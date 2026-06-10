#include "cal_store.h"

#include <string.h>

#include "crash_record.h"
#include "encoder.h"
#include "foc.h"
#include "printf.h"
#include "stm32g4xx_hal.h"

#define CAL_STORE_MAGIC   0x41434131u // "ACA1"
#define CAL_STORE_VERSION 2u          // v2: + bemf_h6[4] FF coefficients (v1 blobs are
                                      // rejected at restore -> one recal+save after upgrading)

_Static_assert(CAL_STORE_LUT_N == ENC_LUT_N, "blob LUT size must match the encoder's table");

#define CAL_STORE_ADDR 0x0807F800u // last 2 KB page: bank 2, page 127 (DBANK=1)
#define CAL_STORE_BANK FLASH_BANK_2
#define CAL_STORE_PAGE 127u

static cal_blob_t staging; // 536 B; static to keep it off the FSM-ISR stack

// Standard reflected CRC-32 (poly 0xEDB88320, init 0xFFFFFFFF, final xor). Bitwise — runs once
// over 532 bytes per op, speed is irrelevant; avoids configuring the CRC peripheral.
static u32 crc32(const void *data, u32 len) {
    const u8 *p = (const u8 *) data;
    u32 crc = 0xFFFFFFFFu;
    for (u32 i = 0; i < len; i++) {
        crc ^= p[i];
        for (u8 b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static u32 blob_crc(const cal_blob_t *b) {
    return crc32(b, offsetof(cal_blob_t, crc));
}

// The page/bank literals above assume dual-bank mode (2 KB pages). DBANK=0 would make them
// misaddress a 4 KB page — refuse loudly instead. (DBANK is factory-default 1 on this part and
// nothing in this project changes it; this is a tripwire, not an expected path.)
static bool dbank_ok() {
    if ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0u)
        return true;
    printf("cal_store: DBANK=0 (single-bank)?! refusing flash ops — page math assumes dual-bank\r\n");
    return false;
}

static const cal_blob_t *stored() {
    return (const cal_blob_t *) CAL_STORE_ADDR;
}

static bool blob_valid(const cal_blob_t *b) {
    if (b->magic != CAL_STORE_MAGIC || b->version != CAL_STORE_VERSION)
        return false;
    if (blob_crc(b) != b->crc)
        return false;
    // Light semantic gates on top of the CRC. NaN rejection uses f32_finite_bits — the old
    // negated compares were legally foldable under -Ofast's -ffinite-math-only into checks
    // that PASS NaN (flight-review P2-14).
    if (!f32_finite_bits(b->encoder_dir) || (b->encoder_dir != 1.f && b->encoder_dir != -1.f))
        return false;
    if (!f32_finite_bits(b->phase_offset) || !(b->phase_offset > -10.f && b->phase_offset < 10.f))
        return false;
    if (b->flags & 1u) { // LUT present: every entry finite + bounded
        for (u32 i = 0; i < CAL_STORE_LUT_N; i++)
            if (!f32_finite_bits(b->lut[i]) || !(b->lut[i] > -0.02f && b->lut[i] < 0.02f))
                return false;
    }
    if (b->flags & 2u) { // 6f_e FF coeffs present: finite + bounded
        for (u32 i = 0; i < 4; i++)
            if (!f32_finite_bits(b->bemf_h6[i]) || !(b->bemf_h6[i] > -1e-3f && b->bemf_h6[i] < 1e-3f))
                return false;
    }
    return true;
}

bool cal_store_present() {
    noinit_cell_clear(FLASH_ECC_LATCH_CELL);
    bool valid = blob_valid(stored());
    if (noinit_cell_valid(FLASH_ECC_LATCH_CELL)) { // ECC-faulting page is not a valid blob
        noinit_cell_clear(FLASH_ECC_LATCH_CELL);
        return false;
    }
    return valid;
}

static bool erase_page() {
    FLASH_EraseInitTypeDef erase = {
            .TypeErase = FLASH_TYPEERASE_PAGES,
            .Banks = CAL_STORE_BANK,
            .Page = CAL_STORE_PAGE,
            .NbPages = 1,
    };
    u32 page_error = 0;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    if (st != HAL_OK) {
        printf("cal_store: page erase FAILED (err 0x%lx)\r\n", (unsigned long) page_error);
        return false;
    }
    return true;
}

bool cal_store_save() {
    if (!dbank_ok()) return false;

    memset(&staging, 0, sizeof(staging));
    staging.magic = CAL_STORE_MAGIC;
    staging.version = CAL_STORE_VERSION;
    staging.phase_offset = encoder_get_phase_offset();
    staging.encoder_dir = encoder_get_direction();
    staging.psi_m = 0.f;
    if (encoder_lut_enabled_get()) {
        staging.flags |= 1u;
        encoder_lut_copy(staging.lut);
    }
    bemf_h6_coeff_t hc;
    foc_get_bemf_harmonic_coeffs(&hc);
    if (hc.Ad != 0.f || hc.Bd != 0.f || hc.Aq != 0.f || hc.Bq != 0.f) {
        staging.flags |= 2u;
        staging.bemf_h6[0] = hc.Ad;
        staging.bemf_h6[1] = hc.Bd;
        staging.bemf_h6[2] = hc.Aq;
        staging.bemf_h6[3] = hc.Bq;
    }
    staging.crc = blob_crc(&staging);

    // Refuse to persist a never-calibrated state. The provenance flags are authoritative: the
    // power-on defaults (dir +1, offset 0) are value-indistinguishable from a real cal, but
    // the flags only go true on a successful procedure or restore.
    if (!encoder_cal_valid()) {
        printf("cal_store: refusing save — dir/offset cal not valid (run 9 then 15/21 first)\r\n");
        return false;
    }
    if (staging.encoder_dir != 1.f && staging.encoder_dir != -1.f) {
        printf("cal_store: refusing save, encoder_dir=%.1f invalid\r\n", staging.encoder_dir);
        return false;
    }

    // ECC bracket on the compare read: a fault here means the stored page is damaged — fall
    // through to erase+program, which repairs the ECC state along with the contents.
    noinit_cell_clear(FLASH_ECC_LATCH_CELL);
    bool unchanged = (memcmp(stored(), &staging, sizeof(staging)) == 0);
    if (noinit_cell_valid(FLASH_ECC_LATCH_CELL)) {
        printf("cal_store: ECC fault during compare — rewriting page\r\n");
        noinit_cell_clear(FLASH_ECC_LATCH_CELL);
        unchanged = false;
    }
    if (unchanged) {
        printf("cal_store: unchanged (offset %.6f dir %+.0f), skipping write\r\n",
               staging.phase_offset, staging.encoder_dir);
        return true;
    }

    if (!erase_page())
        return false;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    bool ok = true;
    for (u32 off = 0; off < sizeof(staging); off += 8) {
        u64 dword;
        memcpy(&dword, ((const u8 *) &staging) + off, 8);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_STORE_ADDR + off, dword) != HAL_OK) {
            ok = false;
            break;
        }
    }
    HAL_FLASH_Lock();

    ok = ok && (memcmp(stored(), &staging, sizeof(staging)) == 0);
    printf("cal_store: save %s (offset %.6f turns, dir %+.0f)\r\n",
           ok ? "OK" : "FAILED", staging.phase_offset, staging.encoder_dir);
    return ok;
}

bool cal_store_restore() {
    const cal_blob_t *b = stored();

    // ECC bracket: a double-bit flash error anywhere in the page (torn save, radiation) NMIs
    // on the validation read below. The NMI handler latches ECCR and returns (the read
    // completes with garbage), so check the latch afterwards — and ERASE the page on a hit:
    // erased cells carry valid ECC, so the next boot reads clean instead of re-faulting
    // forever. Boot continues uncalibrated either way (flight-review P0-3).
    noinit_cell_clear(FLASH_ECC_LATCH_CELL);
    bool valid = blob_valid(b);
    if (noinit_cell_valid(FLASH_ECC_LATCH_CELL)) {
        u32 eccr = noinit_cell_get(FLASH_ECC_LATCH_CELL);
        noinit_cell_clear(FLASH_ECC_LATCH_CELL);
        // Heal-by-erase only if the latched fault was actually in BANK 2 (BK_ECC) — the cal
        // page is the only thing this firmware uses there. An unrelated bank-1 double-bit
        // fault (ISR code/rodata) landing inside this bracket window must not cost a healthy
        // stored cal (review F6).
        if (eccr & FLASH_ECCR_BK_ECC) {
            printf("cal_store: flash ECC fault in cal page (ECCR 0x%08lx) — erasing page\r\n",
                   (unsigned long) eccr);
            if (dbank_ok())
                erase_page();
        } else {
            printf("cal_store: ECC fault during restore was outside bank 2 (ECCR 0x%08lx) — page kept\r\n",
                   (unsigned long) eccr);
        }
        return false;
    }
    if (!valid) {
        printf("cal_store: no valid blob (magic 0x%lx)\r\n", (unsigned long) b->magic);
        return false;
    }

    encoder_set_direction(b->encoder_dir);
    encoder_set_phase_offset(b->phase_offset);
    // The blob is authoritative for the LUT and FF coeffs: apply, or clear if absent.
    // (Restore paths run disarmed/at boot, so swapping these under the PLL/FOC is safe.)
    if (b->flags & 1u)
        encoder_set_lut(b->lut);
    else
        encoder_clear_lut();

    bemf_h6_coeff_t hc = {0.f, 0.f, 0.f, 0.f};
    if (b->flags & 2u) {
        hc.Ad = b->bemf_h6[0];
        hc.Bd = b->bemf_h6[1];
        hc.Aq = b->bemf_h6[2];
        hc.Bq = b->bemf_h6[3];
    }
    foc_set_bemf_harmonic_coeffs(&hc);

    // A CRC-valid blob is a real calibration: mark the live values as calibrated so the
    // closed-loop states arm without re-running procedures.
    encoder_set_dir_cal_valid(true);
    encoder_set_offset_cal_valid(true);

    printf("cal_store: restored offset %.6f turns, dir %+.0f%s%s\r\n",
           b->phase_offset, b->encoder_dir,
           (b->flags & 1u) ? " +LUT" : "", (b->flags & 2u) ? " +h6FF" : "");
    return true;
}

bool cal_store_erase() {
    if (!dbank_ok()) return false;
    bool ok = erase_page();
    if (ok)
        printf("cal_store: erased\r\n");
    return ok;
}
