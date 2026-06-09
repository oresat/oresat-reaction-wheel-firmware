#include "cal_store.h"

#include <string.h>

#include "encoder.h"
#include "printf.h"
#include "stm32g4xx_hal.h"

#define CAL_STORE_MAGIC   0x41434131u // "ACA1"
#define CAL_STORE_VERSION 1u

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
    // Light semantic gates on top of the CRC (also reject NaN via the negated compare)
    if (b->encoder_dir != 1.f && b->encoder_dir != -1.f)
        return false;
    if (!(b->phase_offset > -10.f && b->phase_offset < 10.f))
        return false;
    if (b->flags & 1u) { // LUT present: every entry sane (negated compare also rejects NaN)
        for (u32 i = 0; i < CAL_STORE_LUT_N; i++)
            if (!(b->lut[i] > -0.02f && b->lut[i] < 0.02f))
                return false;
    }
    return true;
}

bool cal_store_present() {
    return blob_valid(stored());
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
        staging.flags = 1u;
        encoder_lut_copy(staging.lut);
    }
    staging.crc = blob_crc(&staging);

    // Refuse to persist a never-calibrated state: dir is +/-1 after dir-cal; the power-on
    // default (+1) is indistinguishable, but offset==0 AND dir==+1 together means nothing ran.
    if (staging.encoder_dir != 1.f && staging.encoder_dir != -1.f) {
        printf("cal_store: refusing save, encoder_dir=%.1f invalid\r\n", staging.encoder_dir);
        return false;
    }

    if (memcmp(stored(), &staging, sizeof(staging)) == 0) {
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
    if (!blob_valid(b)) {
        printf("cal_store: no valid blob (magic 0x%lx)\r\n", (unsigned long) b->magic);
        return false;
    }

    encoder_set_direction(b->encoder_dir);
    encoder_set_phase_offset(b->phase_offset);
    // The blob is authoritative for the LUT too: apply it, or clear any live table if absent.
    // (Restore paths run disarmed/at boot, so swapping the table under the PLL is safe.)
    if (b->flags & 1u)
        encoder_set_lut(b->lut);
    else
        encoder_clear_lut();

    printf("cal_store: restored offset %.6f turns, dir %+.0f%s\r\n",
           b->phase_offset, b->encoder_dir, (b->flags & 1u) ? " +LUT" : "");
    return true;
}

bool cal_store_erase() {
    if (!dbank_ok()) return false;
    bool ok = erase_page();
    if (ok)
        printf("cal_store: erased\r\n");
    return ok;
}
