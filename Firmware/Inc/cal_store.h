#pragma once

#include "typedefs.h"

/*
 * Persistent calibration store — one flash page holding encoder cal so it survives reboots AND
 * firmware updates. Driven over CAN via FSM states (the CLEAR_ERRORS "state as RPC" pattern):
 *   FSM_STATE_SAVE_CALIBRATION    (18) — snapshot live cal (offset/dir) -> flash
 *   FSM_STATE_ERASE_CALIBRATION   (19) — wipe the page
 *   FSM_STATE_RESTORE_CALIBRATION (20) — flash -> live cal
 * plus an automatic restore at boot (app_main). Result is published as procedure_result
 * (fsm_last_state_exit_ok) on TPDO1, so the host sees success/failure.
 *
 * Location: the LAST 2 KB page of flash (dual-bank: bank 2, page 127, 0x0807F800). The app's
 * linker region is capped at 478K (STM32G474VEHX_FLASH.ld) so no firmware image can reach this
 * page, and both update paths (flash.sh SWD `program`, UART DFU via STM32CubeProgrammer default
 * download) erase only image extents -> the blob survives updates. A host-commanded MASS erase
 * still wipes it (and the bootloader itself — stock OpenBL has no self-protection): never mass-
 * erase in update procedures.
 *
 * Runtime context: save/erase run from the FSM tick (TIM17 ISR). The app executes from bank 1
 * (92 KB image << 224 KB bank-1 budget) and the cal page is in bank 2, so dual-bank
 * read-while-write applies: flash ops cause NO instruction stalls. If the app image ever grows
 * past 0x08040000 this stops being true — revisit then.
 *
 * See docs/backemf-coastdown-implementation.md §10.
 */

#define CAL_STORE_LUT_N 128u // mech-angle error-map bins (reserved now; filled by the future
                             // on-board LUT fit so the flash layout doesn't churn)

typedef struct cal_blob {
    u32 magic;                 // CAL_STORE_MAGIC
    u16 version;               // CAL_STORE_VERSION; bump on layout change
    u16 flags;                 // bit0: LUT populated (future). 0 today.
    f32 phase_offset;          // encoder phase offset, mech turns
    f32 encoder_dir;           // +1.f / -1.f
    f32 psi_m;                 // flux linkage [V*s], informational; 0 = unknown
    f32 lut[CAL_STORE_LUT_N];  // mech-angle error map [turns], indexed by measured angle. Zeroed
                               // until the on-board fit lands (see encoder-lut-design.md).
    u32 crc;                   // CRC-32 (reflected, poly 0xEDB88320) over all preceding bytes
} cal_blob_t;

_Static_assert(sizeof(cal_blob_t) == 536, "cal blob layout changed — bump CAL_STORE_VERSION");
_Static_assert(sizeof(cal_blob_t) % 8 == 0, "blob must be a whole number of flash doublewords");

// True if a valid (magic+version+CRC) blob is present in the cal page.
bool cal_store_present();

// Snapshot the live cal (encoder offset/dir) into the page. Skips the write when the stored
// blob is already identical (wear). Refuses if encoder_dir is not +/-1. Returns success.
bool cal_store_save();

// Validate the stored blob and apply it to the live encoder state. Returns false (and changes
// nothing) if absent/corrupt/out-of-range.
bool cal_store_restore();

// Erase the cal page. Returns success.
bool cal_store_erase();
