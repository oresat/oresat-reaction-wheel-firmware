#pragma once

#include "typedefs.h"

// Warm-reset-surviving diagnostic cells at the top of RAM2 (0x20014000–0x20017FFF on the
// contiguous SRAM alias). The app's MAIN STACK also lives in RAM2 and grows DOWN from _estack
// — so the linker explicitly reserves this block by setting
// `_estack = ORIGIN(RAM2)+LENGTH(RAM2)-0x58` (STM32G474VEHX_FLASH.ld; keep the two in sync!).
// The bootloader's statics end ~0x20000FB0 with its stack in the CCM alias, so plain RAM
// retention carries these across NVIC_SystemReset / IWDG resets. Lost on power cycle, which is
// exactly the lifetime a crash/boot-loop breadcrumb wants. Fixed absolute addresses (not
// linker sections) to match the existing bootloader mailbox convention and keep both images'
// layouts decoupled.
//
//   0x20017FF8  bootloader RCSR mailbox (2 words) — owned by bootloader/Src/main.c, keep in sync
//   0x20017FC0  crash record (last fault), 40 B
//   0x20017FB8  boot counter cell
//   0x20017FB0  flash-ECC NMI latch cell (raw ECCR snapshot)
//   0x20017FA8  option-byte write attempt counter cell
//
// Read back over CAN: app_main packs reset cause / boot count / crash kind+PC into the spare
// 0x4006 floats at boot (publish_boot_diagnostics, can_comms.c) — see docs/can-interface.md.

#define CRASH_RECORD_ADDR     0x20017FC0u
#define BOOT_COUNT_ADDR       0x20017FB8u
#define FLASH_ECC_LATCH_ADDR  0x20017FB0u
#define OB_ATTEMPT_ADDR       0x20017FA8u

#define CRASH_RECORD_MAGIC    0x43525348u // "CRSH"
#define NOINIT_CELL_MAGIC     0x4F524553u // "ORES"

typedef enum {
    CRASH_KIND_NONE = 0,
    CRASH_KIND_HARDFAULT = 1,     // any escalated fault (MemManage/Bus/Usage are not enabled)
    CRASH_KIND_NMI_UNKNOWN = 2,   // NMI with no flash-ECC flag set (should be impossible: CSS
                                  // and SRAM parity are not enabled on this configuration)
    CRASH_KIND_ERROR_HANDLER = 3, // HAL init failure path (Error_Handler)
    CRASH_KIND_CCM_SCRUB = 4,     // CCM-resident code failed its CRC scrub (SEU)
} crash_kind_t;

typedef struct {
    u32 magic;
    u32 kind; // crash_kind_t
    u32 cfsr;
    u32 hfsr;
    u32 bfar;
    u32 mmfar;
    u32 stacked_pc;
    u32 stacked_lr;
    u32 controller_errors;
    u32 aux; // kind-specific (CCM scrub: failing word address)
} crash_record_t;

#define CRASH_RECORD ((volatile crash_record_t *) CRASH_RECORD_ADDR)

// Generic magic-tagged value cell.
typedef struct {
    u32 magic;
    u32 value;
} noinit_cell_t;

#define BOOT_COUNT_CELL      ((volatile noinit_cell_t *) BOOT_COUNT_ADDR)
#define FLASH_ECC_LATCH_CELL ((volatile noinit_cell_t *) FLASH_ECC_LATCH_ADDR)
#define OB_ATTEMPT_CELL      ((volatile noinit_cell_t *) OB_ATTEMPT_ADDR)

static inline void noinit_cell_set(volatile noinit_cell_t *c, u32 v) {
    c->value = v;
    c->magic = NOINIT_CELL_MAGIC;
}

static inline bool noinit_cell_valid(const volatile noinit_cell_t *c) {
    return c->magic == NOINIT_CELL_MAGIC;
}

static inline void noinit_cell_clear(volatile noinit_cell_t *c) {
    c->magic = 0u;
}

static inline u32 noinit_cell_get(const volatile noinit_cell_t *c) {
    return noinit_cell_valid(c) ? c->value : 0u;
}
