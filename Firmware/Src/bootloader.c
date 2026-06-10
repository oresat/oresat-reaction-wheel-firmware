#include "bootloader.h"
#include "printf.h"
//#include "app_openbootloader.h" REMOVED FOR TEST BRINGUP

const bool nBOOT0_REQ = 1;
const bool nBOOT1_REQ = 1;
const bool nSWBOOT0_REQ = 0;
// BOR threshold: level 4 ≈ 2.8 V. The factory default (level 0, ~1.7 V) lets the core keep
// executing — and flash keep PROGRAMMING — far below where a sagging rail corrupts a write in
// progress; a torn cal-store save is exactly the ECC boot-loop trap of flight-review P0-3.
// On a regulated 3.3 V rail, 2.8 V only trips when the rail is genuinely dying.
#define BOR_LEVEL_REQ OB_BOR_LEVEL_4

bool check_option_bytes_ok() {
    // Need to make sure:
    // - nBOOT0 is 1, nBOOT1 is 1, nSWBOOT0 is 0 (the custom PB8 bootloader owns the boot path)
    // - BOR_LEV is level 4 (~2.8 V)

    FLASH_OBProgramInitTypeDef ob_cfg;
    HAL_FLASHEx_OBGetConfig(&ob_cfg);
    u32 option_bytes = ob_cfg.USERConfig;

    bool nBOOT1 = (option_bytes & FLASH_OPTR_nBOOT1) != 0;
    bool nSWBOOT0 = (option_bytes & FLASH_OPTR_nSWBOOT0) != 0;
    bool nBOOT0 = (option_bytes & FLASH_OPTR_nBOOT0) != 0;
    bool bor_ok = (option_bytes & FLASH_OPTR_BOR_LEV) == BOR_LEVEL_REQ;

    bool bytes_ok =
            (nBOOT0 == nBOOT0_REQ) &&
            (nBOOT1 == nBOOT1_REQ) &&
            (nSWBOOT0 == nSWBOOT0_REQ) &&
            bor_ok;

    printf("nBOOT0: %s\r\n", nBOOT0 == nBOOT0_REQ ? "OK" : "BAD");
    printf("nBOOT1: %s\r\n", nBOOT1 == nBOOT1_REQ ? "OK" : "BAD");
    printf("nSWBOOT0: %s\r\n", nSWBOOT0 == nSWBOOT0_REQ ? "OK" : "BAD");
    printf("BOR_LEV: %s\r\n", bor_ok ? "OK" : "BAD");

    return bytes_ok;
}

bool write_option_bytes() {
    FLASH_OBProgramInitTypeDef ob_cfg;
    HAL_FLASHEx_OBGetConfig(&ob_cfg);

    ob_cfg.OptionType = OPTIONBYTE_USER;
    ob_cfg.USERType = OB_USER_nSWBOOT0 | OB_USER_nBOOT0 | OB_USER_nBOOT1 | OB_USER_BOR_LEV;
    ob_cfg.USERConfig = OB_nBOOT0_SET | OB_BOOT0_FROM_OB | OB_BOOT1_SYSTEM | BOR_LEVEL_REQ;

    while ((FLASH->SR & FLASH_SR_BSY) != 0);

    HAL_StatusTypeDef flash_unlock_ok = HAL_FLASH_Unlock();
    if (flash_unlock_ok != HAL_OK) goto err;
    HAL_StatusTypeDef ob_unlock_ok = HAL_FLASH_OB_Unlock();
    if (ob_unlock_ok != HAL_OK) goto err;
    HAL_StatusTypeDef flash_ok = HAL_FLASHEx_OBProgram(&ob_cfg);
    if (flash_ok != HAL_OK) goto err;

    // OB_Launch reloads the option bytes, which is a SYSTEM RESET on this part: on success
    // nothing after this line executes. (The previous tail — lock, OPTSTRT, `return true` —
    // was dead code that implied otherwise.) Callers must flush the console BEFORE calling.
    HAL_FLASH_OB_Launch();

err:
    HAL_FLASH_OB_Lock();
    return false; // only reachable on failure
}

// The DFU/jump logic lives in the separate bootloader image (bootloader/). The app-side
// jump_to_bootloader() and System_DeInit() duplicates that used to sit here were dead code.
