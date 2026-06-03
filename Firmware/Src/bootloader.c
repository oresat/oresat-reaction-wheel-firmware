#include "bootloader.h"
#include "printf.h"
//#include "app_openbootloader.h" REMOVED FOR TEST BRINGUP

const bool nBOOT0_REQ = 1;
const bool nBOOT1_REQ = 1;
const bool nSWBOOT0_REQ = 0;

bool check_option_bytes_ok() {
    // Need to make sure:
    // - nBOOT0 is 1
    // - nBOOT1 is 1
    // - nSWBOOT0 is 0

    FLASH_OBProgramInitTypeDef ob_cfg;
    HAL_FLASHEx_OBGetConfig(&ob_cfg);
    u32 option_bytes = ob_cfg.USERConfig;

    bool nBOOT1 = (option_bytes & FLASH_OPTR_nBOOT1) != 0;
    bool nSWBOOT0 = (option_bytes & FLASH_OPTR_nSWBOOT0) != 0;
    bool nBOOT0 = (option_bytes & FLASH_OPTR_nBOOT0) != 0;

    bool bytes_ok =
            (nBOOT0 == nBOOT0_REQ) &&
            (nBOOT1 == nBOOT1_REQ) &&
            (nSWBOOT0 == nSWBOOT0_REQ);

    printf("nBOOT0: %s\r\n", nBOOT0 == nBOOT0_REQ ? "OK" : "BAD");
    printf("nBOOT1: %s\r\n", nBOOT1 == nBOOT1_REQ ? "OK" : "BAD");
    printf("nSWBOOT0: %s\r\n", nSWBOOT0 == nSWBOOT0_REQ ? "OK" : "BAD");

    return bytes_ok;
}

bool write_option_bytes() {
    FLASH_OBProgramInitTypeDef ob_cfg;
    HAL_FLASHEx_OBGetConfig(&ob_cfg);

    ob_cfg.OptionType = OPTIONBYTE_USER;
    ob_cfg.USERType = OB_USER_nSWBOOT0 | OB_USER_nBOOT0 | OB_USER_nBOOT1;
    ob_cfg.USERConfig = OB_nBOOT0_SET | OB_BOOT0_FROM_OB | OB_BOOT1_SYSTEM;
    //ob_cfg.USERType = OB_USER_nSWBOOT0;
    //ob_cfg.USERConfig = OB_BOOT0_FROM_OB;

    while ((FLASH->SR & FLASH_SR_BSY) != 0);

    HAL_StatusTypeDef flash_unlock_ok = HAL_FLASH_Unlock();
    if (flash_unlock_ok != HAL_OK) goto err0;
    HAL_StatusTypeDef ob_unlock_ok = HAL_FLASH_OB_Unlock();
    if (ob_unlock_ok != HAL_OK) goto err1;
    HAL_StatusTypeDef flash_ok = HAL_FLASHEx_OBProgram(&ob_cfg);
    if (flash_ok != HAL_OK) goto err2;
    HAL_StatusTypeDef launch_ok = HAL_FLASH_OB_Launch();
    if (launch_ok != HAL_OK) goto err3;
    HAL_StatusTypeDef lock_ok = HAL_FLASH_OB_Lock();
    if (lock_ok != HAL_OK) goto err4;

    FLASH->CR |= FLASH_CR_OPTSTRT;

    while ((FLASH->SR & FLASH_SR_BSY) != 0);

    return true;

err0:
err1:
err2:
err3:
err4:
    HAL_FLASH_OB_Lock();
    return false;
}

// REMOVED FOR TEST BRINGUP
//void jump_to_bootloader() {
//    OpenBootloader_Init();
//    while (1) {
//        OpenBootloader_ProtocolDetection();
//    }
//}


void System_DeInit() {
    HAL_RCC_DeInit();
}
