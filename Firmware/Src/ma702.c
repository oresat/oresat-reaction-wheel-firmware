#include "ma702.h"


#include "util.h"

#define DEFAULT_MGL_THRESH_REG_VAL ((0b000 << 5) | (0b111 << 2))

// Set the register read to angle
void ma702_preheat() {
    u16 rx_buf[] = {0x0};
    u16 tx_buf[] = {0x0};

    set_nCS(0);
    HAL_SPI_TransmitReceive(&ENCODER_SPI_HANDLE, (u8 *)tx_buf, (u8 *)rx_buf, 1,
                            0xFFFF);
    //printf("%u\r\n", rx_buf[0] >> 2);
    set_nCS(1);
}

ma702_field_strength_flag_t ma702_read_field_strength_flags() {
    // First write the register
    u16 tx_buf[] = {((u16)((0b010 << 5) | 0x1B)) << 8};
    u16 rx_buf[] = {0x0};

    set_nCS(0);
    HAL_SPI_TransmitReceive(&ENCODER_SPI_HANDLE, (u8 *)tx_buf, (u8 *)rx_buf, 1, 0xFFFF);
    set_nCS(1);

    // Spinlock to honor min reg read time
    for (u8 i = 0; i < 128; i++)
        __asm__("nop");


    // Then read the contents, resetting tx_buf to 0 to queue an angle read next
    tx_buf[0] = 0x0;
    rx_buf[0] = 0x0;

    set_nCS(0);
    HAL_SPI_TransmitReceive(&ENCODER_SPI_HANDLE, (u8 *)tx_buf, (u8 *)rx_buf, 1, 0xFFFF);
    set_nCS(1);

    // Spinlock to honor min reg read time
    for (u8 i = 0; i < 128; i++)
        __asm__("nop");

    u16 field_thresh = rx_buf[0] >> 8;

    bool mgh, mgl __maybe_unused, mg1l, mg2l;
    mgh = (field_thresh & 0x80) == 0x80;
    mgl = (field_thresh & 0x40) == 0x40;
    mg1l = (field_thresh & 0x8) == 0x8;
    mg2l = (field_thresh & 0x4) == 0x4;

    bool mgl_corr = !(mg1l | mg2l);
    if (mgh && !mgl_corr)
        return FIELD_HIGH;
    else if (!mgh && mgl_corr)
        return FIELD_LOW;
    else if (!mgh && !mgl_corr)
        return FIELD_IN_RANGE;
    else
        return FIELD_UNK;
}

static bool ma702_write_field_thresh_reg(const u8 reg_val) {
    u16 reg_cmd = (((u16)((0b100 << 5) | 0x6))<<8) | reg_val;

    u16 tx_buf[] = {reg_cmd};
    u16 rx_buf[] = {0x0};

    set_nCS(0);
    HAL_SPI_TransmitReceive(&ENCODER_SPI_HANDLE, (u8 *)tx_buf, (u8 *)rx_buf, 1, 0xFFFF);
    set_nCS(1);

    HAL_Delay(21); // Delay for reg write to work

    tx_buf[0] = 0x0; // Reset to angle read
    rx_buf[0] = 0x0;

    set_nCS(0);
    HAL_SPI_TransmitReceive(&ENCODER_SPI_HANDLE, (u8 *)tx_buf, (u8 *)rx_buf, 1, 0xFFFF);
    set_nCS(1);

    // Spinlock to honor min reg read time
    for (u8 i = 0; i < 128; i++)
        __asm__("nop");

    u16 reg_readback = rx_buf[0] >> 8;

    return reg_readback == reg_val; // Confirm values written correctly
}


u8 ma702_get_estimated_field_strength() {
    ma702_field_strength_flag_t field_regs[8];

    for (u8 i = 0; i < 8; i++) {
        // Set MGLT to i and MGHT to (7-i)
        u8 thresh_reg = (i << 5) | (i << 2);
        if (!ma702_write_field_thresh_reg(thresh_reg))
            goto write_fail;

        field_regs[i] = ma702_read_field_strength_flags();
    }

    if (!ma702_write_field_thresh_reg(DEFAULT_MGL_THRESH_REG_VAL))
        goto write_fail;

    if (field_regs[0] == FIELD_LOW) return 0;
    if (field_regs[1] == FIELD_LOW) return 30;
    if (field_regs[2] == FIELD_LOW) return 45;
    if (field_regs[3] == FIELD_LOW) return 60;
    if (field_regs[4] == FIELD_LOW) return 74;
    if (field_regs[5] == FIELD_LOW) return 88;
    if (field_regs[6] == FIELD_LOW) return 102;
    if (field_regs[7] == FIELD_LOW) return 116;
    return 255;

write_fail:
    return 0;
}

u16 ma702_read_angle() {
    u16 rx_buf[] = {0x0};
    u16 tx_buf[] = {0x0};

    set_nCS(0);
    HAL_SPI_TransmitReceive(&ENCODER_SPI_HANDLE, (u8 *)tx_buf, (u8 *)rx_buf, 1,
                            0xFFFF);
    set_nCS(1);

    return rx_buf[0] >> 2;
}


volatile u16 current_angle_dma_raw = 0;
volatile u32 angle_timestamp_dma = 0;
volatile u32 angle_timestamp_completed_dma = 0;
volatile bool encoder_read_success_dma = false;
volatile bool encoder_dma_in_progress = false;