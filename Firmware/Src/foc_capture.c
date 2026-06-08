#include "foc_capture.h"

volatile foc_capture_t   foc_capture_buf[FOC_CAPTURE_LEN];
volatile u32             foc_capture_idx   = 0;
volatile foc_capture_state_t foc_capture_state = FOC_CAPTURE_IDLE;

void foc_capture_arm(void) {
    foc_capture_idx   = 0;
    foc_capture_state = FOC_CAPTURE_ARMED;
}
