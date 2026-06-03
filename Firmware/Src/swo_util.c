#include "swo_util.h"


bool swo_setup() {
    TpiuOptions tpiu_options = {
        .Protocol = (TpiuProtocol) TpiuProtocolSwoManchester,
        .FormattingEnabled = false,
        .SwoPrescaler = 1,
        .TracePortWidth = 1
    };

    TpiuSetup(&tpiu_options);

    ITMOptions itm_options = {
        .TraceBusID = 0,
        .GlobalTimestampFrequency = 0,
        .LocalTimestampPrescaler = 0,
        .EnableLocalTimestamp = false,
        .ForwardDWT = false,
        .EnableSyncPacket = false,
        .EnabledStimulusPorts = ITM_ENABLE_STIMULUS_PORTS_ALL
    };

    ITMSetup(&itm_options);

    return ITMIsPortEnabled(0);
}

void swo_printbytes(u8 *const bytes, u16 len) {
    for (u16 i = 0; i < len; i++) {
        while (ITM->PORT[0].u32 == 0UL)
            __NOP();
        ITM->PORT[0].u8 = bytes[i];
    }
}

void swo_printwords(u32 *const words, u16 len) {
    for (u16 i = 0; i < len; i++) {
        while (ITM->PORT[0].u32 == 0UL)
            __NOP();
        ITM->PORT[0].u32 = words[i];
    }
}
