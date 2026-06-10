# Bootloader integration (oresat-acs-fw-v8)

> **Status (2026-06-08):** The standalone bootloader (`oresat-acs-bootloader-v8`)
> has been reviewed and is ready. The **§4.1 relink blocker is now done and
> build-verified**: linker FLASH origin `0x08008000`/480K, and `SystemInit` sets
> `SCB->VTOR = 0x08008000` (confirmed by disassembling the ELF; cross-checked
> against the working `oresat-acs-fw-v7`). The §4.2 "do-not-regress" items were
> verified still in place. **Remaining: on-hardware validation (§5)** and the
> optional §4.2 tidy.

## 1. TL;DR

This board runs **two separate firmware images** flashed to different parts of
internal flash: a small **bootloader** at `0x08000000` and **this app** above it
at `0x08008000`. The bootloader runs first on every reset, decides between
"jump to app" and "UART firmware update" based on the BOOT0 pin, and jumps to
the app's vector table at `0x08008000`.

Right now this app is still linked for a no-bootloader world (vector table at
`0x08000000`, full 512K). **It must be relinked to `0x08008000` (§3.1)** or the
bootloader will jump into garbage.

## 2. Background: why two images, and why a *custom* bootloader

This is a reaction-wheel **motor controller**. The STM32's built-in ROM
("system memory") bootloader is deliberately **not** used, because on entry it
reconfigures pins to its own defaults — for a board driving a BLDC inverter,
letting the ROM loader float/redrive the gate-driver pins is unsafe. (See the
`app_main.c` comment: the ROM loader "FUCKS OUR SHIT UP".)

So instead:

- **BOOT0 is taken out of the boot decision in hardware.** The option bytes are
  set `nSWBOOT0=0` + `nBOOT0=1`, which means the chip *always* boots from main
  flash (`0x08000000`) regardless of the BOOT0 pin level. The BOOT0 pin is then
  free to be read as an ordinary GPIO. (This app programs those option bytes —
  see §4.1.)
- **A custom bootloader lives at `0x08000000`.** It reads the BOOT0 pin as a
  GPIO and either jumps to this app or runs ST's OpenBootloader (the same
  AN3155 UART protocol the ROM loader speaks, so `stm32flash` /
  STM32CubeProgrammer still work) for a field firmware update.

This scheme was first done on the v7 board (`oresat-acs-board-v7-bootloader` +
`oresat-acs-fw-v7`). The rev8 board moved to a different STM32G474 package
(**TFBGA100**, the `V` part vs v7's 64-pin `R`), with a different pinout and a
different bootloader UART (**USART1** now, vs USART2 on v7). The bootloader was
re-ported accordingly; this document is the matching app-side checklist.

Repos:
- `oresat-acs-bootloader-v8/` — the bootloader (reviewed, builds at 47.7% of 32K).
- `oresat-acs-fw-v8/` — this app.

## 3. Memory map — the contract

| Image          | Flash region            | Vector table (VTOR) | Role                                       |
|----------------|-------------------------|---------------------|--------------------------------------------|
| bootloader     | `0x08000000` + 32K      | `0x08000000`        | runs first; checks BOOT0; UART DFU         |
| **this app**   | `0x08008000` + 480K     | `0x08008000`        | reaction-wheel firmware                    |

The two tile the 512K flash exactly (`0x08000000`–`0x08008000`–`0x08080000`) and
must not overlap.

Boot flow:

```
reset ──► bootloader @ 0x08000000
            │  HAL_Init → SystemClock (HSE 16 MHz → 144 MHz) → MX_GPIO_Init → MX_IWDG_Init
            │  read BOOT0 pin (PB8)
            ├── HIGH ─► OpenBootloader on USART1 (PE0=TX / PE1=RX, AF7)  ◄── field reflash
            └── LOW  ─► OPENBL_FLASH_JumpToAddress(0x08008000)
                          │  set MSP = *(0x08008000), branch to *(0x08008004)
                          ▼
                        this app's Reset_Handler → SystemInit sets VTOR = 0x08008000
```

The jump sets the app's stack pointer from the first vector-table word and
branches to the second (the reset vector). For interrupts to land in the app,
the app's `SCB->VTOR` must point at `0x08008000` — which is what §3.2 sets up.

## 4. App-side changes required

### 🔴 4.1 BLOCKER — relink the app above the bootloader

Nothing works until this is done. Two values:

1. **`STM32G474VEHX_FLASH.ld`** (currently line ~48):
   ```
   FLASH (rx) : ORIGIN = 0x8000000,  LENGTH = 512K     ← change to:
   FLASH (rx) : ORIGIN = 0x8008000,  LENGTH = 480K
   ```
   (480K = 512K − 32K reserved for the bootloader. RAM regions are unchanged.)

2. **`Core/Src/system_stm32g4xx.c`** — ⚠️ *two* edits, not one. The
   `SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET` write in `SystemInit()` is gated
   behind `#if defined(USER_VECT_TAB_ADDRESS)`, and that macro ships **commented out**.
   So editing the offset alone is a silent no-op — it builds clean and even passes the
   `objdump -h` check below (vector LMA is linker-set), but VTOR stays at reset `0x0` →
   every IRQ dispatches through the bootloader's table. Both edits are required:
   - line ~110: `/* #define USER_VECT_TAB_ADDRESS */` → `#define USER_VECT_TAB_ADDRESS`
   - line ~125: `#define VECT_TAB_OFFSET  0x00000000U` → `#define VECT_TAB_OFFSET  0x00008000U`

   Net: `SCB->VTOR = FLASH_BASE | 0x8000 = 0x08008000`. This is exactly how the working
   `oresat-acs-fw-v7` does it (`USER_VECT_TAB_ADDRESS` defined, `VECT_TAB_OFFSET 0x8000`).

**Verify after building:**
```bash
arm-none-eabi-objdump -h cmake-build-debug/oresat-acs-fw-v8.elf | grep isr_vector
#   -> VMA / LMA should be 08008000
arm-none-eabi-size  cmake-build-debug/oresat-acs-fw-v8.elf
#   -> text+data must be < 480K (current app ~88K, so there is ample margin)
# ^ the two checks above pass even if VTOR is never set. Prove the relocation too:
arm-none-eabi-objdump -d --disassemble=SystemInit cmake-build-debug/oresat-acs-fw-v8.elf
#   -> must store 0x08008000 to VTOR: a `str` to [SCB(0xE000ED00) + 0x8]
```

⚠️ **Regeneration caveat.** Per `CLAUDE.md`, CubeIDE can regenerate the `.ld` and
CubeMX regenerates `Core/`. `system_stm32g4xx.c` is a CMSIS file that is normally
left untouched, but if either of these two files gets regenerated, **re-apply
both values.** Consider adding a line to `docs/footguns.md`.

### 🟢 4.2 Already correct — do not regress

**Option-byte enforcement is wired and must stay on the boot path.**
`check_option_bytes_ok()` / `write_option_bytes()` (`Firmware/Src/bootloader.c`,
`nBOOT0_REQ=1 / nBOOT1_REQ=1 / nSWBOOT0_REQ=0`) are called at
`app_main.c:77-83`. On first boot of a fresh chip these program the option bytes
described in §2 and reboot. Without them the chip can fall into the ROM loader on
a BOOT0-high reset. Keep this early in startup.

> First-boot note: on a *virgin* chip the option bytes are still at factory
> defaults (BOOT0 from pin). Flash both images via SWD with BOOT0 **low**; the
> app then programs the option bytes and reboots, after which BOOT0 is a GPIO and
> the bootloader owns boot. Same behaviour as v7.

**OpenBootloader has already been excised from the app.** `bootloader.c:3` (the
`app_openbootloader.h` include) and `bootloader.c:73-79` (`jump_to_bootloader()`)
are commented out ("REMOVED FOR TEST BRINGUP"), and there are no
`OpenBootloader_APP/` or `Middleware/` dirs. The standalone bootloader owns
BOOT0/DFU now — do **not** re-add OpenBootloader to the app.

*Optional final tidy:* delete the commented-out block and the
`jump_to_bootloader()` declaration in `bootloader.h`, and fix the now-stale
`CLAUDE.md` line ("Boot pin is checked in software and a custom jump-to-bootloader
path is taken") — that path lives in the separate bootloader now, not here.

### 🟡 4.3 Heads-up — IWDG continuity (probably no change needed)

The v8 bootloader calls `MX_IWDG_Init()` **unconditionally** before the jump, so
the app **inherits a live independent watchdog** (~512 ms: LSI ≈ 32 kHz ÷ 4,
reload 4096). A software jump does not reset the IWDG, and once started it cannot
be stopped — so the app *must* service it.

This app already does, via the `HAL_IncTick` override in `Core/Src/iwdg.c` that
reloads the IWDG every 1 ms from the SysTick ISR — kicking `IWDG->KR` directly
(deref-free: safe even before `.bss` init in the post-jump window), and gated on
a **foreground liveness heartbeat** once `app_main`'s main loop starts (a wedged
foreground stops the feed → clean IWDG reboot ≤ ~2.5 s; boot path feeds
unconditionally). Keep that intact, and preserve two invariants:

- The app reaches `MX_IWDG_Init()` within ~512 ms of the jump (it does — it's in
  the early MX-init sequence; startup to there is sub-millisecond).
- Nothing disables interrupts / SysTick for **> ~512 ms** anywhere (e.g. long
  flash erase/program with IRQs off). The option-byte path is fine because it
  reboots. If the firmware rework adds long critical sections, kick the dog
  manually (`__HAL_IWDG_RELOAD_COUNTER(&hiwdg)`).

### ⚪ 4.4 FYI / optional

- **No UART contention.** App debug/printf is **USART2 on PD5/PD6**
  (`DEBUG_UART_HANDLE`); the bootloader's DFU UART is **USART1 on PE0/PE1**.
  Different peripherals and pins — the app keeps full use of USART2.
- **Bootloader entry is BOOT0-pin-only — plus the invalid-app fallback.** A CAN
  reboot (OD `0x4007`) is a plain reset, so the board comes back into the app
  unless BOOT0 is held high at reset. Since 2026-06 the bootloader also validates
  the app vector table before jumping (SP in SRAM, PC thumb-bit + in-image): an
  erased/half-written app drops into the DFU loop instead of HardFault-looping,
  so a power-cut mid-reflash is recoverable over UART without touching BOOT0.
  The bootloader also starts the IWDG *before* clock config (a dead crystal is a
  512 ms retry loop, not a watchdog-less hang) and its `Error_Handler` resets.
  If remote/CAN-commanded reflash is ever needed, add a *soft trigger*: the app
  writes a magic value to an RTC backup register / noinit-RAM word and resets;
  the bootloader enters OpenBootloader on `pin-high OR flag`. This is a net-new
  feature, not required for the split to work.

## 5. Verification checklist

- [x] `.ld` FLASH origin = `0x8008000`, length `480K`.
- [x] `USER_VECT_TAB_ADDRESS` uncommented **and** `VECT_TAB_OFFSET = 0x00008000U`.
- [x] `objdump -h` shows `.isr_vector` at `0x08008000`.
- [x] `SystemInit` disassembly stores `0x08008000` to VTOR.
- [x] App flash usage < 480K (87.8K / 480K).
- [x] Option-byte check/write still called at app_main startup (`app_main.c:77-83`).
- [x] On hardware: flash both images (BOOT0 low) → app runs, sets option bytes,
      reboots, runs again cleanly with no watchdog reset loop.
      (Verified 2026-06-10: weeks of bench runtime behind the bootloader; SWD-read
      `FLASH_OPTR = 0xFBEFF8AA` → nSWBOOT0=0 / nBOOT0=1 / nBOOT1=1.)
- [ ] On hardware: hold BOOT0 high at reset → `stm32flash` / CubeProgrammer can
      talk to the bootloader on USART1 (PE0/PE1) and reflash the app region.
      (Half-verified 2026-06-10: BOOT0-high reset does land in the DFU loop with
      the correct protocol config — SWD-read VTOR=0x00000000, USART1
      CR1=0x0000140D (8E1), CR2=0x00500000 (autobaud-on-0x7F), BRR=0x4E2.
      `stm32flash` init still times out → wiring/adapter side under test.
      Gotcha: in the *app*, PE0/PE1 carry the idle COMMS USART1 at 921600-8N1 —
      electrically alive but deaf to 115200-8E1, so a failed BOOT0 strap
      perfectly mimics a dead DFU.)

## 6. Background facts (for reference)

- MCU: STM32G474VEH (TFBGA100), Cortex-M4F @ 144 MHz, 512K flash / 128K RAM.
- HSE = 16 MHz; PLL `M=1, N=18, R=2` → 144 MHz (identical across bootloader and
  app — confirmed consistent).
- BOOT0 pin = **PB8** (read as GPIO input by the bootloader).
- Bootloader DFU UART = **USART1**, PE0 (TX) / PE1 (RX), AF7, 115200-8E1 with
  autobaud on the `0x7F` sync byte.
- Bootloader review notes (already addressed in the vendored `bootloader/`):
  GPIO-port clock-enable now follows the selected UART handle; redundant USART2
  clock-enable removed; stale pin comment fixed; autobaud re-armed on ABRE error
  (2026-06-10 — a garbage first byte used to latch ABRE and deafen the DFU until
  the next reset; `OPENBL_USART_ProtocolDetection` now writes ABRRQ + flushes RX).

## 7. Cold bootstrap via the ROM bootloader (UART / USB DFU)

For a **virgin board** (or after a full chip erase) the custom bootloader isn't
on the chip yet, so neither SWD `./flash.sh` nor the custom UART DFU is available.
You bootstrap through the STM32 **ROM (system-memory) bootloader**, flash both
images at once, and let the app fix the option bytes on first run.

**Why it's safe to use the ROM loader here:** power **only the 5 V logic test
point**, not the inverter rail. The ROM loader reconfigures pins to its own
defaults on entry — harmless while the gate drivers are unpowered (that pin
behaviour is the whole reason it's bypassed in normal operation).

**⚠️ Wiring gotcha — the ROM loader uses ST's *fixed* pins, NOT the custom DFU
pins.** The custom bootloader's DFU is USART1 on **PE0/PE1**, but the ROM
bootloader only listens on its own hardwired pins (per **AN2606**): typically
**USART1 PA9/PA10**, **USART2 PA2/PA3**, or **USB DFU PA11/PA12** (it auto-scans
several). Connect your adapter to one of *those* — talking to PE0/PE1 will get
you nothing.

**Procedure:**

1. Power the 5 V logic test point only (inverter unpowered).
2. Pull **BOOT0 high**, reset/power-on → ROM bootloader runs (factory option
   bytes are `nSWBOOT0=1`, i.e. BOOT0-from-pin, so BOOT0-high → system memory).
3. Build the combined image:
   ```bash
   ./build.sh           # builds app + bootloader
   ./bootstrap-bin.sh   # -> combined-bootstrap.bin (bootloader @0x08000000 + app @0x08008000)
   ```
4. Flash the **whole file at 0x08000000** (one pass places both images):
   ```bash
   stm32flash -b 115200 -w combined-bootstrap.bin -S 0x08000000 -v /dev/ttyUSB0   # UART
   dfu-util  -a 0 -s 0x08000000 -D combined-bootstrap.bin                          # USB DFU
   ```
5. Set **BOOT0 low** and power-cycle. Now boot goes: main flash → custom
   bootloader → (BOOT0 low) → app → `check_option_bytes_ok()` fails →
   `write_option_bytes()` sets `nBOOT0=1 / nSWBOOT0=0` → reboot. Option bytes are
   now correct, BOOT0 is henceforth a plain GPIO, and the board boots normally.
   (Watch the `nBOOT0/nSWBOOT0 OK` printout on the debug UART — USART2, PD5/PD6.)
6. Done. Use SWD `./flash.sh` from here on; the custom UART DFU (PE0/PE1) is now
   available for field updates.

**Two-file alternative** (same result, two ROM-loader cycles) if you'd rather not
build a combined image:
```bash
stm32flash -w bootloader/cmake-build-debug/oresat-acs-bootloader-v8.bin -S 0x08000000 -v <port>
stm32flash -w cmake-build-debug/oresat-acs-fw-v8.bin                    -S 0x08008000 -v <port>
```

**Gotchas:**
- After flashing with BOOT0 still high, **don't** `-g 0x08000000` (Go): that enters
  the custom bootloader, which sees BOOT0 high and drops into UART DFU on PE0/PE1
  instead of running the app. Use the BOOT0-low power-cycle (step 5), or jump
  straight to the app with `-g 0x08008000`.
- The option-byte write runs *before* any inverter/encoder code
  (`app_main.c:77`, ahead of DAC/CORDIC/encoder at 99+), so 5 V-only power is
  enough for the bootstrap to complete and reboot.
- `combined-bootstrap.bin` is `*.bin`-gitignored; regenerate it with
  `./bootstrap-bin.sh` whenever the images change.
