# OreSat ACS reaction-wheel firmware (rev 8)

Motor-controller firmware for the OreSat reaction wheels: STM32G474, field-oriented control,
CANopen at 1 Mbit/s. Two images live on the chip: a UART bootloader (`bootloader/`, at
`0x08000000`) and the app (at `0x08008000`).

## Flash a release

No compiler needed, only an ST-Link, OpenOCD and curl:

```bash
sudo apt install openocd curl
./flash-release.sh            # lists the releases
./flash-release.sh v1.0.2
```

It downloads the images CI built for that tag, backs up what is on the chip, flashes, and
checks the app came up. The encoder calibration is kept unless you pass `--erase-cal`. If it
cannot reach the ST-Link or the board, it says which and why.

## Build it yourself

```bash
sudo apt install cmake make curl xz-utils
./toolchain.sh                # pinned compiler into .toolchain/ -- no root, nothing system-wide
./build.sh                    # app + bootloader
./flash.sh                    # both over SWD
```

`./toolchain.sh` fetches the same Arm GNU Toolchain the releases are built with, so the distro's
own `arm-none-eabi-gcc` (or the lack of one) does not matter. Without it, `build.sh` falls back
to whatever is on `PATH`.

## Talk to a wheel

`can_console/` is the host-side CAN tool (`python cli.py scan`, `status`, `commission`, ...);
see its README. The CAN interface is described in `docs/can-interface.md`.

## Documentation

`docs/` holds the design notes; `CLAUDE.md` is the map of the codebase and of those notes.

## License

The OreSat ACS firmware in this repository is licensed under the GNU General Public License,
version 3 -- see `LICENSE`. Vendored third-party code keeps its own license:

| Path | License |
|---|---|
| `Drivers/STM32G4xx_HAL_Driver`, `bootloader/Drivers/STM32G4xx_HAL_Driver` | BSD-3-Clause |
| `Drivers/CMSIS`, `bootloader/Drivers/CMSIS`, `DSP/` | Apache-2.0 (`DSP/ComputeLibrary`: MIT) |
| `CANopenSTM32/CANopenNode` | Apache-2.0 |
| `CANopenSTM32/` (STM32 glue, outside `CANopenNode`) | MIT |
| `Firmware/Src/printf.c`, `Firmware/Inc/printf.h` | MIT |
| `openocd.cfg`, `bootloader/st_nucleo_g4.cfg` | GPL-2.0-or-later |
| `bootloader/Middleware/OpenBootloader` | ST Ultimate Liberty SLA0044 (use restricted to ST devices) |

SLA0044 is not GPL-compatible, so the GPL here covers the app image and this project's own
source files, not the bootloader image as a whole: the bootloader links ST's OpenBootloader
middleware and is distributed under ST's terms for that part. The two are separate images and
the app does not link the bootloader, so this does not affect the app's license.
