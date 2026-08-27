<!-- Hebrew version: README.he.md -->

# RADIOROC2 — gamma spectrometer

Firmware for an STM32F722ZE Nucleo driving a RADIOROC2 ASIC, plus the Python
desktop application that configures it, takes measurements, and plots spectra.

Everything needed to build is committed here: the HAL drivers and CMSIS headers
are vendored, so no STM32Cube firmware pack has to be downloaded separately.

**New here? Read this file to get running, then read
[python/TOOLS_README.md](python/TOOLS_README.md) — it is the real manual, and it
covers the protocol, every ASIC parameter, the threshold scan and the
troubleshooting table.**

---

## 1. What you need

| | |
|---|---|
| Board | NUCLEO-F722ZE (STM32F722ZET6) |
| IDE | STM32CubeIDE — the build here is verified on 2.1.1 |
| Python | 3.8 or newer, with `tkinter` |
| Cable | One USB cable to the ST-Link. It carries the debugger **and** the data |

There is no separate USB data connector. The board talks over **USART3 routed to
the ST-Link Virtual COM Port at 921600 baud**, which is a real UART and not a CDC
device — so the baud rate genuinely matters. It is defined once in
[Core/Inc/usart.h](Core/Inc/usart.h) as `RR2_LINK_BAUD` and mirrored in
`python/rr2_decode.py`.

## 2. Get the code

```bash
git clone https://github.com/noamkorinperi/radiocroc2_project.git
```

`main` is the branch you want; it holds the current firmware and GUI.

## 3. Build and flash the firmware

1. **File → Open Projects from File System…**, point it at the cloned folder,
   and import. The Eclipse project is named `RADIOROC2_Firmware` regardless of
   what you called the folder.
   *Do not* use the CubeMX project importer — the `.project` and `.cproject`
   here are already complete.
2. **Project → Build Project** (Ctrl+B). A clean build finishes with
   **0 errors, 0 warnings** and reports roughly `52356 text, 100 data, 12004 bss`.
3. Connect the board and **Run → Run As → STM32 C/C++ Application**. A ready-made
   debug configuration, `RADIOROC2_Firmware Debug`, is committed alongside.

The `Debug/` folder is deliberately not in git — CubeIDE regenerates all of it
from the sources and `.cproject`.

### If you edit the pin configuration

Open [RADIOROC2_Firmware.ioc](RADIOROC2_Firmware.ioc) in CubeMX and regenerate;
do not hand-edit the generated blocks in `gpio.c`, `tim.c` or `adc.c`, because
the next generate will silently discard your changes. Anything of your own goes
between the `USER CODE BEGIN` / `USER CODE END` markers.

## 4. Run the desktop application

```bash
cd python
pip install -r requirements.txt
python rr2_gui.py
```

Only `pyserial` is strictly required. `openpyxl` and `pillow` are optional: without
them Excel export falls back to CSV and the screenshot button explains itself,
and nothing else changes.

### Try it without hardware

```bash
python rr2_gui.py --sim
```

Simulation mode synthesises a Gaussian photopeak with a Compton-like tail at
about 180 counts/s. It is the fastest way to learn the interface before the board
is on the bench.

### Check the link

```bash
python rr2_decode.py --list
```

This lists the serial ports and marks which one is the ST-Link. Both tools find
the board by USB VID/PID, so you never need to know the COM number — which
changes between USB sockets anyway.

## 5. Layout

```
Core/          Firmware. Application code is radioroc2*.c; the rest is CubeMX output
Drivers/       Vendored STM32F7 HAL and CMSIS
python/        Host tools — start with TOOLS_README.md
  rr2_gui.py       the desktop application; run this
  rr2_decode.py    the wire protocol; must sit beside the GUI
  rr2_thscan.py    threshold scan engine, and a command-line tool in its own right
  rr2_i2ctest.py   Slow Control link check
*.ioc          CubeMX configuration
*.ld           Linker scripts
```

Measurements, raw event logs, threshold scans and screenshots are written next to
the Python scripts and are all gitignored — they are bench data, not source.

## 6. A first session

1. Flash the firmware, then start `rr2_gui.py`.
2. **Main** → pick the port → **Connect**. The link indicator should go green
   within a second or two; the board sends a status frame every second whether or
   not you are measuring.
3. **Settings** → enable the channel you are using → **Apply global**.
4. **Scan** → **Start scan** to find where the count rate stops being the source
   and starts being noise, then set the threshold it recommends.
5. **Measure** → fill in **Run label** and the run conditions → **Start**.
6. **History** → select the run → **Export to Excel**.

The firmware already applies channel gating and `Preset CsI` at startup, so you do
not need to do either by hand. `stat` should report `hold_ns` near 5900 in the CsI
configuration — if it reads 3000, the preset did not take.

## 7. When something is wrong

[python/TOOLS_README.md](python/TOOLS_README.md) ends with a symptom/cause table
that covers the common failures. The two that catch people first:

- **Connected but receiving gibberish** — the baud rate does not match. This link
  is a real UART at 921600, not a CDC port that negotiates.
- **Indicator green but `Test link` gets no answer** — frames are arriving but
  commands are not landing, so settings will appear to apply and will not.
