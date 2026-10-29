# STM32 Modbus Data Monitor

An STM32F407 and Qt/C++ monitoring project developed during an embedded-systems internship with mentor guidance. It demonstrates ADC measurement, UART communication, Modbus RTU request/response handling, CRC16 checking, and a desktop interface.

This repository is a snapshot of an internship learning project, not a production-qualified or flight-qualified system. The documentation below describes the **checked-in implementation**, rather than features planned or described in older notes.

## What the project does

- The STM32F407 reads its **internal temperature sensor** and **Vrefint** through ADC1. It refreshes the holding-register values approximately every 500 ms in the main loop.
- A Qt/C++ desktop application connects over a USB-to-UART serial link and **requests** temperature, reference-voltage, or button-state data. The STM32 responds to these requests; it does not continuously stream unsolicited frames.
- Firmware and GUI use **Modbus RTU-style Function 0x03 (Read Holding Registers)**, slave address **1**, and **CRC16/Modbus** for request and response integrity.
- The firmware also reads an onboard button on PA0 and uses LEDs to indicate serial activity.

## Repository layout

- **stm32/** — STM32CubeIDE/CubeMX project, firmware in Core/Src/main.c, startup and vendor HAL/CMSIS sources
- **embeddedGui/** — Qt Widgets/C++ desktop GUI, Qt SerialPort integration, Modbus response validator, and CMake project
- **tests/** — C++ Modbus framing/CRC tests and a host-side test of the actual STM32 Modbus handler using HAL stubs
- **.github/workflows/qt-build.yml** — GitHub Actions builds the Qt desktop application and STM32 ARM ELF, then runs firmware syntax and protocol regression checks
- **[TECHNICAL_DETAILS.md](TECHNICAL_DETAILS.md)** — implementation details, original test notes, maintenance changes, and verification limits

The source-controlled project excludes generated IDE build outputs. Technical notes are in [TECHNICAL_DETAILS.md](TECHNICAL_DETAILS.md).

## Hardware and software

- **Board/project configuration:** STM32F407G-DISC1 (STM32F407VGTx)
- **MCU firmware:** C, STM32CubeIDE, STM32 HAL, ARM GCC toolchain
- **PC interface:** Qt 5/6, Qt Widgets and SerialPort, C++17, CMake
- **Serial link:** UART4, TX on PC10, RX on PC11, 115200 baud, 8 data bits, no parity, 1 stop bit, no hardware flow control
- **ADC:** 12-bit readings from the internal temperature sensor and Vrefint, using HAL calls
- **Execution model:** a bare-metal main loop; UART receive/transmit and ADC acquisition use HAL **polling/blocking APIs**, not interrupt-driven UART or an RTOS

The internship setup used a TTL-232RG USB-to-UART adapter. Match voltage levels and connect a common ground according to the board and adapter documentation.

## Actual communication protocol

The checked-in application uses a Modbus RTU-like register protocol, **not** the older documented 0xAA/length/payload/0x55 custom packet format.

**Request, Function 0x03:**
Slave address (1 byte) | Function 0x03 (1) | Start register (2, big-endian) | Register count (2, big-endian) | CRC16 (2, low byte first)

**Successful response:**
Slave address (1) | Function 0x03 (1) | Byte count (1) | Register values (2 bytes each, high byte first) | CRC16 (2, low byte first)

**Exception response:**
Slave address (1) | Function code with 0x80 set (1) | Exception code (1) | CRC16 (2, low byte first)

**CRC:** reflected polynomial 0xA001, initial value 0xFFFF. The CRC is calculated over all preceding bytes in the Modbus frame.

| Holding register | Meaning |
| --- | --- |
| 0 | Temperature float, high 16-bit word |
| 1 | Temperature float, low 16-bit word |
| 2 | Reference voltage float, high 16-bit word |
| 3 | Reference voltage float, low 16-bit word |
| 4 | Button state (0 = released, 1 = pressed) |

Temperature and reference voltage use 32-bit IEEE-754 float values split across two 16-bit registers. The Qt GUI recombines those words when displaying a value.

## Opening the project

### Firmware

1. Import the **stm32/** project into STM32CubeIDE.
2. Inspect the **deneme3.ioc** CubeMX configuration and build for STM32F407VGTx.
3. Program a compatible STM32F407 board using your available supported programming/debugging hardware.
4. Connect UART4 on PC10/PC11 to a correctly wired, voltage-compatible USB-to-UART adapter.

### Qt desktop application

1. Open **embeddedGui/CMakeLists.txt** with Qt Creator or configure it with CMake and an installed Qt 5/6 toolchain including Widgets and SerialPort.
2. Build and start the GUI.
3. Select the appropriate serial port and **115200, 8N1** settings, then connect.
4. Use the Temperature, Vref, Button, or All controls to send read requests.

The Qt build, cross-compiled STM32 firmware ELF, and protocol tests run in [GitHub Actions](https://github.com/bkaankaya/STM32_Modbus_DataMonitor/actions). Successful software builds are **not** a substitute for flashing the board and running electrical/serial tests.

## Rebuilding and running tests

From the repository root, with **Qt 6 Widgets, SerialPort, LinguistTools**, CMake, and a C++17 compiler installed:

```sh
cmake -S embeddedGui -B build/qt -DCMAKE_BUILD_TYPE=Release
cmake --build build/qt --parallel
```

On Linux with **arm-none-eabi-gcc**, binutils, and the ARM newlib/nano runtime installed, the portable firmware build creates `build/stm32/stm32.elf` and a link map:

```sh
bash tests/build_stm32.sh build/stm32
```

STM32CubeIDE can also import `stm32/` and use the existing CubeMX project. Generated STM32CubeIDE `Debug/` outputs are deliberately excluded from version control; STM32CubeIDE regenerates them when building. Use the portable script for reproducible command-line builds.

On Linux with GCC/G++ and Python 3, run the protocol regression suites:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -pedantic tests/test_modbusframe.cpp -o build/test_modbusframe
./build/test_modbusframe
python3 tests/test_firmware_protocol.py
```

The firmware protocol test extracts the checked-in C handler and compiles it with **host-side HAL stubs**. It checks protocol behavior, not physical UART timing or ADC measurements. The [GitHub Actions workflow](.github/workflows/qt-build.yml) performs both builds and both test suites on Ubuntu.

**Confirmed CI run:** [STM32 ELF + Qt build and protocol checks passed](https://github.com/bkaankaya/STM32_Modbus_DataMonitor/actions/runs/40871817845). The real board still requires flash, serial wiring, and hardware-in-the-loop validation.

## Verified fixes and remaining limits

The original internship implementation has received later maintenance fixes; the Git history distinguishes these changes from the internship work.

- The MCU still uses **polling-based UART**, not interrupt-driven reception or an RTOS. A complete supported **8-byte Function 0x03 request** is now processed immediately; the **10 ms timeout only discards incomplete requests**. This is not a fully standards-compliant RTU frame-timing state machine.
- The existing **100 ms software debounce** routine is now called from the main loop before the button-state register is served.
- The GUI buffers serial bytes until it has an entire response of the **expected length**, including 5-byte Modbus exception frames. It checks slave address, function, register count, and CRC before decoding, and discards stale or unsolicited bytes.
- The GUI rejects overlapping requests and reuses its serial-port object when reconnecting.
- The firmware can still miss bytes while its blocking ADC acquisition runs. A fully interrupt/DMA-driven design and hardware measurements would be required to verify sustained 115200-baud reliability.
- This connection uses **8N1** for the original USB-UART setup. The official Modbus RTU serial-line guide specifies an **11-bit character format** (including two stop bits when parity is disabled), so do not advertise this demonstration as fully serial-line compliant.
- Original notes describe serial experiments and a long-duration run, but raw serial captures and lab logs are not committed. No error-rate or 24-hour reliability guarantee is claimed.

For the current implementation and specific validation limits, see **[TECHNICAL_DETAILS.md](TECHNICAL_DETAILS.md)**. External references: [Modbus specifications](https://www.modbus.org/modbus-specifications), [Qt QSerialPort](https://doc.qt.io/qt-6/qserialport.html), and [ST STM32CubeF4](https://github.com/STMicroelectronics/STM32CubeF4).

## Licensing note

There is **no repository-level LICENSE file** in the checked-in tree. The bundled STMicroelectronics HAL/CMSIS components include their own license notices; do not assume that the entire repository is covered by an MIT license.
