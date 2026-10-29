# Technical Details, Testing and Validation

[Back to project README](README.md)

## Scope and evidence

This document records the behavior visible in the checked-in STM32 firmware, CubeMX project configuration, Qt/C++ desktop application, and earlier project notes. The project was carried out during a mentor-guided embedded-systems internship.

**Verified in this document** means supported by checked-in source/configuration or the explicitly cited automated test results. GitHub Actions successfully builds the Qt desktop application and cross-compiles **and links** the STM32 firmware into an ARM ELF file. The ELF has **not** been flashed or executed on the physical board during this maintenance review; UART timing, electrical measurements, and sensor accuracy remain unverified on real hardware. Historical test claims from the original notes are distinguished below.

## 1. Architecture

**STM32 firmware (C, STM32CubeIDE)**

- Board configuration: STM32F407G-DISC1 / STM32F407VGTx (stm32/deneme3.ioc).
- ADC1 reads the **internal temperature sensor** and **Vrefint**, then computes temperature and supply/reference-voltage values.
- The firmware periodically updates five 16-bit Modbus holding registers and answers **Function 0x03** UART requests as slave address **1**.
- The main loop uses HAL polling/blocking functions for ADC and UART rather than an RTOS or UART RX/TX interrupts.
- TX and RX activity is indicated by onboard LEDs.

**PC application (Qt/C++17)**

- Qt Widgets GUI and Qt SerialPort communicate with the MCU through a USB-to-UART adapter.
- The GUI selects a serial port, sends Modbus read requests, checks response CRC16, reconstructs floats from register words, and shows temperature, voltage, and button state.
- Its QTimer is used as a **one-shot request timeout (1,000 ms)**, not for periodic data streaming.
- Requests are initiated by GUI button actions; displayed values update when requests are made.

## 2. ADC acquisition

| Source | Checked-in implementation |
| --- | --- |
| ADC peripheral | ADC1, 12-bit resolution |
| Inputs | ADC_CHANNEL_TEMPSENSOR and ADC_CHANNEL_VREFINT |
| Sampling time | 480 ADC cycles for each input |
| Runtime method | HAL_ADC_Stop, HAL_ADC_ConfigChannel, HAL_ADC_Start, HAL_ADC_PollForConversion, HAL_ADC_GetValue |
| Holding-register update | Approximately every 500 ms from the main loop |
| Value encoding | 32-bit float split into high/low 16-bit register words |

The generated ADC configuration sets continuous conversion and two configured channels, but the **runtime read function manually stops, reconfigures and polls each channel**. Describing the acquisition solely as interrupt-driven or DMA-driven would therefore be incorrect.

The temperature calculation in the firmware uses fixed constants (1.21 V Vrefint, 0.76 V and 0.0025 V/degree C sensor approximation). No calibration records or accuracy measurement files are included in the repository.

## 3. UART hardware and execution

| Property | Value in source |
| --- | --- |
| Instance | UART4 |
| TX / RX | PC10 / PC11 |
| Baud rate | 115200 |
| Data format | 8 data bits, no parity, 1 stop bit (8N1) |
| Flow control | None |
| MCU receive | HAL_UART_Receive, one byte at a time (polling) |
| MCU transmit | HAL_UART_Transmit (blocking) |
| UART receive buffer | 64-byte allocation; the supported Function 0x03 request uses exactly 8 bytes |
| End-of-request detection | Process exactly 8 accumulated request bytes; discard a partial request after 10 ms inactivity |
| RTOS | Not used |

The 10 ms gap is a **partial-request cleanup timeout**, not an RTU-compliant frame-end delay. The firmware processes a supported 8-byte request as soon as the eighth byte arrives. The firmware still uses blocking ADC operations in the same main loop as UART polling, so rapid requests may lose bytes under certain timing conditions. At 115200 baud, the Modbus serial-line guide recommends 1.750 ms inter-frame delay (and 750 µs inter-character timeout); the original link uses 8N1 rather than the standard's 11-bit framing. This is a Modbus RTU-style demonstration rather than verified full conformance.

No UART interrupt-driven RX implementation, register-level driver implementation, or DMA receive path is established by the application's checked-in source.

## 4. Protocol and register representation

This implementation exchanges **Modbus RTU-style register frames**, not packets delimited by 0xAA and 0x55.

**Read request (8 bytes):**

| Position | Length | Field |
| --- | --- | --- |
| 0 | 1 byte | Slave ID = 0x01 |
| 1 | 1 byte | Function = 0x03 |
| 2–3 | 2 bytes | Start address, high byte then low byte |
| 4–5 | 2 bytes | Quantity, high byte then low byte |
| 6–7 | 2 bytes | CRC16, low byte then high byte |

**Normal response:**

| Position | Length | Field |
| --- | --- | --- |
| 0 | 1 byte | Slave ID |
| 1 | 1 byte | Function = 0x03 |
| 2 | 1 byte | Byte count = 2 × register quantity |
| 3 onward | 2 bytes per register | Values, high byte then low byte |
| Last 2 | 2 bytes | CRC16, low byte then high byte |

The firmware also returns Modbus-style exception frames for unsupported functions and invalid register ranges.

**CRC16/Modbus:** polynomial 0xA001, initial value 0xFFFF, calculated over all bytes preceding the CRC field.

| Register index | Meaning | Encoding |
| --- | --- | --- |
| 0 | Temperature high word | Float bits 31–16 |
| 1 | Temperature low word | Float bits 15–0 |
| 2 | Reference voltage high word | Float bits 31–16 |
| 3 | Reference voltage low word | Float bits 15–0 |
| 4 | Button state | Unsigned 0 or 1 |

The MCU packs each floating-point value into two 16-bit words, and the GUI reconstructs it using the same word order. The All request reads five registers.

## 5. GPIO and button

- The onboard button is read from **PA0**.
- **Button_Debounce_Update** implements a 100 ms stable-high check and is now called in the main loop.
- The debounced state is published in holding register 4; a release resets the state immediately.
- GPIO D13/D15 LEDs indicate UART TX/RX activity.

This corrects a previously inactive debounce routine without replacing the original algorithm.

## 6. Qt desktop application

The checked-in GUI source uses:

- **QSerialPort / QSerialPortInfo** for serial communication and port discovery.
- **QByteArray** as the incoming response buffer.
- **QTimer** for a one-shot 1-second response timeout.
- **QVector<quint16>** and an IEEE-754 word reconstruction helper for returned register values.
- **CRC16/Modbus** validation, slave-ID and function-code checks.
- A small Qt-independent **modbusframe.h** validator that collects complete responses (including 5-byte exception replies) and rejects invalid byte counts, CRCs, and unsolicited frames.
- The serial-port object is reused across connects; requests are serialized to avoid overlapping replies.
- Button controls for temperature, Vref, button state, and reading all registers.

The GUI code does **not** implement a file data logger in the current snapshot; an earlier note's optional logging feature should not be considered implemented.

### Maintenance fixes and known limitations

**Fixed after the internship:**

1. **Partial replies:** Qt no longer parses at an arbitrary 7-byte threshold. It waits for the exact expected length (7 bytes for one register, 9 for two, 15 for five, or 5 for an exception).
2. **Out-of-range register reads:** the decoder verifies the byte count and complete frame length; the All handler requires all five registers.
3. **Button debounce:** the existing firmware function is now invoked from the main loop.
4. **Request processing:** the firmware handles the supported 8-byte Function 0x03 request on receipt of its final byte and rejects non-eight-byte requests.
5. **Serial reconnect and state:** the Qt GUI reuses its QSerialPort instance, clears timed-out requests, and prevents overlapping commands.

**Remaining engineering limitations:**

- The MCU uses polling, including blocking ADC work, so UART overrun/loss under all hardware timing conditions has **not** been ruled out.
- The original 8N1 serial configuration and fixed 10 ms partial-frame timeout do **not** implement a fully compliant Modbus RTU receiver with the standard t1.5/t3.5 intervals.
- There is no MCU-in-the-loop automated test, hardware capture, or independently measured temperature calibration.
- Hardware reliability requires testing on the exact STM32F407 board and adapter used in the field.

The post-internship fixes are traceable in Git history; they should not be misrepresented as original internship milestones.

## 7. Testing and validation record

### Software validation

- **tests/test_modbusframe.cpp** checks Modbus CRC against a known request vector, fragmented replies, complete replies of several lengths, exception responses, wrong slave ID/function, corrupted CRC, mismatched register counts, and unexpected trailing bytes.
- **tests/test_firmware_protocol.py** extracts the actual firmware C functions **Calculate_CRC16** and **Modbus_Process_Request**, compiles them on a host with small HAL stubs and ASan/UBSan, and tests normal and exceptional responses, invalid CRCs, wrong slave IDs, malformed request lengths, and register boundaries. This is not a device emulator.
- The portable protocol test was built and run with **g++ -std=c++17** and AddressSanitizer/UndefinedBehaviorSanitizer in the maintenance environment.
- **tests/build_stm32.sh** compiles and links the checked-in STM32 source and bundled HAL with ARM GCC into a Cortex-M4 `.elf` using the repository's Flash linker script.
- **.github/workflows/qt-build.yml** builds both the Qt desktop application and STM32 firmware, runs both protocol suites, and checks the three STM32 application C sources. See [a successful build and test run](https://github.com/bkaankaya/STM32_Modbus_DataMonitor/actions/runs/40871817845) and the [Actions run history](https://github.com/bkaankaya/STM32_Modbus_DataMonitor/actions) for subsequent commits.
- These are software build and unit checks, **not** end-to-end hardware tests or proof of real-time serial reliability.

### Confirmed by repository inspection

- Both MCU and GUI contain CRC16 functions using 0xA001 and 0xFFFF.
- Function 0x03 register request/response construction and parsing paths are present.
- STM32CubeIDE configuration, vendor HAL sources, the Qt/CMake project, and a reproducible ARM firmware build script are present. Generated `stm32/Debug/` outputs are not checked in.
- GUI displays temperature, reference-voltage, and button values from decoded registers.

### Recorded in earlier project notes; no supporting logs committed

- Serial checks using PuTTY and Tera Term.
- Baud-rate experimentation at 9600, 19200, 38400, 57600, and 115200 bps.
- Comparison of temperature and voltage readings against reference measurements.
- A reported 24-hour continuous operation run.
- Statements about zero CRC mismatches and error rate below 0.01%.

These are **historical notes, not independently reproduced results**. The repository contains no raw serial captures, benchmark logs, device-lab records, automated test reports, or measured results substantiating those figures. No further performance or production-reliability claim is made here.

## 8. Development tools and distribution

| Component | Repository evidence |
| --- | --- |
| IDE/configuration | STM32CubeIDE, STM32CubeMX .ioc project |
| Firmware toolchain | ARM GCC (arm-none-eabi-gcc) project/build artifacts |
| Desktop build | CMake with Qt 5/6 Widgets, SerialPort, LinguistTools |
| Language standard | Qt GUI configured as C++17 |
| Debugging equipment | Serial adapter described in internship notes; no committed debugger or oscilloscope session records |

The repository has **no top-level LICENSE file**. Vendor HAL/CMSIS components carry their own license notices. An earlier document's blanket MIT licensing assertion is not supported by the checked-in files.

## 9. Summary

The repository remains a **mentor-guided embedded-systems internship example** of STM32 ADC acquisition, polling-based UART, Modbus RTU-style communication with CRC16, and a Qt/C++ desktop interface. Targeted post-internship fixes address deterministic parsing and bounds bugs, while hardware-dependent timing and electrical validation remain explicitly open. See [Modbus Serial Line Guide V1.02](https://www.modbus.org/file/secure/modbusoverserial.pdf), [Qt QIODevice](https://doc.qt.io/qt-6/qiodevice.html), and [ST STM32CubeF4](https://github.com/STMicroelectronics/STM32CubeF4) for the referenced behavior.
