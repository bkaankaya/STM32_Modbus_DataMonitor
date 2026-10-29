#!/usr/bin/env python3
"""Compile and exercise the checked-in MCU Modbus handler with host-side HAL stubs.

This tests real production functions, not a rewritten Python protocol model.
It does NOT simulate UART timing, ADC sampling, interrupts or physical hardware.
"""
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = (ROOT / "stm32" / "Core" / "Src" / "main.c").read_text(encoding="utf-8")


def extract(signature: str) -> str:
    # Match the definition, not a forward declaration ending in ';'.
    match = re.search(re.escape(signature) + r"\s*\{", FIRMWARE)
    if match is None:
        raise RuntimeError(f"Could not locate function definition: {signature}")
    start = match.start()
    opening = match.end() - 1
    depth = 0
    for i in range(opening, len(FIRMWARE)):
        if FIRMWARE[i] == "{":
            depth += 1
        elif FIRMWARE[i] == "}":
            depth -= 1
            if depth == 0:
                return FIRMWARE[start : i + 1]
    raise RuntimeError(f"Unbalanced braces in {signature}")


prefix = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MODBUS_SLAVE_ID 1
#define MODBUS_FUNC_READ_HOLD 0x03
#define MODBUS_BUFFER_SIZE 64
#define MODBUS_TIMEOUT 100
#define TOTAL_REGISTERS 5

static int huart4;
static uint8_t modbus_tx_buffer[MODBUS_BUFFER_SIZE];
static uint8_t modbus_rx_buffer[MODBUS_BUFFER_SIZE];
static uint16_t modbus_rx_index;
static uint16_t holding_registers[TOTAL_REGISTERS] = {0};
static uint8_t last_tx[MODBUS_BUFFER_SIZE];
static uint16_t last_tx_size;
static unsigned transmit_calls;
static unsigned led_calls;

static int HAL_UART_Transmit(void *uart, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)uart;
    (void)timeout;
    assert(size <= MODBUS_BUFFER_SIZE);
    memcpy(last_tx, data, size);
    last_tx_size = size;
    ++transmit_calls;
    return 0;
}
static void LED_TX_On(void) { ++led_calls; }
"""

suffix = r"""
static void prepare(uint8_t slave, uint8_t function, uint16_t start, uint16_t count)
{
    uint8_t frame[8] = {
        slave, function, (uint8_t)(start >> 8), (uint8_t)(start & 0xFF),
        (uint8_t)(count >> 8), (uint8_t)(count & 0xFF), 0, 0
    };
    const uint16_t crc = Calculate_CRC16(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)(crc >> 8);
    memcpy(modbus_rx_buffer, frame, sizeof(frame));
    modbus_rx_index = 8;
    transmit_calls = 0;
    last_tx_size = 0;
}

static void assert_valid_reply(uint8_t function, uint16_t length)
{
    assert(transmit_calls == 1);
    assert(last_tx_size == length);
    assert(last_tx[0] == 1);
    assert(last_tx[1] == function);
    const uint16_t expected = Calculate_CRC16(last_tx, (uint8_t)(length - 2));
    const uint16_t actual = (uint16_t)last_tx[length - 2] |
                            ((uint16_t)last_tx[length - 1] << 8);
    assert(expected == actual);
}

int main(void)
{
    // Official CRC16/Modbus test vector: 01 03 00 00 00 02 => C4 0B.
    const uint8_t vector[] = {1, 3, 0, 0, 0, 2};
    assert(Calculate_CRC16((uint8_t *)vector, 6) == 0x0BC4);

    holding_registers[0] = 0x1234;
    holding_registers[1] = 0x5678;
    holding_registers[2] = 0x9ABC;
    holding_registers[3] = 0xDEF0;
    holding_registers[4] = 1;

    prepare(1, 3, 0, 2);
    Modbus_Process_Request();
    assert_valid_reply(3, 9);
    assert(last_tx[2] == 4);
    assert(last_tx[3] == 0x12 && last_tx[4] == 0x34);
    assert(last_tx[5] == 0x56 && last_tx[6] == 0x78);
    assert(modbus_rx_index == 0);

    prepare(1, 3, 0, 5);
    Modbus_Process_Request();
    assert_valid_reply(3, 15);
    assert(last_tx[2] == 10);

    prepare(1, 3, 5, 1);
    Modbus_Process_Request();
    assert_valid_reply(0x83, 5);
    assert(last_tx[2] == 2);

    prepare(1, 3, 0, 0);
    Modbus_Process_Request();
    assert_valid_reply(0x83, 5);
    assert(last_tx[2] == 2);

    prepare(1, 4, 0, 2);
    Modbus_Process_Request();
    assert_valid_reply(0x84, 5);
    assert(last_tx[2] == 1);

    prepare(2, 3, 0, 2);
    Modbus_Process_Request();
    assert(transmit_calls == 0);

    prepare(1, 3, 0, 2);
    modbus_rx_buffer[7] ^= 0x80;
    Modbus_Process_Request();
    assert(transmit_calls == 0);

    prepare(1, 3, 0, 2);
    modbus_rx_index = 7;
    Modbus_Process_Request();
    assert(transmit_calls == 0);

    prepare(1, 3, 0, 2);
    modbus_rx_index = 9;
    Modbus_Process_Request();
    assert(transmit_calls == 0);

    // Check every legal start address and a range of quantities.
    for (uint16_t start = 0; start <= 6; ++start) {
        for (uint16_t count = 0; count <= 8; ++count) {
            prepare(1, 3, start, count);
            Modbus_Process_Request();
            const int valid = count > 0 && start + count <= TOTAL_REGISTERS;
            assert_valid_reply(valid ? 3 : 0x83, valid ? (uint16_t)(5 + 2 * count) : 5);
        }
    }

    assert(led_calls > 0);
    puts("Firmware Modbus request handler tests passed");
    return 0;
}
"""

source = "\n\n".join(
    [
        prefix,
        extract("uint16_t Calculate_CRC16(uint8_t *buffer, uint8_t length)"),
        extract("void Modbus_Process_Request(void)"),
        suffix,
    ]
)

with tempfile.TemporaryDirectory(prefix="stm32-host-test-") as tmp:
    c_file = Path(tmp) / "firmware_test.c"
    exe_file = Path(tmp) / "firmware_test"
    c_file.write_text(source, encoding="utf-8")
    subprocess.run(
        [
            "gcc", "-std=c11", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            str(c_file), "-o", str(exe_file),
        ],
        check=True,
    )
    subprocess.run([str(exe_file)], check=True)
