// Host serial communication initialization
//
// Copyright (C) 2021  Desuuuu <contact@desuuuu.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/serial_irq.h" // serial_prepare
#include "sched.h" // DECL_INIT
#include "command.h" // DECL_CONSTANT

#define RX_BUFFER_SIZE 192

DECL_CONSTANT("SERIAL_BAUD", CONFIG_SERIAL_BAUD);
DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);

#define CONSOLE_UART_BUS 0

DECL_CONSTANT_STR("RESERVE_PINS_serial", "[uart0],gpio1,gpio0");

static uint8_t serial_host_id;

void
serial_init(void)
{
    serial_host_id = serial_prepare(CONSOLE_UART_BUS, CONFIG_SERIAL_BAUD,
                                    RX_BUFFER_SIZE, 96, 0, NULL);
}
DECL_INIT(serial_init);

uint_fast8_t
console_sendf(const struct command_encoder *ce, va_list args)
{
    return serial_send_command(serial_host_id, ce, args);
}
