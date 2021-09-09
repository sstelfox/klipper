// sam3/sam4 serial port
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint32_t
#include "autoconf.h" // CONFIG_MACH_*
#include "board/armcm_boot.h" // DECL_ARMCM_IRQ
#include "board/serial_irq.h" // serial_rx_byte
#include "command.h" // DECL_ENUMERATION
#include "gpio.h" // uart_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown

struct bus_info {
    uint8_t id, is_usart;
    void *dev;
    uint32_t dev_id;
    IRQn_Type irqn;
    uint8_t rx_pin, tx_pin, function;
};

#if CONFIG_MACH_SAM3X
#define UARTx_IRQn UART_IRQn
static Uart * const Port = UART;
static const uint32_t Pmc_id = ID_UART;
static const uint32_t rx_pin = GPIO('A', 8), tx_pin = GPIO('A', 9);
static const char uart_periph = 'A';
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA8,PA9");
#elif CONFIG_MACH_SAM4S
#define UARTx_IRQn UART1_IRQn
static Uart * const Port = UART1;
static const uint32_t Pmc_id = ID_UART1;
static const uint32_t rx_pin = GPIO('B', 2), tx_pin = GPIO('B', 3);
static const char uart_periph = 'A';
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB2,PB3");
#elif CONFIG_MACH_SAM4E
#define UARTx_IRQn UART0_IRQn
static Uart * const Port = UART0;
static const uint32_t Pmc_id = ID_UART0;
static const uint32_t rx_pin = GPIO('A', 9), tx_pin = GPIO('A', 10);
static const char uart_periph = 'A';
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA9,PA10");
#elif CONFIG_MACH_SAME70
#define UARTx_IRQn UART2_IRQn
static Uart * const Port = UART2;
static const uint32_t Pmc_id = ID_UART2;
static const uint32_t rx_pin = GPIO('D', 25), tx_pin = GPIO('D', 26);
static const char uart_periph = 'C';
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PD25,PD26");
#endif

static const struct bus_info bus_data[] = {
    { 0, 0, UART, ID_UART, UART_IRQn, GPIO('A', 8), GPIO('A', 9), 'A' },
    { 1, 1, USART0, ID_USART0, USART0_IRQn, GPIO('A', 10), GPIO('A', 11), 'A' },
    { 2, 1, USART1, ID_USART1, USART1_IRQn, GPIO('A', 12), GPIO('A', 13), 'A' },
    { 3, 1, USART2, ID_USART2, USART2_IRQn, GPIO('B', 21), GPIO('B', 20), 'A' },
#ifdef USART3
    { 4, 1, USART3, ID_USART3, USART3_IRQn, GPIO('D', 5), GPIO('D', 4), 'B' },
#endif
};

void
UART_IRQHandler(void)
{
    uint32_t status = UART->UART_SR;
    if (status & UART_SR_RXRDY)
        serial_rx_byte(0, UART->UART_RHR);
    if (status & UART_SR_TXRDY) {
        uint8_t data;
        int ret = serial_get_tx_byte(0, &data);
        if (ret)
            UART->UART_IDR = UART_IDR_TXRDY;
        else
            UART->UART_THR = data;
    }
}
DECL_ARMCM_IRQ(UART_IRQHandler, UART_IRQn);

void
USART0_IRQHandler(void)
{
    uint32_t status = USART0->US_CSR;
    if (status & US_CSR_RXRDY)
        serial_rx_byte(1, USART0->US_RHR);
    if (status & US_CSR_TXRDY) {
        uint8_t data;
        int ret = serial_get_tx_byte(1, &data);
        if (ret)
            USART0->US_IDR = US_IDR_TXRDY;
        else
            USART0->US_THR = data;
    }
}
DECL_ARMCM_IRQ(USART0_IRQHandler, USART0_IRQn);

void
USART1_IRQHandler(void)
{
    gpio_peripheral(rx_pin, uart_periph, 1);
    gpio_peripheral(tx_pin, uart_periph, 0);

    // Reset uart
    enable_pclock(Pmc_id);
    Port->UART_CR = (UART_CR_RSTRX | UART_CR_RSTTX
                     | UART_CR_RXDIS | UART_CR_TXDIS);
    uart->UART_IDR = 0xFFFFFFFF;

    // Enable uart
    Port->UART_MR = (UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL);
    Port->UART_BRGR = get_pclock_frequency(Pmc_id) / (16 * CONFIG_SERIAL_BAUD);
    Port->UART_IER = UART_IER_RXRDY;
    armcm_enable_irq(UARTx_Handler, UARTx_IRQn, 0);
    Port->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
}
