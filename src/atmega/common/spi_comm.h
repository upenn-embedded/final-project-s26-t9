#pragma once

/*
 * atmega328pb_spi.h
 * =================
 * Public interface for the ATmega328PB bare-metal SPI driver.
 * Supports both Master (Main ATmega) and Slave (Sub ATmega) roles.
 *
 * Master frame format (two CS transactions per exchange):
 *   TX transaction:  CS_low -> [LEN][DATA...] -> CS_high
 *   <guard delay: slave prepares response>
 *   RX transaction:  CS_low -> [LEN][DATA...] (clocked in) -> CS_high
 *
 * Slave mirrors this exactly from the other side.
 */

#include <stdint.h>

/* Maximum payload bytes (excluding the 1-byte length prefix). */
#define SPI_MAX_PAYLOAD 200

/* ── Master API ────────────────────────────────────────────────────── */

/* Configure SPI0 as master, Mode 0, F_CPU/16. Call once at startup. */
void spi_master_init(void);

/*
 * TX phase: assert CS, send [LEN][DATA...], deassert CS.
 * Returns 0 on success, -1 if len > SPI_MAX_PAYLOAD.
 */
int8_t spi_send_message(const uint8_t *msg, uint8_t len);

/*
 * RX phase: applies guard delay, assert CS, reads [LEN][DATA...],
 * deassert CS.  rx_buf must be at least SPI_MAX_PAYLOAD bytes.
 * Returns 0 on success, -1 if the reported length is out of range.
 */
int8_t spi_receive_response(uint8_t *rx_buf, uint8_t *rx_len);

/*
 * Convenience: spi_send_message then spi_receive_response in one call.
 * Returns 0 on success, -1 on any error.
 */
int8_t spi_exchange(const uint8_t *tx_msg, uint8_t tx_len, uint8_t *rx_buf, uint8_t *rx_len);

/* ── Slave API ─────────────────────────────────────────────────────── */

/* Configure SPI0 as slave, Mode 0. Call once at startup. */
void spi_slave_init(void);

/*
 * Blocks until the master's TX transaction completes.
 * Reads [LEN] then LEN bytes into buf, sets *len = LEN.
 * Returns 0 on success, -1 if reported length > SPI_MAX_PAYLOAD.
 */
int8_t spi_slave_receive(uint8_t *buf, uint8_t *len);

/*
 * Blocks until the master's RX transaction completes, shifting out
 * [LEN][DATA...] on MISO one byte at a time.
 * Must be called after spi_slave_receive (during the guard delay window).
 * Returns 0 on success, -1 if len > SPI_MAX_PAYLOAD.
 */
int8_t spi_slave_send(const uint8_t *buf, uint8_t len);