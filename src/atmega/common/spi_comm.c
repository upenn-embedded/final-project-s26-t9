#include "spi_comm.h"
#include <avr/io.h>
#include <util/delay.h>

#define SPI_DDR   DDRB
#define SPI_PORT  PORTB
#define SPI_PIN   PINB
#define PIN_SS    PB2
#define PIN_MOSI  PB3
#define PIN_MISO  PB4
#define PIN_SCK   PB5

#define SPI_GUARD_US    200

static inline uint8_t spi_transfer_byte(uint8_t tx)
{
    SPDR0 = tx;
    while (!(SPSR0 & (1 << SPIF)));
    return SPDR0;
}

static inline void cs_assert(void)   { SPI_PORT &= ~(1 << PIN_SS); }
static inline void cs_deassert(void) { SPI_PORT |=  (1 << PIN_SS); }

static inline void wait_ss_assert(void)
{
    while (SPI_PIN & (1 << PIN_SS));
}

static inline void wait_ss_deassert(void)
{
    while (!(SPI_PIN & (1 << PIN_SS)));
}

void spi_master_init(void)
{
    SPI_DDR |=  (1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PIN_SS);
    SPI_DDR &= ~(1 << PIN_MISO);

    cs_deassert();

    SPCR0 = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR0 &= ~(1 << SPI2X);
}

int8_t spi_send_message(const uint8_t *msg, uint8_t len)
{
    if (len > SPI_MAX_PAYLOAD) return -1;

    cs_assert();
    spi_transfer_byte(len);
    for (uint8_t i = 0; i < len; i++)
        spi_transfer_byte(msg[i]);
    cs_deassert();

    return 0;
}

int8_t spi_receive_response(uint8_t *rx_buf, uint8_t *rx_len)
{
    _delay_us(SPI_GUARD_US);

    cs_assert();

    uint8_t incoming_len = spi_transfer_byte(0x00);
    if (incoming_len > SPI_MAX_PAYLOAD) {
        cs_deassert();
        *rx_len = 0;
        return -1;
    }

    for (uint8_t i = 0; i < incoming_len; i++)
        rx_buf[i] = spi_transfer_byte(0x00);

    cs_deassert();
    *rx_len = incoming_len;
    return 0;
}

int8_t spi_exchange(const uint8_t *tx_msg, uint8_t  tx_len,
                          uint8_t *rx_buf, uint8_t *rx_len)
{
    if (spi_send_message(tx_msg, tx_len) != 0) return -1;
    return spi_receive_response(rx_buf, rx_len);
}

void spi_slave_init(void)
{
    SPI_DDR &= ~((1 << PIN_SS) | (1 << PIN_MOSI) | (1 << PIN_SCK));
    SPI_DDR |= (1 << PIN_MISO);

    SPCR0 = (1 << SPE);

    SPDR0 = 0x00;
}

int8_t spi_slave_receive(uint8_t *buf, uint8_t *len)
{
    wait_ss_assert();

    if (SPSR0 & (1 << SPIF))
        (void)SPDR0;

    while (!(SPSR0 & (1 << SPIF)));
    uint8_t incoming_len = SPDR0;
    SPDR0 = 0x00;

    if (incoming_len > SPI_MAX_PAYLOAD) {
        wait_ss_deassert();
        while (SPSR0 & (1 << SPIF))
            (void)SPDR0;
        *len = 0;
        return -1;
    }

    for (uint8_t i = 0; i < incoming_len; i++) {
        while (!(SPSR0 & (1 << SPIF)));
        buf[i] = SPDR0;
        SPDR0 = 0x00;
    }

    wait_ss_deassert();
    while (SPSR0 & (1 << SPIF))
        (void)SPDR0;
    *len = incoming_len;
    return 0;
}

int8_t spi_slave_send(const uint8_t *buf, uint8_t len)
{
    if (len > SPI_MAX_PAYLOAD) return -1;

    SPDR0 = len;

    wait_ss_assert();

    for (uint8_t i = 0; i < len; i++) {
        while (!(SPSR0 & (1 << SPIF)));
        SPDR0 = buf[i];
        (void)SPDR0;
    }
    while (!(SPSR0 & (1 << SPIF)));
    (void)SPDR0;

    wait_ss_deassert();
    SPDR0 = 0x00;
    return 0;
}