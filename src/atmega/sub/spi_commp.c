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
    SPDR = tx; // Changed from SPDR0
    while (!(SPSR & (1 << SPIF))); // Changed from SPSR0
    return SPDR; // Changed from SPDR0
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

    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // Changed from SPCR0
    SPSR &= ~(1 << SPI2X); // Changed from SPSR0
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

    SPCR = (1 << SPE); // Changed from SPCR0

    SPDR = 0x00; // Changed from SPDR0
}

int8_t spi_slave_receive(uint8_t *buf, uint8_t *len)
{
    wait_ss_assert();

    if (SPSR & (1 << SPIF)) // Changed from SPSR0
        (void)SPDR; // Changed from SPDR0

    while (!(SPSR & (1 << SPIF))); // Changed from SPSR0
    uint8_t incoming_len = SPDR; // Changed from SPDR0
    SPDR = 0x00; // Changed from SPDR0

    if (incoming_len > SPI_MAX_PAYLOAD) {
        wait_ss_deassert();
        while (SPSR & (1 << SPIF)) // Changed from SPSR0
            (void)SPDR; // Changed from SPDR0
        *len = 0;
        return -1;
    }

    for (uint8_t i = 0; i < incoming_len; i++) {
        while (!(SPSR & (1 << SPIF))); // Changed from SPSR0
        buf[i] = SPDR; // Changed from SPDR0
        SPDR = 0x00; // Changed from SPDR0
    }

    wait_ss_deassert();
    while (SPSR & (1 << SPIF)) // Changed from SPSR0
        (void)SPDR; // Changed from SPDR0
    *len = incoming_len;
    return 0;
}

int8_t spi_slave_send(const uint8_t *buf, uint8_t len)
{
    if (len > SPI_MAX_PAYLOAD) return -1;

    SPDR = len; // Changed from SPDR0

    wait_ss_assert();

    while (!(SPSR & (1 << SPIF))); // Changed from SPSR0
    SPDR = (len > 0) ? buf[0] : 0x00; // Changed from SPDR0
    (void)SPDR; // Changed from SPDR0

    for (uint8_t i = 0; i < len; i++) {
        SPDR = buf[i];  // Changed from SPDR0
        while (!(SPSR & (1 << SPIF))); // Changed from SPSR0
        (void)SPDR; // Changed from SPDR0
    }

    wait_ss_deassert();
    SPDR = 0x00; // Changed from SPDR0
    return 0;
}