/*
 * atmega328pb_spi.c
 * =================
 * Bare-metal SPI driver for the ATmega328PB.
 * Supports Master (Main ATmega) and Slave (Sub ATmega) roles.
 *
 * SPI0 pin mapping:
 *   /SS  -> PB2  (master: output driven by firmware)
 *                (slave:  input  driven by ESP32)
 *   MOSI -> PB3  (master: output / slave: input)
 *   MISO -> PB4  (master: input  / slave: output)
 *   SCK  -> PB5  (master: output / slave: input)
 *
 * Master frame format (two CS transactions per exchange):
 *   TX transaction:  CS_low -> [LEN][DATA...] -> CS_high
 *   <guard delay ? slave prepares its response during this window>
 *   RX transaction:  CS_low -> dummy bytes -> reads [LEN][DATA...] -> CS_high
 *
 * Slave mirrors this exactly from the other side.
 *
 * Build (choose the role with -DSPI_ROLE_MASTER or -DSPI_ROLE_SLAVE):
 *   avr-gcc -mmcu=atmega328pb -DF_CPU=16000000UL -O2 -o out.elf atmega328pb_spi.c
 */

#include "spi_comm.h"
#include <avr/io.h>
#include <util/delay.h>

/* ?? Pin / register aliases ????????????????????????????????????????? */
#define SPI_DDR   DDRB
#define SPI_PORT  PORTB
#define SPI_PIN   PINB      /* input register ? used to poll /SS state */
#define PIN_SS    PB2
#define PIN_MOSI  PB3
#define PIN_MISO  PB4
#define PIN_SCK   PB5

/*
 * Guard time between the master's TX and RX transactions (µs).
 * The slave uses this window to prepare its response in SPDR.
 * Increase if the slave needs more processing time.
 */
#define SPI_GUARD_US    200

/* ?? Shared low-level helpers ??????????????????????????????????????? */

static inline uint8_t spi_transfer_byte(uint8_t tx)
{
    SPDR0 = tx;
    while (!(SPSR0 & (1 << SPIF)));
    return SPDR0;
}

/* ?? Master helpers ????????????????????????????????????????????????? */

static inline void cs_assert(void)   { SPI_PORT &= ~(1 << PIN_SS); }
static inline void cs_deassert(void) { SPI_PORT |=  (1 << PIN_SS); }

/* ?? Slave helpers ?????????????????????????????????????????????????? */

/*
 * Poll until the master drives /SS low (start of a transaction).
 * Because the SPI hardware only shifts when /SS is asserted, waiting
 * here guarantees SPIF reflects the current transaction.
 */
static inline void wait_ss_assert(void)
{
    while (SPI_PIN & (1 << PIN_SS));    /* loop while /SS is HIGH */
}

/* Poll until the master releases /SS (end of a transaction). */
static inline void wait_ss_deassert(void)
{
    while (!(SPI_PIN & (1 << PIN_SS))); /* loop while /SS is LOW  */
}

/* ?? Master API ????????????????????????????????????????????????????? */

void spi_master_init(void)
{
    /* MOSI, SCK, /SS -> outputs;  MISO -> input */
    SPI_DDR |=  (1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PIN_SS);
    SPI_DDR &= ~(1 << PIN_MISO);

    cs_deassert();  /* /SS idle-high before enabling peripheral */

    /*
     * SPCR: SPE=1 (enable), MSTR=1 (master), CPOL=0 CPHA=0 (Mode 0)
     * SPR0=1, SPI2X=0  ->  SCK = F_CPU / 16  (1 MHz at 16 MHz clock)
     *
     * Clock rate table:
     *   SPI2X=0, SPR1=0, SPR0=0  ->  F_CPU/4    (fastest)
     *   SPI2X=0, SPR1=0, SPR0=1  ->  F_CPU/16
     *   SPI2X=0, SPR1=1, SPR0=0  ->  F_CPU/64
     *   SPI2X=0, SPR1=1, SPR0=1  ->  F_CPU/128  (slowest)
     */
    SPCR0 = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR0 &= ~(1 << SPI2X);
}

int8_t spi_send_message(const uint8_t *msg, uint8_t len)
{
    if (len > SPI_MAX_PAYLOAD) return -1;

    cs_assert();
    spi_transfer_byte(len);             /* length prefix */
    for (uint8_t i = 0; i < len; i++)
        spi_transfer_byte(msg[i]);
    cs_deassert();

    return 0;
}

int8_t spi_receive_response(uint8_t *rx_buf, uint8_t *rx_len)
{
    _delay_us(SPI_GUARD_US);            /* let slave load its TX buffer */

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

/* ?? Slave API ?????????????????????????????????????????????????????? */

void spi_slave_init(void)
{
    /* /SS, MOSI, SCK -> inputs (driven by ESP32 master) */
    SPI_DDR &= ~((1 << PIN_SS) | (1 << PIN_MOSI) | (1 << PIN_SCK));
    /* MISO -> output (driven by this slave) */
    SPI_DDR |= (1 << PIN_MISO);

    /*
     * SPCR: SPE=1 (enable), MSTR=0 (slave), CPOL=0 CPHA=0 (Mode 0).
     * Clock divider bits are ignored in slave mode.
     * Must match the master's CPOL/CPHA settings.
     */
    SPCR0 = (1 << SPE);

    SPDR0 = 0x00;    /* pre-load idle value; sent if master clocks before
                       we've loaded a real response ? benign filler */
}

int8_t spi_slave_receive(uint8_t *buf, uint8_t *len)
{
    wait_ss_assert();                   /* wait for master's TX transaction */

    /* First byte clocked in is the payload length */
    while (!(SPSR0 & (1 << SPIF)));
    uint8_t incoming_len = SPDR0;
    SPDR0 = 0x00;                        /* keep sending dummy while receiving */

    if (incoming_len > SPI_MAX_PAYLOAD) {
        wait_ss_deassert();
        *len = 0;
        return -1;
    }

    for (uint8_t i = 0; i < incoming_len; i++) {
        while (!(SPSR0 & (1 << SPIF)));
        buf[i] = SPDR0;
        SPDR0 = 0x00;
    }

    wait_ss_deassert();                 /* wait for master to end TX transaction */
    *len = incoming_len;
    return 0;
}

int8_t spi_slave_send(const uint8_t *buf, uint8_t len)
{
    if (len > SPI_MAX_PAYLOAD) return -1;

    /*
     * Pre-load the length byte into SPDR NOW, during the guard delay
     * window.  The master will clock this out as the first byte of its
     * RX transaction.  We must write SPDR before /SS falls again.
     */
    SPDR0 = len;

    wait_ss_assert();                   /* wait for master's RX transaction */

    /* Length byte is being shifted out; master sends a dummy byte */
    while (!(SPSR0 & (1 << SPIF)));
    (void)SPDR0;                         /* discard the master's dummy */

    for (uint8_t i = 0; i < len; i++) {
        SPDR0 = buf[i];                  /* load next byte before next clock */
        while (!(SPSR0 & (1 << SPIF)));
        (void)SPDR0;
    }

    wait_ss_deassert();
    SPDR0 = 0x00;                        /* restore idle value */
    return 0;
}