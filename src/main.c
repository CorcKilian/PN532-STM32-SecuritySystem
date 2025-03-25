#include "spi.h"
#include "eeng1030_lib.h"
#include <stm32l432xx.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include "PN532.h"
#include "auth.h"

void setup(void);
void delay(volatile uint32_t dly);
void initSerial(uint32_t baudrate);
void eputc(char c);

char c = 0;

/**
 * Main entry point. Initializes system, PN532, and loops for UID actions.
 */
int main(void) {
    setup();
    delay(10000000);

    GPIOA->ODR &= ~(1 << 8);  // reset the board
    delay(500000);            // wait a little bit
    GPIOA->ODR |= (1 << 8);   // stop the reset
    delay(500000);            // wait a little bit
    GPIOB->ODR &= ~(1 << 0);  // pull Slave select low for the PN532
    delay(500000);            // Wait a little bit
    GPIOB->ODR |= (1 << 0);   // Pull slave select high
    delay(1000000);           // wait a little bit for everything to be up and running

    typedef enum {
        STATE_WAIT_INPUT,
        STATE_ADD_UID,
        STATE_AUTH_UID,
        STATE_REMOVE_UID
    } SystemState;

    uint8_t sam_cfg[] = {
        PN532_COMMAND_SAMCONFIGURATION,
        0x01,  // Normal mode
        0x14,  // Timeout = 100 ms (0x14 × 5ms)
        0x01   // Enable IRQ
    };

    // Initialise SAM
    sendCommand(sam_cfg, sizeof(sam_cfg));

    if (PN532_readACK()) {
        printf("ACK Read Succesfully.\r\n");
    }

    // read Response
    uint8_t rx_buf[PN532_MAX_FRAME_LEN];
    uint8_t rx_len = 0;

    // Pn532_readResponse() will return true if successful.
    // It also waits until IRQ is pulled low.
    if (PN532_readResponse(rx_buf, &rx_len)) {
        c = waitForByte_USART2();
        printf("Recieved: %c\r\nSAMConfig verification (0x15 expected):\r\n", c);
        for (uint8_t i = 0; i < rx_len; i++) {
            printf("Data[%d] = 0x%02X\n", i, rx_buf[i]);
        }
    }

    // End initialisation of SAM
    SystemState state = STATE_WAIT_INPUT;
    PN532_TagInfo tag;

    // Enter the main loop
    while (1) {
        switch (state) {
            case STATE_WAIT_INPUT:
                printf("\r\n--- UID Authentication System ---\r\n");
                printf("1. Add UID to Whitelist\r\n");
                printf("2. Authenticate UID\r\n");
                printf("3. Remove UID from Whitelist\r\n");
                printf("Select option: ");
                char cmd = waitForByte_USART2();
                printf("%c\r\n", cmd);  // Echo back selection

                if (cmd == '1') state = STATE_ADD_UID;
                else if (cmd == '2') state = STATE_AUTH_UID;
                else if (cmd == '3') state = STATE_REMOVE_UID;
                else {
                    printf("Invalid input.\r\n");
                    state = STATE_WAIT_INPUT;
                }
                break;

            case STATE_ADD_UID:
                printf("Place tag to ADD...\r\n");
                if (PN532_scanForTag(&tag)) {
                    add_uid(&tag); // messaging handled in auth.c
                } else {
                    printf("❌ No tag detected.\r\n");
                }
                state = STATE_WAIT_INPUT;
                break;

            case STATE_AUTH_UID:
                printf("Place tag to AUTHENTICATE...\r\n");
                if (PN532_scanForTag(&tag)) {
                    if (is_uid_authorized(&tag))
                        printf("✅ Access Granted.\r\n");
                    else
                        printf("❌ Access Denied.\r\n");
                } else {
                    printf("❌ No tag detected.\r\n");
                }
                state = STATE_WAIT_INPUT;
                break;

            case STATE_REMOVE_UID:
                printf("Place tag to REMOVE from whitelist...\r\n");
                if (PN532_scanForTag(&tag)) {
                    remove_uid(&tag); // messaging handled in auth.c
                } else {
                    printf("❌ No tag detected.\r\n");
                }
                state = STATE_WAIT_INPUT;
                break;
        }
    }
}

/**
 * setup
 *
 * Initializes clocks, GPIOs, SPI, USART, and enables interrupts.
 */
void setup() {
    initClocks();    
    RCC->AHB2ENR |= (1 << 0) + (1 << 1); // enable GPIOA and GPIOB
    pinMode(GPIOB, 0, 1); // This is the SS line
    pinMode(GPIOB, 1, 0); // This is the IRQ line
    pinMode(GPIOA, 8, 1); // This is the Reset PN532 line

    // This adds a pull-up resistor to PB1 to prevent it from floating.
    GPIOB->PUPDR &= ~(3 << (1 * 2)); // Clear bits 3:2 for PB1           
    GPIOB->PUPDR |=  (1 << (1 * 2)); // Set PB1 to pull-up mode (01)

    initSerial(9600);
    initSPI(SPI1);
    __asm(" cpsie i "); // enable interrupts globally
}

/**
 * Initializes USART2 with a specified baudrate.
 *
 * @param baudrate Desired baud rate (e.g., 9600).
 */
void initSerial(uint32_t baudrate) {
    RCC->AHB2ENR |= (1 << 0); // make sure GPIOA is turned on
    pinMode(GPIOA, 2, 2); // alternate function mode for PA2
    selectAlternateFunction(GPIOA, 2, 7); // AF7 = USART2 TX
    pinMode(GPIOA, 15, 2); // This sets pin A15 to alternative function
    selectAlternateFunction(GPIOA, 15, 3); // This sets up the alternative function for USART2 RX
    RCC->APB1ENR1 |= (1 << 17); // turn on USART2

    const uint32_t CLOCK_SPEED = 80000000;    
    uint32_t BaudRateDivisor = CLOCK_SPEED / baudrate;

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = (1 << 12); // disable over-run errors
    USART2->BRR = BaudRateDivisor;
    USART2->CR1 = (1 << 3);  // enable the transmitter
    USART2->CR1 |= (1 << 2); // enable the receiver
    USART2->CR1 |= (1 << 0); // enable USART
}

/**
 * Redirects printf() to USART2 for output.
 */
int _write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }
    while (len--) {
        eputc(*data);    
        data++;
    }    
    return 0;
}

/**
 * Sends a single character out over USART2.
 *
 * @param c Character to transmit.
 */
void eputc(char c) {
    while ((USART2->ISR & (1 << 6)) == 0); // wait for ongoing transmission to finish
    USART2->TDR = c;
}

/**
 * Delays for a number of CPU cycles.
 *
 * @param dly Loop count (not time-calibrated).
 */
void delay(volatile uint32_t dly) {
    while (dly--);
}
