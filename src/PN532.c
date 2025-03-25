#include "PN532.h"
#include <stm32l432xx.h>
#include "eeng1030_lib.h"
#include "spi.h"
#include <stdio.h>   // for printf, scanf, etc.
#include <stdlib.h>  // for malloc, free, exit
#include <string.h>  // for strcmp, strlen, etc.

uint16_t count = 0;

/**
 * delayPN
 *
 * Simple blocking delay used in PN532-related timing.
 *
 * @param dly Arbitrary delay count (not calibrated to real time).
 */
void delayPN(volatile uint32_t dly) {
    while (dly--);
}

/**
 * PN532_SS_LOW
 *
 * Pulls the PN532 SPI Slave Select line LOW.
 */
void PN532_SS_LOW() {
    GPIOB->ODR &= ~(1 << 0);
}

/**
 * PN532_SS_HIGH
 *
 * Pulls the PN532 SPI Slave Select line HIGH.
 */
void PN532_SS_HIGH() {
    GPIOB->ODR |= (1 << 0);
}

/**
 * sendCommand
 *
 * Sends a command frame to the PN532 over SPI.
 * Based on Adafruit's PN532 library implementation.
 *
 * @param cmd Pointer to command buffer (excluding TFI).
 * @param cmdlen Length of the command (excluding TFI).
 */
void sendCommand(const uint8_t *cmd, uint8_t cmdlen) {
    // Actual command length must include TFI
    uint8_t frameLen = cmdlen + 1;

    PN532_SS_LOW();
    delayPN(PN532_Delay80MHZ);  // Ensure PN532 is awake

    transferSPI8(SPI1, PN532_SPI_DATAWRITE); // Notify PN532 of write

    // Write Preamble, Start Code
    transferSPI8(SPI1, PN532_PREAMBLE);
    transferSPI8(SPI1, PN532_PREAMBLE);
    transferSPI8(SPI1, PN532_STARTCODE2);

    // Write Length and LCS
    transferSPI8(SPI1, frameLen);
    transferSPI8(SPI1, (0x100 - frameLen) & 0xFF);

    // Write TFI (Host to PN532 identifier)
    transferSPI8(SPI1, PN532_HOSTTOPN532);

    uint8_t checksum = PN532_HOSTTOPN532; // Start with TFI

    for (uint8_t i = 0; i < cmdlen; i++) {
        transferSPI8(SPI1, cmd[i]);      // Send command byte
        checksum += cmd[i];              // Update checksum
    }

    transferSPI8(SPI1, ((~checksum + 1) & 0xFF)); // DCS
    transferSPI8(SPI1, PN532_POSTAMBLE);         // Postamble

    PN532_SS_HIGH();
}

/**
 * PN532_readACK
 *
 * Waits for and verifies PN532 ACK response pattern.
 *
 * @return true if valid ACK received, false otherwise.
 */
bool PN532_readACK(void) {
    const uint8_t ACK_PATTERN[6] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00 };
    uint8_t window[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t byte;
    bool ack_found = false;

    while (PN532_IRQcheck); // wait for IRQ

    PN532_SS_LOW();
    delayPN(PN532_Delay80MHZ);

    // Read up to 12 bytes to search for valid ACK frame
    for (int i = 0; i < 12; i++) {
        byte = transferSPI8(SPI1, 0xFF);

        for (int j = 0; j < 5; j++)
            window[j] = window[j + 1];
        window[5] = byte;

        bool match = true;
        for (int j = 0; j < 6; j++) {
            if (window[j] != ACK_PATTERN[j]) {
                match = false;
                break;
            }
        }

        if (match) {
            ack_found = true;
            break;
        }
    }

    PN532_SS_HIGH();
    return ack_found;
}

/**
 * PN532_readResponse
 *
 * Reads a full response frame from the PN532.
 *
 * @param data_out Pointer to buffer to store response payload (excluding TFI).
 * @param data_len_out Pointer to store number of payload bytes.
 * @return true if successful and response is valid, false otherwise.
 */
bool PN532_readResponse(uint8_t *data_out, uint8_t *data_len_out) {
    uint8_t byte;
    uint8_t len, lcs, tfi;
    uint8_t dcs, postamble;
    uint8_t frame_data[PN532_MAX_FRAME_LEN];
    uint8_t sync_window[3] = { 0xFF, 0xFF, 0xFF };
    uint8_t sync_found = 0;

    uint32_t elapsed = 0;
    while (PN532_IRQcheck && elapsed < PN532_RESPONSE_TIMEOUT_MS) {
        delayPN_ms(1);
        elapsed++;
    }

    if (PN532_IRQcheck) {
        *data_len_out = 0;
        return false;  // timeout
    }

    PN532_SS_LOW();
    delayPN(PN532_Delay80MHZ);
    transferSPI8(SPI1, PN532_SPI_DATAREAD);  // SPI read command

    // Find frame preamble
    for (int i = 0; i < 8; i++) {
        byte = transferSPI8(SPI1, 0xFF);
        sync_window[0] = sync_window[1];
        sync_window[1] = sync_window[2];
        sync_window[2] = byte;

        if (sync_window[0] == 0x00 && sync_window[1] == 0x00 && sync_window[2] == 0xFF) {
            sync_found = 1;
            break;
        }
    }

    if (!sync_found)
        goto fail;

    // Read LEN, LCS, TFI
    len = transferSPI8(SPI1, 0xFF);
    lcs = transferSPI8(SPI1, 0xFF);
    tfi = transferSPI8(SPI1, 0xFF);

    if ((uint8_t)(len + lcs) != 0x00 || len < 1 || len > PN532_MAX_FRAME_LEN)
        goto fail;

    for (int i = 0; i < len - 1; i++)
        frame_data[i] = transferSPI8(SPI1, 0xFF);

    dcs = transferSPI8(SPI1, 0xFF);
    postamble = transferSPI8(SPI1, 0xFF);

    PN532_SS_HIGH();

    uint8_t sum = tfi;
    for (int i = 0; i < len - 1; i++)
        sum += frame_data[i];

    if ((uint8_t)(sum + dcs) != 0x00 || postamble != 0x00)
        goto fail;

    for (int i = 0; i < len - 1; i++)
        data_out[i] = frame_data[i];

    *data_len_out = len - 1;
    return true;

fail:
    PN532_SS_HIGH();
    *data_len_out = 0;
    return false;
}

/**
 * checkStatus
 *
 * Performs a simple status check or data-read initiation on the PN532.
 *
 * @param type 2 for status read, 3 for data read.
 */
void checkStatus(uint8_t type) {
    PN532_SS_LOW();
    delayPN(PN532_Delay80MHZ);  // Ensure PN532 is awake

    if (type == 2) {
        transferSPI8(SPI1, 0x02);
    } else if (type == 3) {
        transferSPI8(SPI1, 0x03);
    }

    transferSPI8(SPI1, 0xFF);
    PN532_SS_HIGH();
}

/**
 * PN532_scanForTag
 *
 * Sends an InListPassiveTarget command and parses tag information.
 *
 * @param tag_info Pointer to PN532_TagInfo struct to populate.
 * @return true if a valid tag is detected and parsed, false otherwise.
 */
bool PN532_scanForTag(PN532_TagInfo *tag_info) {
    const uint8_t inlist_cmd[] = {
        0x4A,  // INLISTPASSIVETARGET
        0x01,  // Max number of targets
        0x00   // Baud rate: 106 kbps (Type A)
    };

    for (uint8_t attempt = 0; attempt < PN532_SCAN_MAX_RETRIES; attempt++) {
        sendCommand(inlist_cmd, sizeof(inlist_cmd));

        if (!PN532_readACK()) {
            delayPN_ms(PN532_SCAN_RETRY_DELAY);
            continue;
        }

        uint8_t buf[32] = {0};
        uint8_t len = 0;

        if (!PN532_readResponse(buf, &len)) {
            delayPN_ms(PN532_SCAN_RETRY_DELAY);
            continue;
        }

        if (len < 7 || buf[0] != 0x4B)
            continue;

        tag_info->nb_targets    = buf[1];
        tag_info->target_number = buf[2];
        tag_info->sens_res[0]   = buf[3];
        tag_info->sens_res[1]   = buf[4];
        tag_info->sel_res       = buf[5];
        tag_info->uid_len       = buf[6];

        if (tag_info->uid_len > sizeof(tag_info->uid))
            return false;

        for (uint8_t i = 0; i < tag_info->uid_len; i++) {
            tag_info->uid[i] = buf[7 + i];
        }

        tag_info->response_len = len;
        for (uint8_t i = 0; i < len; i++) {
            tag_info->raw_response[i] = buf[i];
        }

        // Print Tag Results
        printf("Tag detected!\r\n");
        printf("  Target #: %u\r\n", tag_info->target_number);
        printf("  Number of Targets: %u\r\n", tag_info->nb_targets);
        printf("  ATQA (SENS_RES): %02X %02X\r\n", tag_info->sens_res[0], tag_info->sens_res[1]);
        printf("  SAK (SEL_RES): %02X\r\n", tag_info->sel_res);
        printf("  UID [%u bytes]: ", tag_info->uid_len);
        for (uint8_t i = 0; i < tag_info->uid_len; i++) {
            printf("%02X ", tag_info->uid[i]);
        }
        printf("\r\n");

        return true;
    }

    return false;
}

/**
 * delayPN_ms
 *
 * Millisecond blocking delay using NOP loops.
 *
 * @param ms Number of milliseconds to delay.
 */
void delayPN_ms(uint32_t ms) {
    volatile uint32_t count = ms * (SystemCoreClock / 1000);
    while (count--) {
        __NOP(); // Prevent compiler optimization
    }
}
