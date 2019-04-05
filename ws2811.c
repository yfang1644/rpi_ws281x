/*
 * ws2811.c
 *
 * Copyright (c) 2014 Jeremy Garff <jer @ jers.net>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     1.  Redistributions of source code must retain the above copyright notice, this list of
 *         conditions and the following disclaimer.
 *     2.  Redistributions in binary form must reproduce the above copyright notice, this list
 *         of conditions and the following disclaimer in the documentation and/or other materials
 *         provided with the distribution.
 *     3.  Neither the name of the owner nor the names of its contributors may be used to endorse
 *         or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <byteswap.h>
#include <time.h>

#include "ws2811.h"


/* 4 colors (R, G, B + W), 8 bits per byte, 3 symbols per bit + 55uS low for reset signal */
#define LED_COLOURS                              4
#define LED_RESET_uS                             55
#define LED_BIT_COUNT(leds, freq)                ((leds * LED_COLOURS * 8 * 4) \
                                + ((LED_RESET_uS * (freq * 3)) / 1000000))

/* Minimum time to wait for reset to occur in microseconds. */
#define LED_RESET_WAIT_TIME                      300

// Pad out to the nearest uint32 + 32-bits for idle low/high times the number of channels
#define PCM_BYTE_COUNT(leds, freq)               ((((LED_BIT_COUNT(leds, freq) >> 3) & ~0x7) + 4) + 4)
#define PWM_BYTE_COUNT(leds, freq)               (PCM_BYTE_COUNT(leds, freq) * RPI_PWM_CHANNELS)

// Symbol definitions
#define SYMBOL_HIGH                              0xe  // 1 1 1 0
#define SYMBOL_LOW                               0x8  // 1 0 0 0

// Symbol definitions for software inversion (PCM and SPI only)
#define SYMBOL_HIGH_INV                          0x1  // 0 0 0 1
#define SYMBOL_LOW_INV                           0x7  // 0 1 1 1

// Driver mode definitions
#define NONE	0
#define PWM	1
#define PCM	2
#define SPI	3

typedef struct ws2811_device
{
    volatile uint8_t *pxl_raw;
    int spi_fd;
    int max_count;
} ws2811_device_t;

/**
 * Provides monotonic timestamp in microseconds.
 *
 * @returns  Current timestamp in microseconds or 0 on error.
 */
static uint64_t get_microsecond_timestamp()
{
    struct timespec t;

    if (clock_gettime(CLOCK_MONOTONIC_RAW, &t) != 0) {
        return 0;
    }

    return (uint64_t) t.tv_sec * 1000000 + t.tv_nsec / 1000;
}

/**
 * Iterate through the channels and find the largest led count.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  Maximum number of LEDs in all channels.
 */
static int max_channel_led_count(ws2811_t *ws2811)
{
    int chan, max = 0;

    if (ws2811->channel.count > max)
    {
        max = ws2811->channel.count;
    }

    return max;
}

/**
 * Initialize the SPI buffer with all zeros.
 * The buffer length is assumed to be a word multiple.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
void spi_raw_init(ws2811_t *ws2811)
{
    volatile uint32_t *pxl_raw = (uint32_t *)ws2811->device->pxl_raw;
    int maxcount = ws2811->device->max_count;
    int wordcount = PCM_BYTE_COUNT(maxcount, ws2811->freq) / sizeof(uint32_t);
    int i;

    for (i = 0; i < wordcount; i++)
    {
        pxl_raw[i] = 0x0;
    }
}

/**
 * Cleanup previously allocated device memory and buffers.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
void ws2811_cleanup(ws2811_t *ws2811)
{
    ws2811_device_t *device = ws2811->device;
    int chan;

    if (ws2811->channel.leds)
    {
        free(ws2811->channel.leds);
    }
    ws2811->channel.leds = NULL;
    if (ws2811->channel.gamma)
    {
        free(ws2811->channel.gamma);
    }
    ws2811->channel.gamma = NULL;

    if (device && (device->spi_fd > 0))
    {
        close(device->spi_fd);
    }

    if (device) {
        free(device);
    }
    ws2811->device = NULL;
}

static ws2811_return_t spi_init(ws2811_t *ws2811)
{
    int spi_fd;
    static uint8_t mode;
    static uint8_t bits = 8;
    uint32_t speed = ws2811->freq * 3;
    ws2811_device_t *device = ws2811->device;

    spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) {
        fprintf(stderr, "Cannot open /dev/spidev0.0. spi_bcm2835 module not loaded?\n");
        return WS2811_ERROR_SPI_SETUP;
    }
    device->spi_fd = spi_fd;

    // SPI mode
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0)
    {
        return WS2811_ERROR_SPI_SETUP;
    }
    if (ioctl(spi_fd, SPI_IOC_RD_MODE, &mode) < 0)
    {
        return WS2811_ERROR_SPI_SETUP;
    }

    // Bits per word
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
    {
        return WS2811_ERROR_SPI_SETUP;
    }
    if (ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
    {
        return WS2811_ERROR_SPI_SETUP;
    }

    // Max speed Hz
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
        return WS2811_ERROR_SPI_SETUP;
    }
    if (ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
    {
        return WS2811_ERROR_SPI_SETUP;
    }

    // Initialize device structure elements to not used
    // except driver_mode, spi_fd and max_count (already defined when spi_init called)
    device->pxl_raw = NULL;

    // Allocate LED buffer
    ws2811_channel_t *channel = &ws2811->channel;
    channel->leds = malloc(sizeof(ws2811_led_t) * channel->count);
    if (!channel->leds)
    {
        ws2811_cleanup(ws2811);
        return WS2811_ERROR_OUT_OF_MEMORY;
    }
    memset(channel->leds, 0, sizeof(ws2811_led_t) * channel->count);
    if (!channel->strip_type)
    {
      channel->strip_type=WS2811_STRIP_RGB;
    }

    // Set default uncorrected gamma table
    if (!channel->gamma)
    {
      channel->gamma = malloc(sizeof(uint8_t) * 256);
      int x;
      for(x = 0; x < 256; x++){
        channel->gamma[x] = x;
      }
    }

    channel->wshift = (channel->strip_type >> 24) & 0xff;
    channel->rshift = (channel->strip_type >> 16) & 0xff;
    channel->gshift = (channel->strip_type >> 8)  & 0xff;
    channel->bshift = (channel->strip_type >> 0)  & 0xff;

    // Allocate SPI transmit buffer (same size as PCM)
    device->pxl_raw = malloc(PCM_BYTE_COUNT(device->max_count, ws2811->freq));
    if (device->pxl_raw == NULL)
    {
        ws2811_cleanup(ws2811);
        return WS2811_ERROR_OUT_OF_MEMORY;
    }
    spi_raw_init(ws2811);

    return WS2811_SUCCESS;
}

static ws2811_return_t spi_transfer(ws2811_t *ws2811)
{
    int ret;
    struct spi_ioc_transfer tr;

    memset(&tr, 0, sizeof(struct spi_ioc_transfer));
    tr.tx_buf = (unsigned long)ws2811->device->pxl_raw;
    tr.rx_buf = 0;
    tr.len = PCM_BYTE_COUNT(ws2811->device->max_count, ws2811->freq);

    ret = ioctl(ws2811->device->spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
    {
        fprintf(stderr, "Can't send spi message");
        return WS2811_ERROR_SPI_TRANSFER;
    }

    return WS2811_SUCCESS;
}


/*
 *
 * Application API Functions
 *
 */


/**
 * Allocate and initialize memory, buffers, pages, PWM, DMA, and GPIO.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  0 on success, -1 otherwise.
 */
ws2811_return_t ws2811_init(ws2811_t *ws2811)
{
    ws2811_device_t *device;

    ws2811->device = malloc(sizeof(*ws2811->device));
    if (!ws2811->device)
    {
        return WS2811_ERROR_OUT_OF_MEMORY;
    }
    memset(ws2811->device, 0, sizeof(*ws2811->device));
    device = ws2811->device;

    device->max_count = max_channel_led_count(ws2811);

    return spi_init(ws2811);
}

/**
 * Shut down DMA, PWM, and cleanup memory.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
void ws2811_fini(ws2811_t *ws2811)
{
    ws2811_cleanup(ws2811);
}

/**
 * Render the DMA buffer from the user supplied LED arrays and start the DMA
 * controller.  This will update all LEDs on both PWM channels.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
ws2811_return_t  ws2811_render(ws2811_t *ws2811)
{
    volatile uint8_t *pxl_raw = ws2811->device->pxl_raw;
    uint32_t *wordptr;
    int bitpos;
    int i, k, l, chan;
    unsigned j;
    ws2811_return_t ret = WS2811_SUCCESS;
    uint32_t protocol_time = 0;
    static uint64_t previous_timestamp = 0;

    bitpos = 32;

    ws2811_channel_t *channel = &ws2811->channel;

    const int scale = (channel->brightness & 0xff) + 1;
    uint8_t array_size = 24; // Assume 3 color LEDs, RGB, 24bits

    // If our shift mask includes the highest nibble, then we have 4 LEDs, RBGW.
    if (channel->strip_type & SK6812_SHIFT_WMASK)
    {
        array_size = 32;
    }

    // 1.25µs per bit
    const uint32_t channel_protocol_time = channel->count * array_size * 1.25;

    // Only using the channel which takes the longest as both run in parallel
    if (channel_protocol_time > protocol_time)
    {
        protocol_time = channel_protocol_time;
    }

    wordptr = &((uint32_t *)pxl_raw)[0];
    for (i = 0; i < channel->count; i++)                // Led
    {
        uint8_t r, g, b, w;
        uint32_t color;
        r = channel->gamma[(((channel->leds[i] >> channel->rshift) & 0xff) * scale) >> 8];
        g = channel->gamma[(((channel->leds[i] >> channel->gshift) & 0xff) * scale) >> 8];
        b = channel->gamma[(((channel->leds[i] >> channel->bshift) & 0xff) * scale) >> 8];
        w = channel->gamma[(((channel->leds[i] >> channel->wshift) & 0xff) * scale) >> 8];

        color = (r << 24)|(g << 16)|(b << 8)|w;

        for (j = 0; j < array_size; j++)               // Color
        {
            uint32_t symbol = SYMBOL_LOW;
            if (color & 0x80000000)
            symbol = SYMBOL_HIGH;

            color <<= 1;
            // Inversion is handled by hardware for PWM, otherwise by software here
            if (channel->invert)
                symbol = (~symbol) & 0x0f;   /* INVERTED */

            bitpos -= 4;
            *wordptr &= ~(15 << bitpos);
            *wordptr |= (symbol << bitpos);

            if (bitpos == 0)
            {
                *wordptr = bswap_32(*wordptr);
                wordptr += 1;
                bitpos = 32;
            }
        }
    }

    if (ws2811->render_wait_time != 0) {
        const uint64_t current_timestamp = get_microsecond_timestamp();
        uint64_t time_diff = current_timestamp - previous_timestamp;

        if (ws2811->render_wait_time > time_diff) {
            usleep(ws2811->render_wait_time - time_diff);
        }
    }

    ret = spi_transfer(ws2811);

    // LED_RESET_WAIT_TIME is added to allow enough time for the reset to occur.
    previous_timestamp = get_microsecond_timestamp();
    ws2811->render_wait_time = protocol_time + LED_RESET_WAIT_TIME;

    return ret;
}

const char * ws2811_get_return_t_str(const ws2811_return_t state)
{
    const int index = -state;
    static const char * const ret_state_str[] = { WS2811_RETURN_STATES(WS2811_RETURN_STATES_STRING) };

    if (index < (int)(sizeof(ret_state_str) / sizeof(ret_state_str[0])))
    {
        return ret_state_str[index];
    }

    return "";
}
