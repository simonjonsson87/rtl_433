/** @file
    THis is a decoder for unknown signals detected in Southampton, UK. 

    Copyright (C) 2024 Simon Jonsson

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 */

/*
    Use this as a starting point for a new decoder.

    Keep the Doxygen (slash-star-star) comment above to document the file and copyright.

    Keep the Doxygen (slash-star-star) comment below to describe the decoder.
    See http://www.doxygen.nl/manual/markdown.html for the formatting options.

    Remove all other multiline (slash-star) comments.
    Use single-line (slash-slash) comments to annontate important lines if needed.

    To use this:
    - Copy this template to a new file
    - Change at least `new_template` in the source
    - Add to include/rtl_433_devices.h
    - Run ./maintainer_update.py (needs a clean git stage or commit)

    Note that for simple devices doorbell/PIR/remotes a flex conf (see conf dir) is preferred.
*/

/**
The device uses FSK_PULSE_PCM encoding,
- 0 is encoded as 40 us pulse and 132 us gap,
- 1 is encoded as 40 us pulse and 224 us gap.
The device sends a transmission every 60 seconds on average, but the transmissions are not perfectly regular.
A transmission starts with a preamble of 0xAAAAAAAAAAAAAAAA.


Most of the packet structure is not known.
Data layout:
    SS SS II II TU UU UU DD DD UU UU 

- S: These two bytes are almost always the same, so they may be static.
- I: These two bytes appear to be ids for different devices
- U: Unknown
- T: This nibble could be package type, because it corresponds with types of messages.
- D: This is likely two bytes of temperature. The value need to be multiplied by 0.1 to get a realistic celsius reading
*/

#include "decoder.h"
#include <stdio.h>
#include <stdlib.h>


char* bitsToString(unsigned char* b, int n);
char* bitsToString(unsigned char* b, int n) {
    // Calculate the number of characters needed
    //int numChars = (n + 7) / 8; // Number of characters needed
    int numChars = n; // Number of characters needed

    // Allocate memory for the string
    char* str = malloc(numChars + 1); // +1 for the null terminator
    if (str == NULL) {
        return NULL; // Memory allocation failed
    }

    // Parse each bit and convert to character
    

    for (int u = 0; u < (numChars + 7) / 8; ++u) {
        //printf("%d\n", b[u]);
        int ceiling = 0;
        if (numChars - u*8 <= 8) {
            ceiling = numChars - u*8;
        } else {
            ceiling = 8;
        }
        //printf("b[u]: %d; ceiling: %d; numChars: %d\n", b[u], ceiling, numChars);
        for (int i = 0; i < ceiling; ++i) {
            // Extract the ith bit from the rightmost position
            //str[i] = (b & (1 << (7 - i)))? '1' : '0';
            int y = u*8 + i;
            str[y] = ((b[u]) & (1 << (7 - i)))? '1' : '0';
            //printf("y=%d and result=%u\n", y, b[u]);
        }
    }

    // Add null terminator
    str[numChars] = '\0';

    return str;
}

char hex_table[16] = "0123456789ABCDEF";
void uint8_to_hex(uint8_t *data, size_t data_size, char *output, size_t output_size);
void uint8_to_hex(uint8_t *data, size_t data_size, char *output, size_t output_size) {
    //printf("data: %u; output: %u; data_size: %ld; output_size: %ld\n", *data, *output, data_size, output_size);

    for (size_t i = 0; i < data_size; ++i) {
        // Convert the high nibble to its ASCII representation
        output[i * 2] = hex_table[data[i] >> 4];
        // Convert the low nibble to its ASCII representation
        uint8_t lowNibble = data[i];
        lowNibble <<= 4;
        lowNibble >>= 4;
        output[i * 2 + 1] = hex_table[lowNibble];
        //printf("high: %d; low: %d\n", data[i] >> 4, lowNibble);
        //printf("%d%d", hex_table[data[i] >> 4], hex_table[lowNibble]);
    }

    // Null-terminate the output string
    output[data_size * 2] = '\0';

    //printf("===============\n");
    
}


#include <math.h>
#include <string.h>
static int unknown1_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    int r;      // a row index
    uint8_t *b; // bits of a row
    //int currentByteIndex = 0;

    // Check the total size of the 
    int totalLen = 0;
    for (r = 0; r < bitbuffer->num_rows; ++r) {
        totalLen += bitbuffer->bits_per_row[r];
    }

    char* binaryData = (char*)malloc(totalLen);
    //int binaryDataPointer = 0;    

    uint8_t* rawData = (uint8_t*)malloc(totalLen);
    int rawDataPointer = 0;    


    /*
     * Early debugging aid to see demodulated bits in buffer and
     * to determine if your limit settings are matched and firing
     * this decode callback.
     *
     * 1. Enable with -vvv (debug decoders)
     * 2. Delete this block when your decoder is working
     */
    decoder_log_bitbuffer(decoder, 2, __func__, bitbuffer, "");

    /*
     * If you expect the bits flipped with respect to the demod
     * invert the whole bit buffer.
     */

    //bitbuffer_invert(bitbuffer);

    /*
     * The bit buffer will contain multiple rows.
     * Typically a complete message will be contained in a single
     * row if long and reset limits are set correctly.
     * May contain multiple message repeats.
     * Message might not appear in row 0, if protocol uses
     * start/preamble periods of different lengths.
     */

    
    /*
     * Either, if you expect just a single packet
     * loop over all rows and collect or output data:
     */
    //printf("unknown1_decode-num_rows: %d\n", bitbuffer->num_rows);

    // Rows loop
    for (r = 0; r < bitbuffer->num_rows; ++r) {
        b = bitbuffer->bb[r];
        int rowBitCount = bitbuffer->bits_per_row[r];

        //printf("#### Row %d: %s #####\n", r, bitsToString(b, rowBitCount));
        // Bits loop
        for (int signalBitCounter = 0; signalBitCounter < rowBitCount; ++signalBitCounter) {
            int signalByteIndex = floor(signalBitCounter / 8);
            int signalBitInByteIndex = signalBitCounter % 8;
            uint8_t signalBit = b[signalByteIndex];
            

            int rawByteIndex = floor(rawDataPointer / 8);
            int rawBitInByteIndex = rawDataPointer % 8;
            uint8_t rawByte = rawData[rawByteIndex];
            
            signalBit <<= signalBitInByteIndex;
            signalBit >>= 7;
            signalBit <<= (7 - rawBitInByteIndex);

            rawByte |= signalBit;
            rawData[rawByteIndex] = rawByte;
            rawDataPointer++;

            //printf("signalBitCounter: %d; signalBitInByteIndex: %d;signalBit: %s\n", signalBitCounter, signalBitInByteIndex, bitsToString(&signalBit, 8));
            //printf("rawDataPointer: %d; rawByteIndex: %d; rawBitInByteIndex: %d;rawByte: %s\n", rawDataPointer, rawByteIndex, rawBitInByteIndex, bitsToString(&rawByte, 8));
            //printf("%s\n", bitsToString(rawData, rawDataPointer));
        }
    }
    
    // Check preamble
    for (int i = 0; i < 8; i++) {
        if (rawData[i] != 0xAA) return 0;
    }

    char* binary = (char*)malloc(totalLen);
    binary = bitsToString(rawData, rawDataPointer);

    //printf("#####################################################################\n");
    char* hexChars = (char*)malloc(totalLen);
    uint8_to_hex(rawData, ceil(totalLen/8.0), hexChars, ceil(totalLen/4.0));

    
    //printf("rowCount: %d; bitCount: %d; rawData: %s\n", bitbuffer->num_rows, totalLen, bitsToString(rawData, rawDataPointer));

    //printf("%s\n", hexChars);

    data = data_make(
            "model",            "",             DATA_STRING,    "Unknown1",
            "short_width",      NULL,           DATA_DOUBLE,    decoder->short_width,
            "long_width",       NULL,           DATA_DOUBLE,    decoder->long_width,
            "gap_limit",        NULL,           DATA_DOUBLE,    decoder->gap_limit,
            "reset_limit",      NULL,           DATA_DOUBLE,    decoder->reset_limit,
            "lengthBits",       "lengthBits",   DATA_INT,       totalLen,
            "hex_data",         NULL,           DATA_STRING,    hexChars,
            "binary_data",      NULL,           DATA_STRING,    binary,
            "rowCount",         "rowCount",     DATA_INT,       bitbuffer->num_rows,
            NULL);

    free(binaryData);
    free(rawData);
    free(binary);
    /* clang-format on */
    //printf("Before decoder_output_data in unknown1"); // Simon
    decoder_output_data(decoder, data);

    // Return 1 if message successfully decoded
    return 1;
}

/*
 * List of fields that may appear in the output
 *
 * Used to determine what fields will be output in what
 * order for this device when using -F csv.
 *
 */
static char const *const output_fields[] = {
        "model",
        "short_width",
        "long_width", 
        "gap_limit",  
        "reset_limit",
        "lengthBits", 
        "hex_data",
        "binary_data",
        "rowCount",
        NULL,
};

/*
 * r_device - registers device/callback. see rtl_433_devices.h
 *
 * Timings:
 *
 * short, long, and reset - specify pulse/period timings in [us].
 *     These timings will determine if the received pulses
 *     match, so your callback will fire after demodulation.
 *
 * Modulation:
 *
 * The function used to turn the received signal into bits.
 * See:
 * - pulse_slicer.h for descriptions
 * - r_device.h for the list of defined names
 *
 * This device is disabled and hidden, it can not be enabled.
 *
 * To enable your device, append it to the list in include/rtl_433_devices.h
 * and sort it into src/CMakeLists.txt or run ./maintainer_update.py
 *
 */
r_device const unknown1 = {
        .name        = "Unknown1",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 500,  // Nominal width of pulse [us]
        .long_width  = 500,  // Nominal width of bit period [us]
        .gap_limit   = 7000,  // some distance above long
        .reset_limit = 10000, // a bit longer than packet gap. 9000 better than 10000
        .decode_fn   = &unknown1_decode,
        .disabled    = 0, // 3 = disabled and hidden, use 0 if there is a MIC, 1 otherwise
        .fields      = output_fields,
};

// gap-limit    reset-limit
// 7000         10000
// 7000         11000
// 7000         12000
// 8000         10000
// 8000         11000
// 8000         12000
// 9000         10000
// 9000         11000
// 9000         12000
// 7000         9000
// 6000         10000

// 6000 - more than one row per packet
// 7000 159 147
// 8000 159 149
// 9000 159 151
// 10000 - Second message not identified because the first bits are not 10101010

// 9000 400 
// 9000 500  114 111
// 9000 600  114 111
// 9000 1000 124 122
// 9000 2000 136 131
// 9000 4000 151 141 (second row)
// 9000 8000 159 149
// 9000 10000 159 152
// 9000 11000 159 152