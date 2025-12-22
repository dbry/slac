////////////////////////////////////////////////////////////////////////////
//                           ****** SLAC ******                           //
//                    Simple Lossless Audio Compressor                    //
//                 Copyright (c) 2019 - 2022 David Bryant                 //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// libslac.c

// This module provides functions to compress and decompress PCM integer
// audio. The audio is presented in 32-bit integers, but can be arbitrary
// bitdepths (redundant bits at either side of the words are handled). Mono
// and stereo streams are supported and up to 24 significant bits per sample
// (anywhere in the 32-bit word).
//
// No allocation is used; everything is done in place using the passed input
// and output buffers.

//                          Notes on this Version
//                          ---------------------

// This is an experimental version of SLAC to test the concept of having a
// sequence of 'k' values for each frame, as opposed to having a single 'k'
// value for the frame. This optimizes for several situations where the
// residuals may not approximate a Laplace distribution, for example if the
// magnitude is changing rapidly within the context of one frame (e.g., a
// lot of small values and then a lot of large values). This can also help
// in the situation where the optimal Rice 'k' value is midway between
// integers or the preceding decorrelation is sub-optimal.
//
// This code can handle a minimum of two 'k' values, and there is no hard-
// coded upper limit to the number of 'k' values. However, the practical
// limit is around eight, with an optimal count of four or five. The method
// for calculating the best sequence is brute-force recursive and can become
// very slow with a large number of 'k' values, and also the values must
// be encoded and transmitted in the frame for the decoder which reduces
// the gains from higher numbers of values. Note that the maximum number
// of zones is NOT transmitted each frame but IS required by the decoder
// (it's a compile time option for both the encoder and decoder).
//
// As mentioned, the encoding speed is greatly affected with this, however
// the decoding speed is only slightly slower. This method improves the
// compression of my standard test corpus by about 0.34 percent. The
// improvement is much better with 8-bit data where it approaches 1 percent,
// and is also greater when compressing non-standard audio data (like sine
// tones).

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "bitstream.h"
#include "libslac.h"

// If your platform has hardware support for counting leading zeros in 32-bit
// integers, define it here (these are for gcc and friends).

#if INT_MAX == 32767
#define count_leading_zeros __builtin_clzl
#else
#define count_leading_zeros __builtin_clz
#endif

#ifdef NO_BUILTIN_CLZ

// If your platform does not support __builtin_clz() or a variant, define this macro
// to instantiate a native C implementation. It is a hybrid of the Hacker's Delight
// binary search method and a table-driven technique.

static inline int count_leading_zeros (uint32_t x)
{
    static const char nbits_table [] = {
        0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4,     // 0 - 15
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,     // 16 - 31
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,     // 32 - 47
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,     // 48 - 63
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,     // 64 - 79
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,     // 80 - 95
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,     // 96 - 111
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,     // 112 - 127
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,     // 128 - 143
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,     // 144 - 159
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,     // 160 - 175
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,     // 176 - 191
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,     // 192 - 207
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,     // 208 - 223
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,     // 224 - 239
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8      // 240 - 255
    };

    uint32_t y;
    int n = 32;

    if ((y = x >> 16)) {
        n -= 16;
        x = y;
    }

    if ((y = x >> 8)) {
        n -= 8;
        x = y;
    }

    return n - nbits_table [x];
}
#endif

// These complementary macros convert from signed integers into the non-negative
// values that slac uses for analysis and entropy encoding (with the sign moved
// to the LSB), and back.

#define signed_to_non_negative(x)   (((uint32_t)(x)<<1)^((int32_t)(x)>>31))
#define non_negative_to_signed(x)   ((int32_t)(-(int32_t)((x)&1)^(x))>>1)

#define MAX_DECORR 6

// Complementary decorrelate / correlate functions used by both encoding and decoding

static int decorrelate (int32_t *audio_samples, int sample_count, int stride);
static int correlate (int32_t *audio_samples, int sample_count, int stride);

static int32_t decorrs [8], rice_ks [32], shifts [32];  // used for statistics only

/******************************* COMPRESSION *********************************/

static void entropy_encode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride, int flags);
static int redundant_bits (int32_t *audio_samples, int sample_count, int stride);
static int best_decorr (int32_t *audio_samples, int sample_count, int stride);
static int channels_identical (int32_t *audio_samples, int sample_count);
static void lr_to_ms (int32_t *audio_samples, int sample_count);

// Compress an array of audio samples (in either 1 or 2 interleaved channels)
// into the provided buffer (which should be big enough for the data). The
// return value is the number of bytes used (rounded up to the next word) or
// -1 indicating an error. The stereo_mode only applies if num_chans is 2
// (obviously) and can be either MID_SIDE or LEFT_RIGHT (see libslac.h). Note
// that selecting the better of left-right and mid-side encoding is not done
// here, but can easily be done at the next higher level by simply performing
// this operation for both methods and choosing the smaller.

// Note: the audio samples are processed in place, they are not "const" !!

int compress_audio_block (int32_t *audio_samples, int sample_count, int num_chans, int flags, char *outbuffer, int outbufsize)
{
    int sent_chans = num_chans, stereo_mode = flags & 0x3, chan;
    Bitstream bs;

    // this is the processing unique to stereo...from here on it's just 1 or 2 mono channels

    if (num_chans == 2) {
        if (channels_identical (audio_samples, sample_count)) {
            stereo_mode = DUAL_MONO;
            sent_chans = 1;
        }
        else if (stereo_mode == MID_SIDE)
            lr_to_ms (audio_samples, sample_count);
        else
            stereo_mode = LEFT_RIGHT;
    }
    else
        stereo_mode = MONO_MODE;

    // open the bitstream for writing and store the stereo mode in the first two bits

    bs_open_write (&bs, outbuffer, outbuffer + outbufsize);
    putbits (stereo_mode, 2, &bs);
    putbit (flags & EXTRA_K_MASK, &bs);

    // the channels (1 or 2) are processed completely independently and sequentially here

    for (chan = 0; chan < sent_chans; ++chan) {
        int shift, decorr;

        // check whether there are LSB zeros in every sample that can be shifted out

        shift = redundant_bits (audio_samples + chan, sample_count, num_chans);
        shifts [shift]++;

        // find the best decorrelator (from -1 to 6) and write that to the stream in 3 bits

        decorr = best_decorr (audio_samples + chan, sample_count, num_chans);
        putbits (decorr + 1, 3, &bs);
        decorrs [decorr + 1]++;

        // since the shift amount is generally small, store that in unary (1's followed by 0)
        while (shift--)
            putbit_1 (&bs);

        putbit_0 (&bs);

        // because the magnitude of the first few samples can be very different than the rest
        // of the data (because they're not as decorrelated), we store those using their own
        // Rice parameter; this only costs 5 bits per channel, but can make a huge improvement

        if (decorr && abs (decorr) < sample_count) {
            entropy_encode (&bs, audio_samples + chan, abs (decorr), num_chans, flags);
            entropy_encode (&bs, audio_samples + abs (decorr) * num_chans + chan, sample_count - abs (decorr), num_chans, flags);
        }
        else
            entropy_encode (&bs, audio_samples + chan, sample_count, num_chans, flags);
    }

    return bs_close_write (&bs);    // close the bitstream and return the number of bytes written
}

static void entropy_encode_simple (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride);
static void entropy_encode_zones (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride, int flags);
static int best_rice_k (const int32_t *audio_samples, int sample_count, int stride);

// Encode the supplied array of samples into a bitstream using Rice encoding.
// If stereo data is being compressed the channels must be processed in two
// separate passes and the stride parameter should be 2 and the pointer set to
// the first sample of the desired channel.

static void entropy_encode_simple (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride)
{
    int rice_k = best_rice_k (audio_samples, sample_count, stride);

    if (sample_count > MAX_DECORR)  // only count the "real" blocks in the statistics
        rice_ks [rice_k + 1]++;

    putbits (rice_k + 1, 5, bs);    // the Rice k parameter (offset 1) is stored in the first 5 bits

    if (rice_k > 0) {
        uint32_t mask = (1UL << rice_k) - 1;

        while (sample_count--) {
            uint32_t avalue = signed_to_non_negative (*audio_samples);
            int modulo = avalue >> rice_k;

            while (modulo--)
                putbit_1 (bs);

            putbit_0 (bs);
            avalue &= mask;
            putbits (avalue, rice_k, bs);
            audio_samples += stride;
        }
    }
    else if (rice_k == 0)
        while (sample_count--) {
            uint32_t avalue = signed_to_non_negative (*audio_samples);

            while (avalue--)
                putbit_1 (bs);

            putbit_0 (bs);
            audio_samples += stride;
        }
}

#define MAX_NUM_ZONES 8     // number of k-zone slots to allocate

// Zone 'k' Encoding
// -----------------
// 1. first 5 bits are initial 'k' (just like regular entropy encoder)
// 2. each zone (up to MAX_NUM_ZONES - 1) follows:
//
// OPERATION    ENCODING BITS
// ---------    -------------
// terminate    0-0
//  Δk = 0      0-1
//  Δk = -1     1-1
//  Δk = +1     1-1-0
//  Δk = -2     1-1-1-0
//  Δk = +2     1-1-1-1-0
//  Δk = -3     1-1-1-1-1-0
//    ...          ...

// 3. no terminator required when MAX_NUM_ZONES is reached
// 4. last specified 'k' zone is repeated indefinitely

typedef struct {
    uint32_t start_code, num_codes, mask, hits;
    int k;
} Zone;

static int best_next_zone_bits (uint32_t *samples, int sample_count, Zone *zones, int *num_zones, int max_num_zones);

static void entropy_encode_zones (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride, int flags)
{
    int max_num_zones = ((flags & EXTRA_K_MASK) >> EXTRA_K_SHIFT) + 1;
    uint32_t bitcounter = bs_bits_written (bs);
    uint32_t samples [sample_count];
    Zone zones [MAX_NUM_ZONES];
    int num_zones = 0, zone;

    for (int i = 0; i < sample_count; ++i)
        samples [i] = signed_to_non_negative (audio_samples [i * stride]);

    int best_zones_result = best_next_zone_bits (samples, sample_count, zones, &num_zones, max_num_zones);

    // num_zones == 0 signals silent block

    if (!num_zones) {
        putbits (0, 5, bs);
        rice_ks [0]++;
        return;
    }

    rice_ks [zones [0].k + 1]++;
    putbits (zones [0].k + 1, 5, bs);

    for (zone = 1; zone < num_zones; ++zone) {
        int delta_k = zones [zone].k - zones [zone - 1].k, j, ones;

        if (delta_k == 0) {
            for (j = zone + 1; j < num_zones; ++j)
                if (zones [zone].k != zones [j].k)
                    break;

            if (j == num_zones) {
                zone = MAX_NUM_ZONES;
                putbits (0, 2, bs);     // 0-0
                break;
            }
        }

        if (delta_k) {
            ones = signed_to_non_negative (delta_k);

            while (ones--)
                putbit_1 (bs);

            putbit_0 (bs);
        }
        else
            putbits (2, 2, bs);         // 0-1
    }

    if (zone < MAX_NUM_ZONES)
        putbits (0, 2, bs);             // 0-0

    for (zone = 0; zone < num_zones; ++zone)
        zones [zone].mask = (1UL << zones [zone].k) - 1;

    for (int i = 0; i < sample_count; ++i) {
        uint32_t avalue = samples [i];
        int ones;

        zone = 0;

        while (++zone < num_zones)
            if (avalue < zones [zone].start_code)
                break;

        avalue -= zones [--zone].start_code;
        ones = (avalue >> zones [zone].k) + zone;

        while (ones--)
            putbit_1 (bs);

        putbit_0 (bs);
        avalue &= zones [zone].mask;
        putbits (avalue, zones [zone].k, bs);
    }

    bitcounter = bs_bits_written (bs) - bitcounter;

    if (bitcounter != best_zones_result) {
        fprintf (stderr, "best_next_zone_bits() returned %d bits, but we wrote %d bits\n", best_zones_result, bitcounter);
        exit (1);
    }
}

static void entropy_encode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride, int flags)
{
    if (sample_count < 64 || !(flags & EXTRA_K_MASK))
        entropy_encode_simple (bs, audio_samples, sample_count, stride);
    else
        entropy_encode_zones (bs, audio_samples, sample_count, stride, flags);
}

// Calculate the number of bits required to encode the given 'k' zone sequence

static int zone_overhead_bits (Zone *zones, int num_zones)
{
    int zone_bits = 5, zone;

    for (zone = 1; zone < num_zones; ++zone) {
        int delta_k = zones [zone].k - zones [zone - 1].k;

        if (delta_k == 0) {
            int j;

            for (j = zone + 1; j < num_zones; ++j)
                if (zones [zone].k != zones [j].k)
                    break;

            if (j == num_zones) {
                zone = MAX_NUM_ZONES;
                zone_bits += 2;
                break;
            }
        }

        if (delta_k)
            zone_bits += signed_to_non_negative (delta_k) + 1;
        else
            zone_bits += 2;
    }

    if (zone < MAX_NUM_ZONES)
        zone_bits += 2;

    return zone_bits;
}

// Calculate the number of bits required to encode the given samples using up to "max_num_zones" zones. The
// "zones" array is returned set to the best sequence and the "num_zones" parameter returns the number of zones
// specified (the final zone is assumed repeated indefinitely). The "num_zones" value must be preset to zero
// before calling this function. Note that this function is recursive.

static int best_next_zone_bits (uint32_t *samples, int sample_count, Zone *zones, int *num_zones, int max_num_zones)
{
    int samples_left = sample_count, encoding_bits = 0, k_prev = -1, k_last = -1;
    const int curr_zone = *num_zones;
    uint32_t zone_start_code = 0;

    if (curr_zone)
        k_prev = zones [curr_zone - 1].k;

    for (int i = 0; i < curr_zone; ++i) {
        zone_start_code += 1UL << zones [i].k;
        samples_left -= zones [i].hits;
    }

    if (!samples_left || max_num_zones <= 1) {
        fprintf (stderr, "fatal error in best_next_zone_bits(), samples left = %d, max_num_zones = %d\n",
            samples_left, max_num_zones);
        exit (1);
    }

    if (curr_zone < max_num_zones - 1) {
        uint16_t histogram [32] = { 0 };

        for (int i = 0; i < sample_count; ++i)
            if (samples [i] > zone_start_code)
                histogram [32 - count_leading_zeros (samples [i] - zone_start_code)]++;
            else if (samples [i] == zone_start_code)
                histogram [0]++;

        for (int i = 1; i < 32; ++i)
            if ((histogram [i] += histogram [i - 1]) == samples_left && k_last < 0)
                k_last = i;

        if (curr_zone == 0 && histogram [0] == histogram [31]) {
            zones [0].k = -1;
            *num_zones = 0;
            return 5;
        }

        for (int i = 0; i < 32; ++i)
            if (histogram [i] * 2 >= samples_left) {
                int k_low = i, k_high = i, best_num_zones = 0, trial_bits;
                Zone trial_zones [max_num_zones];

                memcpy (trial_zones, zones, sizeof (Zone) * curr_zone);

                if (k_low && histogram [i] * 2 > samples_left)
                    k_low--;

                if (k_prev >= 0) {
                    if      (k_prev < k_low)    k_low = k_prev;
                    else if (k_prev > k_high)   k_high = k_prev;
                }

                for (int k = k_low; k <= k_high; ++k) {
                    int trial_num_zones = curr_zone;

                    trial_zones [curr_zone].k = k;
                    trial_zones [curr_zone].start_code = zone_start_code;
                    trial_zones [curr_zone].num_codes = 1UL << k;
                    trial_zones [curr_zone].hits = histogram [k];
                    trial_bits = histogram [k] * (k + trial_num_zones + 1);
                    trial_num_zones++;

                    if (encoding_bits && trial_bits + 6 >= encoding_bits)
                        continue;

                    if (histogram [k] < samples_left)
                        trial_bits += best_next_zone_bits (samples, sample_count, trial_zones, &trial_num_zones, max_num_zones);
                    else
                        trial_bits += zone_overhead_bits (trial_zones, trial_num_zones);

                    if (!encoding_bits || trial_bits < encoding_bits) {
                        memcpy (zones, trial_zones, sizeof (Zone) * trial_num_zones);
                        best_num_zones = trial_num_zones;
                        encoding_bits = trial_bits;
                    }
                }

                *num_zones = best_num_zones;
                break;
            }
    }
    else {
        int best_k = -1;

        for (int trial_k = 0; ; trial_k++) {
            int trial_bits = 0;

            for (int i = 0; i < sample_count; ++i)
                if (samples [i] >= zone_start_code &&
                    (trial_bits += (samples [i] - zone_start_code) >> trial_k) > sample_count * 32)
                        break;

            if (trial_bits > sample_count * 32)
                continue;

            zones [curr_zone].k = trial_k;
            trial_bits += samples_left * (curr_zone + trial_k + 1);
            trial_bits += zone_overhead_bits (zones, curr_zone + 1);

            if (!encoding_bits || trial_bits < encoding_bits) {
                encoding_bits = trial_bits;
                best_k = trial_k;
            }
            else if (trial_k > k_prev)
                break;
        }

        if (best_k < 0) {
            fprintf (stderr, "best_next_zone_bits() termination test didn't get a best_k value\n");
            exit (1);
        }

        zones [curr_zone].k = best_k;
        zones [curr_zone].start_code = zone_start_code;
        zones [curr_zone].num_codes = 1UL << best_k;
        zones [curr_zone].hits = samples_left;
        (*num_zones)++;
    }

    return encoding_bits;
}

// Calculate the optimum Rice k parameter (i.e. number of bits sent literally
// for each sample) for the supplied sample data. If all the samples are zero
// this function returns -1 so that silent blocks can be efficiently handled.

static int best_rice_k (const int32_t *audio_samples, int sample_count, int stride)
{
    uint32_t sum_bits = 0, min_bits;
    int rice_k = 0, i;

    // go through the samples initially calculating the sum of all the samples
    // but bump the rice_k and right shift the sum when we exceed 32 bits per
    // sample (which is definitely too high)

    for (i = 0; i < sample_count; ++i) {
        if ((sum_bits += (uint32_t) signed_to_non_negative (*audio_samples) >> rice_k) > sample_count * 32) {
            sum_bits = (sum_bits + 1) >> 1;
            rice_k++;
        }

        audio_samples += stride;
    }

    if (!sum_bits)                  // return -1 for complete silence
        return -1;

    min_bits = sum_bits + (1 + rice_k) * sample_count;

    // min_bits this is now the total number of bits used for encoding all the
    // samples with k=rice_k; next we'll increase k and estimate the resulting bit
    // count as long as it continues to improve

    while (1) {
        uint32_t bits = (sum_bits >>= 1) + ((1 + ++rice_k) * sample_count) - (sample_count / 6);

        if (bits < min_bits)
            min_bits = bits;
        else
            return rice_k - 1;
    }
}

// Scan sample array for redundant LSB's (zeros) and remove them through a
// shift. The number of bits removed is returned so that the samples can be
// correctly reconstructed on decode. This should not normally be time-
// consuming because we break out as soon as we see the LSB set (which would
// normally be very early, unless we are really going to have success).

static int redundant_bits (int32_t *audio_samples, int sample_count, int stride)
{
    int count = sample_count, redundant_bits = 0;
    int32_t *dptr = audio_samples;
    int32_t ordata = 0;

    while (count--)
        if ((ordata |= *dptr) & 1)
            break;
        else
            dptr += stride;

    if (ordata)
        while (!(ordata & 1)) {
            redundant_bits++;
            ordata >>= 1;
        }

    if (redundant_bits)
        while (sample_count--) {
            *audio_samples >>= redundant_bits;
            audio_samples += stride;
        }

    return redundant_bits;
}

// Scan a buffer of sample data and return an estimated "magnitude" value
// based on the most significant 1 bit of the absolute value of each sample.
// There is no real significance to this number because it is simply used to
// compare arrays with different decorrelation processing. This should match
// the value returned by decorrelate() and correlate(). We add 1 because
// __builtin_clz() is undefined for zero input.

static int magnitude_bits (int32_t *audio_samples, int sample_count, int stride)
{
    int32_t *dptr, *eptr = audio_samples + sample_count * stride;
    int bits = sample_count * 31;

    for (dptr = audio_samples; dptr < eptr; dptr += stride)
        bits -= count_leading_zeros (signed_to_non_negative (*dptr) + 1);

    return bits;
}

// Given an array of samples, determine the best decorrelation level (or count)
// that produces the smallest output, and leave the data decorrelated to that
// level so that it can be immediately entropy encoded. If even the first level
// of decorrelation shows degradation, then we try correlation() to see if that
// helps instead. Negative correlation would not normally be seen in regular
// audio, but might be seen in test signals, and it kind of comes free here.

static int best_decorr (int32_t *audio_samples, int sample_count, int stride)
{
    int best_magnitude = magnitude_bits (audio_samples, sample_count, stride);
    int decorr_index = 0, best_decorr_index = 0;

    // first, continue to call decorrelate() until the result gets worse, then back up 1

    while (decorr_index < MAX_DECORR) {
        int magnitude = decorrelate (audio_samples, sample_count, stride); decorr_index++;

        if (magnitude < best_magnitude) {
            best_decorr_index = decorr_index;
            best_magnitude = magnitude;
        }
        else {
            correlate (audio_samples, sample_count, stride); decorr_index--;
            break;
        }
    }

    // if that didn't work at all, maybe negative decorrelation (i.e. correlation) will

    if (decorr_index == 0) {
        int magnitude = correlate (audio_samples, sample_count, stride); decorr_index--;

        if (magnitude < best_magnitude) {
            best_decorr_index = decorr_index;
            best_magnitude = magnitude;
        }
        else {
            decorrelate (audio_samples, sample_count, stride); decorr_index++;
        }
    }

    return best_decorr_index;
}

// Scans the provided stereo samples to see if they are identical and can
// therefore be encoded as a "dual-mono" block. To avoid delays, we break out
// of the loop as soon as we se a difference (which would normally be the
// first or second sample).

static int channels_identical (int32_t *audio_samples, int sample_count)
{
    while (sample_count--)
        if (audio_samples [0] == audio_samples [1])
            audio_samples += 2;
        else
            return 0;

    return 1;
}

// Converts the supplied buffer of stereo samples from left-right encoding to
// mid-side encoding. Note that since L+R and L-R have identical LSBs, we
// don't really need to store the LSB of both, which is why it's "mid" instead
// of "sum".

static void lr_to_ms (int32_t *audio_samples, int sample_count)
{
    while (sample_count--) {
        audio_samples [1] += ((audio_samples [0] -= audio_samples [1]) >> 1);
        audio_samples += 2;
    }
}

/******************************** STATISTICS *********************************/

// Display some encoder statistics to the specified stream.

void dump_compression_stats (FILE *file)
{
    int maxnz = -1, i;
    char string [256];

    fprintf (file, "decorr: (%ld) %ld %ld %ld %ld %ld %ld %ld\n",
        (long) decorrs[0], (long) decorrs[1], (long) decorrs[2], (long) decorrs[3],
        (long) decorrs[4], (long) decorrs[5], (long) decorrs[6], (long) decorrs[7]);

    for (i = 0; i < 32; ++i) if (rice_ks [i]) maxnz = i;
    for (string [0] = i = 0; i <= maxnz; ++i) sprintf (string + strlen(string), i ? " %ld" : " (%ld)", (long) rice_ks [i]);
    fprintf (file, "rice-k:%s (max = %d)\n", string, maxnz - 1);

    for (i = 0; i < 32; ++i) if (shifts [i]) maxnz = i;
    if (maxnz) {
        for (string [0] = i = 0; i <= maxnz; ++i) sprintf (string + strlen(string), " %ld", (long) shifts [i]);
        fprintf (file, "shifts:%s (max = %d)\n", string, maxnz);
    }

    memset (decorrs, 0, sizeof (decorrs));
    memset (rice_ks, 0, sizeof (rice_ks));
    memset (shifts, 0, sizeof (shifts));
}

/****************************** DECOMPRESSION ********************************/

static void entropy_decode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride, int k_zones);
static void leftshift_bits (int32_t *audio_samples, int sample_count, int stride, int shift);
static void ms_to_lr (int32_t *audio_samples, int sample_count);
static void dm_to_lr (int32_t *audio_samples, int sample_count);

// Decompress the supplied compressed audio data into the original samples.
// Note that the number of samples and channels must be passed in, so these
// must be stored with the block somehow (although checking that both of the
// 2 LSB's of the first compressed data byte are zero can be used to determine
// if the block is mono, although this is kind of cheating). All the compressed
// data provided by compress_audio_block() must be present, or -1 will be
// returned indicating an error.

int decompress_audio_block (int32_t *audio_samples, int sample_count, int num_chans, char *inbuffer, int inbufsize)
{
    int read_chans = num_chans, res = 0, stereo_mode, k_zones, chan;
    Bitstream bs;

    // open the passed buffer as a bitstream and get the first 2 bits (stereo mode)

    bs_open_read (&bs, inbuffer, inbuffer + inbufsize);
    getbits (&stereo_mode, 2, &bs);
    stereo_mode &= 0x3;

    if (stereo_mode == DUAL_MONO)   // for dual-mono, we only read one channel of data from the bitstream
        read_chans = 1;

    k_zones = getbit (&bs);

    for (chan = 0; chan < read_chans && !bs.error; ++chan) {
        int shift = 0, decorr;

        getbits (&decorr, 3, &bs);      // the first 3 bits for each channel is the decorrelation info
        decorr = (decorr & 0x7) - 1;

        while (getbit (&bs))            // the number of 1's that follow is the final left-shift amount
            shift++;

        // as with encoding, we read the first few still correlated samples in a separate Rice operation

        if (decorr && abs (decorr) < sample_count) {
            entropy_decode (&bs, audio_samples + chan, abs (decorr), num_chans, k_zones);
            entropy_decode (&bs, audio_samples + abs (decorr) * num_chans + chan, sample_count - abs (decorr), num_chans, k_zones);
        }
        else
            entropy_decode (&bs, audio_samples + chan, sample_count, num_chans, k_zones);

        // next undo the decorrelation step, and finally the shift

        if (decorr < 0)
            decorrelate (audio_samples + chan, sample_count, num_chans);
        else while (decorr--)
            correlate (audio_samples + chan, sample_count, num_chans);

        if (shift)
            leftshift_bits (audio_samples + chan, sample_count, num_chans, shift);
    }

    // if stereo and not simply stored left-right, we might have one more step

    if (stereo_mode == MID_SIDE)
        ms_to_lr (audio_samples, sample_count);
    else if (stereo_mode == DUAL_MONO)
        dm_to_lr (audio_samples, sample_count);

    res = bs.error;         // if we went past the end of the buffer, flag an error
    bs_close_read (&bs);

    return res;
}

// Decode the supplied array of samples from the bitstream using Rice encoding.
// If stereo data is being decompressed the channels are processed sequentially
// and the stride parameter should be 2 and the pointer set to the first sample
// of the desired channel. The first 5 bits are the Rice "k" value (offset 1)
// which specifies how many bits are sent literally, with 0 signalling silence.

static void entropy_decode_simple (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride)
{
    int rice_k;

    getbits (&rice_k, 5, bs);   // Rice K is offset by 1 (so we send -1 for silence)
    rice_k = (rice_k & 0x1f) - 1;

    if (rice_k < 0)             // if silence, just clear the samples and get out
        while (sample_count--) {
            *audio_samples = 0;
            audio_samples += stride;
        }
    else if (rice_k > 0) {      // normal case for k > 0
        uint32_t mask = (1UL << rice_k) - 1, rice_k_bits;

        while (sample_count--) {
            uint32_t avalue = 0;

            while (getbit (bs))
                avalue++;

            avalue <<= rice_k;
            getbits (&rice_k_bits, rice_k, bs);
            avalue |= (rice_k_bits & mask);
            *audio_samples = non_negative_to_signed (avalue);
            audio_samples += stride;
        }
    }
    else                        // optimized case for k == 0
        while (sample_count--) {
            uint32_t avalue = 0;

            while (getbit (bs))
                avalue++;

            *audio_samples = non_negative_to_signed (avalue);
            audio_samples += stride;
        }
}

static void entropy_decode_zones (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride)
{
    Zone zones [MAX_NUM_ZONES];

    getbits (&zones [0].k, 5, bs);
    zones [0].k = (zones [0].k & 0x1f) - 1;

    if (zones [0].k < 0) {              // if silence, just clear the samples and get out
        while (sample_count--) {
            *audio_samples = 0;
            audio_samples += stride;
        }

        return;
    }

    for (int z = 1; z < MAX_NUM_ZONES; ++z) {
        int ones = 0, prev_k = zones [z - 1].k;

        while (getbit (bs))
            ones++;

        if (!ones) {
            if (!getbit (bs)) {
                for (int j = z; j < MAX_NUM_ZONES; ++j)     // "00" = done
                    zones [j].k = prev_k;

                break;
            }
        }

        zones [z].k = prev_k + non_negative_to_signed (ones);

        if (zones [z].k < 0) {
            fprintf (stderr, "zone decoding error at zone %d\n", z);
            exit (1);
        }
    }

    uint32_t start_code = 0;

    for (int z = 0; z < MAX_NUM_ZONES; ++z) {
        zones [z].num_codes = 1UL << zones [z].k;
        zones [z].start_code = start_code;
        zones [z].mask = zones [z].num_codes - 1;
        start_code += zones [z].num_codes;
    }

    for (int i = 0; i < sample_count; ++i) {
        uint32_t avalue = 0, rice_k_bits;

        while (getbit (bs))
            avalue++;

        if (avalue < MAX_NUM_ZONES) {
            getbits (&rice_k_bits, zones [avalue].k, bs);
            avalue = zones [avalue].start_code + (rice_k_bits & zones [avalue].mask);
        }
        else {
            avalue = ((avalue - MAX_NUM_ZONES + 1) << zones [MAX_NUM_ZONES - 1].k) + zones [MAX_NUM_ZONES - 1].start_code;
            getbits (&rice_k_bits, zones [MAX_NUM_ZONES - 1].k, bs);
            avalue += rice_k_bits & zones [MAX_NUM_ZONES - 1].mask;
        }

        *audio_samples = non_negative_to_signed (avalue);
        audio_samples += stride;
    }
}

static void entropy_decode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride, int k_zones)
{
    if (sample_count < 64 || !k_zones)
        entropy_decode_simple (bs, audio_samples, sample_count, stride);
    else
        entropy_decode_zones (bs, audio_samples, sample_count, stride);
}

// Leftshift specified number of zeros into the audio samples. This is to undo
// the redundant zeros detected and rightshifted by redundant_bits().

static void leftshift_bits (int32_t *audio_samples, int sample_count, int stride, int shift)
{
    while (sample_count--) {
        * (uint32_t *) audio_samples <<= shift;
        audio_samples += stride;
    }
}

// Duplicate the single left stereo channel into both channels for decoding
// "dual-mono" mode.

static void dm_to_lr (int32_t *audio_samples, int sample_count)
{
    while (sample_count--) {
        audio_samples [1] = audio_samples [0];
        audio_samples += 2;
    }
}

// Converts the supplied buffer of stereo samples from mid-side encoding back
// to left-right encoding.

static void ms_to_lr (int32_t *audio_samples, int sample_count)
{
    while (sample_count--) {
        audio_samples [0] += (audio_samples [1] -= (audio_samples [0] >> 1));
        audio_samples += 2;
    }
}

/********************* COMMON CORRELATION / DECORRELATION ********************/

// These two complementary functions are at the heart of the compressor and
// are probably the simplest implementation of decorrelation that exists.
// Essentially, for decorrelation, each sample is replaced by the difference
// between that sample and the previous one. In the reverse operation,
// called correlation, each sample is replaced by the sum of itself and all
// the previous samples. The first sample is never changed. Each function
// exactly undoes the operation of the other, which is why they can be used
// for lossless compression. So that they can be used during the encoder side
// analysis, they also calculate and return a "magnitude" value identical to
// the value returned by magnitude_bits() that can be used to determine if
// the operation reduced the average magnitude of the samples.

static int decorrelate (int32_t *audio_samples, int sample_count, int stride)
{
    int bits = sample_count * 31;
    uint32_t temp = 0;

    while (sample_count--) {
        temp += *audio_samples -= temp;
        bits -= count_leading_zeros (signed_to_non_negative (*audio_samples) + 1);
        audio_samples += stride;
    }

    return bits;
}

static int correlate (int32_t *audio_samples, int sample_count, int stride)
{
    int bits = sample_count * 31;
    uint32_t value = 0;

    while (sample_count--) {
        value = *audio_samples += value;
        bits -= count_leading_zeros (signed_to_non_negative (value) + 1);
        audio_samples += stride;
    }

    return bits;
}
