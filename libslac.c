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

static void entropy_encode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride);
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

int compress_audio_block (int32_t *audio_samples, int sample_count, int num_chans, int stereo_mode, char *outbuffer, int outbufsize)
{
    int sent_chans = num_chans, chan;
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
            entropy_encode (&bs, audio_samples + chan, abs (decorr), num_chans);
            entropy_encode (&bs, audio_samples + abs (decorr) * num_chans + chan, sample_count - abs (decorr), num_chans);
        }
        else
            entropy_encode (&bs, audio_samples + chan, sample_count, num_chans);
    }

    return bs_close_write (&bs);    // close the bitstream and return the number of bytes written
}

static int best_rice_k (int32_t *audio_samples, int sample_count, int stride);

// Encode the supplied array of samples into a bitstream using Rice encoding.
// If stereo data is being compressed the channels must be processed in two
// separate passes and the stride parameter should be 2 and the pointer set to
// the first sample of the desired channel.

static void entropy_encode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride)
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

// Calculate the optimum Rice k parameter (i.e. number of bits sent literally
// for each sample) for the supplied sample data. We calculate the sum of all
// the (converted to non-negative) sample values which requires more than just
// 32 bits of magnitude. To avoid requiring a 64-bit integer type (which some
// platforms don't have) or a float (which can be slow on some platforms) this
// is done with a 32-bit unsigned integer and another integer for overflows.
// Just the final calculation is done in floats. If there are only zero samples
// this function returns -1 so that silent blocks can be efficiently handled.

static int best_rice_k (int32_t *audio_samples, int sample_count, int stride)
{
    int count = sample_count, upper32 = 0, rice_k = 0;
    float float_sum = 0, min_bits;
    int32_t *dptr = audio_samples;
    uint32_t lower32 = 0;

    while (count--) {
        uint32_t temp = lower32;

        if ((lower32 += signed_to_non_negative (*dptr)) < temp)
            upper32++;

        dptr += stride;
    }

    if (!upper32 && !lower32)        // return -1 for complete silence
        return -1;

    float_sum = upper32 * 4294967296.0 + lower32;
    min_bits = float_sum + sample_count;

    // min_bits this is now the exact total number of bits used for encoding the
    // samples with k=0; next we'll increase k and estimate the resulting bit
    // count as long as it continues to improve

    while (1) {
        float bits = (float_sum /= 2) + (++rice_k * sample_count) + sample_count - (sample_count / 6);

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

static void entropy_decode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride);
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
    int read_chans = num_chans, res = 0, stereo_mode, chan;
    Bitstream bs;

    // open the passed buffer as a bitstream and get the first 2 bits (stereo mode)

    bs_open_read (&bs, inbuffer, inbuffer + inbufsize);
    getbits (&stereo_mode, 2, &bs);
    stereo_mode &= 0x3;

    if (stereo_mode == DUAL_MONO)   // for dual-mono, we only read one channel of data from the bitstream
        read_chans = 1;

    for (chan = 0; chan < read_chans && !bs.error; ++chan) {
        int shift = 0, decorr;

        getbits (&decorr, 3, &bs);      // the first 3 bits for each channel is the decorrelation info
        decorr = (decorr & 0x7) - 1;

        while (getbit (&bs))            // the number of 1's that follow is the final left-shift amount
            shift++;

        // as with encoding, we read the first few still correlated samples in a separate Rice operation

        if (decorr && abs (decorr) < sample_count) {
            entropy_decode (&bs, audio_samples + chan, abs (decorr), num_chans);
            entropy_decode (&bs, audio_samples + abs (decorr) * num_chans + chan, sample_count - abs (decorr), num_chans);
        }
        else
            entropy_decode (&bs, audio_samples + chan, sample_count, num_chans);

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

static void entropy_decode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride)
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
