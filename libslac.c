////////////////////////////////////////////////////////////////////////////
//                           ****** SLAC ******                           //
//                    Simple Lossless Audio Compressor                    //
//                    Copyright (c) 2019 David Bryant.                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// libslac.c

// This module provides functions to compress and decompress PCM integer
// audio. The audio is presented in 32-bit integers, but can be arbitrary
// bitdepths (redundant bits at either side of the words are handled). Mono
// and stereo streams are supported and up to significant 24-bits per sample
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

// Unfortunately a variable with about 40 bits bits of magnitude is required
// to calculate the optimum Rice code size. This is typedef'd here to be either
// a float or a 64-bit integer (int64_t), so use whatever is best for your
// platform.

typedef int64_t slac_mag_t;

// These complementary macros convert from signed integers into the positive
// values that slac uses for analysis and entropy encoding (with the sign moved
// to the LSB), and back.

#define signed_to_positive(x)   (((x)<<1)^((int32_t)(x)>>31))
#define positive_to_signed(x)   ((((x)&1)?~(x):(x))>>1)

#define MAX_DECORR 6

// Complementary decorrelate / correlate functions used by both encoding and decoding

static int decorrelate (int32_t *audio_samples, int sample_count, int stride);
static int correlate (int32_t *audio_samples, int sample_count, int stride);

static int decorrs [8], bases [32], shifts [32];    // used for statistics only

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
// (obviously) and can be either MID_SIDE or LEFT_RIGHT (see libslac.h).

int compress_audio_block (int32_t *audio_samples, int sample_count, int num_chans, int stereo_mode, char *outbuffer, int outbufsize)
{
    int sent_chans = num_chans, chan;
    Bitstream bs;

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

    bs_open_write (&bs, outbuffer, outbuffer + outbufsize);
    putbits (stereo_mode, 2, &bs);

    for (chan = 0; chan < sent_chans; ++chan) {
        int shift, decorr;

        shift = redundant_bits (audio_samples + chan, sample_count, num_chans);
        shifts [shift]++;
        decorr = best_decorr (audio_samples + chan, sample_count, num_chans);
        decorrs [decorr + 1]++;
        putbits (decorr + 1, 3, &bs);

        while (shift--)
            putbit_1 (&bs);

        putbit_0 (&bs);

        if (decorr && abs (decorr) < sample_count) {
            entropy_encode (&bs, audio_samples + chan, abs (decorr), num_chans);
            entropy_encode (&bs, audio_samples + abs (decorr) * num_chans + chan, sample_count - abs (decorr), num_chans);
        }
        else
            entropy_encode (&bs, audio_samples + chan, sample_count, num_chans);
    }

    return bs_close_write (&bs);
}

static int best_base (int32_t *audio_samples, int sample_count, int stride);

// Encode the supplied array of samples into a bitstream using Rice encoding.
// If stereo data is being compressed the channels must be processed in two
// separate passes and the stride parameter should be 2 and the pointer set to
// the first sample of the desired channel.

static void entropy_encode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride)
{
    int base = best_base (audio_samples, sample_count, stride);

    if (sample_count > MAX_DECORR)
        bases [base]++;

    putbits (base, 5, bs);
    uint32_t mask = (1 << base) - 1;

    while (sample_count--) {
        uint32_t avalue = signed_to_positive (*audio_samples);
        int modulo = avalue >> base;

        while (modulo--)
            putbit_1 (bs);

        putbit_0 (bs);

        if (base) {
            avalue &= mask;
            putbits (avalue, base, bs);
        }

        audio_samples += stride;
    }
}

// Calculate the optimum base (i.e. number of bits sent literally) for the
// supplied sample data. This requires a variable type with greater than
// 32 bits of magnitude (not resolution), so this can be either a 32-bit
// float or a 64-bit integer, whichever is better for the platform.

static int best_base (int32_t *audio_samples, int sample_count, int stride)
{
    slac_mag_t avalue_sum = 0, min_bits;
    int count = sample_count, base = 0;
    int32_t *dptr = audio_samples;

    while (count--) {
        avalue_sum += signed_to_positive (*dptr);
        dptr += stride;
    }

    min_bits = avalue_sum + sample_count;

    while (1) {
        slac_mag_t bits = (avalue_sum /= 2) + (++base * sample_count) + (sample_count / 2);

        if (bits < min_bits)
            min_bits = bits;
        else
            return base - 1;
    }
}

// Scan sample array for redundant LSB's (zeros) and remove them through a
// shift. The number of bits removed is returned so that the samples can be
// correctly reconstructed on decode.

static int redundant_bits (int32_t *audio_samples, int sample_count, int stride)
{
    int redundant_bits = 0;

    int32_t *dptr = audio_samples;
    int count = sample_count;
    int32_t ordata = 0;

    while (count--)
        if ((ordata |= *dptr) & 1)
            break;
        else
            dptr += stride;

    redundant_bits = ordata ? __builtin_ctz (ordata) : 0;

    if (redundant_bits)
        while (sample_count--) {
            *audio_samples >>= redundant_bits;
            audio_samples += stride;
        }

    return redundant_bits;
}

// Scan a buffer of sample data and return an estimated "magnitude" value
// based on the first significant bit of the absolute value of each sample.
// There is no real significance to this number because it is simply used to
// compare arrays with different decorrelation processing. This should match
// the value returned by decorrelate() and correlate().

static int magnitude_bits (int32_t *audio_samples, int sample_count, int stride)
{
    int32_t *dptr, *eptr = audio_samples + sample_count * stride;
    int bits = sample_count * 31;

    for (dptr = audio_samples; dptr < eptr; dptr += stride)
        bits -= __builtin_clz (signed_to_positive (*dptr) + 1);

    return bits;
}

// Given an array of samples, determine the best decorrelation level (or count)
// the produces the smallest output, and leave the data decorrelated to that
// level so that it can be immediately entropy encoded. If even the first level
// of decorrelation shows degradation then we try correlation() to see if that
// helps. Negative correlation would not normally be seen in regular audio, but
// might be seen in test signals, and it kind of comes free here.

static int best_decorr (int32_t *audio_samples, int sample_count, int stride)
{
    int best_magnitude = magnitude_bits (audio_samples, sample_count, stride);
    int decorr_index = 0, best_decorr_index = 0;

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

    fprintf (file, "decorr: %d %d %d %d %d %d %d %d\n",
        decorrs[0],decorrs[1],decorrs[2],decorrs[3],decorrs[4],decorrs[5],decorrs[6],decorrs[7]);

    for (i = 0; i < 32; ++i) if (bases [i]) maxnz = i;
    for (string [0] = i = 0; i <= maxnz; ++i) sprintf (string + strlen(string), " %d", bases [i]);
    fprintf (file, "bases:%s (max = %d)\n", string, maxnz - 1);

    for (i = 0; i < 32; ++i) if (shifts [i]) maxnz = i;
    if (maxnz) {
        for (string [0] = i = 0; i <= maxnz; ++i) sprintf (string + strlen(string), " %d", shifts [i]);
        fprintf (file, "shifts:%s (max = %d)\n", string, maxnz);
    }

    memset (decorrs, 0, sizeof (decorrs));
    memset (bases, 0, sizeof (bases));
    memset (shifts, 0, sizeof (shifts));
}

/****************************** DECOMPRESSION ********************************/

static void entropy_decode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride);
static void correlate_factor (int32_t *audio_samples, int sample_count, int stride, int factor);
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

    bs_open_read (&bs, inbuffer, inbuffer + inbufsize);
    getbits (&stereo_mode, 2, &bs);
    stereo_mode &= 0x3;

    if (stereo_mode == DUAL_MONO)
        read_chans = 1;

    for (chan = 0; chan < read_chans && !bs.error; ++chan) {
        int shift = 0, decorr;

        getbits (&decorr, 3, &bs);
        decorr = (decorr & 0x7) - 1;

        while (getbit (&bs))
            shift++;

        if (decorr && abs (decorr) < sample_count) {
            entropy_decode (&bs, audio_samples + chan, abs (decorr), num_chans);
            entropy_decode (&bs, audio_samples + abs (decorr) * num_chans + chan, sample_count - abs (decorr), num_chans);
        }
        else
            entropy_decode (&bs, audio_samples + chan, sample_count, num_chans);

        if (decorr < 0)
            decorrelate (audio_samples + chan, sample_count, num_chans);
        else if (decorr == 1)
            correlate (audio_samples + chan, sample_count, num_chans);
        else if (decorr > 1)
            correlate_factor (audio_samples + chan, sample_count, num_chans, decorr);

        if (shift)
            leftshift_bits (audio_samples + chan, sample_count, num_chans, shift);
    }

    if (stereo_mode == MID_SIDE)
        ms_to_lr (audio_samples, sample_count);
    else if (stereo_mode == DUAL_MONO)
        dm_to_lr (audio_samples, sample_count);

    res = bs.error;
    bs_close_read (&bs);

    return res;
}

// Decode the supplied array of samples from the bitstream using Rice encoding.
// If stereo data is being decompressed the channels are processed sequentially
// and the stride parameter should be 2 and the pointer set to the first sample
// of the desired channel. The first 5 bits are the Rice "base" value which
// specifies how many bits are sent literally.

static void entropy_decode (Bitstream *bs, int32_t *audio_samples, int sample_count, int stride)
{
    int base;

    getbits (&base, 5, bs);
    base &= 0x1f;

    uint32_t mask = (1 << base) - 1;

    while (sample_count--) {
        uint32_t avalue = 0, base_bits;

        while (getbit (bs))
            avalue++;

        if (base) {
            avalue <<= base;
            getbits (&base_bits, base, bs);
            avalue |= (base_bits & mask);
        }

        *audio_samples = positive_to_signed (avalue);
        audio_samples += stride;
    }
}

// Leftshift specified number of zeros into the audio samples. This is to undo
// the redundant zeros detected and rightshifted by redundant_bits().

static void leftshift_bits (int32_t *audio_samples, int sample_count, int stride, int shift)
{
    while (sample_count--) {
        *audio_samples <<= shift;
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

// Converts the supplied buffer of stereo samples from mid-side encoding to
// left-right encoding. Note that since L+R and L-R have identical LSBs, we
// don't really need to store the LSB of both, which is why it's "mid" instead
// of "sum".

static void ms_to_lr (int32_t *audio_samples, int sample_count)
{
    while (sample_count--) {
        audio_samples [0] += (audio_samples [1] -= (audio_samples [0] >> 1));
        audio_samples += 2;
    }
}

// This function is identical to the regular correlate() function except for
// two things. First, it can concatenate several correlate operations into one
// without making multiple passes through the audio, so it is faster for
// decoding when we know how many operations are required. Also, since it's
// for decoding, we don't calculate and return the magnitude bits.

static void correlate_factor (int32_t *audio_samples, int sample_count, int stride, int factor)
{
    int32_t *fptr = audio_samples + (sample_count + factor) * stride;
    int32_t *eptr = audio_samples + sample_count * stride;
    int32_t *dptr, *aptr, i;

    for (dptr = audio_samples + stride; dptr < fptr; dptr += stride)
        for (aptr = dptr, i = factor; i--; aptr -= stride)
            if (aptr > audio_samples && aptr < eptr)
                *aptr += aptr [-stride];
}

/********************* COMMON CORRELATION / DECORRELATION ********************/

// These two complementary functions are at the heart of the compressor and
// are probably the simplest implementation of decorrelation that exists.
// Essentially, for decorrelation, each sample is replaced by the difference
// between that sample and the previous one. In the reverse operation,
// called correlation, each sample is replaced by the sum of itself and all
// the previous samples. The first sample is never changed. Each function
// exactly undoes the operation of the other, which is why they can be used
// for lossless compression. So that they can be used on the encoder side,
// they also calculate and return a "magnitude" value identical to the
// value returned by magnitude_bits().

static int decorrelate (int32_t *audio_samples, int sample_count, int stride)
{
    int bits = sample_count * 31;
    int32_t temp = 0;

    while (sample_count--) {
        temp += *audio_samples -= temp;
        bits -= __builtin_clz (signed_to_positive (*audio_samples) + 1);
        audio_samples += stride;
    }

    return bits;
}

static int correlate (int32_t *audio_samples, int sample_count, int stride)
{
    int bits = sample_count * 31;
    int32_t value = 0;

    while (sample_count--) {
        value = *audio_samples += value;
        bits -= __builtin_clz (signed_to_positive (value) + 1);
        audio_samples += stride;
    }

    return bits;
}
