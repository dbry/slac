////////////////////////////////////////////////////////////////////////////
//                           ****** SLAC ******                           //
//                    Simple Lossless Audio Compressor                    //
//                 Copyright (c) 2019 - 2022 David Bryant                 //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// main.c

// This module provide a command-line frontend to libslac allowing conversion
// of standard Microsoft WAV audio files to and from SLAC files. Note that
// a SLAC file format does not really exist, but this program implements a
// simple 8-byte header to allow testing and demonstration of the libslac
// library. Note that this header limits SLAC file blocks to 4095 audio
// samples and/or 4095 compressed words, but this is not an actual
// limitation of libslac.

// !! THIS SHOULD NOT BE USED FOR ARCHIVAL PURPOSES !!

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>

#include "libslac.h"

static const char *sign_on = "\n"
" SLAC  Simple Lossless Audio Compressor Demo  Version 0.3\n"
" Copyright (c) 2022 David Bryant. All Rights Reserved.\n\n";

static const char *usage =
" Usage:     SLAC [-options] infile.wav outfile.slac\n"
"            SLAC -d [-options] infile.slac outfile.wav\n\n"
" Pipes:     specify '-' for filename, but not WAV output\n\n"
" Options:  -d     = decode operation (default = encode))\n"
"           -bn    = override block samples (default = 1024)\n"
"           -h     = display this help message\n"
"           -j0    = encode stereo files as L/R\n"
"           -j1    = encode stereo files as M/S (default)\n"
"           -j2    = encode stereo files using best (slow)\n"
"           -q     = quiet mode (display errors only)\n"
"           -r     = raw output (no WAV header written)\n"
"           -v     = verbose (display lots of info)\n"
"           -y     = overwrite outfile if it exists\n\n"
" Web:       Visit www.github.com/dbry/slac for latest version and info\n\n";

static const uint32_t sample_rates [] = { 6000, 8000, 9600, 11025, 12000, 16000, 22050,
    24000, 32000, 44100, 48000, 64000, 88200, 96000, 192000 };

static int encode_wav_to_slac (char *infilename, char *outfilename);
static int decode_slac_to_wav (char *infilename, char *outfilename);

static void little_endian_to_native (void *data, char *format);
static void native_to_little_endian (void *data, char *format);

static int block_samples = 1024, joint_stereo = 1;
static int verbose_mode, quiet_mode, raw_decode;

int main (argc, argv) int argc; char **argv;
{
    int decode_mode = 0, asked_help = 0, overwrite = 0;
    char *infilename = NULL, *outfilename = NULL;
    FILE *outfile;

    // loop through command-line arguments

    while (--argc) {
#ifdef _WIN32
        if ((**++argv == '-' || **argv == '/') && (*argv)[1])
#else
        if ((**++argv == '-') && (*argv)[1])
#endif
            while (*++*argv)
                switch (**argv) {

                    case 'B': case 'b':
                        block_samples = strtol (++*argv, argv, 10);

                        if (block_samples < 0 || block_samples > 4095) {
                            fprintf (stderr, "\nblock size must be 1 to 4095!\n");
                            return -1;
                        }

                        --*argv;
                        break;

                    case 'J': case 'j':
                        joint_stereo = strtol (++*argv, argv, 10);

                        if (joint_stereo < 0 || joint_stereo > 2) {
                            fprintf (stderr, "\njoint stereo must be 0 to 2!\n");
                            return -1;
                        }

                        --*argv;
                        break;

                    case 'H': case 'h':
                        asked_help = 1;
                        break;

                    case 'D': case 'd':
                        decode_mode = 1;
                        break;

                    case 'V': case 'v':
                        verbose_mode = 1;
                        break;

                    case 'Q': case 'q':
                        quiet_mode = 1;
                        break;

                    case 'R': case 'r':
                        raw_decode = 1;
                        break;

                    case 'Y': case 'y':
                        overwrite = 1;
                        break;

                    default:
                        fprintf (stderr, "\nillegal option: %c !\n", **argv);
                        return -1;
                }
        else if (!infilename) {
            infilename = malloc (strlen (*argv) + 10);
            strcpy (infilename, *argv);
        }
        else if (!outfilename) {
            outfilename = malloc (strlen (*argv) + 10);
            strcpy (outfilename, *argv);
        }
        else {
            fprintf (stderr, "\nextra unknown argument: %s !\n", *argv);
            return -1;
        }
    }

    if (!quiet_mode)
        fprintf (stderr, "%s", sign_on);

    if (!outfilename || asked_help) {
        printf ("%s", usage);
        return 0;
    }

    if (decode_mode && !raw_decode && *outfilename == '-') {
        fprintf (stderr, "can't write WAV files to stdout, use file or raw mode\n");
        return -1;
    }

    if (*infilename != '-' && !strcmp (infilename, outfilename)) {
        fprintf (stderr, "can't overwrite input file (specify different/new output file name)\n");
        return -1;
    }

    if (!overwrite && (outfile = fopen (outfilename, "r"))) {
        fclose (outfile);
        fprintf (stderr, "output file \"%s\" exists (use -y to overwrite)\n", outfilename);
        return -1;
    }

    return decode_mode ?
        decode_slac_to_wav (infilename, outfilename) :
        encode_wav_to_slac (infilename, outfilename);
}

/**************************** ENCODE WAV TO SLAC ******************************/

typedef struct {
    char ckID [4];
    uint32_t ckSize;
    char formType [4];
} RiffChunkHeader;

typedef struct {
    char ckID [4];
    uint32_t ckSize;
} ChunkHeader;

#define ChunkHeaderFormat "4L"

typedef struct {
    uint16_t FormatTag, NumChannels;
    uint32_t SampleRate, BytesPerSecond;
    uint16_t BlockAlign, BitsPerSample;
    uint16_t cbSize;
    union {
        uint16_t ValidBitsPerSample;
        uint16_t SamplesPerBlock;
        uint16_t Reserved;
    } Samples;
    int32_t ChannelMask;
    uint16_t SubFormat;
    char GUID [14];
} WaveHeader;

#define WaveHeaderFormat "SSLLSSSSLS"

#define WAVE_FORMAT_PCM         0x1
#define WAVE_FORMAT_EXTENSIBLE  0xfffe

static int encode_slac (FILE *infile, FILE *outfile, uint32_t total_samples, int num_chans, int sample_rate_index, int bytes_per_sample);

static int encode_wav_to_slac (char *infilename, char *outfilename)
{
    int format = 0, res = 0, bits_per_sample = 0, num_channels = 0;
    uint32_t num_samples = 0, sample_rate = 0;
    FILE *infile, *outfile;
    RiffChunkHeader riff_chunk_header;
    ChunkHeader chunk_header;
    WaveHeader WaveHeader;

    if (*infilename == '-') {
        infile = stdin;
#ifdef _WIN32
        _setmode (fileno (stdin), O_BINARY);
#endif
    }
    else if (!(infile = fopen (infilename, "rb"))) {
        fprintf (stderr, "can't open file \"%s\" for reading!\n", infilename);
        return -1;
    }

    // read initial RIFF form header

    if (!fread (&riff_chunk_header, sizeof (RiffChunkHeader), 1, infile) ||
        strncmp (riff_chunk_header.ckID, "RIFF", 4) ||
        strncmp (riff_chunk_header.formType, "WAVE", 4)) {
            fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
            return -1;
    }

    // loop through all elements of the RIFF wav header (until the data chuck)

    while (1) {

        if (!fread (&chunk_header, sizeof (ChunkHeader), 1, infile)) {
            fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
            return -1;
        }

        little_endian_to_native (&chunk_header, ChunkHeaderFormat);

        // if it's the format chunk, we want to get some info out of there and
        // make sure it's a .wav file we can handle

        if (!strncmp (chunk_header.ckID, "fmt ", 4)) {
            int supported = 1;

            if (chunk_header.ckSize < 16 || chunk_header.ckSize > sizeof (WaveHeader) ||
                !fread (&WaveHeader, chunk_header.ckSize, 1, infile)) {
                    fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                    return -1;
            }

            little_endian_to_native (&WaveHeader, WaveHeaderFormat);

            format = (WaveHeader.FormatTag == WAVE_FORMAT_EXTENSIBLE && chunk_header.ckSize == 40) ?
                WaveHeader.SubFormat : WaveHeader.FormatTag;

            bits_per_sample = (chunk_header.ckSize == 40 && WaveHeader.Samples.ValidBitsPerSample) ?
                WaveHeader.Samples.ValidBitsPerSample : WaveHeader.BitsPerSample;

            num_channels = WaveHeader.NumChannels;;

            if (format == WAVE_FORMAT_PCM) {
                int i;

                for (i = 0; i < 15; ++i)
                    if (WaveHeader.SampleRate == sample_rates [i]) {
                        sample_rate = i;
                        break;
                    }

                if (i == 15)
                    supported = 0;

                if (bits_per_sample != 8 && bits_per_sample != 16 && bits_per_sample != 24)
                    supported = 0;

                if (!WaveHeader.NumChannels || WaveHeader.NumChannels > 2 ||
                    WaveHeader.BlockAlign / WaveHeader.NumChannels < (bits_per_sample + 7) / 8 ||
                    WaveHeader.BlockAlign / WaveHeader.NumChannels > 3 ||
                    WaveHeader.BlockAlign % WaveHeader.NumChannels)
                        supported = 0;
            }
            else
                supported = 0;

            if (!supported) {
                fprintf (stderr, "\"%s\" is an unsupported .WAV format!\n", infilename);
                return -1;
            }

            if (verbose_mode) {
                fprintf (stderr, "format tag size = %lu\n", (unsigned long) chunk_header.ckSize);
                fprintf (stderr, "FormatTag = 0x%x, NumChannels = %d, BitsPerSample = %d\n",
                    WaveHeader.FormatTag, WaveHeader.NumChannels, WaveHeader.BitsPerSample);
                fprintf (stderr, "BlockAlign = %d, SampleRate = %lu, BytesPerSecond = %lu\n",
                    WaveHeader.BlockAlign, (unsigned long) WaveHeader.SampleRate,
                    (unsigned long) WaveHeader.BytesPerSecond);

                if (chunk_header.ckSize > 16)
                        fprintf (stderr, "cbSize = %d, ValidBitsPerSample = %d\n", WaveHeader.cbSize,
                            WaveHeader.Samples.ValidBitsPerSample);

                if (chunk_header.ckSize > 20)
                    fprintf (stderr, "ChannelMask = %lx, SubFormat = %d\n",
                        (unsigned long) WaveHeader.ChannelMask, WaveHeader.SubFormat);
            }
        }
        else if (!strncmp (chunk_header.ckID, "data", 4)) {

            // on the data chunk, get size and exit parsing loop

            if (!WaveHeader.NumChannels) {      // make sure we saw a "fmt" chunk...
                fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                return -1;
            }

            if (!chunk_header.ckSize) {
                fprintf (stderr, "this .WAV file has no audio samples, probably is corrupt!\n");
                return -1;
            }

            if (format == WAVE_FORMAT_PCM) {
                if (chunk_header.ckSize % WaveHeader.BlockAlign) {
                    fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                    return -1;
                }

                num_samples = chunk_header.ckSize / WaveHeader.BlockAlign;
            }

            if (!num_samples) {
                fprintf (stderr, "this .WAV file has no audio samples, probably is corrupt!\n");
                return -1;
            }

            if (verbose_mode)
                fprintf (stderr, "num samples = %lu\n", (unsigned long) num_samples);

            break;
        }
        else {          // just ignore unknown chunks
            int bytes_to_eat = (chunk_header.ckSize + 1) & ~1L;
            char dummy;

            if (verbose_mode)
                fprintf (stderr, "extra unknown chunk \"%c%c%c%c\" of %lu bytes\n",
                    chunk_header.ckID [0], chunk_header.ckID [1], chunk_header.ckID [2],
                    chunk_header.ckID [3], (unsigned long) chunk_header.ckSize);

            while (bytes_to_eat--)
                if (!fread (&dummy, 1, 1, infile)) {
                    fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                    return -1;
                }
        }
    }

    if (*outfilename == '-') {
        outfile = stdout;
#ifdef _WIN32
        _setmode (fileno (stdout), O_BINARY);
#endif
    }
    else if (!(outfile = fopen (outfilename, "wb"))) {
        fprintf (stderr, "can't open file \"%s\" for writing!\n", outfilename);
        return -1;
    }

    res = encode_slac (infile, outfile, num_samples, num_channels, (int) sample_rate, bits_per_sample / 8);

    fclose (outfile);
    fclose (infile);
    return res;
}

static int encode_slac (FILE *infile, FILE *outfile, uint32_t total_samples, int num_chans, int sample_rate, int bytes_per_sample)
{
    unsigned char raw_audio_block [block_samples * num_chans * bytes_per_sample];
    char encoded_block [256 + block_samples * 4 * num_chans];
    uint32_t samples_left = total_samples, total_bytes = 0;
    int32_t audio_block [block_samples * num_chans];
    int block_count = 0, stereo [4] = { 0 };

    while (samples_left) {
        int samples_to_read = block_samples, samples_read, cnt;
        unsigned char *sptr = raw_audio_block;
        int32_t *dptr = audio_block;

        if (samples_to_read > samples_left)
            samples_to_read = samples_left;

        samples_read = fread (raw_audio_block, bytes_per_sample * num_chans, samples_to_read, infile);
        cnt = samples_read * num_chans;

        if (!samples_read) {
            fprintf (stderr, "input file exhausted early, may be corrupt!\n");
            break;
        }

        switch (bytes_per_sample) {
            case 1:
                while (cnt--)
                    *dptr++ = *sptr++ - 128;

                break;

            case 2:
                while (cnt--) {
                    *dptr++ = sptr [0] | ((int32_t)(signed char) sptr [1] << 8);
                    sptr += 2;
                }

                break;

            case 3:
                while (cnt--) {
                    *dptr++ = sptr [0] | ((int32_t) sptr [1] << 8) | ((int32_t)(signed char) sptr [2] << 16);
                    sptr += 3;
                }

                break;
        }

        int block_bytes;

        if (num_chans == 2 && joint_stereo == 2) {
            char mid_side_encoded_block [256 + block_samples * 4 * num_chans];
            int32_t mid_side_audio_block [block_samples * num_chans];

            memcpy (mid_side_audio_block, audio_block, block_samples * num_chans * sizeof (int32_t));

            block_bytes = compress_audio_block (audio_block, samples_read, num_chans, LEFT_RIGHT,
                encoded_block, sizeof (encoded_block));

            if (block_bytes <= 0 || (block_bytes & 1) || block_bytes > 8190) {
                fprintf (stderr, "compress_audio_block() returned error or overflow, block_bytes = %d\n", block_bytes);
                return -1;
            }

            if ((encoded_block [0] & 0x3) == LEFT_RIGHT) {
                int mid_side_block_bytes = compress_audio_block (mid_side_audio_block, samples_read, num_chans, MID_SIDE,
                    mid_side_encoded_block, sizeof (encoded_block));

                if (mid_side_block_bytes < block_bytes) {
                    memcpy (encoded_block, mid_side_encoded_block, mid_side_block_bytes);
                    block_bytes = mid_side_block_bytes;
                }
            }
        }
        else
            block_bytes = compress_audio_block (audio_block, samples_read, num_chans, joint_stereo ? MID_SIDE : LEFT_RIGHT,
                encoded_block, sizeof (encoded_block));

        if (block_bytes <= 0 || (block_bytes & 1) || block_bytes > 8190) {
            fprintf (stderr, "compress_audio_block() returned error or overflow, block_bytes = %d\n", block_bytes);
            return -1;
        }

        stereo [encoded_block [0] & 0x3]++;
        fwrite ("slac", 1, 4, outfile);
        fputc (block_bytes >> 1, outfile);
        fputc ((block_bytes >> 9) | (bytes_per_sample << 4), outfile);
        fputc (samples_read, outfile);
        fputc ((samples_read >> 8) | (sample_rate << 4), outfile);
        fwrite (encoded_block, 1, block_bytes, outfile);
        total_bytes += block_bytes + 8;
        samples_left -= samples_read;
        block_count++;
    }

    if (!quiet_mode) {
        uint32_t source_bytes = (total_samples - samples_left) * num_chans * bytes_per_sample;

        fprintf (stderr, "compressed %d blocks, of %d %d-bit %s samples each, into %lu bytes\n",
            block_count, block_samples, bytes_per_sample * 8, num_chans == 2 ? "stereo" : "mono", (unsigned long) total_bytes);
        fprintf (stderr, "overall compression ratio was %.2f%%, or %.2f bits per sample\n",
            total_bytes * 100.0 / source_bytes, total_bytes * 8.0 / num_chans / (total_samples - samples_left));
    }

    if (verbose_mode) {
        dump_compression_stats (stderr);

        if (stereo [1] || stereo [2] || stereo [3])
            fprintf (stderr, "dual-mono = %d, mid+side = %d, left+right = %d\n", stereo [1], stereo [2], stereo [3]);
    }

    return 0;
}

/***************************** DECODE SLAC TO WAV *****************************/

static int write_pcm_wav_header (FILE *outfile, uint32_t num_samples, int num_channels, int bytes_per_sample, uint32_t sample_rate);

static int decode_slac_to_wav (char *infilename, char *outfilename)
{
    int block_byte_count, sample_count, bytes_per_sample = 0, num_chans = 0, res = 0;
    uint32_t total_samples = 0, sample_rate = 0;
    FILE *infile, *outfile;

    if (*infilename == '-') {
        infile = stdin;
#ifdef _WIN32
        _setmode (fileno (stdin), O_BINARY);
#endif
    }
    else if (!(infile = fopen (infilename, "rb"))) {
        fprintf (stderr, "can't open file \"%s\" for reading!\n", infilename);
        return -1;
    }

    if (*outfilename == '-') {
        outfile = stdout;
#ifdef _WIN32
        _setmode (fileno (stdout), O_BINARY);
#endif
    }
    else if (!(outfile = fopen (outfilename, "wb"))) {
        fprintf (stderr, "can't open file \"%s\" for writing!\n", outfilename);
        return -1;
    }

    if (!raw_decode)
        write_pcm_wav_header (outfile, 0, 2, 2, 44100);

    while (1) {
        char *block_buffer, fourcc [4];
        char *raw_output_buffer;
        int32_t *sample_buffer;
        int ch;

        if (fread (fourcc, 1, 4, infile) != 4)
            break;

        if (memcmp (fourcc, "slac", 4)) {
            fprintf (stderr, "got invalid SLAC block!\n");
            res = -1;
            break;
        }

        if ((ch = fgetc (infile)) == EOF)
            break;

        block_byte_count = ch << 1;

        if ((ch = fgetc (infile)) == EOF)
            break;

        block_byte_count |= (ch << 9) & 0x1F00;
        bytes_per_sample = (ch >> 4) & 0x3;

        if (!block_byte_count)
            break;

        if ((ch = fgetc (infile)) == EOF)
            break;

        sample_count = ch;

        if ((ch = fgetc (infile)) == EOF)
            break;

        sample_count |= (ch << 8) & 0xF00;
        sample_rate = sample_rates [(ch >> 4) & 0xF];

        block_buffer = malloc (block_byte_count);

        if (fread (block_buffer, 1, block_byte_count, infile) != block_byte_count)
            break;

        num_chans = (block_buffer [0] & 0x3) ? 2 : 1;
        sample_buffer = malloc (sample_count * num_chans * sizeof (int32_t));
        raw_output_buffer = malloc (sample_count * num_chans * bytes_per_sample);

        if (decompress_audio_block (sample_buffer, sample_count, num_chans, block_buffer, block_byte_count)) {
            fprintf (stderr, "decompress_audio_block() returned error!\n");
            res = -1;
            break;
        }

        int count = sample_count * num_chans;
        int32_t *src = sample_buffer, temp;
        char *dst = raw_output_buffer;

        switch (bytes_per_sample) {
            case 1:
                while (count--)
                    *dst++ = *src++ + 128;

                break;

            case 2:
                while (count--) {
                    *dst++ = temp = *src++;
                    *dst++ = temp >> 8;
                }

                break;

            case 3:
                while (count--) {
                    *dst++ = temp = *src++;
                    *dst++ = temp >> 8;
                    *dst++ = temp >> 16;
                }

                break;
        }

        fwrite (raw_output_buffer, bytes_per_sample * num_chans, sample_count, outfile);
        total_samples += sample_count;
        free (raw_output_buffer);
        free (sample_buffer);
        free (block_buffer);
    }

    if (!raw_decode) {
        rewind (outfile);
        write_pcm_wav_header (outfile, total_samples, num_chans, bytes_per_sample, sample_rate);
    }

    fclose (outfile);

    return res;
}

static int write_pcm_wav_header (FILE *outfile, uint32_t num_samples, int num_channels, int bytes_per_sample, uint32_t sample_rate)
{
    RiffChunkHeader riffhdr;
    ChunkHeader datahdr, fmthdr;
    WaveHeader wavhdr;

    int wavhdrsize = 16;
    uint32_t total_data_bytes = num_samples * bytes_per_sample * num_channels;

    memset (&wavhdr, 0, sizeof (wavhdr));

    wavhdr.FormatTag = WAVE_FORMAT_PCM;
    wavhdr.NumChannels = num_channels;
    wavhdr.SampleRate = sample_rate;
    wavhdr.BytesPerSecond = sample_rate * num_channels * bytes_per_sample;
    wavhdr.BlockAlign = bytes_per_sample * num_channels;
    wavhdr.BitsPerSample = bytes_per_sample * 8;

    memcpy (riffhdr.ckID, "RIFF", sizeof (riffhdr.ckID));
    memcpy (riffhdr.formType, "WAVE", sizeof (riffhdr.formType));
    riffhdr.ckSize = sizeof (riffhdr) + wavhdrsize + sizeof (datahdr) + total_data_bytes;
    memcpy (fmthdr.ckID, "fmt ", sizeof (fmthdr.ckID));
    fmthdr.ckSize = wavhdrsize;

    memcpy (datahdr.ckID, "data", sizeof (datahdr.ckID));
    datahdr.ckSize = total_data_bytes;

    // write the RIFF chunks up to just before the data starts

    native_to_little_endian (&riffhdr, ChunkHeaderFormat);
    native_to_little_endian (&fmthdr, ChunkHeaderFormat);
    native_to_little_endian (&wavhdr, WaveHeaderFormat);
    native_to_little_endian (&datahdr, ChunkHeaderFormat);

    return fwrite (&riffhdr, sizeof (riffhdr), 1, outfile) &&
        fwrite (&fmthdr, sizeof (fmthdr), 1, outfile) &&
        fwrite (&wavhdr, wavhdrsize, 1, outfile) &&
        fwrite (&datahdr, sizeof (datahdr), 1, outfile);
}

/******************************** ENDIAN HELPERS ******************************/

static void little_endian_to_native (void *data, char *format)
{
    unsigned char *cp = (unsigned char *) data;
    int32_t temp;

    while (*format) {
        switch (*format) {
            case 'L':
                temp = cp [0] + ((int32_t) cp [1] << 8) + ((int32_t) cp [2] << 16) + ((int32_t) cp [3] << 24);
                * (int32_t *) cp = temp;
                cp += 4;
                break;

            case 'S':
                temp = cp [0] + (cp [1] << 8);
                * (short *) cp = (short) temp;
                cp += 2;
                break;

            default:
                if (isdigit ((unsigned char) *format))
                    cp += *format - '0';

                break;
        }

        format++;
    }
}

static void native_to_little_endian (void *data, char *format)
{
    unsigned char *cp = (unsigned char *) data;
    int32_t temp;

    while (*format) {
        switch (*format) {
            case 'L':
                temp = * (int32_t *) cp;
                *cp++ = (unsigned char) temp;
                *cp++ = (unsigned char) (temp >> 8);
                *cp++ = (unsigned char) (temp >> 16);
                *cp++ = (unsigned char) (temp >> 24);
                break;

            case 'S':
                temp = * (short *) cp;
                *cp++ = (unsigned char) temp;
                *cp++ = (unsigned char) (temp >> 8);
                break;

            default:
                if (isdigit ((unsigned char) *format))
                    cp += *format - '0';

                break;
        }

        format++;
    }
}

