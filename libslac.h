////////////////////////////////////////////////////////////////////////////
//                           ****** SLAC ******                           //
//                    Simple Lossless Audio Compressor                    //
//                    Copyright (c) 2025 David Bryant.                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// libslac.h

// This module provides functions to compress and decompress PCM integer
// audio. The audio is presented in 32-bit integers, but can be arbitrary
// bitdepths (redundant bits at either side of the words are handled). Mono
// streams are supported and up to 24-bits per sample (although the
// significant bits can be anywhere in the 32-bit word).
//
// No allocation is used; everything is done in place using the passed input
// and output buffers.

#ifndef LIBSLAC_H
#define LIBSLAC_H

#ifdef __cplusplus
extern "C" {
#endif

int compress_audio_buffer (int32_t *audio_samples, int sample_count, int stride, char *outbuffer, int outbufsize);
int decompress_audio_buffer (int32_t *audio_samples, int sample_count, int stride, char *inbuffer, int inbufsize);
void dump_compression_stats (FILE *file);

#ifdef __cplusplus
}
#endif

#endif
