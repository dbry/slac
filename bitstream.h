////////////////////////////////////////////////////////////////////////////
//                           ****** SLAC ******                           //
//                    Simple Lossless Audio Compressor                    //
//                    Copyright (c) 2019 David Bryant.                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// bitstream.h

// This helper module provides functions and macros to read and write
// bitstreams that hide the messy details of reading and writing individual
// bits.

typedef struct bs {
    unsigned char *buf, *end, *ptr;
    void (*wrap)(struct bs *bs);
    int wrapc, bc;
    uint32_t sr;
} Bitstream;

// Read a unary count. This is currently represented in the bitstream as a sequence
// of ones (the count, which may be zero) followed by a zero. However, the reverse
// could also be true, and might be faster. The count may be arbitrarily large.

#define getcount(count, bs) do {                                \
    *(count) = 0;                                               \
    while (1) {                                                 \
        while ((bs)->bc < 25) {                                 \
            if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs);    \
            (bs)->sr |= (int32_t)*((bs)->ptr) << (bs)->bc;      \
            (bs)->bc += 8;                                      \
        }                                                       \
        if (((bs)->sr & 0x1FFFFFF) == 0x1FFFFFF) {              \
            (bs)->sr >>= 25;                                    \
            (bs)->bc -= 25;                                     \
            *(count) += 25;                                     \
        }                                                       \
        else {                                                  \
            int temp = __builtin_ctz (~(bs)->sr);               \
            (bs)->sr >>= temp + 1;                              \
            (bs)->bc -= temp + 1;                               \
            *(count) += temp;                                   \
            break;                                              \
        }                                                       \
    }                                                           \
} while (0)

// Read a value consuming just the specified number of bits. The value is returned
// right-justified but may contain bits set higher than those requested, so a mask
// is required afterward if this is problematic (which it normally would be).

#define getbits(value, nbits, bs) do {                          \
    while ((nbits) > (bs)->bc) {                                \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs);        \
        (bs)->sr |= (int32_t)*((bs)->ptr) << (bs)->bc;          \
        (bs)->bc += 8;                                          \
    }                                                           \
    *(value) = (bs)->sr;                                        \
    if ((bs)->bc > 32) {                                        \
        (bs)->bc -= (nbits);                                    \
        (bs)->sr = *((bs)->ptr) >> (8 - (bs)->bc);              \
    }                                                           \
    else {                                                      \
        (bs)->bc -= (nbits);                                    \
        (bs)->sr >>= (nbits);                                   \
    }                                                           \
} while (0)

// Write a unary count. This is currently represented in the bitstream as a sequence
// of ones (the count, which may be zero) followed by a zero. However, the reverse
// could also be true, and might be faster. The count may be arbitrarily large.

#define putcount(count, bs) do {                                \
    while ((count) >= 8 - (bs)->bc) {                           \
        *((bs)->ptr) = (bs)->sr | (-1U << (bs)->bc);            \
        (count) -= 8 - (bs)->bc;                                \
        (bs)->sr = (bs)->bc = 0;                                \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs);        \
    }                                                           \
    (bs)->sr |= (((1U << (count)) - 1) << (bs)->bc);            \
    if (((bs)->bc += (count) + 1) == 8) {                       \
        *((bs)->ptr) = (bs)->sr;                                \
        (bs)->sr = (bs)->bc = 0;                                \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs);        \
    }                                                           \
} while (0)

// Write a one to the bitstream. This is currently used only by bs_close_write().

#define putbit_1(bs) do { (bs)->sr |= (1 << (bs)->bc);          \
    if (++((bs)->bc) == 8) {                                    \
        *((bs)->ptr) = (bs)->sr;                                \
        (bs)->sr = (bs)->bc = 0;                                \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs);        \
    }} while (0)

// Write a value consuming just the specified number of bits. The value is passed
// right-justified and may not contain bits set higher than those specified, so a
// mask is required beforehand if this is possible.

#define putbits(value, nbits, bs) do {                          \
    (bs)->sr |= (int32_t)(value) << (bs)->bc;                   \
    if (((bs)->bc += (nbits)) >= 8)                             \
        do {                                                    \
            *((bs)->ptr) = (bs)->sr;                            \
            (bs)->sr >>= 8;                                     \
            if (((bs)->bc -= 8) > 24)                           \
                (bs)->sr |= ((value) >> ((nbits) - (bs)->bc));  \
            if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs);    \
        } while ((bs)->bc >= 8);                                \
} while (0)

// Open the specified BitStream and associate with the specified buffer.

static void bs_read (Bitstream *bs);

static void bs_open_read (Bitstream *bs, void *buffer_start, void *buffer_end)
{
    bs->wrapc = bs->sr = bs->bc = 0;
    bs->ptr = ((bs->buf = buffer_start) - 1);
    bs->end = buffer_end;
    bs->wrap = bs_read;
}

// This function is only called from the getbit() and getbits() macros when
// the BitStream has been exhausted and more data is required. Since these
// bistreams no longer access files, this function simply resets the buffer
// and bumps the wrap count.

static void bs_read (Bitstream *bs)
{
    bs->ptr = bs->buf;
    bs->wrapc++;
}

// This function is called to close the bitstream. It returns the number of
// full bytes actually read as bits, rounded up to the next word.

static uint32_t bs_close_read (Bitstream *bs)
{
    uint32_t bytes_read;

    // just because we wrapped, we didn't necessarily overrun if we didn't use the bits

    while (bs->bc >= 8) {
        if (bs->ptr == bs->buf) {
            bs->ptr = bs->end - 1;
            bs->wrapc--;
        }
        else
            bs->ptr--;

        bs->bc -= 8;
    }

    if (bs->wrapc)
        return (uint32_t) -1;

    if (bs->bc < 8)
        bs->ptr++;

    bytes_read = (uint32_t)(bs->ptr - bs->buf);

    if (!(bytes_read & 1))
        ++bytes_read;

    memset (bs, 0, sizeof (*bs));
    return bytes_read;
}

// Open the specified Bitstream using the specified buffer pointers. It is
// assumed that enough buffer space has been allocated for all data that will
// be written, otherwise an error will be generated.

static void bs_write (Bitstream *bs);

static void bs_open_write (Bitstream *bs, void *buffer_start, void *buffer_end)
{
    bs->wrapc = bs->sr = bs->bc = 0;
    bs->ptr = bs->buf = buffer_start;
    bs->end = buffer_end;
    bs->wrap = bs_write;
}

// This function is only called from the putbit() and putbits() macros when
// the buffer is full, which is now flagged as an error.

static void bs_write (Bitstream *bs)
{
    bs->ptr = bs->buf;
    bs->wrapc++;
}

// This function forces a flushing write of the specified Bitstream, and
// returns the total number of bytes written into the buffer, rounded up
// to the next word.

static uint32_t bs_close_write (Bitstream *bs)
{
    uint32_t bytes_written;

    if (bs->wrapc)
        return (uint32_t) -1;

    while (1) {
        while (bs->bc)
            putbit_1 (bs);

        bytes_written = (uint32_t)(bs->ptr - bs->buf);

        if (bytes_written & 1)
            putbit_1 (bs);
        else
            break;
    }

    memset (bs, 0, sizeof (*bs));
    return bytes_written;
}
