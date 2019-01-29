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
    int error, bc;
    uint32_t sr;
} Bitstream;

#define getbit(bs) ( \
    (((bs)->bc) ? \
        ((bs)->bc--, (bs)->sr & 1) : \
            (((++((bs)->ptr) != (bs)->end) ? (void) 0 : (bs)->wrap (bs)), (bs)->bc = 7, ((bs)->sr = *((bs)->ptr)) & 1) \
    ) ? \
        ((bs)->sr >>= 1, 1) : \
        ((bs)->sr >>= 1, 0) \
)

// Note that the value retrieved from getbits() may contain bits outside those
// requested, so a mask is required afterward if this is problematic.

#define getbits(value, nbits, bs) do { \
    while ((nbits) > (bs)->bc) { \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs); \
        (bs)->sr |= (int32_t)*((bs)->ptr) << (bs)->bc; \
        (bs)->bc += 8; \
    } \
    *(value) = (bs)->sr; \
    if ((bs)->bc > 32) { \
        (bs)->bc -= (nbits); \
        (bs)->sr = *((bs)->ptr) >> (8 - (bs)->bc); \
    } \
    else { \
        (bs)->bc -= (nbits); \
        (bs)->sr >>= (nbits); \
    } \
} while (0)

#define putbit(bit, bs) do { if (bit) (bs)->sr |= (1 << (bs)->bc); \
    if (++((bs)->bc) == 8) { \
        *((bs)->ptr) = (bs)->sr; \
        (bs)->sr = (bs)->bc = 0; \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs); \
    }} while (0)

#define putbit_0(bs) do { \
    if (++((bs)->bc) == 8) { \
        *((bs)->ptr) = (bs)->sr; \
        (bs)->sr = (bs)->bc = 0; \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs); \
    }} while (0)

#define putbit_1(bs) do { (bs)->sr |= (1 << (bs)->bc); \
    if (++((bs)->bc) == 8) { \
        *((bs)->ptr) = (bs)->sr; \
        (bs)->sr = (bs)->bc = 0; \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs); \
    }} while (0)

// Note that the putbits() requires that no other bits are set in the value
// provided, so a mask is required beforehand if this might happen.

#define putbits(value, nbits, bs) do { \
    (bs)->sr |= (int32_t)(value) << (bs)->bc; \
    if (((bs)->bc += (nbits)) >= 8) \
        do { \
            *((bs)->ptr) = (bs)->sr; \
            (bs)->sr >>= 8; \
            if (((bs)->bc -= 8) > 24) \
                (bs)->sr |= ((value) >> ((nbits) - (bs)->bc)); \
            if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs); \
        } while ((bs)->bc >= 8); \
} while (0)

// Open the specified BitStream and associate with the specified buffer.

static void bs_read (Bitstream *bs);

static void bs_open_read (Bitstream *bs, void *buffer_start, void *buffer_end)
{
    bs->error = bs->sr = bs->bc = 0;
    bs->ptr = ((bs->buf = buffer_start) - 1);
    bs->end = buffer_end;
    bs->wrap = bs_read;
}

// This function is only called from the getbit() and getbits() macros when
// the BitStream has been exhausted and more data is required. Sinve these
// bistreams no longer access files, this function simply sets an error and
// resets the buffer.

static void bs_read (Bitstream *bs)
{
    bs->ptr = bs->buf;
    bs->error = 1;
}

// This function is called to close the bitstream. It returns the number of
// full bytes actually read as bits, rounded up to the next word.

static uint32_t bs_close_read (Bitstream *bs)
{
    uint32_t bytes_read;

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
    bs->error = bs->sr = bs->bc = 0;
    bs->ptr = bs->buf = buffer_start;
    bs->end = buffer_end;
    bs->wrap = bs_write;
}

// This function is only called from the putbit() and putbits() macros when
// the buffer is full, which is now flagged as an error.

static void bs_write (Bitstream *bs)
{
    bs->ptr = bs->buf;
    bs->error = 1;
}

// This function forces a flushing write of the specified Bitstream, and
// returns the total number of bytes written into the buffer, rounded up
// to the next word.

static uint32_t bs_close_write (Bitstream *bs)
{
    uint32_t bytes_written;

    if (bs->error)
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
