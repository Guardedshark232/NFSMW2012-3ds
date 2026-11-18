/*
 * tinf - tiny inflate library (zlib-compatible decompressor)
 * Implements RFC 1950 (zlib) and RFC 1951 (deflate).
 */

#include "tinf.h"
#include <stdint.h>

typedef struct {
    unsigned short table[16];
    unsigned short trans[288];
} TINF_TREE;

typedef struct {
    const unsigned char *source;
    unsigned int tag;
    unsigned int bitcount;

    unsigned char *dest;
    unsigned int destLen;
    unsigned int curLen;

    TINF_TREE ltree;
    TINF_TREE dtree;
} TINF_DATA;

/* ----------------------------------------------------------------- Adler-32 */

static uint32_t tinf_adler32(const void *data, unsigned int len) {
    const unsigned char *buf = (const unsigned char *)data;
    uint32_t s1 = 1;
    uint32_t s2 = 0;

    for (unsigned int i = 0; i < len; ++i) {
        s1 = (s1 + buf[i]) % 65521;
        s2 = (s2 + s1) % 65521;
    }
    return (s2 << 16) | s1;
}

/* ----------------------------------------------------------------- Bit stream */

static unsigned int tinf_getbit(TINF_DATA *d) {
    unsigned int bit;

    if (!d->bitcount--) {
        d->tag = *d->source++;
        d->bitcount = 7;
    }

    bit = d->tag & 1;
    d->tag >>= 1;

    return bit;
}

static unsigned int tinf_read_bits(TINF_DATA *d, int num, int base) {
    unsigned int val = 0;

    if (num) {
        unsigned int limit = 1 << num;
        unsigned int mask;

        for (mask = 1; mask < limit; mask <<= 1) {
            if (tinf_getbit(d)) val += mask;
        }
    }

    return val + base;
}

/* ----------------------------------------------------------------- Huffman */

static void tinf_build_tree(TINF_TREE *t, const unsigned char *lengths, unsigned int num) {
    unsigned short offs[16];
    unsigned int i, sum = 0;

    for (i = 0; i < 16; ++i) t->table[i] = 0;
    for (i = 0; i < num; ++i) t->table[lengths[i]]++;

    t->table[0] = 0;

    for (i = 0; i < 16; ++i) {
        offs[i] = sum;
        sum += t->table[i];
    }

    for (i = 0; i < num; ++i) {
        if (lengths[i]) t->trans[offs[lengths[i]]++] = i;
    }
}

static int tinf_decode_symbol(TINF_DATA *d, const TINF_TREE *t) {
    int base = 0, walk = 0;
    int len = 1;

    for (;;) {
        walk |= tinf_getbit(d);

        if (walk < t->table[len]) {
            return t->trans[base + walk];
        }

        base += t->table[len];
        walk -= t->table[len];

        walk <<= 1;
        len++;

        if (len >= 16) return TINF_DATA_ERROR;
    }
}

static void tinf_build_fixed_trees(TINF_TREE *lt, TINF_TREE *dt) {
    unsigned char lengths[288];
    unsigned int i;

    for (i = 0; i < 144; ++i) lengths[i] = 8;
    for (; i < 256; ++i) lengths[i] = 9;
    for (; i < 280; ++i) lengths[i] = 7;
    for (; i < 288; ++i) lengths[i] = 8;
    tinf_build_tree(lt, lengths, 288);

    for (i = 0; i < 32; ++i) lengths[i] = 5;
    tinf_build_tree(dt, lengths, 32);
}

static int tinf_decode_trees(TINF_DATA *d, TINF_TREE *lt, TINF_TREE *dt) {
    static const unsigned char clcidx[19] = {
        16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15
    };
    unsigned char lengths[288 + 32];
    unsigned char clen[19];
    TINF_TREE codetree;
    unsigned int hlit, hdist, hclen;
    unsigned int i, num;

    hlit = tinf_read_bits(d, 5, 257);
    hdist = tinf_read_bits(d, 5, 1);
    hclen = tinf_read_bits(d, 4, 4);

    for (i = 0; i < 19; ++i) clen[i] = 0;
    for (i = 0; i < hclen; ++i) {
        clen[clcidx[i]] = tinf_read_bits(d, 3, 0);
    }

    tinf_build_tree(&codetree, clen, 19);

    for (num = 0; num < hlit + hdist;) {
        int sym = tinf_decode_symbol(d, &codetree);
        if (sym < 0) return sym;

        switch (sym) {
        case 16: {
            unsigned char prev = lengths[num - 1];
            unsigned int fill = tinf_read_bits(d, 2, 3);
            while (fill--) lengths[num++] = prev;
            break;
        }
        case 17: {
            unsigned int fill = tinf_read_bits(d, 3, 3);
            while (fill--) lengths[num++] = 0;
            break;
        }
        case 18: {
            unsigned int fill = tinf_read_bits(d, 7, 11);
            while (fill--) lengths[num++] = 0;
            break;
        }
        default:
            lengths[num++] = sym;
            break;
        }
    }

    tinf_build_tree(lt, lengths, hlit);
    tinf_build_tree(dt, lengths + hlit, hdist);

    return TINF_OK;
}

/* ----------------------------------------------------------------- Block inflate */

static const unsigned short length_base[30] = {
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
    35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258, 0
};
static const unsigned char length_extra[30] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
    3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0, 0
};
static const unsigned short dist_base[30] = {
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193,
    257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145,
    8193, 12289, 16385, 24577
};
static const unsigned char dist_extra[30] = {
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6,
    7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13
};

static int tinf_inflate_block_data(TINF_DATA *d, TINF_TREE *lt, TINF_TREE *dt) {
    for (;;) {
        int sym = tinf_decode_symbol(d, lt);
        if (sym < 0) return sym;

        if (sym == 256) return TINF_OK;

        if (sym < 256) {
            if (d->curLen >= d->destLen) return TINF_BUF_ERROR;
            d->dest[d->curLen++] = sym;
        } else {
            unsigned int len, dist, i;
            sym -= 257;
            len = tinf_read_bits(d, length_extra[sym], length_base[sym]);
            dist = tinf_decode_symbol(d, dt);
            dist = tinf_read_bits(d, dist_extra[dist], dist_base[dist]);

            if (d->curLen + len > d->destLen) return TINF_BUF_ERROR;

            for (i = 0; i < len; ++i) {
                d->dest[d->curLen] = d->dest[d->curLen - dist];
                d->curLen++;
            }
        }
    }
}

static int tinf_inflate_uncompressed_block(TINF_DATA *d) {
    unsigned int len, nlen, i;

    d->bitcount = 0;
    len = d->source[0] | (d->source[1] << 8);
    nlen = d->source[2] | (d->source[3] << 8);
    d->source += 4;

    if (len != (~nlen & 0xFFFF)) return TINF_DATA_ERROR;
    if (d->curLen + len > d->destLen) return TINF_BUF_ERROR;

    for (i = 0; i < len; ++i) {
        d->dest[d->curLen++] = *d->source++;
    }

    return TINF_OK;
}

/* ----------------------------------------------------------------- Public API */

int tinf_uncompress(void *dest, unsigned int *destLen,
                    const void *source, unsigned int sourceLen) {
    TINF_DATA d;
    int bfinal;

    d.source = (const unsigned char *)source;
    d.tag = 0;
    d.bitcount = 0;
    d.dest = (unsigned char *)dest;
    d.destLen = *destLen;
    d.curLen = 0;

    do {
        int btype, res;

        bfinal = tinf_getbit(&d);
        btype = tinf_read_bits(&d, 2, 0);

        switch (btype) {
        case 0:
            res = tinf_inflate_uncompressed_block(&d);
            break;
        case 1:
            tinf_build_fixed_trees(&d.ltree, &d.dtree);
            res = tinf_inflate_block_data(&d, &d.ltree, &d.dtree);
            break;
        case 2:
            res = tinf_decode_trees(&d, &d.ltree, &d.dtree);
            if (res == TINF_OK) {
                res = tinf_inflate_block_data(&d, &d.ltree, &d.dtree);
            }
            break;
        default:
            res = TINF_DATA_ERROR;
            break;
        }

        if (res != TINF_OK) return res;

    } while (!bfinal);

    *destLen = d.curLen;
    return TINF_OK;
}

int tinf_zlib_uncompress(void *dest, unsigned int *destLen,
                         const void *source, unsigned int sourceLen) {
    const unsigned char *src = (const unsigned char *)source;
    unsigned int cmf, flg;
    int res;

    if (sourceLen < 6) return TINF_DATA_ERROR;

    cmf = src[0];
    flg = src[1];

    /* Check deflate compression method (CM=8, CINFO<=7) */
    if ((cmf & 0x0F) != 8 || (cmf >> 4) > 7) return TINF_DATA_ERROR;

    /* Check header checksum */
    if (((cmf << 8) + flg) % 31 != 0) return TINF_DATA_ERROR;

    /* Skip preset dictionary if present */
    if (flg & 0x20) return TINF_DATA_ERROR;

    res = tinf_uncompress(dest, destLen, src + 2, sourceLen - 6);
    if (res != TINF_OK) return res;

    /* Verify Adler-32 checksum */
    uint32_t a32 = ((uint32_t)src[sourceLen - 4] << 24) |
                   ((uint32_t)src[sourceLen - 3] << 16) |
                   ((uint32_t)src[sourceLen - 2] << 8)  |
                   ((uint32_t)src[sourceLen - 1]);

    if (tinf_adler32(dest, *destLen) != a32) {
        return TINF_DATA_ERROR;
    }

    return TINF_OK;
}
