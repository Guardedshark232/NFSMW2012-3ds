#ifndef TINF_H
#define TINF_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TINF_OK             0
#define TINF_DATA_ERROR    (-3)
#define TINF_BUF_ERROR     (-5)

/* Decompress zlib stream (including 2-byte header and 4-byte Adler32) */
int tinf_zlib_uncompress(void *dest, unsigned int *destLen,
                         const void *source, unsigned int sourceLen);

/* Decompress raw deflate data without headers */
int tinf_uncompress(void *dest, unsigned int *destLen,
                    const void *source, unsigned int sourceLen);

#ifdef __cplusplus
}
#endif

#endif /* TINF_H */
