/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <string.h>
#include "messaging/gains_t.h"

static int __gains_t_hash_computed;
static int64_t __gains_t_hash;

int64_t __gains_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __gains_t_get_hash)
            return 0;

    const __lcm_hash_ptr cp = { p, (void*)__gains_t_get_hash };
    (void) cp;

    int64_t hash = 0xd0500d89c9fa09bcLL
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __int64_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __gains_t_get_hash(void)
{
    if (!__gains_t_hash_computed) {
        __gains_t_hash = __gains_t_hash_recursive(NULL);
        __gains_t_hash_computed = 1;
    }

    return __gains_t_hash;
}

int __gains_t_encode_array(void *buf, int offset, int maxlen, const gains_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, p[element].xGains, 6);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, p[element].yGains, 6);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, p[element].zGains, 6);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, p[element].yawGains, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int gains_t_encode(void *buf, int offset, int maxlen, const gains_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __gains_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __gains_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __gains_t_encoded_array_size(const gains_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __float_encoded_array_size(p[element].xGains, 6);

        size += __float_encoded_array_size(p[element].yGains, 6);

        size += __float_encoded_array_size(p[element].zGains, 6);

        size += __float_encoded_array_size(p[element].yawGains, 3);

        size += __int64_t_encoded_array_size(&(p[element].utime), 1);

    }
    return size;
}

int gains_t_encoded_size(const gains_t *p)
{
    return 8 + __gains_t_encoded_array_size(p, 1);
}

int __gains_t_decode_array(const void *buf, int offset, int maxlen, gains_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, p[element].xGains, 6);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, p[element].yGains, 6);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, p[element].zGains, 6);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, p[element].yawGains, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __gains_t_decode_array_cleanup(gains_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __float_decode_array_cleanup(p[element].xGains, 6);

        __float_decode_array_cleanup(p[element].yGains, 6);

        __float_decode_array_cleanup(p[element].zGains, 6);

        __float_decode_array_cleanup(p[element].yawGains, 3);

        __int64_t_decode_array_cleanup(&(p[element].utime), 1);

    }
    return 0;
}

int gains_t_decode(const void *buf, int offset, int maxlen, gains_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __gains_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __gains_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int gains_t_decode_cleanup(gains_t *p)
{
    return __gains_t_decode_array_cleanup(p, 1);
}

int __gains_t_clone_array(const gains_t *p, gains_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __float_clone_array(p[element].xGains, q[element].xGains, 6);

        __float_clone_array(p[element].yGains, q[element].yGains, 6);

        __float_clone_array(p[element].zGains, q[element].zGains, 6);

        __float_clone_array(p[element].yawGains, q[element].yawGains, 3);

        __int64_t_clone_array(&(p[element].utime), &(q[element].utime), 1);

    }
    return 0;
}

gains_t *gains_t_copy(const gains_t *p)
{
    gains_t *q = (gains_t*) malloc(sizeof(gains_t));
    __gains_t_clone_array(p, q, 1);
    return q;
}

void gains_t_destroy(gains_t *p)
{
    __gains_t_decode_array_cleanup(p, 1);
    free(p);
}

