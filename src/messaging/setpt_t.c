// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "setpt_t.h"

static int __setpt_t_hash_computed;
static int64_t __setpt_t_hash;

int64_t __setpt_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __setpt_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__setpt_t_get_hash;
    (void) cp;

    int64_t hash = (int64_t)0xe71a1a8a0aea9316LL
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __setpt_t_get_hash(void)
{
    if (!__setpt_t_hash_computed) {
        __setpt_t_hash = __setpt_t_hash_recursive(NULL);
        __setpt_t_hash_computed = 1;
    }

    return __setpt_t_hash;
}

int __setpt_t_encode_array(void *buf, int offset, int maxlen, const setpt_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].z), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].yaw), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].flags), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int setpt_t_encode(void *buf, int offset, int maxlen, const setpt_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __setpt_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __setpt_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __setpt_t_encoded_array_size(const setpt_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __float_encoded_array_size(&(p[element].x), 1);

        size += __float_encoded_array_size(&(p[element].y), 1);

        size += __float_encoded_array_size(&(p[element].z), 1);

        size += __float_encoded_array_size(&(p[element].yaw), 1);

        size += __int8_t_encoded_array_size(&(p[element].flags), 1);

        size += __int32_t_encoded_array_size(&(p[element].utime), 1);

    }
    return size;
}

int setpt_t_encoded_size(const setpt_t *p)
{
    return 8 + __setpt_t_encoded_array_size(p, 1);
}

int __setpt_t_decode_array(const void *buf, int offset, int maxlen, setpt_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].z), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].yaw), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].flags), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __setpt_t_decode_array_cleanup(setpt_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __float_decode_array_cleanup(&(p[element].x), 1);

        __float_decode_array_cleanup(&(p[element].y), 1);

        __float_decode_array_cleanup(&(p[element].z), 1);

        __float_decode_array_cleanup(&(p[element].yaw), 1);

        __int8_t_decode_array_cleanup(&(p[element].flags), 1);

        __int32_t_decode_array_cleanup(&(p[element].utime), 1);

    }
    return 0;
}

int setpt_t_decode(const void *buf, int offset, int maxlen, setpt_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __setpt_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __setpt_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int setpt_t_decode_cleanup(setpt_t *p)
{
    return __setpt_t_decode_array_cleanup(p, 1);
}

int __setpt_t_clone_array(const setpt_t *p, setpt_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __float_clone_array(&(p[element].x), &(q[element].x), 1);

        __float_clone_array(&(p[element].y), &(q[element].y), 1);

        __float_clone_array(&(p[element].z), &(q[element].z), 1);

        __float_clone_array(&(p[element].yaw), &(q[element].yaw), 1);

        __int8_t_clone_array(&(p[element].flags), &(q[element].flags), 1);

        __int32_t_clone_array(&(p[element].utime), &(q[element].utime), 1);

    }
    return 0;
}

setpt_t *setpt_t_copy(const setpt_t *p)
{
    setpt_t *q = (setpt_t*) malloc(sizeof(setpt_t));
    __setpt_t_clone_array(p, q, 1);
    return q;
}

void setpt_t_destroy(setpt_t *p)
{
    __setpt_t_decode_array_cleanup(p, 1);
    free(p);
}

