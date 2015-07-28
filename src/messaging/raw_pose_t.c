/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <string.h>
#include "messaging/raw_pose_t.h"

static int __raw_pose_t_hash_computed;
static int64_t __raw_pose_t_hash;

int64_t __raw_pose_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __raw_pose_t_get_hash)
            return 0;

    const __lcm_hash_ptr cp = { p, (void*)__raw_pose_t_get_hash };
    (void) cp;

    int64_t hash = 0x5c5dd8cc22d56939LL
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __int64_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __raw_pose_t_get_hash(void)
{
    if (!__raw_pose_t_hash_computed) {
        __raw_pose_t_hash = __raw_pose_t_hash_recursive(NULL);
        __raw_pose_t_hash_computed = 1;
    }

    return __raw_pose_t_hash;
}

int __raw_pose_t_encode_array(void *buf, int offset, int maxlen, const raw_pose_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].yaw), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int raw_pose_t_encode(void *buf, int offset, int maxlen, const raw_pose_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __raw_pose_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __raw_pose_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __raw_pose_t_encoded_array_size(const raw_pose_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __float_encoded_array_size(&(p[element].x), 1);

        size += __float_encoded_array_size(&(p[element].y), 1);

        size += __float_encoded_array_size(&(p[element].yaw), 1);

        size += __int64_t_encoded_array_size(&(p[element].utime), 1);

    }
    return size;
}

int raw_pose_t_encoded_size(const raw_pose_t *p)
{
    return 8 + __raw_pose_t_encoded_array_size(p, 1);
}

int __raw_pose_t_decode_array(const void *buf, int offset, int maxlen, raw_pose_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].yaw), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __raw_pose_t_decode_array_cleanup(raw_pose_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __float_decode_array_cleanup(&(p[element].x), 1);

        __float_decode_array_cleanup(&(p[element].y), 1);

        __float_decode_array_cleanup(&(p[element].yaw), 1);

        __int64_t_decode_array_cleanup(&(p[element].utime), 1);

    }
    return 0;
}

int raw_pose_t_decode(const void *buf, int offset, int maxlen, raw_pose_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __raw_pose_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __raw_pose_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int raw_pose_t_decode_cleanup(raw_pose_t *p)
{
    return __raw_pose_t_decode_array_cleanup(p, 1);
}

int __raw_pose_t_clone_array(const raw_pose_t *p, raw_pose_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __float_clone_array(&(p[element].x), &(q[element].x), 1);

        __float_clone_array(&(p[element].y), &(q[element].y), 1);

        __float_clone_array(&(p[element].yaw), &(q[element].yaw), 1);

        __int64_t_clone_array(&(p[element].utime), &(q[element].utime), 1);

    }
    return 0;
}

raw_pose_t *raw_pose_t_copy(const raw_pose_t *p)
{
    raw_pose_t *q = (raw_pose_t*) malloc(sizeof(raw_pose_t));
    __raw_pose_t_clone_array(p, q, 1);
    return q;
}

void raw_pose_t_destroy(raw_pose_t *p)
{
    __raw_pose_t_decode_array_cleanup(p, 1);
    free(p);
}

