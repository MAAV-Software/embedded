/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <stdint.h>
#include <stdlib.h>
#include <lcm/lcm_coretypes.h>

#ifndef _target_t_h
#define _target_t_h

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _target_t target_t;
struct _target_t
{
    int32_t    timestamp;
    int32_t    cmd;
    float      x;
    float      y;
    float      z;
    float      h;
};

target_t   *target_t_copy(const target_t *p);
void target_t_destroy(target_t *p);

int  target_t_encode(void *buf, int offset, int maxlen, const target_t *p);
int  target_t_decode(const void *buf, int offset, int maxlen, target_t *p);
int  target_t_decode_cleanup(target_t *p);
int  target_t_encoded_size(const target_t *p);

// LCM support functions. Users should not call these
int64_t __target_t_get_hash(void);
int64_t __target_t_hash_recursive(const __lcm_hash_ptr *p);
int     __target_t_encode_array(void *buf, int offset, int maxlen, const target_t *p, int elements);
int     __target_t_decode_array(const void *buf, int offset, int maxlen, target_t *p, int elements);
int     __target_t_decode_array_cleanup(target_t *p, int elements);
int     __target_t_encoded_array_size(const target_t *p, int elements);
int     __target_t_clone_array(const target_t *p, target_t *q, int elements);

#ifdef __cplusplus
}
#endif

#endif