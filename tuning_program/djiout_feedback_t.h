/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <stdint.h>
#include <stdlib.h>
#include <lcm/lcm_coretypes.h>

#ifndef _djiout_feedback_t_h
#define _djiout_feedback_t_h

#ifdef __cplusplus
extern "C" {
#endif

#include "int32.h"
typedef struct _djiout_feedback_t djiout_feedback_t;
struct _djiout_feedback_t
{
    int32      timestamp;
    float      roll;
    float      pitch;
    float      yawdot;
    float      fz;
};

djiout_feedback_t   *djiout_feedback_t_copy(const djiout_feedback_t *p);
void djiout_feedback_t_destroy(djiout_feedback_t *p);

int  djiout_feedback_t_encode(void *buf, int offset, int maxlen, const djiout_feedback_t *p);
int  djiout_feedback_t_decode(const void *buf, int offset, int maxlen, djiout_feedback_t *p);
int  djiout_feedback_t_decode_cleanup(djiout_feedback_t *p);
int  djiout_feedback_t_encoded_size(const djiout_feedback_t *p);

// LCM support functions. Users should not call these
int64_t __djiout_feedback_t_get_hash(void);
int64_t __djiout_feedback_t_hash_recursive(const __lcm_hash_ptr *p);
int     __djiout_feedback_t_encode_array(void *buf, int offset, int maxlen, const djiout_feedback_t *p, int elements);
int     __djiout_feedback_t_decode_array(const void *buf, int offset, int maxlen, djiout_feedback_t *p, int elements);
int     __djiout_feedback_t_decode_array_cleanup(djiout_feedback_t *p, int elements);
int     __djiout_feedback_t_encoded_array_size(const djiout_feedback_t *p, int elements);
int     __djiout_feedback_t_clone_array(const djiout_feedback_t *p, djiout_feedback_t *q, int elements);

#ifdef __cplusplus
}
#endif

#endif
