/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <stdint.h>
#include <stdlib.h>
#include "lcm/lcm_coretypes.h"

#ifndef _gains_t_h
#define _gains_t_h

#ifdef __cplusplus
extern "C" {
#endif

#define GAINS_T_VAL_KP 0
#define GAINS_T_VAL_KI 1
#define GAINS_T_VAL_KD 2
#define GAINS_T_RATE_KP 3
#define GAINS_T_RATE_KI 4
#define GAINS_T_RATE_KD 5

typedef struct _gains_t gains_t;
struct _gains_t
{
    float      xGains[6];
    float      yGains[6];
    float      zGains[6];
    float      yawGains[3];
    int64_t    utime;
};

gains_t   *gains_t_copy(const gains_t *p);
void gains_t_destroy(gains_t *p);

int  gains_t_encode(void *buf, int offset, int maxlen, const gains_t *p);
int  gains_t_decode(const void *buf, int offset, int maxlen, gains_t *p);
int  gains_t_decode_cleanup(gains_t *p);
int  gains_t_encoded_size(const gains_t *p);

// LCM support functions. Users should not call these
int64_t __gains_t_get_hash(void);
int64_t __gains_t_hash_recursive(const __lcm_hash_ptr *p);
int     __gains_t_encode_array(void *buf, int offset, int maxlen, const gains_t *p, int elements);
int     __gains_t_decode_array(const void *buf, int offset, int maxlen, gains_t *p, int elements);
int     __gains_t_decode_array_cleanup(gains_t *p, int elements);
int     __gains_t_encoded_array_size(const gains_t *p, int elements);
int     __gains_t_clone_array(const gains_t *p, gains_t *q, int elements);

#ifdef __cplusplus
}
#endif

#endif