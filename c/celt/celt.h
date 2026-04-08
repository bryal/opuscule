/* Copyright (c) 2007-2012 IETF Trust, CSIRO, Xiph.Org Foundation,
                           Gregory Maxwell. All rights reserved.
   Written by Jean-Marc Valin and Gregory Maxwell */
/**
  @file celt.h
  @brief Contains all the functions for encoding and decoding audio
 */

/*

   This file is extracted from RFC6716. Please see that RFC for additional
   information.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   - Neither the name of Internet Society, IETF or IETF Trust, nor the
   names of specific contributors, may be used to endorse or promote
   products derived from this software without specific prior written
   permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CELT_H
#define CELT_H

#include "opus_types.h"
#include "opus_defines.h"
#include "opus_custom.h"
#include "entenc.h"
#include "entdec.h"
#include "arch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CELTEncoder OpusCustomEncoder
#define CELTDecoder OpusCustomDecoder
#define CELTMode OpusCustomMode

/* C equivalent of the Rust CeltDecCtl enum (#[repr(C, i32)]).
   Layout: struct { int32_t tag; union { ... } payload; } */
typedef struct {
    opus_int32 tag;
    union {
        opus_int32   i;
        opus_int32  *ip;
        opus_uint32 *up;
        const CELTMode **mp;
    } payload;
} CeltDecCtl;

#define _celt_check_mode_ptr_ptr(ptr) ((ptr) + ((ptr) - (const CELTMode**)(ptr)))

/* Encoder/decoder Requests */

#define CELT_SET_PREDICTION_REQUEST    10002
/** Controls the use of interframe prediction.
    0=Independent frames
    1=Short term interframe prediction allowed
    2=Long term prediction allowed
 */
#define CELT_SET_PREDICTION(x) CELT_SET_PREDICTION_REQUEST, __opus_check_int(x)

#define CELT_SET_INPUT_CLIPPING_REQUEST    10004
#define CELT_SET_INPUT_CLIPPING(x) CELT_SET_INPUT_CLIPPING_REQUEST, __opus_check_int(x)

#define CELT_GET_AND_CLEAR_ERROR_REQUEST   10007
#define CELT_GET_AND_CLEAR_ERROR(x) \
    ((CeltDecCtl){ .tag = 10007, .payload.ip = (x) })

#define CELT_SET_CHANNELS_REQUEST    10008
#define CELT_SET_CHANNELS(x) \
    ((CeltDecCtl){ .tag = 10008, .payload.i = (x) })


/* Internal */
#define CELT_SET_START_BAND_REQUEST    10010
#define CELT_SET_START_BAND(x) \
    ((CeltDecCtl){ .tag = 10010, .payload.i = (x) })

#define CELT_SET_END_BAND_REQUEST    10012
#define CELT_SET_END_BAND(x) \
    ((CeltDecCtl){ .tag = 10012, .payload.i = (x) })

#define CELT_GET_MODE_REQUEST    10015
/** Get the CELTMode used by an encoder or decoder */
#define CELT_GET_MODE(x) \
    ((CeltDecCtl){ .tag = 10015, .payload.mp = (x) })

#define CELT_SET_SIGNALLING_REQUEST    10016
#define CELT_SET_SIGNALLING(x) \
    ((CeltDecCtl){ .tag = 10016, .payload.i = (x) })

/* CeltDecCtl aliases for OPUS-level requests used with celt_decoder_ctl.
   The original OPUS_* macros in opus_defines.h are unchanged (they are
   still needed as varargs / case-labels in opus_decoder_ctl). */
#define CELT_RESET_STATE \
    ((CeltDecCtl){ .tag = 4028, .payload.i = 0 })
#define CELT_GET_FINAL_RANGE(p) \
    ((CeltDecCtl){ .tag = 4031, .payload.up = (p) })
#define CELT_GET_PITCH(p) \
    ((CeltDecCtl){ .tag = 4033, .payload.ip = (p) })



/* Encoder stuff */

int celt_encoder_get_size(int channels);

int celt_encode_with_ec(OpusCustomEncoder * restrict st, const opus_val16 * pcm, int frame_size, unsigned char *compressed, int nbCompressedBytes, ec_enc *enc);

int celt_encoder_init(CELTEncoder *st, opus_int32 sampling_rate, int channels);



/* Decoder stuff */

int celt_decoder_get_size(int channels);


int celt_decoder_init(CELTDecoder *st, opus_int32 sampling_rate, int channels);

int celt_decode_with_ec(OpusCustomDecoder * restrict st, const unsigned char *data, int len, opus_val16 * restrict pcm, int frame_size, ec_dec *dec);

#define celt_encoder_ctl opus_custom_encoder_ctl

/* celt_decoder_ctl / opus_custom_decoder_ctl: translated to Rust (src/celt.rs) */
int opus_custom_decoder_ctl(CELTDecoder *st, CeltDecCtl request);
int celt_decoder_ctl(CELTDecoder *st, CeltDecCtl request);

#ifdef __cplusplus
}
#endif

#endif /* CELT_H */
