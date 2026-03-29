/* Copyright (c) 2007-2012 IETF Trust, CSIRO, Xiph.Org Foundation,
                           Timothy B. Terriberry. All rights reserved.
   Written by Timothy B. Terriberry and Jean-Marc Valin */
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

/* decode_pulses: translated to Rust in src/cwrs.rs */

/* Encoder-side functions retained below because vq.c still calls
   encode_pulses from C. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "os_support.h"
#include "cwrs.h"
#include "mathops.h"
#include "arch.h"

#ifndef SMALL_FOOTPRINT

#define MASK32 (0xFFFFFFFF)

static const opus_uint32 INV_TABLE[53]={
  0x00000001,0xAAAAAAAB,0xCCCCCCCD,0xB6DB6DB7,
  0x38E38E39,0xBA2E8BA3,0xC4EC4EC5,0xEEEEEEEF,
  0xF0F0F0F1,0x286BCA1B,0x3CF3CF3D,0xE9BD37A7,
  0xC28F5C29,0x684BDA13,0x4F72C235,0xBDEF7BDF,
  0x3E0F83E1,0x8AF8AF8B,0x914C1BAD,0x96F96F97,
  0xC18F9C19,0x2FA0BE83,0xA4FA4FA5,0x677D46CF,
  0x1A1F58D1,0xFAFAFAFB,0x8C13521D,0x586FB587,
  0xB823EE09,0xA08AD8F3,0xC10C9715,0xBEFBEFBF,
  0xC0FC0FC1,0x07A44C6B,0xA33F128D,0xE327A977,
  0xC7E3F1F9,0x962FC963,0x3F2B3885,0x613716AF,
  0x781948B1,0x2B2E43DB,0xFCFCFCFD,0x6FD0EB67,
  0xFA3F47E9,0xD2FD2FD3,0x3F4FD3F5,0xD4E25B9F,
  0x5F02A3A1,0xBF5A814B,0x7C32B16D,0xD3431B57,
  0xD8FD8FD9,
};

static inline opus_uint32 imusdiv32odd(opus_uint32 _a,opus_uint32 _b,
 opus_uint32 _c,int _d){
  celt_assert(_d<=52);
  return (_a*_b-_c)*INV_TABLE[_d]&MASK32;
}

static inline unsigned ucwrs2(unsigned _k){
  celt_assert(_k>0);
  return _k+(_k-1);
}

static inline opus_uint32 ncwrs2(int _k){
  celt_assert(_k>0);
  return 4*(opus_uint32)_k;
}

static inline opus_uint32 ucwrs3(unsigned _k){
  celt_assert(_k>0);
  return (2*(opus_uint32)_k-2)*_k+1;
}

static inline opus_uint32 ncwrs3(int _k){
  celt_assert(_k>0);
  return 2*(2*(unsigned)_k*(opus_uint32)_k+1);
}

static inline opus_uint32 ucwrs4(int _k){
  celt_assert(_k>0);
  return imusdiv32odd(2*_k,(2*_k-3)*(opus_uint32)_k+4,3,1);
}

static inline opus_uint32 ncwrs4(int _k){
  celt_assert(_k>0);
  return ((_k*(opus_uint32)_k+2)*_k)/3<<3;
}

#endif /* SMALL_FOOTPRINT */

static inline opus_uint32 icwrs1(const int *_y,int *_k){
  *_k=abs(_y[0]);
  return _y[0]<0;
}

#ifndef SMALL_FOOTPRINT

static inline opus_uint32 icwrs2(const int *_y,int *_k){
  opus_uint32 i;
  int           k;
  i=icwrs1(_y+1,&k);
  i+=k?ucwrs2(k):0;
  k+=abs(_y[0]);
  if(_y[0]<0)i+=ucwrs2(k+1U);
  *_k=k;
  return i;
}

static inline opus_uint32 icwrs3(const int *_y,int *_k){
  opus_uint32 i;
  int           k;
  i=icwrs2(_y+1,&k);
  i+=k?ucwrs3(k):0;
  k+=abs(_y[0]);
  if(_y[0]<0)i+=ucwrs3(k+1U);
  *_k=k;
  return i;
}

static inline opus_uint32 icwrs4(const int *_y,int *_k){
  opus_uint32 i;
  int           k;
  i=icwrs3(_y+1,&k);
  i+=k?ucwrs4(k):0;
  k+=abs(_y[0]);
  if(_y[0]<0)i+=ucwrs4(k+1);
  *_k=k;
  return i;
}

#endif /* SMALL_FOOTPRINT */

static inline void unext(opus_uint32 *_ui,unsigned _len,opus_uint32 _ui0){
  opus_uint32 ui1;
  unsigned      j;
  j=1; do {
    ui1=UADD32(UADD32(_ui[j],_ui[j-1]),_ui0);
    _ui[j-1]=_ui0;
    _ui0=ui1;
  } while (++j<_len);
  _ui[j-1]=_ui0;
}

static inline opus_uint32 icwrs(int _n,int _k,opus_uint32 *_nc,const int *_y,
 opus_uint32 *_u){
  opus_uint32 i;
  int           j;
  int           k;
  celt_assert(_n>=2);
  _u[0]=0;
  for(k=1;k<=_k+1;k++)_u[k]=(k<<1)-1;
  i=icwrs1(_y+_n-1,&k);
  j=_n-2;
  i+=_u[k];
  k+=abs(_y[j]);
  if(_y[j]<0)i+=_u[k+1];
  while(j-->0){
    unext(_u,_k+2,0);
    i+=_u[k];
    k+=abs(_y[j]);
    if(_y[j]<0)i+=_u[k+1];
  }
  *_nc=_u[k]+_u[k+1];
  return i;
}

void encode_pulses(const int *_y,int _n,int _k,ec_enc *_enc){
  opus_uint32 i;
  celt_assert(_k>0);
#ifndef SMALL_FOOTPRINT
  switch(_n){
    case 2:{
      i=icwrs2(_y,&_k);
      ec_enc_uint(_enc,i,ncwrs2(_k));
    }break;
    case 3:{
      i=icwrs3(_y,&_k);
      ec_enc_uint(_enc,i,ncwrs3(_k));
    }break;
    case 4:{
      i=icwrs4(_y,&_k);
      ec_enc_uint(_enc,i,ncwrs4(_k));
    }break;
     default:
    {
#endif
      VARDECL(opus_uint32,u);
      opus_uint32 nc;
      SAVE_STACK;
      ALLOC(u,_k+2U,opus_uint32);
      i=icwrs(_n,_k,&nc,_y,u);
      ec_enc_uint(_enc,i,nc);
      RESTORE_STACK;
#ifndef SMALL_FOOTPRINT
    }
    break;
  }
#endif
}
