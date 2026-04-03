/* Copyright (c) 2007-2012 IETF Trust, CSIRO, Xiph.Org Foundation. All rights reserved.
   Written by Jean-Marc Valin */
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

/* This is a simple MDCT implementation that uses a N/4 complex FFT
   to do most of the work. It should be relatively straightforward to
   plug in pretty much and FFT here.

   This replaces the Vorbis FFT (and uses the exact same API), which
   was a bit too messy and that was ending up duplicating code
   (might as well use the same FFT everywhere).

   The algorithm is similar to (and inspired from) Fabrice Bellard's
   MDCT implementation in FFMPEG, but has differences in signs, ordering
   and scaling in many places.
*/

#ifndef SKIP_CONFIG_H
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#endif

#include "mdct.h"
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"
#include <math.h>
#include "os_support.h"
#include "mathops.h"
#include "stack_alloc.h"

#ifdef CUSTOM_MODES

int clt_mdct_init(mdct_lookup *l,int N, int maxshift)
{
   int i;
   int N4;
   kiss_twiddle_scalar *trig;
#if defined(FIXED_POINT)
   int N2=N>>1;
#endif
   l->n = N;
   N4 = N>>2;
   l->maxshift = maxshift;
   for (i=0;i<=maxshift;i++)
   {
      if (i==0)
         l->kfft[i] = opus_fft_alloc(N>>2>>i, 0, 0);
      else
         l->kfft[i] = opus_fft_alloc_twiddles(N>>2>>i, 0, 0, l->kfft[0]);
#ifndef ENABLE_TI_DSPLIB55
      if (l->kfft[i]==NULL)
         return 0;
#endif
   }
   l->trig = trig = (kiss_twiddle_scalar*)opus_alloc((N4+1)*sizeof(kiss_twiddle_scalar));
   if (l->trig==NULL)
     return 0;
   /* We have enough points that sine isn't necessary */
#if defined(FIXED_POINT)
   for (i=0;i<=N4;i++)
      trig[i] = TRIG_UPSCALE*celt_cos_norm(DIV32(ADD32(SHL32(EXTEND32(i),17),N2),N));
#else
   for (i=0;i<=N4;i++)
      trig[i] = (kiss_twiddle_scalar)cos(2*PI*i/N);
#endif
   return 1;
}

void clt_mdct_clear(mdct_lookup *l)
{
   int i;
   for (i=0;i<=l->maxshift;i++)
      opus_fft_free(l->kfft[i]);
   opus_free((kiss_twiddle_scalar*)l->trig);
}

#endif /* CUSTOM_MODES */

/* clt_mdct_forward: encoder-only, removed in decoder-only build */
/* clt_mdct_backward: translated to Rust (src/mdct.rs) */
