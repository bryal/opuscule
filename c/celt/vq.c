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

/* alg_unquant, renormalise_vector, stereo_itheta: translated to Rust in src/vq.rs */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mathops.h"
#include "cwrs.h"
#include "vq.h"
#include "arch.h"
#include "os_support.h"
#include "bands.h"
#include "rate.h"

static void exp_rotation1(celt_norm *X, int len, int stride, opus_val16 c, opus_val16 s)
{
   int i;
   celt_norm *Xptr;
   Xptr = X;
   for (i=0;i<len-stride;i++)
   {
      celt_norm x1, x2;
      x1 = Xptr[0];
      x2 = Xptr[stride];
      Xptr[stride] = EXTRACT16(SHR32(MULT16_16(c,x2) + MULT16_16(s,x1), 15));
      *Xptr++      = EXTRACT16(SHR32(MULT16_16(c,x1) - MULT16_16(s,x2), 15));
   }
   Xptr = &X[len-2*stride-1];
   for (i=len-2*stride-1;i>=0;i--)
   {
      celt_norm x1, x2;
      x1 = Xptr[0];
      x2 = Xptr[stride];
      Xptr[stride] = EXTRACT16(SHR32(MULT16_16(c,x2) + MULT16_16(s,x1), 15));
      *Xptr--      = EXTRACT16(SHR32(MULT16_16(c,x1) - MULT16_16(s,x2), 15));
   }
}

static void exp_rotation(celt_norm *X, int len, int dir, int stride, int K, int spread)
{
   static const int SPREAD_FACTOR[3]={15,10,5};
   int i;
   opus_val16 c, s;
   opus_val16 gain, theta;
   int stride2=0;
   int factor;

   if (2*K>=len || spread==SPREAD_NONE)
      return;
   factor = SPREAD_FACTOR[spread-1];

   gain = celt_div((opus_val32)MULT16_16(Q15_ONE,len),(opus_val32)(len+factor*K));
   theta = HALF16(MULT16_16_Q15(gain,gain));

   c = celt_cos_norm(EXTEND32(theta));
   s = celt_cos_norm(EXTEND32(SUB16(Q15ONE,theta))); /*  sin(theta) */

   if (len>=8*stride)
   {
      stride2 = 1;
      while ((stride2*stride2+stride2)*stride + (stride>>2) < len)
         stride2++;
   }
   len /= stride;
   for (i=0;i<stride;i++)
   {
      if (dir < 0)
      {
         if (stride2)
            exp_rotation1(X+i*len, len, stride2, s, c);
         exp_rotation1(X+i*len, len, 1, c, s);
      } else {
         exp_rotation1(X+i*len, len, 1, c, -s);
         if (stride2)
            exp_rotation1(X+i*len, len, stride2, s, -c);
      }
   }
}

static unsigned extract_collapse_mask(int *iy, int N, int B)
{
   unsigned collapse_mask;
   int N0;
   int i;
   if (B<=1)
      return 1;
   N0 = N/B;
   collapse_mask = 0;
   i=0; do {
      int j;
      j=0; do {
         collapse_mask |= (iy[i*N0+j]!=0)<<i;
      } while (++j<N0);
   } while (++i<B);
   return collapse_mask;
}

unsigned alg_quant(celt_norm *X, int N, int K, int spread, int B, ec_enc *enc
#ifdef RESYNTH
   , opus_val16 gain
#endif
   )
{
   VARDECL(celt_norm, y);
   VARDECL(int, iy);
   VARDECL(opus_val16, signx);
   int i, j;
   opus_val16 s;
   int pulsesLeft;
   opus_val32 sum;
   opus_val32 xy;
   opus_val16 yy;
   unsigned collapse_mask;
   SAVE_STACK;

   celt_assert2(K>0, "alg_quant() needs at least one pulse");
   celt_assert2(N>1, "alg_quant() needs at least two dimensions");

   ALLOC(y, N, celt_norm);
   ALLOC(iy, N, int);
   ALLOC(signx, N, opus_val16);

   exp_rotation(X, N, 1, B, K, spread);

   /* Get rid of the sign */
   sum = 0;
   j=0; do {
      if (X[j]>0)
         signx[j]=1;
      else {
         signx[j]=-1;
         X[j]=-X[j];
      }
      iy[j] = 0;
      y[j] = 0;
   } while (++j<N);

   xy = yy = 0;

   pulsesLeft = K;

   /* Do a pre-search by projecting on the pyramid */
   if (K > (N>>1))
   {
      opus_val16 rcp;
      j=0; do {
         sum += X[j];
      }  while (++j<N);

      /* If X is too small, just replace it with a pulse at 0 */
#ifdef FIXED_POINT
      if (sum <= K)
#else
      /* Prevents infinities and NaNs from causing too many pulses
         to be allocated. 64 is an approximation of infinity here. */
      if (!(sum > EPSILON && sum < 64))
#endif
      {
         X[0] = QCONST16(1.f,14);
         j=1; do
            X[j]=0;
         while (++j<N);
         sum = QCONST16(1.f,14);
      }
      rcp = EXTRACT16(MULT16_32_Q16(K-1, celt_rcp(sum)));
      j=0; do {
#ifdef FIXED_POINT
         /* It's really important to round *towards zero* here */
         iy[j] = MULT16_16_Q15(X[j],rcp);
#else
         iy[j] = (int)floor(rcp*X[j]);
#endif
         y[j] = (celt_norm)iy[j];
         yy = MAC16_16(yy, y[j],y[j]);
         xy = MAC16_16(xy, X[j],y[j]);
         y[j] *= 2;
         pulsesLeft -= iy[j];
      }  while (++j<N);
   }
   celt_assert2(pulsesLeft>=1, "Allocated too many pulses in the quick pass");

#ifdef FIXED_POINT_DEBUG
   celt_assert2(pulsesLeft<=N+3, "Not enough pulses in the quick pass");
#endif
   if (pulsesLeft > N+3)
   {
      opus_val16 tmp = (opus_val16)pulsesLeft;
      yy = MAC16_16(yy, tmp, tmp);
      yy = MAC16_16(yy, tmp, y[0]);
      iy[0] += pulsesLeft;
      pulsesLeft=0;
   }

   s = 1;
   for (i=0;i<pulsesLeft;i++)
   {
      int best_id;
      opus_val32 best_num = -VERY_LARGE16;
      opus_val16 best_den = 0;
#ifdef FIXED_POINT
      int rshift;
#endif
#ifdef FIXED_POINT
      rshift = 1+celt_ilog2(K-pulsesLeft+i+1);
#endif
      best_id = 0;
      yy = ADD32(yy, 1);
      j=0;
      do {
         opus_val16 Rxy, Ryy;
         Rxy = EXTRACT16(SHR32(ADD32(xy, EXTEND32(X[j])),rshift));
         Ryy = ADD16(yy, y[j]);
         Rxy = MULT16_16_Q15(Rxy,Rxy);
         if (MULT16_16(best_den, Rxy) > MULT16_16(Ryy, best_num))
         {
            best_den = Ryy;
            best_num = Rxy;
            best_id = j;
         }
      } while (++j<N);

      xy = ADD32(xy, EXTEND32(X[best_id]));
      yy = ADD16(yy, y[best_id]);
      y[best_id] += 2*s;
      iy[best_id]++;
   }

   /* Put the original sign back */
   j=0;
   do {
      X[j] = MULT16_16(signx[j],X[j]);
      if (signx[j] < 0)
         iy[j] = -iy[j];
   } while (++j<N);
   encode_pulses(iy, N, K, enc);

#ifdef RESYNTH
   {
      /* normalise_residual inlined here since it's only needed for RESYNTH */
      int ii;
#ifdef FIXED_POINT
      int k;
#endif
      opus_val32 t;
      opus_val16 g;
#ifdef FIXED_POINT
      k = celt_ilog2(yy)>>1;
#endif
      t = VSHR32(yy, 2*(k-7));
      g = MULT16_16_P15(celt_rsqrt_norm(t),gain);
      ii=0;
      do
         X[ii] = EXTRACT16(PSHR32(MULT16_16(g, iy[ii]), k+1));
      while (++ii < N);
      exp_rotation(X, N, -1, B, K, spread);
   }
#endif

   collapse_mask = extract_collapse_mask(iy, N, B);
   RESTORE_STACK;
   return collapse_mask;
}
