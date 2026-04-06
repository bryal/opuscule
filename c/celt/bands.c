/* Copyright (c) 2007-2012 IETF Trust, CSIRO, Xiph.Org Foundation,
                           Gregory Maxwell. All rights reserved.
   Written by Jean-Marc Valin and Gregory Maxwell */
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include "bands.h"
#include "modes.h"
#include "vq.h"
#include "cwrs.h"
#include "stack_alloc.h"
#include "os_support.h"
#include "mathops.h"
#include "rate.h"

/* This is a cos() approximation designed to be bit-exact on any platform. Bit exactness
   with this approximation is important because it has an impact on the bit allocation */
static opus_int16 bitexact_cos(opus_int16 x)
{
   opus_int32 tmp;
   opus_int16 x2;
   tmp = (4096+((opus_int32)(x)*(x)))>>13;
   celt_assert(tmp<=32767);
   x2 = tmp;
   x2 = (32767-x2) + FRAC_MUL16(x2, (-7651 + FRAC_MUL16(x2, (8277 + FRAC_MUL16(-626, x2)))));
   celt_assert(x2<=32766);
   return 1+x2;
}

static int bitexact_log2tan(int isin,int icos)
{
   int lc;
   int ls;
   lc=EC_ILOG(icos);
   ls=EC_ILOG(isin);
   icos<<=15-lc;
   isin<<=15-ls;
   return (ls-lc)*(1<<11)
         +FRAC_MUL16(isin, FRAC_MUL16(isin, -2597) + 7932)
         -FRAC_MUL16(icos, FRAC_MUL16(icos, -2597) + 7932);
}

#ifdef FIXED_POINT
/* Compute the amplitude (sqrt energy) in each of the bands */
void compute_band_energies(const CELTMode *m, const celt_sig *X, celt_ener *bandE, int end, int C, int M)
{
   int i, c, N;
   const opus_int16 *eBands = m->eBands;
   N = M*m->shortMdctSize;
   c=0; do {
      for (i=0;i<end;i++)
      {
         int j;
         opus_val32 maxval=0;
         opus_val32 sum = 0;

         j=M*eBands[i]; do {
            maxval = MAX32(maxval, X[j+c*N]);
            maxval = MAX32(maxval, -X[j+c*N]);
         } while (++j<M*eBands[i+1]);

         if (maxval > 0)
         {
            int shift = celt_ilog2(maxval)-10;
            j=M*eBands[i]; do {
               sum = MAC16_16(sum, EXTRACT16(VSHR32(X[j+c*N],shift)),
                                   EXTRACT16(VSHR32(X[j+c*N],shift)));
            } while (++j<M*eBands[i+1]);
            /* We're adding one here to ensure the normalized band isn't larger than unity norm */
            bandE[i+c*m->nbEBands] = EPSILON+VSHR32(EXTEND32(celt_sqrt(sum)),-shift);
         } else {
            bandE[i+c*m->nbEBands] = EPSILON;
         }
         /*printf ("%f ", bandE[i+c*m->nbEBands]);*/
      }
   } while (++c<C);
   /*printf ("\n");*/
}

/* Normalise each band such that the energy is one. */
void normalise_bands(const CELTMode *m, const celt_sig * restrict freq, celt_norm * restrict X, const celt_ener *bandE, int end, int C, int M)
{
   int i, c, N;
   const opus_int16 *eBands = m->eBands;
   N = M*m->shortMdctSize;
   c=0; do {
      i=0; do {
         opus_val16 g;
         int j,shift;
         opus_val16 E;
         shift = celt_zlog2(bandE[i+c*m->nbEBands])-13;
         E = VSHR32(bandE[i+c*m->nbEBands], shift);
         g = EXTRACT16(celt_rcp(SHL32(E,3)));
         j=M*eBands[i]; do {
            X[j+c*N] = MULT16_16_Q15(VSHR32(freq[j+c*N],shift-1),g);
         } while (++j<M*eBands[i+1]);
      } while (++i<end);
   } while (++c<C);
}

#else /* FIXED_POINT */
/* Compute the amplitude (sqrt energy) in each of the bands */
void compute_band_energies(const CELTMode *m, const celt_sig *X, celt_ener *bandE, int end, int C, int M)
{
   int i, c, N;
   const opus_int16 *eBands = m->eBands;
   N = M*m->shortMdctSize;
   c=0; do {
      for (i=0;i<end;i++)
      {
         int j;
         opus_val32 sum = 1e-27f;
         for (j=M*eBands[i];j<M*eBands[i+1];j++)
            sum += X[j+c*N]*X[j+c*N];
         bandE[i+c*m->nbEBands] = celt_sqrt(sum);
         /*printf ("%f ", bandE[i+c*m->nbEBands]);*/
      }
   } while (++c<C);
   /*printf ("\n");*/
}

/* Normalise each band such that the energy is one. */
void normalise_bands(const CELTMode *m, const celt_sig * restrict freq, celt_norm * restrict X, const celt_ener *bandE, int end, int C, int M)
{
   int i, c, N;
   const opus_int16 *eBands = m->eBands;
   N = M*m->shortMdctSize;
   c=0; do {
      for (i=0;i<end;i++)
      {
         int j;
         opus_val16 g = 1.f/(1e-27f+bandE[i+c*m->nbEBands]);
         for (j=M*eBands[i];j<M*eBands[i+1];j++)
            X[j+c*N] = freq[j+c*N]*g;
      }
   } while (++c<C);
}

#endif /* FIXED_POINT */


extern void intensity_stereo(const CELTMode *m, celt_norm *X, const celt_norm *Y, const celt_ener *bandE, int bandID, int N);
extern void stereo_split(celt_norm *X, celt_norm *Y, int N);
extern void stereo_merge(celt_norm *X, celt_norm *Y, opus_val16 mid, int N);

/* Decide whether we should spread the pulses in the current frame */
int spreading_decision(const CELTMode *m, celt_norm *X, int *average,
      int last_decision, int *hf_average, int *tapset_decision, int update_hf,
      int end, int C, int M)
{
   int i, c, N0;
   int sum = 0, nbBands=0;
   const opus_int16 * restrict eBands = m->eBands;
   int decision;
   int hf_sum=0;

   celt_assert(end>0);

   N0 = M*m->shortMdctSize;

   if (M*(eBands[end]-eBands[end-1]) <= 8)
      return SPREAD_NONE;
   c=0; do {
      for (i=0;i<end;i++)
      {
         int j, N, tmp=0;
         int tcount[3] = {0,0,0};
         celt_norm * restrict x = X+M*eBands[i]+c*N0;
         N = M*(eBands[i+1]-eBands[i]);
         if (N<=8)
            continue;
         /* Compute rough CDF of |x[j]| */
         for (j=0;j<N;j++)
         {
            opus_val32 x2N; /* Q13 */

            x2N = MULT16_16(MULT16_16_Q15(x[j], x[j]), N);
            if (x2N < QCONST16(0.25f,13))
               tcount[0]++;
            if (x2N < QCONST16(0.0625f,13))
               tcount[1]++;
            if (x2N < QCONST16(0.015625f,13))
               tcount[2]++;
         }

         /* Only include four last bands (8 kHz and up) */
         if (i>m->nbEBands-4)
            hf_sum += 32*(tcount[1]+tcount[0])/N;
         tmp = (2*tcount[2] >= N) + (2*tcount[1] >= N) + (2*tcount[0] >= N);
         sum += tmp*256;
         nbBands++;
      }
   } while (++c<C);

   if (update_hf)
   {
      if (hf_sum)
         hf_sum /= C*(4-m->nbEBands+end);
      *hf_average = (*hf_average+hf_sum)>>1;
      hf_sum = *hf_average;
      if (*tapset_decision==2)
         hf_sum += 4;
      else if (*tapset_decision==0)
         hf_sum -= 4;
      if (hf_sum > 22)
         *tapset_decision=2;
      else if (hf_sum > 18)
         *tapset_decision=1;
      else
         *tapset_decision=0;
   }
   /*printf("%d %d %d\n", hf_sum, *hf_average, *tapset_decision);*/
   celt_assert(nbBands>0); /*M*(eBands[end]-eBands[end-1]) <= 8 assures this*/
   sum /= nbBands;
   /* Recursive averaging */
   sum = (sum+*average)>>1;
   *average = sum;
   /* Hysteresis */
   sum = (3*sum + (((3-last_decision)<<7) + 64) + 2)>>2;
   if (sum < 80)
   {
      decision = SPREAD_AGGRESSIVE;
   } else if (sum < 256)
   {
      decision = SPREAD_NORMAL;
   } else if (sum < 384)
   {
      decision = SPREAD_LIGHT;
   } else {
      decision = SPREAD_NONE;
   }
#ifdef FUZZING
   decision = rand()&0x3;
   *tapset_decision=rand()%3;
#endif
   return decision;
}

#ifdef MEASURE_NORM_MSE

float MSE[30] = {0};
int nbMSEBands = 0;
int MSECount[30] = {0};

void dump_norm_mse(void)
{
   int i;
   for (i=0;i<nbMSEBands;i++)
   {
      printf ("%g ", MSE[i]/MSECount[i]);
   }
   printf ("\n");
}

void measure_norm_mse(const CELTMode *m, float *X, float *X0, float *bandE, float *bandE0, int M, int N, int C)
{
   static int init = 0;
   int i;
   if (!init)
   {
      atexit(dump_norm_mse);
      init = 1;
   }
   for (i=0;i<m->nbEBands;i++)
   {
      int j;
      int c;
      float g;
      if (bandE0[i]<10 || (C==2 && bandE0[i+m->nbEBands]<1))
         continue;
      c=0; do {
         g = bandE[i+c*m->nbEBands]/(1e-15+bandE0[i+c*m->nbEBands]);
         for (j=M*m->eBands[i];j<M*m->eBands[i+1];j++)
            MSE[i] += (g*X[j+c*N]-X0[j+c*N])*(g*X[j+c*N]-X0[j+c*N]);
      } while (++c<C);
      MSECount[i]+=C;
   }
   nbMSEBands = m->nbEBands;
}

#endif

extern void deinterleave_hadamard(celt_norm *X, int N0, int stride, int hadamard);
extern void interleave_hadamard(celt_norm *X, int N0, int stride, int hadamard);
extern int compute_qn(int N, int b, int offset, int pulse_cap, int stereo);

extern unsigned quant_band(int encode, const CELTMode *m, int i, celt_norm *X, celt_norm *Y,
      int N, int b, int spread, int B, int intensity, int tf_change, celt_norm *lowband, ec_ctx *ec,
      opus_int32 *remaining_bits, int LM, celt_norm *lowband_out, const celt_ener *bandE, int level,
      opus_uint32 *seed, opus_val16 gain, celt_norm *lowband_scratch, int fill);
extern void quant_all_bands(int encode, const CELTMode *m, int start, int end,
      celt_norm *X_, celt_norm *Y_, unsigned char *collapse_masks, const celt_ener *bandE, int *pulses,
      int shortBlocks, int spread, int dual_stereo, int intensity, int *tf_res,
      opus_int32 total_bits, opus_int32 balance, ec_ctx *ec, int LM, int codedBands, opus_uint32 *seed);

