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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include "modes.h"
#include "cwrs.h"
#include "arch.h"
#include "os_support.h"

#include "entcode.h"
#include "rate.h"

static const unsigned char LOG2_FRAC_TABLE[24]={
   0,
   8,13,
  16,19,21,23,
  24,26,27,28,29,30,31,32,
  32,33,34,34,35,36,36,37,37
};

#ifdef CUSTOM_MODES

/*Determines if V(N,K) fits in a 32-bit unsigned integer.
  N and K are themselves limited to 15 bits.*/
static int fits_in32(int _n, int _k)
{
   static const opus_int16 maxN[15] = {
      32767, 32767, 32767, 1476, 283, 109,  60,  40,
       29,  24,  20,  18,  16,  14,  13};
   static const opus_int16 maxK[15] = {
      32767, 32767, 32767, 32767, 1172, 238,  95,  53,
       36,  27,  22,  18,  16,  15,  13};
   if (_n>=14)
   {
      if (_k>=14)
         return 0;
      else
         return _n <= maxN[_k];
   } else {
      return _k <= maxK[_n];
   }
}

void compute_pulse_cache(CELTMode *m, int LM)
{
   int C;
   int i;
   int j;
   int curr=0;
   int nbEntries=0;
   int entryN[100], entryK[100], entryI[100];
   const opus_int16 *eBands = m->eBands;
   PulseCache *cache = &m->cache;
   opus_int16 *cindex;
   unsigned char *bits;
   unsigned char *cap;

   cindex = opus_alloc(sizeof(cache->index[0])*m->nbEBands*(LM+2));
   cache->index = cindex;

   /* Scan for all unique band sizes */
   for (i=0;i<=LM+1;i++)
   {
      for (j=0;j<m->nbEBands;j++)
      {
         int k;
         int N = (eBands[j+1]-eBands[j])<<i>>1;
         cindex[i*m->nbEBands+j] = -1;
         /* Find other bands that have the same size */
         for (k=0;k<=i;k++)
         {
            int n;
            for (n=0;n<m->nbEBands && (k!=i || n<j);n++)
            {
               if (N == (eBands[n+1]-eBands[n])<<k>>1)
               {
                  cindex[i*m->nbEBands+j] = cindex[k*m->nbEBands+n];
                  break;
               }
            }
         }
         if (cache->index[i*m->nbEBands+j] == -1 && N!=0)
         {
            int K;
            entryN[nbEntries] = N;
            K = 0;
            while (fits_in32(N,get_pulses(K+1)) && K<MAX_PSEUDO)
               K++;
            entryK[nbEntries] = K;
            cindex[i*m->nbEBands+j] = curr;
            entryI[nbEntries] = curr;

            curr += K+1;
            nbEntries++;
         }
      }
   }
   bits = opus_alloc(sizeof(unsigned char)*curr);
   cache->bits = bits;
   cache->size = curr;
   /* Compute the cache for all unique sizes */
   for (i=0;i<nbEntries;i++)
   {
      unsigned char *ptr = bits+entryI[i];
      opus_int16 tmp[MAX_PULSES+1];
      get_required_bits(tmp, entryN[i], get_pulses(entryK[i]), BITRES);
      for (j=1;j<=entryK[i];j++)
         ptr[j] = tmp[get_pulses(j)]-1;
      ptr[0] = entryK[i];
   }

   /* Compute the maximum rate for each band at which we'll reliably use as
       many bits as we ask for. */
   cache->caps = cap = opus_alloc(sizeof(cache->caps[0])*(LM+1)*2*m->nbEBands);
   for (i=0;i<=LM;i++)
   {
      for (C=1;C<=2;C++)
      {
         for (j=0;j<m->nbEBands;j++)
         {
            int N0;
            int max_bits;
            N0 = m->eBands[j+1]-m->eBands[j];
            /* N=1 bands only have a sign bit and fine bits. */
            if (N0<<i == 1)
               max_bits = C*(1+MAX_FINE_BITS)<<BITRES;
            else
            {
               const unsigned char *pcache;
               opus_int32           num;
               opus_int32           den;
               int                  LM0;
               int                  N;
               int                  offset;
               int                  ndof;
               int                  qb;
               int                  k;
               LM0 = 0;
               /* Even-sized bands bigger than N=2 can be split one more time.
                  As of commit 44203907 all bands >1 are even, including custom modes.*/
               if (N0 > 2)
               {
                  N0>>=1;
                  LM0--;
               }
               /* N0=1 bands can't be split down to N<2. */
               else if (N0 <= 1)
               {
                  LM0=IMIN(i,1);
                  N0<<=LM0;
               }
               /* Compute the cost for the lowest-level PVQ of a fully split
                   band. */
               pcache = bits + cindex[(LM0+1)*m->nbEBands+j];
               max_bits = pcache[pcache[0]]+1;
               /* Add in the cost of coding regular splits. */
               N = N0;
               for(k=0;k<i-LM0;k++){
                  max_bits <<= 1;
                  /* Offset the number of qtheta bits by log2(N)/2
                      + QTHETA_OFFSET compared to their "fair share" of
                      total/N */
                  offset = ((m->logN[j]+((LM0+k)<<BITRES))>>1)-QTHETA_OFFSET;
                  /* The number of qtheta bits we'll allocate if the remainder
                      is to be max_bits.
                     The average measured cost for theta is 0.89701 times qb,
                      approximated here as 459/512. */
                  num=459*(opus_int32)((2*N-1)*offset+max_bits);
                  den=((opus_int32)(2*N-1)<<9)-459;
                  qb = IMIN((num+(den>>1))/den, 57);
                  celt_assert(qb >= 0);
                  max_bits += qb;
                  N <<= 1;
               }
               /* Add in the cost of a stereo split, if necessary. */
               if (C==2)
               {
                  max_bits <<= 1;
                  offset = ((m->logN[j]+(i<<BITRES))>>1)-(N==2?QTHETA_OFFSET_TWOPHASE:QTHETA_OFFSET);
                  ndof = 2*N-1-(N==2);
                  /* The average measured cost for theta with the step PDF is
                      0.95164 times qb, approximated here as 487/512. */
                  num = (N==2?512:487)*(opus_int32)(max_bits+ndof*offset);
                  den = ((opus_int32)ndof<<9)-(N==2?512:487);
                  qb = IMIN((num+(den>>1))/den, (N==2?64:61));
                  celt_assert(qb >= 0);
                  max_bits += qb;
               }
               /* Add the fine bits we'll use. */
               /* Compensate for the extra DoF in stereo */
               ndof = C*N + ((C==2 && N>2) ? 1 : 0);
               /* Offset the number of fine bits by log2(N)/2 + FINE_OFFSET
                   compared to their "fair share" of total/N */
               offset = ((m->logN[j] + (i<<BITRES))>>1)-FINE_OFFSET;
               /* N=2 is the only point that doesn't match the curve */
               if (N==2)
                  offset += 1<<BITRES>>2;
               /* The number of fine bits we'll allocate if the remainder is
                   to be max_bits. */
               num = max_bits+ndof*offset;
               den = (ndof-1)<<BITRES;
               qb = IMIN((num+(den>>1))/den, MAX_FINE_BITS);
               celt_assert(qb >= 0);
               max_bits += C*qb<<BITRES;
            }
            max_bits = (4*max_bits/(C*((m->eBands[j+1]-m->eBands[j])<<i)))-64;
            celt_assert(max_bits >= 0);
            celt_assert(max_bits < 256);
            *cap++ = (unsigned char)max_bits;
         }
      }
   }
}

#endif /* CUSTOM_MODES */

/* interp_bits2pulses + compute_allocation: translated to Rust in src/rate.rs */
