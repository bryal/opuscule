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

#include "quant_bands.h"
#include "laplace.h"
#include <math.h>
#include "os_support.h"
#include "arch.h"
#include "mathops.h"
#include "stack_alloc.h"
#include "rate.h"

#ifdef FIXED_POINT
/* Mean energy in each band quantized in Q6 */
static const signed char eMeans[25] = {
      103,100, 92, 85, 81,
       77, 72, 70, 78, 75,
       73, 71, 78, 74, 69,
       72, 70, 74, 76, 71,
       60, 60, 60, 60, 60
};
#else
/* Mean energy in each band quantized in Q6 and converted back to float */
static const opus_val16 eMeans[25] = {
      6.437500f, 6.250000f, 5.750000f, 5.312500f, 5.062500f,
      4.812500f, 4.500000f, 4.375000f, 4.875000f, 4.687500f,
      4.562500f, 4.437500f, 4.875000f, 4.625000f, 4.312500f,
      4.500000f, 4.375000f, 4.625000f, 4.750000f, 4.437500f,
      3.750000f, 3.750000f, 3.750000f, 3.750000f, 3.750000f
};
#endif
/* prediction coefficients: 0.9, 0.8, 0.65, 0.5 */
#ifdef FIXED_POINT
static const opus_val16 pred_coef[4] = {29440, 26112, 21248, 16384};
static const opus_val16 beta_coef[4] = {30147, 22282, 12124, 6554};
static const opus_val16 beta_intra = 4915;
#else
static const opus_val16 pred_coef[4] = {29440/32768., 26112/32768., 21248/32768., 16384/32768.};
static const opus_val16 beta_coef[4] = {30147/32768., 22282/32768., 12124/32768., 6554/32768.};
static const opus_val16 beta_intra = 4915/32768.;
#endif

/*Parameters of the Laplace-like probability models used for the coarse energy.
  There is one pair of parameters for each frame size, prediction type
   (inter/intra), and band number.
  The first number of each pair is the probability of 0, and the second is the
   decay rate, both in Q8 precision.*/
static const unsigned char e_prob_model[4][2][42] = {
   /*120 sample frames.*/
   {
      /*Inter*/
      {
          72, 127,  65, 129,  66, 128,  65, 128,  64, 128,  62, 128,  64, 128,
          64, 128,  92,  78,  92,  79,  92,  78,  90,  79, 116,  41, 115,  40,
         114,  40, 132,  26, 132,  26, 145,  17, 161,  12, 176,  10, 177,  11
      },
      /*Intra*/
      {
          24, 179,  48, 138,  54, 135,  54, 132,  53, 134,  56, 133,  55, 132,
          55, 132,  61, 114,  70,  96,  74,  88,  75,  88,  87,  74,  89,  66,
          91,  67, 100,  59, 108,  50, 120,  40, 122,  37,  97,  43,  78,  50
      }
   },
   /*240 sample frames.*/
   {
      /*Inter*/
      {
          83,  78,  84,  81,  88,  75,  86,  74,  87,  71,  90,  73,  93,  74,
          93,  74, 109,  40, 114,  36, 117,  34, 117,  34, 143,  17, 145,  18,
         146,  19, 162,  12, 165,  10, 178,   7, 189,   6, 190,   8, 177,   9
      },
      /*Intra*/
      {
          23, 178,  54, 115,  63, 102,  66,  98,  69,  99,  74,  89,  71,  91,
          73,  91,  78,  89,  86,  80,  92,  66,  93,  64, 102,  59, 103,  60,
         104,  60, 117,  52, 123,  44, 138,  35, 133,  31,  97,  38,  77,  45
      }
   },
   /*480 sample frames.*/
   {
      /*Inter*/
      {
          61,  90,  93,  60, 105,  42, 107,  41, 110,  45, 116,  38, 113,  38,
         112,  38, 124,  26, 132,  27, 136,  19, 140,  20, 155,  14, 159,  16,
         158,  18, 170,  13, 177,  10, 187,   8, 192,   6, 175,   9, 159,  10
      },
      /*Intra*/
      {
          21, 178,  59, 110,  71,  86,  75,  85,  84,  83,  91,  66,  88,  73,
          87,  72,  92,  75,  98,  72, 105,  58, 107,  54, 115,  52, 114,  55,
         112,  56, 129,  51, 132,  40, 150,  33, 140,  29,  98,  35,  77,  42
      }
   },
   /*960 sample frames.*/
   {
      /*Inter*/
      {
          42, 121,  96,  66, 108,  43, 111,  40, 117,  44, 123,  32, 120,  36,
         119,  33, 127,  33, 134,  34, 139,  21, 147,  23, 152,  20, 158,  25,
         154,  26, 166,  21, 173,  16, 184,  13, 184,  10, 150,  13, 139,  15
      },
      /*Intra*/
      {
          22, 178,  63, 114,  74,  82,  84,  83,  92,  82, 103,  62,  96,  72,
          96,  67, 101,  73, 107,  72, 113,  55, 118,  52, 125,  52, 118,  52,
         117,  55, 135,  49, 137,  39, 157,  32, 145,  29,  97,  33,  77,  40
      }
   }
};

static const unsigned char small_energy_icdf[3]={2,1,0};

/* Encoder functions (loss_distortion, quant_coarse_energy_impl,
   quant_coarse_energy, quant_fine_energy, quant_energy_finalise)
   removed — decoder-only build. */

void unquant_coarse_energy(const CELTMode *m, int start, int end, opus_val16 *oldEBands, int intra, ec_dec *dec, int C, int LM)
{
   const unsigned char *prob_model = e_prob_model[LM][intra];
   int i, c;
   opus_val32 prev[2] = {0, 0};
   opus_val16 coef;
   opus_val16 beta;
   opus_int32 budget;
   opus_int32 tell;

   if (intra)
   {
      coef = 0;
      beta = beta_intra;
   } else {
      beta = beta_coef[LM];
      coef = pred_coef[LM];
   }

   budget = dec->storage*8;

   /* Decode at a fixed coarse resolution */
   for (i=start;i<end;i++)
   {
      c=0;
      do {
         int qi;
         opus_val32 q;
         opus_val32 tmp;
         /* It would be better to express this invariant as a
            test on C at function entry, but that isn't enough
            to make the static analyzer happy. */
         celt_assert(c<2);
         tell = ec_tell(dec);
         if(budget-tell>=15)
         {
            int pi;
            pi = 2*IMIN(i,20);
            qi = ec_laplace_decode(dec,
                  prob_model[pi]<<7, prob_model[pi+1]<<6);
         }
         else if(budget-tell>=2)
         {
            qi = ec_dec_icdf(dec, small_energy_icdf, 2);
            qi = (qi>>1)^-(qi&1);
         }
         else if(budget-tell>=1)
         {
            qi = -ec_dec_bit_logp(dec, 1);
         }
         else
            qi = -1;
         q = (opus_val32)SHL32(EXTEND32(qi),DB_SHIFT);

         oldEBands[i+c*m->nbEBands] = MAX16(-QCONST16(9.f,DB_SHIFT), oldEBands[i+c*m->nbEBands]);
         tmp = PSHR32(MULT16_16(coef,oldEBands[i+c*m->nbEBands]),8) + prev[c] + SHL32(q,7);
#ifdef FIXED_POINT
         tmp = MAX32(-QCONST32(28.f, DB_SHIFT+7), tmp);
#endif
         oldEBands[i+c*m->nbEBands] = PSHR32(tmp, 7);
         prev[c] = prev[c] + SHL32(q,7) - MULT16_16(beta,PSHR32(q,8));
      } while (++c < C);
   }
}

void unquant_fine_energy(const CELTMode *m, int start, int end, opus_val16 *oldEBands, int *fine_quant, ec_dec *dec, int C)
{
   int i, c;
   /* Decode finer resolution */
   for (i=start;i<end;i++)
   {
      if (fine_quant[i] <= 0)
         continue;
      c=0;
      do {
         int q2;
         opus_val16 offset;
         q2 = ec_dec_bits(dec, fine_quant[i]);
#ifdef FIXED_POINT
         offset = SUB16(SHR32(SHL32(EXTEND32(q2),DB_SHIFT)+QCONST16(.5f,DB_SHIFT),fine_quant[i]),QCONST16(.5f,DB_SHIFT));
#else
         offset = (q2+.5f)*(1<<(14-fine_quant[i]))*(1.f/16384) - .5f;
#endif
         oldEBands[i+c*m->nbEBands] += offset;
      } while (++c < C);
   }
}

void unquant_energy_finalise(const CELTMode *m, int start, int end, opus_val16 *oldEBands, int *fine_quant,  int *fine_priority, int bits_left, ec_dec *dec, int C)
{
   int i, prio, c;

   /* Use up the remaining bits */
   for (prio=0;prio<2;prio++)
   {
      for (i=start;i<end && bits_left>=C ;i++)
      {
         if (fine_quant[i] >= MAX_FINE_BITS || fine_priority[i]!=prio)
            continue;
         c=0;
         do {
            int q2;
            opus_val16 offset;
            q2 = ec_dec_bits(dec, 1);
#ifdef FIXED_POINT
            offset = SHR16(SHL16(q2,DB_SHIFT)-QCONST16(.5f,DB_SHIFT),fine_quant[i]+1);
#else
            offset = (q2-.5f)*(1<<(14-fine_quant[i]-1))*(1.f/16384);
#endif
            oldEBands[i+c*m->nbEBands] += offset;
            bits_left--;
         } while (++c < C);
      }
   }
}

void log2Amp(const CELTMode *m, int start, int end,
      celt_ener *eBands, const opus_val16 *oldEBands, int C)
{
   int c, i;
   c=0;
   do {
      for (i=0;i<start;i++)
         eBands[i+c*m->nbEBands] = 0;
      for (;i<end;i++)
      {
         opus_val16 lg = ADD16(oldEBands[i+c*m->nbEBands],
                         SHL16((opus_val16)eMeans[i],6));
         eBands[i+c*m->nbEBands] = PSHR32(celt_exp2(lg),4);
      }
      for (;i<m->nbEBands;i++)
         eBands[i+c*m->nbEBands] = 0;
   } while (++c < C);
}

void amp2Log2(const CELTMode *m, int effEnd, int end,
      celt_ener *bandE, opus_val16 *bandLogE, int C)
{
   int c, i;
   c=0;
   do {
      for (i=0;i<effEnd;i++)
         bandLogE[i+c*m->nbEBands] =
               celt_log2(SHL32(bandE[i+c*m->nbEBands],2))
               - SHL16((opus_val16)eMeans[i],6);
      for (i=effEnd;i<end;i++)
         bandLogE[c*m->nbEBands+i] = -QCONST16(14.f,DB_SHIFT);
   } while (++c < C);
}
