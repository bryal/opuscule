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

#define CELT_C

#include "os_support.h"
#include "mdct.h"
#include <math.h>
#include "celt.h"
#include "pitch.h"
#include "bands.h"
#include "modes.h"
#include "entcode.h"
#include "quant_bands.h"
#include "rate.h"
#include "stack_alloc.h"
#include "mathops.h"
#include "float_cast.h"
#include <stdarg.h>
#include "celt_lpc.h"
#include "vq.h"

#ifndef OPUS_VERSION
#define OPUS_VERSION "unknown"
#endif

#ifdef CUSTOM_MODES
#define OPUS_CUSTOM_NOSTATIC
#else
#define OPUS_CUSTOM_NOSTATIC static inline
#endif

static const unsigned char trim_icdf[11] = {126, 124, 119, 109, 87, 41, 19, 9, 4, 2, 0};
/* Probs: NONE: 21.875%, LIGHT: 6.25%, NORMAL: 65.625%, AGGRESSIVE: 6.25% */
static const unsigned char spread_icdf[4] = {25, 23, 2, 0};

static const unsigned char tapset_icdf[3]={2,1,0};

#ifdef CUSTOM_MODES
static const unsigned char toOpusTable[20] = {
      0xE0, 0xE8, 0xF0, 0xF8,
      0xC0, 0xC8, 0xD0, 0xD8,
      0xA0, 0xA8, 0xB0, 0xB8,
      0x00, 0x00, 0x00, 0x00,
      0x80, 0x88, 0x90, 0x98,
};

static const unsigned char fromOpusTable[16] = {
      0x80, 0x88, 0x90, 0x98,
      0x40, 0x48, 0x50, 0x58,
      0x20, 0x28, 0x30, 0x38,
      0x00, 0x08, 0x10, 0x18
};

static inline int toOpus(unsigned char c)
{
   int ret=0;
   if (c<0xA0)
      ret = toOpusTable[c>>3];
   if (ret == 0)
      return -1;
   else
      return ret|(c&0x7);
}

static inline int fromOpus(unsigned char c)
{
   if (c<0x80)
      return -1;
   else
      return fromOpusTable[(c>>3)-16] | (c&0x7);
}
#endif /* CUSTOM_MODES */

#define COMBFILTER_MAXPERIOD 1024
#define COMBFILTER_MINPERIOD 15

extern int resampling_factor(opus_int32 rate);
extern opus_val16 sig2word16(celt_sig x);
#define SIG2WORD16(x) sig2word16(x)
/** Compute the IMDCT and apply window for all sub-frames and
    all channels in a frame */
static void compute_inv_mdcts(const CELTMode *mode, int shortBlocks, celt_sig *X,
      celt_sig * restrict out_mem[],
      celt_sig * restrict overlap_mem[], int C, int LM)
{
   int c;
   const int N = mode->shortMdctSize<<LM;
   const int overlap = OVERLAP(mode);
   VARDECL(opus_val32, x);
   SAVE_STACK;

   ALLOC(x, N+overlap, opus_val32);
   c=0; do {
      int j;
      int b;
      int N2 = N;
      int B = 1;

      if (shortBlocks)
      {
         N2 = mode->shortMdctSize;
         B = shortBlocks;
      }
      /* Prevents problems from the imdct doing the overlap-add */
      OPUS_CLEAR(x, overlap);

      for (b=0;b<B;b++)
      {
         /* IMDCT on the interleaved the sub-frames */
         clt_mdct_backward(&mode->mdct, &X[b+c*N2*B], x+N2*b, mode->window, overlap, shortBlocks ? mode->maxLM : mode->maxLM-LM, B);
      }

      for (j=0;j<overlap;j++)
         out_mem[c][j] = x[j] + overlap_mem[c][j];
      for (;j<N;j++)
         out_mem[c][j] = x[j];
      for (j=0;j<overlap;j++)
         overlap_mem[c][j] = x[N+j];
   } while (++c<C);
   RESTORE_STACK;
}

static void deemphasis(celt_sig *in[], opus_val16 *pcm, int N, int C, int downsample, const opus_val16 *coef, celt_sig *mem)
{
   int c;
   int count=0;
   c=0; do {
      int j;
      celt_sig * restrict x;
      opus_val16  * restrict y;
      celt_sig m = mem[c];
      x =in[c];
      y = pcm+c;
      for (j=0;j<N;j++)
      {
         celt_sig tmp = *x + m;
         m = MULT16_32_Q15(coef[0], tmp)
           - MULT16_32_Q15(coef[1], *x);
         tmp = SHL32(MULT16_32_Q15(coef[3], tmp), 2);
         x++;
         /* Technically the store could be moved outside of the if because
            the stores we don't want will just be overwritten */
         if (count==0)
            *y = SCALEOUT(SIG2WORD16(tmp));
         if (++count==downsample)
         {
            y+=C;
            count=0;
         }
      }
      mem[c] = m;
   } while (++c<C);
}

static void comb_filter(opus_val32 *y, opus_val32 *x, int T0, int T1, int N,
      opus_val16 g0, opus_val16 g1, int tapset0, int tapset1,
      const opus_val16 *window, int overlap)
{
   int i;
   /* printf ("%d %d %f %f\n", T0, T1, g0, g1); */
   opus_val16 g00, g01, g02, g10, g11, g12;
   static const opus_val16 gains[3][3] = {
         {QCONST16(0.3066406250f, 15), QCONST16(0.2170410156f, 15), QCONST16(0.1296386719f, 15)},
         {QCONST16(0.4638671875f, 15), QCONST16(0.2680664062f, 15), QCONST16(0.f, 15)},
         {QCONST16(0.7998046875f, 15), QCONST16(0.1000976562f, 15), QCONST16(0.f, 15)}};
   g00 = MULT16_16_Q15(g0, gains[tapset0][0]);
   g01 = MULT16_16_Q15(g0, gains[tapset0][1]);
   g02 = MULT16_16_Q15(g0, gains[tapset0][2]);
   g10 = MULT16_16_Q15(g1, gains[tapset1][0]);
   g11 = MULT16_16_Q15(g1, gains[tapset1][1]);
   g12 = MULT16_16_Q15(g1, gains[tapset1][2]);
   for (i=0;i<overlap;i++)
   {
      opus_val16 f;
      f = MULT16_16_Q15(window[i],window[i]);
      y[i] = x[i]
               + MULT16_32_Q15(MULT16_16_Q15((Q15ONE-f),g00),x[i-T0])
               + MULT16_32_Q15(MULT16_16_Q15((Q15ONE-f),g01),x[i-T0-1])
               + MULT16_32_Q15(MULT16_16_Q15((Q15ONE-f),g01),x[i-T0+1])
               + MULT16_32_Q15(MULT16_16_Q15((Q15ONE-f),g02),x[i-T0-2])
               + MULT16_32_Q15(MULT16_16_Q15((Q15ONE-f),g02),x[i-T0+2])
               + MULT16_32_Q15(MULT16_16_Q15(f,g10),x[i-T1])
               + MULT16_32_Q15(MULT16_16_Q15(f,g11),x[i-T1-1])
               + MULT16_32_Q15(MULT16_16_Q15(f,g11),x[i-T1+1])
               + MULT16_32_Q15(MULT16_16_Q15(f,g12),x[i-T1-2])
               + MULT16_32_Q15(MULT16_16_Q15(f,g12),x[i-T1+2]);

   }
   for (i=overlap;i<N;i++)
      y[i] = x[i]
               + MULT16_32_Q15(g10,x[i-T1])
               + MULT16_32_Q15(g11,x[i-T1-1])
               + MULT16_32_Q15(g11,x[i-T1+1])
               + MULT16_32_Q15(g12,x[i-T1-2])
               + MULT16_32_Q15(g12,x[i-T1+2]);
}

extern void tf_decode(int start, int end, int isTransient, int *tf_res, int LM, ec_dec *dec);
extern void init_caps(const CELTMode *m, int *cap, int LM, int C);
/**********************************************************************/
/*                                                                    */
/*                             DECODER                                */
/*                                                                    */
/**********************************************************************/
#define DECODE_BUFFER_SIZE 2048

/** Decoder state
 @brief Decoder state
 */
struct OpusCustomDecoder {
   const OpusCustomMode *mode;
   int overlap;
   int channels;
   int stream_channels;

   int downsample;
   int start, end;
   int signalling;

   /* Everything beyond this point gets cleared on a reset */
#define DECODER_RESET_START rng

   opus_uint32 rng;
   int error;
   int last_pitch_index;
   int loss_count;
   int postfilter_period;
   int postfilter_period_old;
   opus_val16 postfilter_gain;
   opus_val16 postfilter_gain_old;
   int postfilter_tapset;
   int postfilter_tapset_old;

   celt_sig preemph_memD[2];

   celt_sig _decode_mem[1]; /* Size = channels*(DECODE_BUFFER_SIZE+mode->overlap) */
   /* opus_val16 lpc[],  Size = channels*LPC_ORDER */
   /* opus_val16 oldEBands[], Size = 2*mode->nbEBands */
   /* opus_val16 oldLogE[], Size = 2*mode->nbEBands */
   /* opus_val16 oldLogE2[], Size = 2*mode->nbEBands */
   /* opus_val16 backgroundLogE[], Size = 2*mode->nbEBands */
};

int celt_decoder_get_size(int channels)
{
   const CELTMode *mode = opus_custom_mode_create(48000, 960, NULL);
   return opus_custom_decoder_get_size(mode, channels);
}

OPUS_CUSTOM_NOSTATIC int opus_custom_decoder_get_size(const CELTMode *mode, int channels)
{
   int size = sizeof(struct CELTDecoder)
            + (channels*(DECODE_BUFFER_SIZE+mode->overlap)-1)*sizeof(celt_sig)
            + channels*LPC_ORDER*sizeof(opus_val16)
            + 4*2*mode->nbEBands*sizeof(opus_val16);
   return size;
}

#ifdef CUSTOM_MODES
CELTDecoder *opus_custom_decoder_create(const CELTMode *mode, int channels, int *error)
{
   int ret;
   CELTDecoder *st = (CELTDecoder *)opus_alloc(opus_custom_decoder_get_size(mode, channels));
   ret = opus_custom_decoder_init(st, mode, channels);
   if (ret != OPUS_OK)
   {
      opus_custom_decoder_destroy(st);
      st = NULL;
   }
   if (error)
      *error = ret;
   return st;
}
#endif /* CUSTOM_MODES */

int celt_decoder_init(CELTDecoder *st, opus_int32 sampling_rate, int channels)
{
   int ret;
   ret = opus_custom_decoder_init(st, opus_custom_mode_create(48000, 960, NULL), channels);
   if (ret != OPUS_OK)
      return ret;
   st->downsample = resampling_factor(sampling_rate);
   if (st->downsample==0)
      return OPUS_BAD_ARG;
   else
      return OPUS_OK;
}

OPUS_CUSTOM_NOSTATIC int opus_custom_decoder_init(CELTDecoder *st, const CELTMode *mode, int channels)
{
   if (channels < 0 || channels > 2)
      return OPUS_BAD_ARG;

   if (st==NULL)
      return OPUS_ALLOC_FAIL;

   OPUS_CLEAR((char*)st, opus_custom_decoder_get_size(mode, channels));

   st->mode = mode;
   st->overlap = mode->overlap;
   st->stream_channels = st->channels = channels;

   st->downsample = 1;
   st->start = 0;
   st->end = st->mode->effEBands;
   st->signalling = 1;

   st->loss_count = 0;

   opus_custom_decoder_ctl(st, OPUS_RESET_STATE);

   return OPUS_OK;
}

#ifdef CUSTOM_MODES
void opus_custom_decoder_destroy(CELTDecoder *st)
{
   opus_free(st);
}
#endif /* CUSTOM_MODES */

static void celt_decode_lost(CELTDecoder * restrict st, opus_val16 * restrict pcm, int N, int LM)
{
   int c;
   int pitch_index;
   int overlap = st->mode->overlap;
   opus_val16 fade = Q15ONE;
   int i, len;
   const int C = st->channels;
   int offset;
   celt_sig *out_mem[2];
   celt_sig *decode_mem[2];
   celt_sig *overlap_mem[2];
   opus_val16 *lpc;
   opus_val32 *out_syn[2];
   opus_val16 *oldBandE, *oldLogE, *oldLogE2, *backgroundLogE;
   SAVE_STACK;

   c=0; do {
      decode_mem[c] = st->_decode_mem + c*(DECODE_BUFFER_SIZE+st->overlap);
      out_mem[c] = decode_mem[c]+DECODE_BUFFER_SIZE-MAX_PERIOD;
      overlap_mem[c] = decode_mem[c]+DECODE_BUFFER_SIZE;
   } while (++c<C);
   lpc = (opus_val16*)(st->_decode_mem+(DECODE_BUFFER_SIZE+st->overlap)*C);
   oldBandE = lpc+C*LPC_ORDER;
   oldLogE = oldBandE + 2*st->mode->nbEBands;
   oldLogE2 = oldLogE + 2*st->mode->nbEBands;
   backgroundLogE = oldLogE2  + 2*st->mode->nbEBands;

   out_syn[0] = out_mem[0]+MAX_PERIOD-N;
   if (C==2)
      out_syn[1] = out_mem[1]+MAX_PERIOD-N;

   len = N+st->mode->overlap;

   if (st->loss_count >= 5 || st->start!=0)
   {
      /* Noise-based PLC/CNG */
      VARDECL(celt_sig, freq);
      VARDECL(celt_norm, X);
      VARDECL(celt_ener, bandE);
      opus_uint32 seed;
      int effEnd;

      effEnd = st->end;
      if (effEnd > st->mode->effEBands)
         effEnd = st->mode->effEBands;

      ALLOC(freq, C*N, celt_sig); /**< Interleaved signal MDCTs */
      ALLOC(X, C*N, celt_norm);   /**< Interleaved normalised MDCTs */
      ALLOC(bandE, st->mode->nbEBands*C, celt_ener);

      if (st->loss_count >= 5)
         log2Amp(st->mode, st->start, st->end, bandE, backgroundLogE, C);
      else {
         /* Energy decay */
         opus_val16 decay = st->loss_count==0 ? QCONST16(1.5f, DB_SHIFT) : QCONST16(.5f, DB_SHIFT);
         c=0; do
         {
            for (i=st->start;i<st->end;i++)
               oldBandE[c*st->mode->nbEBands+i] -= decay;
         } while (++c<C);
         log2Amp(st->mode, st->start, st->end, bandE, oldBandE, C);
      }
      seed = st->rng;
      for (c=0;c<C;c++)
      {
         for (i=0;i<(st->mode->eBands[st->start]<<LM);i++)
            X[c*N+i] = 0;
         for (i=st->start;i<st->mode->effEBands;i++)
         {
            int j;
            int boffs;
            int blen;
            boffs = N*c+(st->mode->eBands[i]<<LM);
            blen = (st->mode->eBands[i+1]-st->mode->eBands[i])<<LM;
            for (j=0;j<blen;j++)
            {
               seed = celt_lcg_rand(seed);
               X[boffs+j] = (celt_norm)((opus_int32)seed>>20);
            }
            renormalise_vector(X+boffs, blen, Q15ONE);
         }
         for (i=(st->mode->eBands[st->end]<<LM);i<N;i++)
            X[c*N+i] = 0;
      }
      st->rng = seed;

      denormalise_bands(st->mode, X, freq, bandE, st->mode->effEBands, C, 1<<LM);

      c=0; do
         for (i=0;i<st->mode->eBands[st->start]<<LM;i++)
            freq[c*N+i] = 0;
      while (++c<C);
      c=0; do {
         int bound = st->mode->eBands[effEnd]<<LM;
         if (st->downsample!=1)
            bound = IMIN(bound, N/st->downsample);
         for (i=bound;i<N;i++)
            freq[c*N+i] = 0;
      } while (++c<C);
      compute_inv_mdcts(st->mode, 0, freq, out_syn, overlap_mem, C, LM);
   } else {
      /* Pitch-based PLC */
      if (st->loss_count == 0)
      {
         opus_val16 pitch_buf[DECODE_BUFFER_SIZE>>1];
         /* Corresponds to a min pitch of 67 Hz. It's possible to save CPU in this
         search by using only part of the decode buffer */
         int poffset = 720;
         pitch_downsample(decode_mem, pitch_buf, DECODE_BUFFER_SIZE, C);
         /* Max pitch is 100 samples (480 Hz) */
         pitch_search(pitch_buf+((poffset)>>1), pitch_buf, DECODE_BUFFER_SIZE-poffset,
               poffset-100, &pitch_index);
         pitch_index = poffset-pitch_index;
         st->last_pitch_index = pitch_index;
      } else {
         pitch_index = st->last_pitch_index;
         fade = QCONST16(.8f,15);
      }

      c=0; do {
         VARDECL(opus_val32, e);
         opus_val16 exc[MAX_PERIOD];
         opus_val32 ac[LPC_ORDER+1];
         opus_val16 decay = 1;
         opus_val32 S1=0;
         opus_val16 mem[LPC_ORDER]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

         ALLOC(e, MAX_PERIOD+2*st->mode->overlap, opus_val32);

         offset = MAX_PERIOD-pitch_index;
         for (i=0;i<MAX_PERIOD;i++)
            exc[i] = ROUND16(out_mem[c][i], SIG_SHIFT);

         if (st->loss_count == 0)
         {
            _celt_autocorr(exc, ac, st->mode->window, st->mode->overlap,
                  LPC_ORDER, MAX_PERIOD);

            /* Noise floor -40 dB */
#ifdef FIXED_POINT
            ac[0] += SHR32(ac[0],13);
#else
            ac[0] *= 1.0001f;
#endif
            /* Lag windowing */
            for (i=1;i<=LPC_ORDER;i++)
            {
               /*ac[i] *= exp(-.5*(2*M_PI*.002*i)*(2*M_PI*.002*i));*/
#ifdef FIXED_POINT
               ac[i] -= MULT16_32_Q15(2*i*i, ac[i]);
#else
               ac[i] -= ac[i]*(.008f*i)*(.008f*i);
#endif
            }

            _celt_lpc(lpc+c*LPC_ORDER, ac, LPC_ORDER);
         }
         for (i=0;i<LPC_ORDER;i++)
            mem[i] = ROUND16(out_mem[c][MAX_PERIOD-1-i], SIG_SHIFT);
         celt_fir(exc, lpc+c*LPC_ORDER, exc, MAX_PERIOD, LPC_ORDER, mem);
         /*for (i=0;i<MAX_PERIOD;i++)printf("%d ", exc[i]); printf("\n");*/
         /* Check if the waveform is decaying (and if so how fast) */
         {
            opus_val32 E1=1, E2=1;
            int period;
            if (pitch_index <= MAX_PERIOD/2)
               period = pitch_index;
            else
               period = MAX_PERIOD/2;
            for (i=0;i<period;i++)
            {
               E1 += SHR32(MULT16_16(exc[MAX_PERIOD-period+i],exc[MAX_PERIOD-period+i]),8);
               E2 += SHR32(MULT16_16(exc[MAX_PERIOD-2*period+i],exc[MAX_PERIOD-2*period+i]),8);
            }
            if (E1 > E2)
               E1 = E2;
            decay = celt_sqrt(frac_div32(SHR(E1,1),E2));
         }

         /* Copy excitation, taking decay into account */
         for (i=0;i<len+st->mode->overlap;i++)
         {
            opus_val16 tmp;
            if (offset+i >= MAX_PERIOD)
            {
               offset -= pitch_index;
               decay = MULT16_16_Q15(decay, decay);
            }
            e[i] = SHL32(EXTEND32(MULT16_16_Q15(decay, exc[offset+i])), SIG_SHIFT);
            tmp = ROUND16(out_mem[c][offset+i],SIG_SHIFT);
            S1 += SHR32(MULT16_16(tmp,tmp),8);
         }
         for (i=0;i<LPC_ORDER;i++)
            mem[i] = ROUND16(out_mem[c][MAX_PERIOD-1-i], SIG_SHIFT);
         for (i=0;i<len+st->mode->overlap;i++)
            e[i] = MULT16_32_Q15(fade, e[i]);
         celt_iir(e, lpc+c*LPC_ORDER, e, len+st->mode->overlap, LPC_ORDER, mem);

         {
            opus_val32 S2=0;
            for (i=0;i<len+overlap;i++)
            {
               opus_val16 tmp = ROUND16(e[i],SIG_SHIFT);
               S2 += SHR32(MULT16_16(tmp,tmp),8);
            }
            /* This checks for an "explosion" in the synthesis */
#ifdef FIXED_POINT
            if (!(S1 > SHR32(S2,2)))
#else
               /* Float test is written this way to catch NaNs at the same time */
               if (!(S1 > 0.2f*S2))
#endif
               {
                  for (i=0;i<len+overlap;i++)
                     e[i] = 0;
               } else if (S1 < S2)
               {
                  opus_val16 ratio = celt_sqrt(frac_div32(SHR32(S1,1)+1,S2+1));
                  for (i=0;i<len+overlap;i++)
                     e[i] = MULT16_32_Q15(ratio, e[i]);
               }
         }

         /* Apply post-filter to the MDCT overlap of the previous frame */
         comb_filter(out_mem[c]+MAX_PERIOD, out_mem[c]+MAX_PERIOD, st->postfilter_period, st->postfilter_period, st->overlap,
               st->postfilter_gain, st->postfilter_gain, st->postfilter_tapset, st->postfilter_tapset,
               NULL, 0);

         for (i=0;i<MAX_PERIOD+st->mode->overlap-N;i++)
            out_mem[c][i] = out_mem[c][N+i];

         /* Apply TDAC to the concealed audio so that it blends with the
         previous and next frames */
         for (i=0;i<overlap/2;i++)
         {
            opus_val32 tmp;
            tmp = MULT16_32_Q15(st->mode->window[i],           e[N+overlap-1-i]) +
                  MULT16_32_Q15(st->mode->window[overlap-i-1], e[N+i          ]);
            out_mem[c][MAX_PERIOD+i] = MULT16_32_Q15(st->mode->window[overlap-i-1], tmp);
            out_mem[c][MAX_PERIOD+overlap-i-1] = MULT16_32_Q15(st->mode->window[i], tmp);
         }
         for (i=0;i<N;i++)
            out_mem[c][MAX_PERIOD-N+i] = e[i];

         /* Apply pre-filter to the MDCT overlap for the next frame (post-filter will be applied then) */
         comb_filter(e, out_mem[c]+MAX_PERIOD, st->postfilter_period, st->postfilter_period, st->overlap,
               -st->postfilter_gain, -st->postfilter_gain, st->postfilter_tapset, st->postfilter_tapset,
               NULL, 0);
         for (i=0;i<overlap;i++)
            out_mem[c][MAX_PERIOD+i] = e[i];
      } while (++c<C);
   }

   deemphasis(out_syn, pcm, N, C, st->downsample, st->mode->preemph, st->preemph_memD);

   st->loss_count++;

   RESTORE_STACK;
}

int celt_decode_with_ec(CELTDecoder * restrict st, const unsigned char *data, int len, opus_val16 * restrict pcm, int frame_size, ec_dec *dec)
{
   int c, i, N;
   int spread_decision;
   opus_int32 bits;
   ec_dec _dec;
   VARDECL(celt_sig, freq);
   VARDECL(celt_norm, X);
   VARDECL(celt_ener, bandE);
   VARDECL(int, fine_quant);
   VARDECL(int, pulses);
   VARDECL(int, cap);
   VARDECL(int, offsets);
   VARDECL(int, fine_priority);
   VARDECL(int, tf_res);
   VARDECL(unsigned char, collapse_masks);
   celt_sig *out_mem[2];
   celt_sig *decode_mem[2];
   celt_sig *overlap_mem[2];
   celt_sig *out_syn[2];
   opus_val16 *lpc;
   opus_val16 *oldBandE, *oldLogE, *oldLogE2, *backgroundLogE;

   int shortBlocks;
   int isTransient;
   int intra_ener;
   const int CC = st->channels;
   int LM, M;
   int effEnd;
   int codedBands;
   int alloc_trim;
   int postfilter_pitch;
   opus_val16 postfilter_gain;
   int intensity=0;
   int dual_stereo=0;
   opus_int32 total_bits;
   opus_int32 balance;
   opus_int32 tell;
   int dynalloc_logp;
   int postfilter_tapset;
   int anti_collapse_rsv;
   int anti_collapse_on=0;
   int silence;
   int C = st->stream_channels;
   ALLOC_STACK;

   frame_size *= st->downsample;

   c=0; do {
      decode_mem[c] = st->_decode_mem + c*(DECODE_BUFFER_SIZE+st->overlap);
      out_mem[c] = decode_mem[c]+DECODE_BUFFER_SIZE-MAX_PERIOD;
      overlap_mem[c] = decode_mem[c]+DECODE_BUFFER_SIZE;
   } while (++c<CC);
   lpc = (opus_val16*)(st->_decode_mem+(DECODE_BUFFER_SIZE+st->overlap)*CC);
   oldBandE = lpc+CC*LPC_ORDER;
   oldLogE = oldBandE + 2*st->mode->nbEBands;
   oldLogE2 = oldLogE + 2*st->mode->nbEBands;
   backgroundLogE = oldLogE2  + 2*st->mode->nbEBands;

#ifdef CUSTOM_MODES
   if (st->signalling && data!=NULL)
   {
      int data0=data[0];
      /* Convert "standard mode" to Opus header */
      if (st->mode->Fs==48000 && st->mode->shortMdctSize==120)
      {
         data0 = fromOpus(data0);
         if (data0<0)
            return OPUS_INVALID_PACKET;
      }
      st->end = IMAX(1, st->mode->effEBands-2*(data0>>5));
      LM = (data0>>3)&0x3;
      C = 1 + ((data0>>2)&0x1);
      data++;
      len--;
      if (LM>st->mode->maxLM)
         return OPUS_INVALID_PACKET;
      if (frame_size < st->mode->shortMdctSize<<LM)
         return OPUS_BUFFER_TOO_SMALL;
      else
         frame_size = st->mode->shortMdctSize<<LM;
   } else {
#else
   {
#endif
      for (LM=0;LM<=st->mode->maxLM;LM++)
         if (st->mode->shortMdctSize<<LM==frame_size)
            break;
      if (LM>st->mode->maxLM)
         return OPUS_BAD_ARG;
   }
   M=1<<LM;

   if (len<0 || len>1275 || pcm==NULL)
      return OPUS_BAD_ARG;

   N = M*st->mode->shortMdctSize;

   effEnd = st->end;
   if (effEnd > st->mode->effEBands)
      effEnd = st->mode->effEBands;

   ALLOC(freq, IMAX(CC,C)*N, celt_sig); /**< Interleaved signal MDCTs */
   ALLOC(X, C*N, celt_norm);   /**< Interleaved normalised MDCTs */
   ALLOC(bandE, st->mode->nbEBands*C, celt_ener);
   c=0; do
      for (i=0;i<M*st->mode->eBands[st->start];i++)
         X[c*N+i] = 0;
   while (++c<C);
   c=0; do
      for (i=M*st->mode->eBands[effEnd];i<N;i++)
         X[c*N+i] = 0;
   while (++c<C);

   if (data == NULL || len<=1)
   {
      celt_decode_lost(st, pcm, N, LM);
      RESTORE_STACK;
      return frame_size/st->downsample;
   }

   if (dec == NULL)
   {
      ec_dec_init(&_dec,(unsigned char*)data,len);
      dec = &_dec;
   }

   if (C==1)
   {
      for (i=0;i<st->mode->nbEBands;i++)
         oldBandE[i]=MAX16(oldBandE[i],oldBandE[st->mode->nbEBands+i]);
   }

   total_bits = len*8;
   tell = ec_tell(dec);

   if (tell >= total_bits)
      silence = 1;
   else if (tell==1)
      silence = ec_dec_bit_logp(dec, 15);
   else
      silence = 0;
   if (silence)
   {
      /* Pretend we've read all the remaining bits */
      tell = len*8;
      dec->nbits_total+=tell-ec_tell(dec);
   }

   postfilter_gain = 0;
   postfilter_pitch = 0;
   postfilter_tapset = 0;
   if (st->start==0 && tell+16 <= total_bits)
   {
      if(ec_dec_bit_logp(dec, 1))
      {
         int qg, octave;
         octave = ec_dec_uint(dec, 6);
         postfilter_pitch = (16<<octave)+ec_dec_bits(dec, 4+octave)-1;
         qg = ec_dec_bits(dec, 3);
         if (ec_tell(dec)+2<=total_bits)
            postfilter_tapset = ec_dec_icdf(dec, tapset_icdf, 2);
         postfilter_gain = QCONST16(.09375f,15)*(qg+1);
      }
      tell = ec_tell(dec);
   }

   if (LM > 0 && tell+3 <= total_bits)
   {
      isTransient = ec_dec_bit_logp(dec, 3);
      tell = ec_tell(dec);
   }
   else
      isTransient = 0;

   if (isTransient)
      shortBlocks = M;
   else
      shortBlocks = 0;

   /* Decode the global flags (first symbols in the stream) */
   intra_ener = tell+3<=total_bits ? ec_dec_bit_logp(dec, 3) : 0;
   /* Get band energies */
   unquant_coarse_energy(st->mode, st->start, st->end, oldBandE,
         intra_ener, dec, C, LM);

   ALLOC(tf_res, st->mode->nbEBands, int);
   tf_decode(st->start, st->end, isTransient, tf_res, LM, dec);

   tell = ec_tell(dec);
   spread_decision = SPREAD_NORMAL;
   if (tell+4 <= total_bits)
      spread_decision = ec_dec_icdf(dec, spread_icdf, 5);

   ALLOC(pulses, st->mode->nbEBands, int);
   ALLOC(cap, st->mode->nbEBands, int);
   ALLOC(offsets, st->mode->nbEBands, int);
   ALLOC(fine_priority, st->mode->nbEBands, int);

   init_caps(st->mode,cap,LM,C);

   dynalloc_logp = 6;
   total_bits<<=BITRES;
   tell = ec_tell_frac(dec);
   for (i=st->start;i<st->end;i++)
   {
      int width, quanta;
      int dynalloc_loop_logp;
      int boost;
      width = C*(st->mode->eBands[i+1]-st->mode->eBands[i])<<LM;
      /* quanta is 6 bits, but no more than 1 bit/sample
         and no less than 1/8 bit/sample */
      quanta = IMIN(width<<BITRES, IMAX(6<<BITRES, width));
      dynalloc_loop_logp = dynalloc_logp;
      boost = 0;
      while (tell+(dynalloc_loop_logp<<BITRES) < total_bits && boost < cap[i])
      {
         int flag;
         flag = ec_dec_bit_logp(dec, dynalloc_loop_logp);
         tell = ec_tell_frac(dec);
         if (!flag)
            break;
         boost += quanta;
         total_bits -= quanta;
         dynalloc_loop_logp = 1;
      }
      offsets[i] = boost;
      /* Making dynalloc more likely */
      if (boost>0)
         dynalloc_logp = IMAX(2, dynalloc_logp-1);
   }

   ALLOC(fine_quant, st->mode->nbEBands, int);
   alloc_trim = tell+(6<<BITRES) <= total_bits ?
         ec_dec_icdf(dec, trim_icdf, 7) : 5;

   bits = (((opus_int32)len*8)<<BITRES) - ec_tell_frac(dec) - 1;
   anti_collapse_rsv = isTransient&&LM>=2&&bits>=((LM+2)<<BITRES) ? (1<<BITRES) : 0;
   bits -= anti_collapse_rsv;
   codedBands = compute_allocation(st->mode, st->start, st->end, offsets, cap,
         alloc_trim, &intensity, &dual_stereo, bits, &balance, pulses,
         fine_quant, fine_priority, C, LM, dec, 0, 0);

   unquant_fine_energy(st->mode, st->start, st->end, oldBandE, fine_quant, dec, C);

   /* Decode fixed codebook */
   ALLOC(collapse_masks, C*st->mode->nbEBands, unsigned char);
   quant_all_bands(0, st->mode, st->start, st->end, X, C==2 ? X+N : NULL, collapse_masks,
         NULL, pulses, shortBlocks, spread_decision, dual_stereo, intensity, tf_res,
         len*(8<<BITRES)-anti_collapse_rsv, balance, dec, LM, codedBands, &st->rng);

   if (anti_collapse_rsv > 0)
   {
      anti_collapse_on = ec_dec_bits(dec, 1);
   }

   unquant_energy_finalise(st->mode, st->start, st->end, oldBandE,
         fine_quant, fine_priority, len*8-ec_tell(dec), dec, C);

   if (anti_collapse_on)
      anti_collapse(st->mode, X, collapse_masks, LM, C, N,
            st->start, st->end, oldBandE, oldLogE, oldLogE2, pulses, st->rng);

   log2Amp(st->mode, st->start, st->end, bandE, oldBandE, C);

   if (silence)
   {
      for (i=0;i<C*st->mode->nbEBands;i++)
      {
         bandE[i] = 0;
         oldBandE[i] = -QCONST16(28.f,DB_SHIFT);
      }
   }
   /* Synthesis */
   denormalise_bands(st->mode, X, freq, bandE, effEnd, C, M);

   OPUS_MOVE(decode_mem[0], decode_mem[0]+N, DECODE_BUFFER_SIZE-N);
   if (CC==2)
      OPUS_MOVE(decode_mem[1], decode_mem[1]+N, DECODE_BUFFER_SIZE-N);

   c=0; do
      for (i=0;i<M*st->mode->eBands[st->start];i++)
         freq[c*N+i] = 0;
   while (++c<C);
   c=0; do {
      int bound = M*st->mode->eBands[effEnd];
      if (st->downsample!=1)
         bound = IMIN(bound, N/st->downsample);
      for (i=bound;i<N;i++)
         freq[c*N+i] = 0;
   } while (++c<C);

   out_syn[0] = out_mem[0]+MAX_PERIOD-N;
   if (CC==2)
      out_syn[1] = out_mem[1]+MAX_PERIOD-N;

   if (CC==2&&C==1)
   {
      for (i=0;i<N;i++)
         freq[N+i] = freq[i];
   }
   if (CC==1&&C==2)
   {
      for (i=0;i<N;i++)
         freq[i] = HALF32(ADD32(freq[i],freq[N+i]));
   }

   /* Compute inverse MDCTs */
   compute_inv_mdcts(st->mode, shortBlocks, freq, out_syn, overlap_mem, CC, LM);

   c=0; do {
      st->postfilter_period=IMAX(st->postfilter_period, COMBFILTER_MINPERIOD);
      st->postfilter_period_old=IMAX(st->postfilter_period_old, COMBFILTER_MINPERIOD);
      comb_filter(out_syn[c], out_syn[c], st->postfilter_period_old, st->postfilter_period, st->mode->shortMdctSize,
            st->postfilter_gain_old, st->postfilter_gain, st->postfilter_tapset_old, st->postfilter_tapset,
            st->mode->window, st->overlap);
      if (LM!=0)
         comb_filter(out_syn[c]+st->mode->shortMdctSize, out_syn[c]+st->mode->shortMdctSize, st->postfilter_period, postfilter_pitch, N-st->mode->shortMdctSize,
               st->postfilter_gain, postfilter_gain, st->postfilter_tapset, postfilter_tapset,
               st->mode->window, st->mode->overlap);

   } while (++c<CC);
   st->postfilter_period_old = st->postfilter_period;
   st->postfilter_gain_old = st->postfilter_gain;
   st->postfilter_tapset_old = st->postfilter_tapset;
   st->postfilter_period = postfilter_pitch;
   st->postfilter_gain = postfilter_gain;
   st->postfilter_tapset = postfilter_tapset;
   if (LM!=0)
   {
      st->postfilter_period_old = st->postfilter_period;
      st->postfilter_gain_old = st->postfilter_gain;
      st->postfilter_tapset_old = st->postfilter_tapset;
   }

   if (C==1) {
      for (i=0;i<st->mode->nbEBands;i++)
         oldBandE[st->mode->nbEBands+i]=oldBandE[i];
   }

   /* In case start or end were to change */
   if (!isTransient)
   {
      for (i=0;i<2*st->mode->nbEBands;i++)
         oldLogE2[i] = oldLogE[i];
      for (i=0;i<2*st->mode->nbEBands;i++)
         oldLogE[i] = oldBandE[i];
      for (i=0;i<2*st->mode->nbEBands;i++)
         backgroundLogE[i] = MIN16(backgroundLogE[i] + M*QCONST16(0.001f,DB_SHIFT), oldBandE[i]);
   } else {
      for (i=0;i<2*st->mode->nbEBands;i++)
         oldLogE[i] = MIN16(oldLogE[i], oldBandE[i]);
   }
   c=0; do
   {
      for (i=0;i<st->start;i++)
      {
         oldBandE[c*st->mode->nbEBands+i]=0;
         oldLogE[c*st->mode->nbEBands+i]=oldLogE2[c*st->mode->nbEBands+i]=-QCONST16(28.f,DB_SHIFT);
      }
      for (i=st->end;i<st->mode->nbEBands;i++)
      {
         oldBandE[c*st->mode->nbEBands+i]=0;
         oldLogE[c*st->mode->nbEBands+i]=oldLogE2[c*st->mode->nbEBands+i]=-QCONST16(28.f,DB_SHIFT);
      }
   } while (++c<2);
   st->rng = dec->rng;

   deemphasis(out_syn, pcm, N, CC, st->downsample, st->mode->preemph, st->preemph_memD);
   st->loss_count = 0;
   RESTORE_STACK;
   if (ec_tell(dec) > 8*len)
      return OPUS_INTERNAL_ERROR;
   if(ec_get_error(dec))
      st->error = 1;
   return frame_size/st->downsample;
}


#ifdef CUSTOM_MODES

#ifdef FIXED_POINT
int opus_custom_decode(CELTDecoder * restrict st, const unsigned char *data, int len, opus_int16 * restrict pcm, int frame_size)
{
   return celt_decode_with_ec(st, data, len, pcm, frame_size, NULL);
}

#ifndef DISABLE_FLOAT_API
int opus_custom_decode_float(CELTDecoder * restrict st, const unsigned char *data, int len, float * restrict pcm, int frame_size)
{
   int j, ret, C, N;
   VARDECL(opus_int16, out);
   ALLOC_STACK;

   if (pcm==NULL)
      return OPUS_BAD_ARG;

   C = st->channels;
   N = frame_size;

   ALLOC(out, C*N, opus_int16);
   ret=celt_decode_with_ec(st, data, len, out, frame_size, NULL);
   if (ret>0)
      for (j=0;j<C*ret;j++)
         pcm[j]=out[j]*(1.f/32768.f);

   RESTORE_STACK;
   return ret;
}
#endif /* DISABLE_FLOAT_API */

#else

int opus_custom_decode_float(CELTDecoder * restrict st, const unsigned char *data, int len, float * restrict pcm, int frame_size)
{
   return celt_decode_with_ec(st, data, len, pcm, frame_size, NULL);
}

int opus_custom_decode(CELTDecoder * restrict st, const unsigned char *data, int len, opus_int16 * restrict pcm, int frame_size)
{
   int j, ret, C, N;
   VARDECL(celt_sig, out);
   ALLOC_STACK;

   if (pcm==NULL)
      return OPUS_BAD_ARG;

   C = st->channels;
   N = frame_size;
   ALLOC(out, C*N, celt_sig);

   ret=celt_decode_with_ec(st, data, len, out, frame_size, NULL);

   if (ret>0)
      for (j=0;j<C*ret;j++)
         pcm[j] = FLOAT2INT16 (out[j]);

   RESTORE_STACK;
   return ret;
}

#endif
#endif /* CUSTOM_MODES */

int opus_custom_decoder_ctl(CELTDecoder * restrict st, int request, ...)
{
   va_list ap;

   va_start(ap, request);
   switch (request)
   {
      case CELT_SET_START_BAND_REQUEST:
      {
         opus_int32 value = va_arg(ap, opus_int32);
         if (value<0 || value>=st->mode->nbEBands)
            goto bad_arg;
         st->start = value;
      }
      break;
      case CELT_SET_END_BAND_REQUEST:
      {
         opus_int32 value = va_arg(ap, opus_int32);
         if (value<1 || value>st->mode->nbEBands)
            goto bad_arg;
         st->end = value;
      }
      break;
      case CELT_SET_CHANNELS_REQUEST:
      {
         opus_int32 value = va_arg(ap, opus_int32);
         if (value<1 || value>2)
            goto bad_arg;
         st->stream_channels = value;
      }
      break;
      case CELT_GET_AND_CLEAR_ERROR_REQUEST:
      {
         int *value = va_arg(ap, opus_int32*);
         if (value==NULL)
            goto bad_arg;
         *value=st->error;
         st->error = 0;
      }
      break;
      case OPUS_GET_LOOKAHEAD_REQUEST:
      {
         int *value = va_arg(ap, opus_int32*);
         if (value==NULL)
            goto bad_arg;
         *value = st->overlap/st->downsample;
      }
      break;
      case OPUS_RESET_STATE:
      {
         int i;
         opus_val16 *lpc, *oldBandE, *oldLogE, *oldLogE2;
         lpc = (opus_val16*)(st->_decode_mem+(DECODE_BUFFER_SIZE+st->overlap)*st->channels);
         oldBandE = lpc+st->channels*LPC_ORDER;
         oldLogE = oldBandE + 2*st->mode->nbEBands;
         oldLogE2 = oldLogE + 2*st->mode->nbEBands;
         OPUS_CLEAR((char*)&st->DECODER_RESET_START,
               opus_custom_decoder_get_size(st->mode, st->channels)-
               ((char*)&st->DECODER_RESET_START - (char*)st));
         for (i=0;i<2*st->mode->nbEBands;i++)
            oldLogE[i]=oldLogE2[i]=-QCONST16(28.f,DB_SHIFT);
      }
      break;
      case OPUS_GET_PITCH_REQUEST:
      {
         int *value = va_arg(ap, opus_int32*);
         if (value==NULL)
            goto bad_arg;
         *value = st->postfilter_period;
      }
      break;
#ifdef OPUS_BUILD
      case CELT_GET_MODE_REQUEST:
      {
         const CELTMode ** value = va_arg(ap, const CELTMode**);
         if (value==0)
            goto bad_arg;
         *value=st->mode;
      }
      break;
      case CELT_SET_SIGNALLING_REQUEST:
      {
         opus_int32 value = va_arg(ap, opus_int32);
         st->signalling = value;
      }
      break;
      case OPUS_GET_FINAL_RANGE_REQUEST:
      {
         opus_uint32 * value = va_arg(ap, opus_uint32 *);
         if (value==0)
            goto bad_arg;
         *value=st->rng;
      }
      break;
#endif
      default:
         goto bad_request;
   }
   va_end(ap);
   return OPUS_OK;
bad_arg:
   va_end(ap);
   return OPUS_BAD_ARG;
bad_request:
      va_end(ap);
  return OPUS_UNIMPLEMENTED;
}



const char *opus_strerror(int error)
{
   static const char *error_strings[8] = {
      "success",
      "invalid argument",
      "buffer too small",
      "internal error",
      "corrupted stream",
      "request not implemented",
      "invalid state",
      "memory allocation failed"
   };
   if (error > 0 || error < -7)
      return "unknown error";
   else
      return error_strings[-error];
}

const char *opus_get_version_string(void)
{
    return "libopus " OPUS_VERSION
#ifdef FUZZING
          "-fuzzing"
#endif
          ;
}
