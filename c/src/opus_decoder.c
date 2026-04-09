/* Copyright (c) 2010-2012 IETF Trust, Xiph.Org Foundation, Skype Limited. All rights reserved.
   Written by Jean-Marc Valin and Koen Vos */
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

/* All functions translated to Rust in src/opus_decoder.rs.
   Extern declarations kept for linking. */

#include "opus.h"
#include "opus_private.h"

extern int opus_decoder_get_size(int channels);
extern int opus_decoder_init(OpusDecoder *st, opus_int32 Fs, int channels);
extern OpusDecoder *opus_decoder_create(opus_int32 Fs, int channels, int *error);
extern void opus_decoder_destroy(OpusDecoder *st);

extern int opus_decode_native(OpusDecoder *st, const unsigned char *data, int len,
      opus_val16 *pcm, int frame_size, int decode_fec, int self_delimited, int *packet_offset);

extern int opus_decode(OpusDecoder *st, const unsigned char *data,
      int len, opus_int16 *pcm, int frame_size, int decode_fec);

extern int opus_decoder_ctl(OpusDecoder *st, OpusDecCtl request);

extern int opus_decoder_get_nb_samples(const OpusDecoder *dec,
      const unsigned char packet[], int len);

/* opus_packet_get_bandwidth, opus_packet_get_samples_per_frame,
   opus_packet_get_nb_channels, opus_packet_get_nb_frames:
   translated to Rust in src/packet.rs */
