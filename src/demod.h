#ifndef RTL_DEMOD_H
#define RTL_DEMOD_H

/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 * Copyright (C) 2015 by Hayati Ayguen <h_ayguen@web.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * written because people could not do real time
 * FM demod on Atom hardware with GNU radio
 * based on rtl_sdr.c and rtl_tcp.c and rtl_fm.c
 */

#include <stdint.h>

#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)
#define MAXIMUM_DOWNSAMPLE_PASSES      10


struct mixer_state
{
	int16_t   inv_sqrt[32];   /* precalculated inverse square root() */
	int16_t   mix[16][2];	  /* out[i] = inp[i] * mix[(i-1) % 16] * rot */
	int16_t   rot[2];         /* complex rotation multiplicator: mix[k] = mix[k-1] * rot[0] */
	int	  index;          /* next index to use of mix[] */
	int	  skip;           /* can skip mixing? - cause frequency is 0 */
};

struct demod_state
{
	int16_t * lowpassed;	/* input and decimated quadrature I/Q sample-pairs */
	int16_t * result;	/* demodulated inphase signal */
	int	  lp_len;	/* number of valid samples in lowpassed[] - NOT quadrature I/Q sample-pairs! */
	int	  result_len;	/* number of valid samples in result[] */

#if 1
	int16_t   lp_i_hist[MAXIMUM_DOWNSAMPLE_PASSES][6];
	int16_t   lp_q_hist[MAXIMUM_DOWNSAMPLE_PASSES][6];
#else
	int16_t   lp_iq_hist[MAXIMUM_DOWNSAMPLE_PASSES][20+4];	/* 20 would be sufficient - but fill cache-line */
#endif

	int16_t   droop_i_hist[9];
	int16_t   droop_q_hist[9];
	int	  comp_fir_size;	/* 9 activates droop compensation of fifth_order() with cic_9_tables[] */

	int	  rate_in;		/* samplerate from rtlsdr dongle */
	int	  rate_out;		/* samplerate for demodulation (=CBB and MPX) */
	int	  rate_out2;		/* samplerate for additional decimation after demodulation */

	int	  downsample;		/* decimation ratio: min 1, max 256 */
	int	  downsample_passes;	/* number of decimation steps for downsample */
	int	  post_downsample;	/* requested oversampling ratio for demodulated signal - to limit Δθ to ±π/2 */

	int	  lp_sum_r, lp_sum_j;	/* state for low_pass() */
	int	  lp_index;		/* state for low_pass() - index/number of summed input values */

	int	  custom_atan;		/* 0: polar_discriminant(), 1: polar_disc_fast(),
					* 2: polar_disc_lut() - see atan_lut_init(),
					* 3: esbensen() */
	int	  pre_r, pre_j;	/* state (=last I/Q sample) for FM fm_demod() */

	int	  output_scale;	/* scaling factor after demodulation */

	int	  squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;

	int	  deemph;		/* flag: activated deemph_filter() on result[] */
	int	  deemph_a;		/* alpha for deemph_filter() */
	int	  deemph_avg;		/* state for deemph_filter() */

	int	  lpr_sum;		/* state for low_pass_real() */
	int	  lpr_index;		/* state for low_pass_real() */

	int	  dc_block_audio;	/* flag: activate dc_block_audio_filter() */
	int	  adc_avg;		/* state for dc_block_audio_filter() */
	int	  adc_block_const;	/* parameter for dc_block_audio_filter() */

	int	  omit_dc_fix;		/* flag: indicates activate dc_block_raw_filter() */

	void	 (*mode_demod)(struct demod_state*);	/* function point to one of fm_demod(), .. raw_demod() */
};

void demod_init(struct demod_state *s, int init_fields, int init_mem);
void demod_copy_fields(struct demod_state *dest, struct demod_state *src);
void demod_cleanup(struct demod_state *s);

void mixer_init(struct mixer_state *mixer, double rel_freq, double samplerate);
void mixer_apply(struct mixer_state *mixer, int len, const int16_t *inp, int16_t *out);


void rotate16_neg90(int16_t *buf, uint32_t len);

/* simple square window FIR for inphase signal
 * with decimation by (s->rate_out / s->rate_out2)
 * input is s->result[] with size d->result_len
 * output is put inplace to s->result[]; s->result_len is updated
 */
void low_pass_real(struct demod_state *s);

/* simple square window FIR for inphase signal
 * with decimation by step
 * and length len must be multiple of step
 *   => no state or history required
 * input is signal2[] with size len
 * output is put inplace to signal2[];
 * output length is returned
 */
int low_pass_simple(int16_t *signal2, int len, int step);

/* calculate root-mean-square on inphase samples[] of size len
 *   step is to skip samples, that rms() is calculated only every skip-samples.
 *   omitDCfix option/flag can be set, to skip removing the DC
 */
int rms(const int16_t *samples, int len, int step, int omitDCfix);

/* initialize lookup-table for polar_disc_lut() used with custom_atan == 2 */
int atan_lut_init(void);

/* downsample (filter and decimate) quadrature samples
 * input is d->lowpassed[] with size d->lp_len
 * output is put inplace to d->lowpassed[]; d->lp_len is updated
 * this functions combines
 *    multiple stages of fifth_order() + generic_fir()
 * or - by default - runs
 *    low_pass()
 */
void downsample_input(struct demod_state *d);

void fm_demod(struct demod_state *fm);
void am_demod(struct demod_state *fm);
void usb_demod(struct demod_state *fm);
void lsb_demod(struct demod_state *fm);
void raw_demod(struct demod_state *fm);

void deemph_filter(struct demod_state *fm);

/* remove DC from result[] */
void dc_block_audio_filter(int16_t *buf, int len, int *adc_avg_state, int adc_block_const);

/* remove DC from quadrature I/Q input - before up-mixing */
void dc_block_raw_filter(int16_t *buf, int len, int rdc_avg_state[2], int rdc_block_const);

#endif

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
