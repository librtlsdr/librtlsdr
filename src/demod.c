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
 *
 * lots of locks, but that is okay
 * (no many-to-many locks)
 *
 * todo:
 *	   sanity checks
 *	   scale squelch to other input parameters
 *	   test all the demodulations
 *	   pad output on hop
 *	   frequency ranges could be stored better
 *	   scaled AM demod amplification
 *	   auto-hop after time limit
 *	   peak detector to tune onto stronger signals
 *	   fifo for active hop frequency
 *	   clips
 *	   noise squelch
 *	   merge stereo patch
 *	   merge soft agc patch
 *	   merge udp patch
 *	   testmode to detect overruns
 *	   watchdog to reset bad dongle
 *	   fix oversampling
 */

#include "demod.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#ifdef _WIN32
#if defined(_MSC_VER) && (_MSC_VER < 1800)
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <math.h>

#define DEFAULT_SAMPLE_RATE		24000


/* detect compiler flavour */
#if defined(_MSC_VER)
#  define RESTRICT __restrict
#pragma warning( disable : 4244 4305 4204 4456 )
#elif defined(__GNUC__)
#  define RESTRICT __restrict
#else
#  define RESTRICT
#endif

#if defined(__GNUC__) || defined(__clang__)
#pragma push_macro("FORCE_INLINE")
#define FORCE_INLINE static inline __attribute__((always_inline))
#elif defined(_MSC_VER)
#define FORCE_INLINE static __forceinline
#else
#error "Macro name collisions may happen with unsupported compiler."
#ifdef FORCE_INLINE
#undef FORCE_INLINE
#endif
#define FORCE_INLINE static inline
#endif


typedef int16_t lut_type;  /* 16 bit reduces CPU cache consumption */
static lut_type *atan_lut_tab = NULL;
static int atan_lut_size = 131072; /* => 256 KB for int16_t , 512 KB for int32_t */
static int atan_lut_coef = 8;


/* {length, coef, coef, coef}  and scaled by 2^15
   for now, only length 9, optimal way to get +85% bandwidth */
static const int cic_9_tables[][10] = {
	{0,},
	{9, -156,  -97, 2798, -15489, 61019, -15489, 2798,  -97, -156},
	{9, -128, -568, 5593, -24125, 74126, -24125, 5593, -568, -128},
	{9, -129, -639, 6187, -26281, 77511, -26281, 6187, -639, -129},
	{9, -122, -612, 6082, -26353, 77818, -26353, 6082, -612, -122},
	{9, -120, -602, 6015, -26269, 77757, -26269, 6015, -602, -120},
	{9, -120, -582, 5951, -26128, 77542, -26128, 5951, -582, -120},
	{9, -119, -580, 5931, -26094, 77505, -26094, 5931, -580, -119},
	{9, -119, -578, 5921, -26077, 77484, -26077, 5921, -578, -119},
	{9, -119, -577, 5917, -26067, 77473, -26067, 5917, -577, -119},
	{9, -199, -362, 5303, -25505, 77489, -25505, 5303, -362, -199},
};


/* uint8_t negation = 255 - x */
#define NEG_U8( x )     ( 255 - x )
/* MUL_PLUS_J:    (a + j*b ) * j =  -b + j *  a */
/* MUL_MINUS_ONE: (a + j*b ) * -1 = -a + j * -b */
/* MUL_MINUS_J:   (a + j*b ) * -j =  b + j * -a */
#define MUL_PLUS_J_U8( X, J )	\
    tmp = X[J]; \
    X[J] = NEG_U8( X[J+1] ); \
    X[J+1] = tmp

#define MUL_MINUS_ONE_U8( X, J ) \
    X[J] = NEG_U8( X[J] ); \
    X[J+1] = NEG_U8( X[J+1] )

#define MUL_MINUS_J_U8( X, J ) \
    tmp = X[J]; \
    X[J] = X[J+1]; \
    X[J+1] = NEG_U8( tmp )


#define MUL_PLUS_J_INT( X, J )	\
    tmp = X[J]; \
    X[J] = - X[J+1]; \
    X[J+1] = tmp

#define MUL_MINUS_ONE_INT( X, J ) \
    X[J] = - X[J]; \
    X[J+1] = - X[J+1]

#define MUL_MINUS_J_INT( X, J ) \
    tmp = X[J]; \
    X[J] = X[J+1]; \
    X[J+1] = -tmp


void rotate16_90(int16_t *buf, uint32_t len)
{
	/* 90 degree rotation is 1, +j, -1, -j */
	uint32_t i;
	int16_t tmp;
	for (i=0; i<len; i+=8) {
		MUL_PLUS_J_INT( buf, i+2 );
		MUL_MINUS_ONE_INT( buf, i+4 );
		MUL_MINUS_J_INT( buf, i+6 );
	}
}

void rotate16_neg90(int16_t *buf, uint32_t len)
{
	/* -90 degree rotation is 1, -j, -1, +j */
	uint32_t i;
	int16_t tmp;
	for (i=0; i<len; i+=8) {
		MUL_MINUS_J_INT( buf, i+2 );
		MUL_MINUS_ONE_INT( buf, i+4 );
		MUL_PLUS_J_INT( buf, i+6 );
	}
}


void rotate_90(unsigned char *buf, uint32_t len)
{
	/* 90 degree rotation is 1, +j, -1, -j */
	uint32_t i;
	unsigned char tmp;
	for (i=0; i<len; i+=8) {
		MUL_PLUS_J_U8( buf, i+2 );
		MUL_MINUS_ONE_U8( buf, i+4 );
		MUL_MINUS_J_U8( buf, i+6 );
	}
}

void rotate_neg90(unsigned char *buf, uint32_t len)
{
	/* -90 degree rotation is 1, -j, -1, +j */
	uint32_t i;
	unsigned char tmp;
	for (i=0; i<len; i+=8) {
		MUL_MINUS_J_U8( buf, 2 );
		MUL_MINUS_ONE_U8( buf, 4 );
		MUL_PLUS_J_U8( buf, 6 );
	}
}

#define MIXER_PREC          14  /* mixer values have precision */
#define MIXER_ADD_PREC      2   /* add precision from input[] to output[] */
#define MIXER_INV_SQRT_SZ   32  /* size of mixer->inv_sqrt[] */
#define MIXER_TAB_SZ        16  /* size of mixer->mix[] */

FORCE_INLINE void mixer_fp_unit_normalize(const struct mixer_state * RESTRICT mixer, int16_t * RESTRICT c)
{
	int32_t inv_sqrt, pwr = ( (int32_t)c[0] * c[0] + (int32_t)c[1] * c[1] ) >> MIXER_PREC;
	int index = pwr - (1 << MIXER_PREC) + (MIXER_INV_SQRT_SZ / 2);
	assert( index >= 0 && index < MIXER_INV_SQRT_SZ );
	inv_sqrt = mixer->inv_sqrt[index];
	c[0] = (c[0] * inv_sqrt) >> MIXER_PREC;
	c[1] = (c[1] * inv_sqrt) >> MIXER_PREC;
}

FORCE_INLINE void mixer_fp_complex_mul(const int16_t * RESTRICT a, const int16_t * RESTRICT b, int16_t * RESTRICT c)
{
	c[0] = ( (int32_t)a[0] * b[0] - (int32_t)a[1] * b[1] ) >> MIXER_PREC;
	c[1] = ( (int32_t)a[0] * b[1] + (int32_t)a[1] * b[0] ) >> MIXER_PREC;
}

FORCE_INLINE void mixer_fp_complex_mul_pprec(const int16_t * RESTRICT a, const int16_t * RESTRICT b, int16_t * RESTRICT c)
{
	c[0] = ( (int32_t)a[0] * b[0] - (int32_t)a[1] * b[1] ) >> (MIXER_PREC - MIXER_ADD_PREC);
	c[1] = ( (int32_t)a[0] * b[1] + (int32_t)a[1] * b[0] ) >> (MIXER_PREC - MIXER_ADD_PREC);
}

typedef int16_t cplx16[2];

void mixer_init(struct mixer_state *mixer, double rel_freq, double samplerate)
{
	cplx16 * const mix = mixer->mix;
	const double flt_one = (double)(1 << MIXER_PREC);
	const double dphi = -2.0 * M_PI * rel_freq / samplerate;

	for (int k = 0; k < MIXER_INV_SQRT_SZ; ++k) {
		double v = ((1 << MIXER_PREC) + k - (MIXER_INV_SQRT_SZ / 2)) / flt_one;
		double inv_sqrt = flt_one / sqrt(v);
		mixer->inv_sqrt[k] = (int16_t)round(inv_sqrt);
	}

	mix[0][0] = (int16_t)( flt_one * cos(dphi) );
	mix[0][1] = (int16_t)( flt_one * sin(dphi) );
	mixer_fp_unit_normalize(mixer, mix[0]);
	mixer->skip = (mix[0][0] == (1 << MIXER_PREC) && mix[0][1] == 0) ? 1 : 0;

	for (int k = 1; k < MIXER_TAB_SZ; ++k) {
		mixer_fp_complex_mul(mix[k-1], mix[0], mix[k]);
		mixer_fp_unit_normalize(mixer, mix[k]);
	}

	mixer->rot[0] = 1 << MIXER_PREC;
	mixer->rot[1] = 0;

	mixer->index = -1;
}

void mixer_apply(struct mixer_state *mixer, int len, const int16_t *inp_, int16_t *out_)
{
	const cplx16 * const RESTRICT mix = mixer->mix;
	const int16_t * RESTRICT inp = inp_;
	int16_t * RESTRICT out = out_;
	int16_t rot[2], tmp[2];
	int index;

	if (mixer->skip)
	{
		if (out == inp)
			return;
		for (int i = 0; i < len; ++i)
			out[i] = inp[i];
		return;
	}

	index = mixer->index;
	rot[0] = mixer->rot[0];
	rot[1] = mixer->rot[1];
	for (int off = 0; off < len; off += 2 * MIXER_TAB_SZ) {
		const int left = (len - off >= 2 * MIXER_TAB_SZ) ? (2 * MIXER_TAB_SZ) : (len - off);
		if (index >= 0) {
			/* advance rot: rot = rot * mix[index] */
			tmp[0] = rot[0];
			tmp[1] = rot[1];
			mixer_fp_complex_mul(tmp, mix[index], rot);
			/* and normalize: rot = rot / |rot| */
			mixer_fp_unit_normalize(mixer, rot);
		}

		/* tmp allows out == inp */
		tmp[0] = inp[off];
		tmp[1] = inp[off+1];
		mixer_fp_complex_mul_pprec(tmp, rot, &out[off]);
		index = 0;
		for (int k = 2; k < left; k += 2) {
			mixer_fp_complex_mul_pprec(&inp[off + k], rot, tmp); /* tmp = inp[] * rot */
			mixer_fp_complex_mul(tmp, mix[index++], &out[off + k]); /* out[] = tmp * mix[index] */
		}
	}
	mixer->index = index;
	mixer->rot[0] = rot[0];
	mixer->rot[1] = rot[1];
}


/* simple square window FIR for quadrature signal
 * with decimation by d->downsample
 * input is d->lowpassed[] with size d->lp_len
 * output is put inplace to d->lowpassed[]; d->lp_len is updated
 * lowpass does not compensate the sum
 * produces additional ceil(log2(downsample)) bits
 * e.g. additional 5 bits for decimation from 3200 to 100 kHz
 * or   additional 4 bits for decimation from 2400 to 171 kHz
 */
void low_pass(struct demod_state *d)
/* simple square window FIR */
{
	int16_t *lowpassed = d->lowpassed;
	const int lp_len = d->lp_len;
	const int downsample = d->downsample;
	int i=0, i2=0;
	int lp_index = d->lp_index;
	int lp_sum_r = d->lp_sum_r;
	int lp_sum_j = d->lp_sum_j;

	while (i < lp_len) {
		lp_sum_r += lowpassed[i++];
		lp_sum_j += lowpassed[i++];
		lp_index++;
		if (lp_index < downsample) {
			continue;
		}
		lowpassed[i2++] = lp_sum_r; /* * d->output_scale; */
		lowpassed[i2++] = lp_sum_j; /* * d->output_scale; */
		lp_index = 0;
		lp_sum_r = 0;
		lp_sum_j = 0;
	}
	d->lp_len = i2;
	d->lp_index = lp_index;
	d->lp_sum_r = lp_sum_r;
	d->lp_sum_j = lp_sum_j;
}

int low_pass_simple(int16_t *signal2, int len, int step)
/* no wrap around, length must be multiple of step */
{
	int i, i2, sum;
	for(i=0; i < len; i+=step) {
		sum = 0;
		for(i2=0; i2<step; i2++) {
			sum += (int)signal2[i + i2];
		}
		/* signal2[i/step] = (int16_t)(sum / step); */
		signal2[i/step] = (int16_t)(sum);
	}
	signal2[i/step + 1] = signal2[i/step];
	return len / step;
}

void low_pass_real(struct demod_state *s)
/* simple square window FIR */
/* add support for upsampling? */
{
	int16_t * result = s->result;
	const int result_len = s->result_len;
	int i=0, i2=0;
	int lpr_index = s->lpr_index;
	int lpr_sum = s->lpr_sum;
	const int fast = s->rate_out;
	const int slow = s->rate_out2;
	while (i < result_len) {
		lpr_sum += result[i++];
		lpr_index += slow;
		if (lpr_index < fast) {
			continue;
		}
		result[i2++] = (int16_t)(lpr_sum / (fast/slow));
		lpr_index -= fast;
		lpr_sum = 0;
	}
	s->result_len = i2;
	s->lpr_index = lpr_index;
	s->lpr_sum = lpr_sum;
}

/* (looks like) a 6-tap decimation-by-2 filter
 * for interleaved quadrature I/Q data
 * processes every 2nd sample of data[],
 * thus needs to be called twice
 * produces 1 additional bit per call (stage)
 */
void fifth_order(int16_t *data, int length, int16_t *hist)
/* for half of interleaved data */
{
	int i;
	int16_t a, b, c, d, e, f;
	a = hist[1];
	b = hist[2];
	c = hist[3];
	d = hist[4];
	e = hist[5];
	f = data[0];
	/* downsampling improves resolution, so don't fully shift:
	 * considering multiplication with 5 and 10,
	 * one input is multiplied up to 32 (= 1+5+10+10+5+1)
	 * that would need up to additional 5 bits.
	 * => use one additional bit per decimation step
	 */
	data[0] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	for (i=4; i<length; i+=4) {
		a = c;
		b = d;
		c = e;
		d = f;
		e = data[i-2];
		f = data[i];
		data[i/2] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	}
	/* archive */
	hist[0] = a;
	hist[1] = b;
	hist[2] = c;
	hist[3] = d;
	hist[4] = e;
	hist[5] = f;
}

void fifth_order_cx(int16_t *d, int length, int16_t *hist)
{
	/* same as fifth_order() - but processes both interleaved I and Q in one call
	 *   1- idea is, that this improves cpu's memory cache utilization
	 *   2- get rid of rolling  a,b,c,d,e,f  for every 4 samples
	 * hist[] needs size 20 !
	 *
	 * h[0..9] = already in history
	 * convolution over 6 coefficients: c[0..5]
	 *     c0   c1  c2  c3  c4  c5   first convolution requires d[0] - d[1] for imag
	 *              c0  c1  c2  c3  c4  c5
	 *                      c0  c1  c2  c3  c4  c5   last convolution requires hist[10] - 11 for imag
         *                                               last convolution requires d[8] - 9 for imag
	 *     a    b   c   d   e   d0  d2  d4  d6  d8 |d10  d12
	 *                          d1  d3  d5  d7  d9 |d11  d13
	 *                           *       *       *       (*)
	 *     h0   h2  h4  h6  h8 h10 h12 h14 h16 h18
	 *     h1   h3  h5  h7  h9 h11 h13 h15 h17 h19
	 */

	int16_t *p;
	int i, len;

	assert( length >= 10 && (length % 4) == 0 );

	/* fillup history buffer with begin of data[] */
	for (i=0; i < 10; ++i)
		hist[10+i] = d[i];

	/* process samples in history buffer */
	p = hist;
	len = 20 - 10;
	for (i = 0; i < len; i += 4) {	/* i = 0, 4, 8 */
		/*          = (     a  + (    b  +     e )*5 + (    c +    d )*10 +     f   ) >> 4; */
		d[i/2  ] = ( p[i+0] + (p[i+2] + p[i+8])*5 + (p[i+4]+p[i+6])*10 + p[i+10] ) >> 4;
		d[i/2+1] = ( p[i+1] + (p[i+3] + p[i+9])*5 + (p[i+5]+p[i+7])*10 + p[i+11] ) >> 4;
	}

	/* process new samples from data[] */
	p = d;
	len = length - 10;
	for ( ; i < len; i += 4) {
		/*          = (     a  + (    b  +     e )*5 + (    c +    d )*10 +     f   ) >> 4; */
		d[i/2  ] = ( p[i+0] + (p[i+2] + p[i+8])*5 + (p[i+4]+p[i+6])*10 + p[i+10] ) >> 4;
		d[i/2+1] = ( p[i+1] + (p[i+3] + p[i+9])*5 + (p[i+5]+p[i+7])*10 + p[i+11] ) >> 4;
	}

	/* save last samples from data[] into hist[] */
	for (int k = 0; k < 10; ++k)
		hist[k] = d[i+k];
}


/* 9-tap FIR for interleaved quadrature I/Q data,
 * processes every 2nd sample of data[],
 * thus needs to be called twice.
 * FIR state (=history) is saved/read from hist[]
 * FIR coefficients are expexted in fir[1..9] - symmetric around fir[5]
 *   => only fir[1..5] are used
 * produces additional 3 bits
 */
void generic_fir(int16_t *data, int length, const int *fir, int16_t *hist)
/* Okay, not at all generic.  Assumes length 9, fix that eventually. */
{
	int d, temp, sum;
	for (d=0; d<length; d+=2) {
		temp = data[d];
		sum = 0;
		sum += (hist[0] + hist[8]) * fir[1];
		sum += (hist[1] + hist[7]) * fir[2];
		sum += (hist[2] + hist[6]) * fir[3];
		sum += (hist[3] + hist[5]) * fir[4];
		sum +=		hist[4]  * fir[5];
		/* convolution (multiplications & sum) requires additional bits.
		 * one input is multiplied up to 144156 (=sum(abs(cic_9_tables[])))
		 * that would need up to additional 18 bits (=ceil(log2(144156)))
		 * shift right by 15 => use up to 3 additional bits
		 */
		data[d] = sum >> 15;
		hist[0] = hist[1];
		hist[1] = hist[2];
		hist[2] = hist[3];
		hist[3] = hist[4];
		hist[4] = hist[5];
		hist[5] = hist[6];
		hist[6] = hist[7];
		hist[7] = hist[8];
		hist[8] = temp;
	}
}

/* define our own complex math ops
   because ARMv5 has no hardware float */

static void multiply(int ar, int aj, int br, int bj, int *cr, int *cj)
{
	*cr = ar*br - aj*bj;
	*cj = aj*br + ar*bj;
}

static int16_t polar_discriminant(int16_t ar, int16_t aj, int16_t br, int16_t bj)
{
	int cr, cj;
	float angle;
	multiply(ar, aj, br, -bj, &cr, &cj);
	angle = atan2f((float)cj, (float)cr);  /* float precision is sufficient */
	return (int16_t)(angle / 3.14159F * (1<<14));
}

static int16_t fast_atan2(int y, int x)
/* pre scaled for int16 */
{
	int yabs, angle;
	int pi4=(1<<12), pi34=3*(1<<12);  /* note pi = 1<<14 */
	if (x==0 && y==0) {
		return 0;
	}
	yabs = y;
	if (yabs < 0) {
		yabs = -yabs;
	}
	if (x >= 0) {
		angle = pi4  - pi4 * (x-yabs) / (x+yabs);
	} else {
		angle = pi34 - pi4 * (x+yabs) / (yabs-x);
	}
	if (y < 0) {
		return -angle;
	}
	return (int16_t)angle;
}

static int16_t polar_disc_fast(int16_t ar, int16_t aj, int16_t br, int16_t bj)
{
	int cr, cj;
	multiply(ar, aj, br, -bj, &cr, &cj);
	return fast_atan2(cj, cr);
}

int atan_lut_init(void)
{
	if (atan_lut_tab)  /* already initialized? */
		return 1;
	atan_lut_tab = malloc(atan_lut_size * sizeof(lut_type));

	for (int i = 0; i < atan_lut_size; i++) {
		atan_lut_tab[i] = (lut_type) (atan((double) i / (1<<atan_lut_coef)) / 3.14159 * (1<<14));
	}

	return 0;
}

static int16_t polar_disc_lut(int16_t ar, int16_t aj, int16_t br, int16_t bj)
{
	int cr, cj, x, x_abs;

	multiply(ar, aj, br, -bj, &cr, &cj);

	/* special cases */
	if (cr == 0 || cj == 0) {
		if (cr == 0 && cj == 0)
			{return 0;}
		if (cr == 0 && cj > 0)
			{return 1 << 13;}
		if (cr == 0 && cj < 0)
			{return -(1 << 13);}
		if (cj == 0 && cr > 0)
			{return 0;}
		if (cj == 0 && cr < 0)
			{return 1 << 14;}
	}

	/* real range -32768 - 32768 use 64x range -> absolute maximum: 2097152 */
	x = (cj << atan_lut_coef) / cr;
	x_abs = abs(x);

	if (x_abs >= atan_lut_size) {
		/* we can use linear range, but it is not necessary */
		return (cj > 0) ? 1<<13 : -(1<<13);
	}

	if (x > 0) {
		return (cj > 0) ? atan_lut_tab[x] : atan_lut_tab[x] - (1<<14);
	} else {
		return (cj > 0) ? (1<<14) - atan_lut_tab[-x] : -atan_lut_tab[-x];
	}

	return 0;
}

static int16_t esbensen(int ar, int aj, int br, int bj)
/*
  input signal: s(t) = a*exp(-i*w*t+p)
  a = amplitude, w = angular freq, p = phase difference
  solve w
  s' = -i(w)*a*exp(-i*w*t+p)
  s'*conj(s) = -i*w*a*a
  s'*conj(s) / |s|^2 = -i*w
*/
{
	const int scaled_pi = 2608; /* 1<<14 / (2*pi) */
	int dr = (br - ar) * 2;
	int dj = (bj - aj) * 2;
	int cj = bj*dr - br*dj; /* imag(ds*conj(s)) */
	return (scaled_pi * cj / (ar*ar+aj*aj+1));
}

void fm_demod(struct demod_state *fm)
{
	const int16_t *lp = fm->lowpassed;
	int16_t *result = fm->result;
	const int lp_len = fm->lp_len;
	switch (fm->custom_atan) {
	default:
	case 0:
		result[0] = polar_discriminant(lp[0], lp[1], fm->pre_r, fm->pre_j);
		for (int i = 2; i < (lp_len-1); i += 2)
			result[i/2] = polar_discriminant(lp[i], lp[i+1], lp[i-2], lp[i-1]);
		break;
	case 1:
		result[0] = polar_disc_fast(lp[0], lp[1], fm->pre_r, fm->pre_j);
		for (int i = 2; i < (lp_len-1); i += 2)
			result[i/2] = polar_disc_fast(lp[i], lp[i+1], lp[i-2], lp[i-1]);
		break;
	case 2:
		result[0] = polar_disc_lut(lp[0], lp[1], fm->pre_r, fm->pre_j);
		for (int i = 2; i < (lp_len-1); i += 2)
			result[i/2] = polar_disc_lut(lp[i], lp[i+1], lp[i-2], lp[i-1]);
		break;
	case 3:
		result[0] = esbensen(lp[0], lp[1], fm->pre_r, fm->pre_j);
		for (int i = 2; i < (lp_len-1); i += 2)
			result[i/2] = esbensen(lp[i], lp[i+1], lp[i-2], lp[i-1]);
		break;
	}
	fm->pre_r = lp[lp_len - 2];
	fm->pre_j = lp[lp_len - 1];
	fm->result_len = lp_len/2;
}

void am_demod(struct demod_state *fm)
/* todo, fix this extreme laziness */
{
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	const int output_scale = fm->output_scale;
	int pcm;
	for (int i = 0; i < fm->lp_len; i += 2) {
		/* hypot uses floats but won't overflow
		* r[i/2] = (int16_t)hypot(lp[i], lp[i+1]);
		*/
		pcm = (int)lp[i] * lp[i];
		pcm += (int)lp[i+1] * lp[i+1];
		r[i/2] = (int16_t)(sqrt(pcm) * output_scale);
	}
	fm->result_len = fm->lp_len/2;
	/* lowpass? (3khz)  highpass?  (dc) */
}

void usb_demod(struct demod_state *fm)
{
	const int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	const int output_scale = fm->output_scale;
	int pcm;
	for (int i = 0; i < fm->lp_len; i += 2) {
		pcm = (int)lp[i] + lp[i+1];
		r[i/2] = (int16_t)(pcm * output_scale);
	}
	fm->result_len = fm->lp_len/2;
}

void lsb_demod(struct demod_state *fm)
{
	const int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	const int output_scale = fm->output_scale;
	int pcm;
	for (int i = 0; i < fm->lp_len; i += 2) {
		pcm = (int)lp[i] - lp[i+1];
		r[i/2] = (int16_t)(pcm * output_scale);
	}
	fm->result_len = fm->lp_len/2;
}

void raw_demod(struct demod_state *fm)
{
	for (int i = 0; i < fm->lp_len; i++) {
		fm->result[i] = fm->lowpassed[i];
	}
	fm->result_len = fm->lp_len;
}

void deemph_filter(struct demod_state *fm)
{
	const int alpha = fm->deemph_a;
	const int alpha_2 = alpha /2;
	int avg = fm->deemph_avg;
	int i, d;
	/* de-emph IIR
	 * avg = avg * (1 - alpha) + sample * alpha;
	 */
	for (i = 0; i < fm->result_len; i++) {
		d = fm->result[i] - avg;
		if (d > 0) {
			avg += (d + alpha_2) / alpha;
		} else {
			avg += (d - alpha_2) / alpha;
		}
		fm->result[i] = (int16_t)avg;
	}
	fm->deemph_avg = avg;
}

void dc_block_audio_filter(int16_t *buf, int len, int *adc_avg_state, int adc_block_const)
{
	int i, avg;
	int64_t sum = 0;
	for (i=0; i < len; i++) {
		sum += buf[i];
	}
	avg = sum / len;
	avg = (avg + (*adc_avg_state) * adc_block_const) / ( adc_block_const + 1 );
	for (i=0; i < len; i++) {
		buf[i] -= avg;
	}
	*adc_avg_state = avg;
}

void dc_block_raw_filter(int16_t *buf, int len, int rdc_avg_state[2], int rdc_block_const)
{
	/* derived from dc_block_audio_filter,
		running over the raw I/Q components
	*/
	int i, avgI, avgQ;
	int64_t sumI = 0;
	int64_t sumQ = 0;
	for (i = 0; i < len; i += 2) {
		sumI += buf[i];
		sumQ += buf[i+1];
	}
	avgI = sumI / ( len / 2 );
	avgQ = sumQ / ( len / 2 );
	avgI = (avgI + rdc_avg_state[0] * rdc_block_const) / ( rdc_block_const + 1 );
	avgQ = (avgQ + rdc_avg_state[1] * rdc_block_const) / ( rdc_block_const + 1 );
	for (i = 0; i < len; i += 2) {
		buf[i] -= avgI;
		buf[i+1] -= avgQ;
	}
	rdc_avg_state[0] = avgI;
	rdc_avg_state[1] = avgQ;
}

static int mad(const int16_t *samples, int len, int step)
/* mean average deviation */
{
	int i=0, sum=0, ave=0;
	if (len == 0)
		{return 0;}
	for (i=0; i<len; i+=step) {
		sum += samples[i];
	}
	ave = sum / (len * step);
	sum = 0;
	for (i=0; i<len; i+=step) {
		sum += abs(samples[i] - ave);
	}
	return sum / (len / step);
}

int rms(const int16_t *samples, int len, int step, int omitDCfix)
/* largely lifted from rtl_power */
{
	double dc, err;
	int i, num;
	int32_t t, s;
	uint32_t p;  /* use sign bit to prevent overflow */

	p = 0;
	t = 0L;
	while (len > step * 32768) /* 8 bit squared = 16 bit. limit to 2^16 for 32 bit squared sum */
		++step;  /* increase step to prevent overflow */
	for (i=0; i<len; i+=step) {
		s = (long)samples[i];
		t += s;
		p += s * s;
	}

	if (omitDCfix) {
		/* DC is already corrected. No need to do it again */
		num = len / step;
		return (int)sqrt( (double)(p) / num );
	}

	/* correct for dc offset in squares */
	dc = (double)(t*step) / (double)len;
	err = t * 2 * dc - dc * dc * len;

	return (int)sqrt((p-err) / len);
}

static void arbitrary_upsample(const int16_t *buf1, int16_t *buf2, int len1, int len2)
/* linear interpolation, len1 < len2 */
{
	int i = 1;
	int j = 0;
	int tick = 0;
	double frac;  /* use integers... */
	while (j < len2) {
		frac = (double)tick / (double)len2;
		buf2[j] = (int16_t)(buf1[i-1]*(1-frac) + buf1[i]*frac);
		j++;
		tick += len1;
		if (tick > len2) {
			tick -= len2;
			i++;
		}
		if (i >= len1) {
			i = len1 - 1;
			tick = len2;
		}
	}
}

static void arbitrary_downsample(const int16_t *buf1, int16_t *buf2, int len1, int len2)
/* fractional boxcar lowpass, len1 > len2 */
{
	int i = 1;
	int j = 0;
	int tick = 0;
	double remainder = 0;
	double frac;  /* use integers... */
	buf2[0] = 0;
	while (j < len2) {
		frac = 1.0;
		if ((tick + len2) > len1) {
			frac = (double)(len1 - tick) / (double)len2;}
		buf2[j] += (int16_t)((double)buf1[i] * frac + remainder);
		remainder = (double)buf1[i] * (1.0-frac);
		tick += len2;
		i++;
		if (tick > len1) {
			j++;
			buf2[j] = 0;
			tick -= len1;
		}
		if (i >= len1) {
			i = len1 - 1;
			tick = len1;
		}
	}
	for (j=0; j<len2; j++) {
		buf2[j] = buf2[j] * len2 / len1;}
}

static void arbitrary_resample(const int16_t *buf1, int16_t *buf2, int len1, int len2)
/* up to you to calculate lengths and make sure it does not go OOB
 * okay for buffers to overlap, if you are downsampling */
{
	if (len1 < len2) {
		arbitrary_upsample(buf1, buf2, len1, len2);
	} else {
		arbitrary_downsample(buf1, buf2, len1, len2);
	}
}


void downsample_input(struct demod_state *d)
{
	const int ds_p = d->downsample_passes;
	if (ds_p) {
		/* this is executed with "rtl_fm -F 9".
		 * this consumes approx twice the cpu usage (time: user)
		 *   compared to low_pass() in other branch
		 */
		/* this branch might overflow 16 bits!:
		 *   8 bits from input
		 * + 3 bits from generic_fir()
		 * + 1 bit per per stage
		 * = 11 + number of stages
		 * => overflow with more than 5 stages!
		 */
		for (int i=0; i < ds_p; i++) {
#if 1
			fifth_order(d->lowpassed,   (d->lp_len >> i), d->lp_i_hist[i]);
			fifth_order(d->lowpassed+1, (d->lp_len >> i) - 1, d->lp_q_hist[i]);
#else
			fifth_order_cx(d->lowpassed, (d->lp_len >> i), d->lp_iq_hist[i]);
#endif
		}
		d->lp_len = d->lp_len >> ds_p;
		/* droop compensation */
		if (d->comp_fir_size == 9 && ds_p <= MAXIMUM_DOWNSAMPLE_PASSES) {
			generic_fir(d->lowpassed, d->lp_len,
				cic_9_tables[ds_p], d->droop_i_hist);
			generic_fir(d->lowpassed+1, d->lp_len-1,
				cic_9_tables[ds_p], d->droop_q_hist);
		}
	} else {
		/* default */
		low_pass(d);
	}
}

static void full_demod_light(struct demod_state *d)
/* copied from rtl_fm.c full_demod(), removed squelch, level printing, ..
 * demonstrates full usage
 */
{
	downsample_input(d);

	d->mode_demod(d);  /* lowpassed -> result */
	if (d->mode_demod == &raw_demod) {
		return;
	}
	/* todo, fm noise squelch */
	/* use nicer filter here too? */
	if (d->post_downsample > 1) {
		d->result_len = low_pass_simple(d->result, d->result_len, d->post_downsample);}
	if (d->deemph) {
		deemph_filter(d);}
	if (d->dc_block_audio) {
		dc_block_audio_filter(d->result, d->result_len, &(d->adc_avg), d->adc_block_const);}
	if (d->rate_out2 > 0) {
		low_pass_real(d);
		/* arbitrary_resample(d->result, d->result, d->result_len, d->result_len * d->rate_out2 / d->rate_out); */
	}
}

void demod_init(struct demod_state *s)
{
	s->comp_fir_size = 0;

	s->rate_in = DEFAULT_SAMPLE_RATE;
	s->rate_out = DEFAULT_SAMPLE_RATE;
	s->rate_out2 = -1;	// flag for disabled

	s->downsample = 1;
	s->downsample_passes = 0;
	s->post_downsample = 1;	// once this works, default = 4

	s->squelch_level = 0;
	s->conseq_squelch = 10;
	s->terminate_on_squelch = 0;
	s->squelch_hits = 11;

	s->custom_atan = 0;
	s->pre_j = s->pre_r = 0;

	s->mode_demod = &fm_demod;

	s->lp_index = 0;
	s->lp_sum_r = s->lp_sum_j = 0;

	s->lpr_index = 0;
	s->lpr_sum = 0;

	s->deemph = 0;
	s->deemph_a = 0;
	s->deemph_avg = 0;

	s->dc_block_audio = 0;
	s->adc_avg = 0;
	s->adc_block_const = 9;

	s->omit_dc_fix = 0;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
