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

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#if defined(_MSC_VER) && (_MSC_VER < 1900)
#define snprintf _snprintf
#endif
#if defined(_MSC_VER) && (_MSC_VER < 1800)
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#ifdef NEED_PTHREADS_WORKARROUND
#define HAVE_STRUCT_TIMESPEC
#endif

#include <math.h>
#include <pthread.h>
#include <libusb.h>

#include <rtl-sdr.h>
#include <rtl_app_ver.h>
#include "convenience/convenience.h"
#include "convenience/rtl_convenience.h"
#include "convenience/wavewrite.h"
#include "demod.h"

#define DEFAULT_SAMPLE_RATE		24000
#define AUTO_GAIN				-100

#define FREQUENCIES_LIMIT		64
#define NUM_CIRCULAR_BUFFERS		4	/* power of 2 ! */

static int MinCaptureRate = 1000000;

static volatile int do_exit = 0;
static int verbosity = 0;


time_t stop_time;
int duration = 0;

struct demod_input_buffer
{
	int16_t   lowpassed[MAXIMUM_BUF_LENGTH];	/* input and decimated quadrature I/Q sample-pairs */
	int	  lp_len;		/* number of valid samples in lowpassed[] - NOT quadrature I/Q sample-pairs! */
	int	  is_free;	/* 0 == free; 1 == occupied with data */
	pthread_rwlock_t	rw;
};

struct demod_thread_state
{

	struct mixer_state mixers[FREQUENCIES_LIMIT];
	struct demod_state demods[FREQUENCIES_LIMIT];
	int	  num_channels;

	struct demod_input_buffer	buffers[NUM_CIRCULAR_BUFFERS];
	int	  buffer_write_idx;
	int	  buffer_read_idx;

	pthread_t		thread;
	pthread_cond_t		ready;
	pthread_mutex_t	ready_m;
	struct output_state	*output_target;
};

struct dongle_state
{
	pthread_t thread;
	rtlsdr_dev_t *dev;
	int	  dev_index;
	uint64_t freq;
	uint32_t rate;
	uint32_t bandwidth;
	int	  gain;
	int16_t  buf16[MAXIMUM_BUF_LENGTH];
	uint32_t buf_len;
	int	  ppm_error;
	int	  direct_sampling;
	int	  mute;
	struct demod_thread_state *demod_target;

	int	  dc_block_raw;	/* flag: activate dc_block_raw_filter() */
	int	  rdc_avg[2];		/* state for dc_block_raw_filter() */
	int	  rdc_block_const;	/* parameter for dc_block_raw_filter() */
};

struct output_state
{
	FILE	 *file;
	char	 *filename;
	char	 *tempfilename;
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int	  result_len;
	int	  rate;
};

struct controller_state
{
	int32_t   freqs[FREQUENCIES_LIMIT];	/* relative to center_freq */
	int	  freq_len;
	uint32_t  center_freq;
};

/* multiple of these, eventually */
struct dongle_state dongle;
struct demod_thread_state dm_thr;
struct output_state output;
struct controller_state controller;


void usage(void)
{
	fprintf(stderr,
		"rtl_multichannel, a multichannel demodulator for RTL2832 based SDR-receivers\n"
		"rtl_multichannel  version %d.%d %s (%s)\n"
		"rtl-sdr library %d.%d %s\n\n",
		APP_VER_MAJOR, APP_VER_MINOR, APP_VER_ID, __DATE__,
		rtlsdr_get_version() >>16, rtlsdr_get_version() & 0xFFFF,
		rtlsdr_get_ver_id() );
	fprintf(stderr,
		"Usage:\trtl_multichannel -f freq [-options] [filename]\n"
		"\t-f frequency_to_tune_to [Hz]\n"
		"\t	use multiple -f for parallel demodulation\n"
		"\t	ranges supported, -f 118M:137M:25k\n"
		"\t[-v increase verbosity (default: 0)]\n"
		"\t[-M modulation (default: fm)]\n"
		"\t	fm or nbfm or nfm, wbfm or wfm, raw or iq, am, usb, lsb\n"
		"\t	wbfm == -M fm -s 170k -A fast -r 32k -E deemp\n"
		"\t	raw mode outputs 2x16 bit IQ pairs\n"
		"\t[-m minimum_capture_rate Hz (default: 1m, min=900k, max=3.2m)]\n"
		"\t[-s sample_rate (default: 24k)]\n"
		"\t[-r resample_rate (default: none / same as -s)]\n"
		"\t[-d device_index or serial (default: 0)]\n"
		"\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]\n"
		"\t[-D direct_sampling_mode (default: 0, 1 = I, 2 = Q, 3 = I below threshold, 4 = Q below threshold)]\n"
		"\t[-D direct_sampling_threshold_frequency (default: 0 use tuner specific frequency threshold for 3 and 4)]\n"
		"\t[-g tuner_gain (default: automatic)]\n"
		"\t[-w tuner_bandwidth in Hz (default: automatic)]\n"
		"\t[-W length of single buffer in units of 512 samples (default: 32 was 256)]\n"
		"\t[-c de-emphasis_time_constant in us for wbfm. 'us' or 'eu' for 75/50 us (default: us)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-R run_seconds] specify number of seconds to run\n"
		"\t[-E enable_option (default: none)]\n"
		"\t	use multiple -E to enable multiple options\n"
		"\t	rdc:    enable dc blocking filter on raw I/Q data at capture rate\n"
		"\t	adc:    enable dc blocking filter on demodulated audio\n"
		"\t	rtlagc: enable rtl2832's digital agc (default: off)\n"
		"\t	deemp:  enable de-emphasis filter\n"
		"\t	direct: enable direct sampling (bypasses tuner, uses rtl2832 xtal)\n"
		"%s"
		"\t[-q dc_avg_factor for option rdc (default: 9)]\n"
		"\t[-H write wave Header to file (default: off)]\n"
		"\t	limitation: only 1st tuned frequency will be written into the header!\n"
		"\tfilename ('-' means stdout)\n"
		"\t	omitting the filename also uses stdout\n\n"
		"Experimental options:\n"
		"\t[-F fir_size (default: off)]\n"
		"\t	enables low-leakage downsample filter\n"
		"\t	size can be 0 or 9.  0 has bad roll off\n"
		"\t[-A std/fast/lut/ale choose atan math (default: std)]\n"
		"\n"
		"Produces signed 16 bit ints, use Sox or aplay to hear them.\n"
		"\trtl_multichannel ... | play -t raw -r 24k -es -b 16 -c 1 -V1 -\n"
		"\t		   | aplay -r 24000 -f S16_LE -t raw -c 1\n"
		"\t  -M wbfm  | play -r 32k ... \n"
		"\t  -s 22050 | multimon -t raw /dev/stdin\n\n"
		, rtlsdr_get_opt_help(1) );
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dongle.dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dongle.dev);
}
#endif

/* more cond dumbness */
#define safe_cond_signal(n, m) do { pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m); } while (0)
#define safe_cond_wait(n, m)   do { pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m); } while (0)

#if defined(_MSC_VER) && (_MSC_VER < 1800)
static double log2(double n)
{
	return log(n) / log(2.0);
}
#endif


void full_demod(struct demod_state *d)
{
	downsample_input(d);

	d->mode_demod(d);  /* lowpassed -> result */
	if (d->mode_demod == &raw_demod) {
		return;
	}
	/* use nicer filter here too? */
	if (d->deemph) {
		deemph_filter(d);}
	if (d->dc_block_audio) {
		dc_block_audio_filter(d->result, d->result_len, &(d->adc_avg), d->adc_block_const);}
	if (d->rate_out2 > 0) {
		low_pass_real(d);
		/* arbitrary_resample(d->result, d->result, d->result_len, d->result_len * d->rate_out2 / d->rate_out); */
	}
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct dongle_state *s = ctx;
	struct demod_thread_state *dt = s->demod_target;
	struct demod_input_buffer *buffer;
	int i, write_idx;
	time_t rawtime;

	if (do_exit) {
		return;}
	if (!ctx) {
		return;}
	time(&rawtime);
	if (duration > 0 && rawtime >= stop_time) {
		do_exit = 1;
		fprintf(stderr, "Time expired, exiting!\n");
		rtlsdr_cancel_async(dongle.dev);
	}
	/* OR all samples to allow checking overflow
	 * - before conversion to 16 bit and before DC filtering.
	 * we only get bitmask of positive samples (after -127) but that won't matter */
	/* 1st: convert to 16 bit - to allow easier calculation of DC */
	for (i=0; i<(int)len; i++) {
		s->buf16[i] = ( (int16_t)buf[i] - 127 );
	}
	/* 2nd: do DC filtering BEFORE up-mixing */
	if (s->dc_block_raw) {
		dc_block_raw_filter(s->buf16, (int)len, s->rdc_avg, s->rdc_block_const);
	}

	write_idx = dt->buffer_write_idx;
	dt->buffer_write_idx = (dt->buffer_write_idx + 1) % NUM_CIRCULAR_BUFFERS;
	buffer = &dt->buffers[write_idx];
	pthread_rwlock_wrlock(&buffer->rw);	/* lock before writing into demod_thread_state.lowpassed */
	if (!buffer->is_free) {
		do_exit = 1;
		fprintf(stderr, "Overflow of circular input buffers, exiting!\n");
		rtlsdr_cancel_async(dongle.dev);
		pthread_rwlock_unlock(&buffer->rw);
		return;
	}
	memcpy(buffer->lowpassed, s->buf16, 2*len);
	buffer->lp_len = len;
	buffer->is_free = 0;
	pthread_rwlock_unlock(&buffer->rw);
	safe_cond_signal(&dt->ready, &dt->ready_m);
}

static void *dongle_thread_fn(void *arg)
{
	struct dongle_state *s = arg;
	rtlsdr_read_async(s->dev, rtlsdr_callback, s, 0, s->buf_len);
	return 0;
}

static void *demod_thread_fn(void *arg)
{
	struct demod_thread_state *dt = arg;
	struct demod_state *d0 = &dt->demods[0];
	struct output_state *o = &output;
	struct demod_input_buffer *buffer;
	int ch, read_idx;
	while (!do_exit) {
		safe_cond_wait(&dt->ready, &dt->ready_m);

		read_idx = dt->buffer_read_idx;
		dt->buffer_read_idx = (dt->buffer_read_idx + 1) % NUM_CIRCULAR_BUFFERS;
		buffer = &dt->buffers[read_idx];
		pthread_rwlock_wrlock(&buffer->rw);	/* lock before reading into demod_thread_state.lowpassed */

		for (ch = 0; ch < dt->num_channels; ++ch) {
			dt->demods[ch].lp_len = buffer->lp_len;
			mixer_apply(&dt->mixers[ch], buffer->lp_len, buffer->lowpassed, dt->demods[ch].lowpassed);
		}

		buffer->is_free = 1;	/* buffer can be written again */
		/* we only need to lock the lowpassed buffer of demod_thread_state */
		pthread_rwlock_unlock(&buffer->rw);

		for (ch = 0; ch < dt->num_channels; ++ch) {
			full_demod(&dt->demods[ch]);
			/* todo: have a popen() where to post results for each channel */
		}

		if (do_exit)
			break;

		/* only write result of 1st channel to file - for now */
		if (!waveHdrStarted)
			fwrite(d0->result, 2, d0->result_len, o->file);
		else
			waveWriteSamples(o->file, d0->result, d0->result_len, 0);
	}
	return 0;
}

static void *output_fn(void *arg)
{
	struct output_state *s = arg;
	if (!waveHdrStarted) {
		while (!do_exit) {
			/* use timedwait and pad out under runs */
			fwrite(s->result, 2, s->result_len, s->file);
		}
	} else {
		while (!do_exit) {
			/* use timedwait and pad out under runs */
			/* distinguish for endianness: wave requires little endian */
			waveWriteSamples(s->file, s->result, s->result_len, 0);
		}
	}
	return 0;
}

static int optimal_settings(uint64_t freq, uint32_t rate)
{
	/* giant ball of hacks
	 * seems unable to do a single pass, 2:1
	 */
	uint32_t capture_rate;
	struct dongle_state *d = &dongle;
	struct demod_thread_state *dt = &dm_thr;
	struct demod_state *dm = &dt->demods[0];	/* calculate for 1st one */
	struct controller_state *cs = &controller;
	dm->downsample = (MinCaptureRate / dm->rate_in) + 1;
	if (dm->downsample_passes) {
		dm->downsample_passes = (int)log2(dm->downsample) + 1;
		if (dm->downsample_passes > MAXIMUM_DOWNSAMPLE_PASSES) {
			fprintf(stderr, "downsample_passes = %d exceeds it's limit. setting to %d\n", dm->downsample, MAXIMUM_DOWNSAMPLE_PASSES);
			dm->downsample_passes = MAXIMUM_DOWNSAMPLE_PASSES;
		}
		dm->downsample = 1 << dm->downsample_passes;
	}
	if (verbosity >= 2) {
		fprintf(stderr, "downsample_passes = %d (= # of fifth_order() iterations), downsample = %d\n", dm->downsample_passes, dm->downsample );
	}
	capture_rate = dm->downsample * dm->rate_in;
	if (capture_rate > 3200U*1000U) {
		fprintf(stderr, "Error: Capture rate of %u Hz exceedds 3200k!\n", (unsigned)capture_rate);
		return 1;
	}
	else if (capture_rate > 2400U*1000U) {
		fprintf(stderr, "Warning: Capture rate of %u Hz is too big (exceeds 2400k) for continous transfer!\n", (unsigned)capture_rate);
	}
	if (verbosity >= 2)
		fprintf(stderr, "capture_rate = dm->downsample * dm->rate_in = %d * %d = %d\n", dm->downsample, dm->rate_in, capture_rate );
	dm->output_scale = (1<<15) / (128 * dm->downsample);
	if (dm->output_scale < 1) {
		dm->output_scale = 1;}
	if (dm->mode_demod == &fm_demod) {
		dm->output_scale = 1;}
	d->freq = freq;
	d->rate = capture_rate;
	if (verbosity >= 2)
		fprintf(stderr, "optimal_settings(freq = %f MHz) delivers freq %f MHz, rate %.0f\n", freq * 1E-6, d->freq * 1E-6, (double)d->rate );
	return 0;
}

static int controller_fn(struct controller_state *s)
{
	int i, r;
	int32_t dongle_rate, nyq_min, nyq_max;
	struct demod_state *demod0 = &dm_thr.demods[0];

	r = optimal_settings(s->center_freq, demod0->rate_in);
	if (r) {
		return r;
	}
	if (dongle.direct_sampling) {
		verbose_direct_sampling(dongle.dev, 1);}

	/* Set the frequency */
	if (verbosity) {
		fprintf(stderr, "verbose_set_frequency(%f MHz)\n", dongle.freq * 1E-6);
	}
	verbose_set_frequency(dongle.dev, dongle.freq);

	dongle_rate = dongle.rate;
	nyq_max = dongle_rate /2;
	for (i = 0; i < s->freq_len; ++i) {
		if (s->freqs[i] <= -nyq_max || s->freqs[i] >= nyq_max) {
			fprintf(stderr, "Warning: frequency for channel %d (=%d Hz) is out of Nyquist band!\n", i, (int)s->freqs[i]);
			fprintf(stderr, "  nyquist band is from %d .. %d Hz\n", -nyq_max, nyq_max);
		}
		mixer_init(&dm_thr.mixers[i], s->freqs[i], dongle.rate);
		if (i)
			/* distribute demod setting from 1st to all */
			demod_copy_fields(&dm_thr.demods[i], demod0);
		/* allocate memory per channel */
		demod_init(&dm_thr.demods[i], 0, 1);
	}
	dm_thr.num_channels = s->freq_len;
	fprintf(stderr, "Multichannel will demodulate %d channels.\n", dm_thr.num_channels);

	fprintf(stderr, "Oversampling input by: %ix.\n", demod0->downsample);
	fprintf(stderr, "Buffer size: %u Bytes == %u quadrature samples == %0.2fms\n",
		(unsigned)dongle.buf_len,
		(unsigned)dongle.buf_len / 2,
		1000 * 0.5 * (float)dongle.buf_len / (float)dongle.rate);

	/* Set the sample rate */
	if (verbosity)
		fprintf(stderr, "verbose_set_sample_rate(%.0f Hz)\n", (double)dongle.rate);
	verbose_set_sample_rate(dongle.dev, dongle.rate);
	fprintf(stderr, "Output at %u Hz.\n", demod0->rate_in/demod0->post_downsample);

	return 0;
}

void frequency_range(struct controller_state *s, char *arg)
{
	char *start, *stop, *step;
	int i;
	start = arg;
	stop = strchr(start, ':') + 1;
	stop[-1] = '\0';
	step = strchr(stop, ':') + 1;
	step[-1] = '\0';
	for(i=(int)atofs(start); i<=(int)atofs(stop); i+=(int)atofs(step))
	{
		s->freqs[s->freq_len] = (int32_t)i;
		s->freq_len++;
		if (s->freq_len >= FREQUENCIES_LIMIT) {
			break;}
	}
	stop[-1] = ':';
	step[-1] = ':';
}

void dongle_init(struct dongle_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	s->gain = AUTO_GAIN; /* tenths of a dB */
	s->mute = 0;
	s->direct_sampling = 0;
	s->demod_target = &dm_thr;
	s->bandwidth = 0;
	s->buf_len = 32 * 512;  /* see rtl_tcp */

	s->dc_block_raw = 0;
	s->rdc_avg[0] = 0;
	s->rdc_avg[1] = 0;
	s->rdc_block_const = 9;
}

void demod_thread_init(struct demod_thread_state *s)
{
	s->num_channels = 0;
	for (int ch = 0; ch < FREQUENCIES_LIMIT; ++ch)
		demod_init(&s->demods[ch], 1, 0);

	for (int idx = 0; idx < NUM_CIRCULAR_BUFFERS; ++idx) {
		pthread_rwlock_init(&s->buffers[idx].rw, NULL);
		s->buffers[idx].is_free = 1;
	}
	s->buffer_write_idx = 0;
	s->buffer_read_idx = 0;

	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
}

void demod_thread_cleanup(struct demod_thread_state *s)
{
	for (int ch = 0; ch < FREQUENCIES_LIMIT; ++ch)
		demod_cleanup(&s->demods[ch]);
	for (int idx = 0; idx < NUM_CIRCULAR_BUFFERS; ++idx) {
		pthread_rwlock_destroy(&s->buffers[idx].rw);
	}
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void output_init(struct output_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
}

void output_cleanup(struct output_state *s)
{
}

void controller_init(struct controller_state *s)
{
	s->center_freq = 100000000;
	s->freq_len = -1;
}

void sanity_checks(void)
{
	if (controller.freq_len <= 0) {
		fprintf(stderr, "Please specify a center frequency (RF) and one more relative frequency\n");
		exit(1);
	}
	else if (controller.freq_len == 1) {
		fprintf(stderr, "Warning: With only one channel, prefer 'rtl_fm', which will have better performance.\n");
	}

	if (controller.freq_len >= FREQUENCIES_LIMIT) {
		fprintf(stderr, "Too many channels, maximum %i.\n", FREQUENCIES_LIMIT);
		exit(1);
	}

}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	int r, opt;
	int dev_given = 0;
	int writeWav = 0;
	int enable_biastee = 0;
	const char * rtlOpts = NULL;
	enum rtlsdr_ds_mode ds_mode = RTLSDR_DS_IQ;
	uint32_t ds_temp, ds_threshold = 0;
	int timeConstant = 75; /* default: U.S. 75 uS */
	int rtlagc = 0;
	struct demod_state *demod = &dm_thr.demods[0];	/* prepare settings for 1st demod/channel */
	dongle_init(&dongle);
	demod_thread_init(&dm_thr);
	output_init(&output);
	controller_init(&controller);

	while ((opt = getopt(argc, argv, "d:f:g:m:s:r:p:R:E:O:F:A:M:hTq:c:w:W:D:Hv")) != -1) {
		switch (opt) {
		case 'd':
			dongle.dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			if (controller.freq_len >= FREQUENCIES_LIMIT) {
				break;}
			if (strchr(optarg, ':')) {
				if ( controller.freq_len == -1 ) {
					fprintf(stderr, "error: 1st frequency parameter must be single frequency\n");
					exit(1);
				}
				frequency_range(&controller, optarg);
			}
			else
			{
				if ( controller.freq_len >= 0 )
					controller.freqs[controller.freq_len] = (int32_t)atofs(optarg);
				else
					controller.center_freq = (uint32_t)atofs(optarg);
				controller.freq_len++;
			}
			break;
		case 'g':
			dongle.gain = (int)(atof(optarg) * 10);
			break;
		case 'm':
			MinCaptureRate = (int)atofs(optarg);
			break;
		case 's':
			demod->rate_in = (uint32_t)atofs(optarg);
			demod->rate_out = (uint32_t)atofs(optarg);
			break;
		case 'r':
			output.rate = (int)atofs(optarg);
			demod->rate_out2 = (int)atofs(optarg);
			break;
		case 'p':
			dongle.ppm_error = atoi(optarg);
			break;
		case 'R':
			time(&stop_time);
			duration = atoi(optarg);
			if (duration < 1) {
				fprintf(stderr, "Duration '%s' was not positive integer; will continue indefinitely\n", optarg);
			} else {
				stop_time += duration;
			}
			break;
		case 'E':
			if (strcmp("adc", optarg) == 0) {
				demod->dc_block_audio = 1;}
			if (strcmp("rdc", optarg) == 0) {
				dongle.dc_block_raw = demod->omit_dc_fix = 1;}
			if (strcmp("deemp",  optarg) == 0) {
				demod->deemph = 1;}
			if (strcmp("direct",  optarg) == 0) {
				dongle.direct_sampling = 1;}
			if (strcmp("rtlagc", optarg) == 0) {
				rtlagc = 1;}
			break;
		case 'O':
			rtlOpts = optarg;
			break;
		case 'q':
			dongle.rdc_block_const = atoi(optarg);
			break;
		case 'F':
			demod->downsample_passes = 1;  /* truthy placeholder */
			demod->comp_fir_size = atoi(optarg);
			break;
		case 'A':
			if (strcmp("std",  optarg) == 0) {
				demod->custom_atan = 0;}
			if (strcmp("fast", optarg) == 0) {
				demod->custom_atan = 1;}
			if (strcmp("lut",  optarg) == 0) {
				atan_lut_init();
				demod->custom_atan = 2;}
			if (strcmp("ale", optarg) == 0) {
				demod->custom_atan = 3;}
			break;
		case 'M':
			if (strcmp("nbfm",  optarg) == 0 || strcmp("nfm",  optarg) == 0 || strcmp("fm",  optarg) == 0) {
				demod->mode_demod = &fm_demod;}
			if (strcmp("raw",  optarg) == 0 || strcmp("iq",  optarg) == 0) {
				demod->mode_demod = &raw_demod;}
			if (strcmp("am",  optarg) == 0) {
				demod->mode_demod = &am_demod;}
			if (strcmp("usb", optarg) == 0) {
				demod->mode_demod = &usb_demod;}
			if (strcmp("lsb", optarg) == 0) {
				demod->mode_demod = &lsb_demod;}
			if (strcmp("wbfm",  optarg) == 0 || strcmp("wfm",  optarg) == 0) {
				demod->mode_demod = &fm_demod;
				demod->rate_in = 170000;
				demod->rate_out = 170000;
				demod->rate_out2 = 32000;
				output.rate = 32000;
				demod->custom_atan = 1;
				//demod.post_downsample = 4;
				demod->deemph = 1;
				demod->squelch_level = 0;}
			break;
		case 'T':
			enable_biastee = 1;
			break;
		case 'c':
			if (strcmp("us",  optarg) == 0)
				timeConstant = 75;
			else if (strcmp("eu", optarg) == 0)
				timeConstant = 50;
			else
				timeConstant = (int)atof(optarg);
			break;
		case 'D':
			ds_temp = (uint32_t)( atofs(optarg) + 0.5 );
			if (ds_temp <= RTLSDR_DS_Q_BELOW)
				ds_mode = (enum rtlsdr_ds_mode)ds_temp;
			else
				ds_threshold = ds_temp;
			break;
		case 'H':
			writeWav = 1;
			break;
		case 'v':
			++verbosity;
			break;
		case 'w':
			dongle.bandwidth = (uint32_t)atofs(optarg);
			break;
		case 'W':
			dongle.buf_len = 512 * atoi(optarg);
			if (dongle.buf_len > MAXIMUM_BUF_LENGTH) {
				fprintf(stderr, "Warning: limiting buffers from option -W to %d\n", MAXIMUM_BUF_LENGTH / 512);
				dongle.buf_len = MAXIMUM_BUF_LENGTH;
			}
			break;
		case 'h':
		case '?':
		default:
			usage();
			break;
		}
	}

	if (verbosity)
		fprintf(stderr, "verbosity set to %d\n", verbosity);

	/* quadruple sample_rate to limit to Δθ to ±π/2 */
	demod->rate_in *= demod->post_downsample;

	if (!output.rate) {
		output.rate = demod->rate_out;}

	sanity_checks();

	if (optind < argc) {
		output.filename = argv[optind];
	} else {
		output.filename = "-";
	}

	if (!dev_given) {
		dongle.dev_index = verbose_device_search("0");
	}

	if (dongle.dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dongle.dev, (uint32_t)dongle.dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dongle.dev_index);
		exit(1);
	}
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	if (demod->deemph) {
		double tc = (double)timeConstant * 1e-6;
		demod->deemph_a = (int)round(1.0/((1.0-exp(-1.0/(demod->rate_out * tc)))));
		if (verbosity)
			fprintf(stderr, "using wbfm deemphasis filter with time constant %d us\n", timeConstant );
	}

	/* Set the tuner gain */
	if (dongle.gain == AUTO_GAIN) {
		verbose_auto_gain(dongle.dev);
	} else {
		dongle.gain = nearest_gain(dongle.dev, dongle.gain);
		verbose_gain_set(dongle.dev, dongle.gain);
	}

	rtlsdr_set_agc_mode(dongle.dev, rtlagc);

	rtlsdr_set_bias_tee(dongle.dev, enable_biastee);
	if (enable_biastee)
		fprintf(stderr, "activated bias-T on GPIO PIN 0\n");

	verbose_ppm_set(dongle.dev, dongle.ppm_error);

	/* Set direct sampling with threshold */
	rtlsdr_set_ds_mode(dongle.dev, ds_mode, ds_threshold);

	verbose_set_bandwidth(dongle.dev, dongle.bandwidth);

	if (verbosity && dongle.bandwidth)
	{
		int r;
		uint32_t in_bw, out_bw, last_bw = 0;
		fprintf(stderr, "Supported bandwidth values in kHz:\n");
		for ( in_bw = 1; in_bw < 3200; ++in_bw )
		{
			r = rtlsdr_set_and_get_tuner_bandwidth(dongle.dev, in_bw*1000, &out_bw, 0 /* =apply_bw */);
			if ( r == 0 && out_bw != 0 && ( out_bw != last_bw || in_bw == 1 ) )
				fprintf(stderr, "%s%.1f", (in_bw==1 ? "" : ", "), out_bw/1000.0 );
			last_bw = out_bw;
		}
		fprintf(stderr,"\n");
	}

	if (rtlOpts) {
		rtlsdr_set_opt_string(dongle.dev, rtlOpts, verbosity);
	}

	if (strcmp(output.filename, "-") == 0) { /* Write samples to stdout */
		output.file = stdout;
#ifdef _WIN32
		_setmode(_fileno(output.file), _O_BINARY);
#endif
	} else {
		const char * filename_to_open = output.filename;
		if (writeWav) {
			output.tempfilename = malloc( strlen(output.filename)+8 );
			strcpy(output.tempfilename, output.filename);
			strcat(output.tempfilename, ".tmp");
			filename_to_open = output.tempfilename;
		}
 		output.file = fopen(filename_to_open, "wb");
		if (!output.file) {
			fprintf(stderr, "Failed to open %s\n", filename_to_open);
			exit(1);
		}
		else
		{
			fprintf(stderr, "Open %s for write\n", filename_to_open);
			if (writeWav) {
				int nChan = (demod->mode_demod == &raw_demod) ? 2 : 1;
				int srate = (demod->rate_out2 > 0) ? demod->rate_out2 : demod->rate_out;
				uint32_t f = controller.freqs[0];	/* only 1st frequency!!! */
				waveWriteHeader(srate, f, 16, nChan, output.file);
			}
		}
	}

	//r = rtlsdr_set_testmode(dongle.dev, 1);

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dongle.dev);

	r = controller_fn(&controller);
	if (r) {
		rtlsdr_close(dongle.dev);
		return 1;
	}
	/* usleep(1000000); * it looks, that startup of dongle level takes some time at startup! */
	pthread_create(&dm_thr.thread, NULL, demod_thread_fn, (void *)(&dm_thr));
	pthread_create(&dongle.thread, NULL, dongle_thread_fn, (void *)(&dongle));

	while (!do_exit) {
		usleep(100000);
	}

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");}
	else {
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);}

	rtlsdr_cancel_async(dongle.dev);
	pthread_join(dongle.thread, NULL);
	safe_cond_signal(&dm_thr.ready, &dm_thr.ready_m);
	pthread_join(dm_thr.thread, NULL);

	/* dongle_cleanup(&dongle); */
	demod_thread_cleanup(&dm_thr);

	if (output.file != stdout) {
		if (writeWav) {
			int r;
			waveFinalizeHeader(output.file);
			fclose(output.file);
			remove(output.filename);	/* delete, in case file already exists */
			r = rename( output.tempfilename, output.filename );	/* #include <stdio.h> */
			if ( r )
				fprintf( stderr, "%s: error %d '%s' renaming'%s' to '%s'\n"
					, argv[0], errno, strerror(errno), output.tempfilename, output.filename );
		} else {
			fclose(output.file);
		}
	}

	rtlsdr_close(dongle.dev);
	return r >= 0 ? r : -r;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
