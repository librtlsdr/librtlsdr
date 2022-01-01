/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 * Copyright (C) 2015 by Hayati Ayguen <h_ayguen@web.de>
 * Copyright (C) 2021 by Ahmet Genç <ahmetgenc93@gmail.com>
 * Copyright (C) 2021 by Omer Faruk Kirli <omerfarukkirli@gmail.com>
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

#include <stdatomic.h>

#include <rtl-sdr.h>
#include <rtl_app_ver.h>
#include "convenience/convenience.h"
#include "convenience/rtl_convenience.h"
#include "convenience/wavewrite.h"
#include "demod.h"

#define DEFAULT_SAMPLE_RATE		24000
#define AUTO_GAIN				-100

#define MAX_NUM_CHANNELS        32
#define MAX_CIRCULAR_BUFFERS    32

#define DEFAULT_MPX_PIPE_PATTERN	"redsea -r %sf >%F_%H-%M-%S_%#_ch-%cf_rds.txt"
#define DEFAULT_MPX_FILE_PATTERN	"%F_%H-%M-%S_%#_ch-%cf_mpx.raw"
#define DEFAULT_AUDIO_PIPE_PATTERN	"ffmpeg -f s16le -ar %sf -ac 1 -i pipe: -loglevel quiet %F_%H-%M-%S_%#_ch-%cf_audio.mp3"
#define DEFAULT_AUDIO_FILE_PATTERN	"%F_%H-%M-%S_%#_ch-%cf_audio.raw"

/* type: 0: nothing, 1: file, 2: pipe */
#define DEFAULT_AUDIO_WRITE_TYPE	1
#define DEFAULT_MPX_WRITE_TYPE		0

#define likely(x)	__builtin_expect(!!(x), 1)
#define unlikely(x)	__builtin_expect(!!(x), 0)
#define ARRAY_LEN(x)	(sizeof(x)/sizeof(x[0]))

static int MinCaptureRate = 1000000;

static volatile int do_exit = 0;
static int verbosity = 0;

time_t stop_time;
int duration = 0;

struct demod_input_buffer
{
	int16_t * lowpassed; /* input and decimated quadrature I/Q sample-pairs */
	int	  lp_len;		/* number of valid samples in lowpassed[] - NOT quadrature I/Q sample-pairs! */
	atomic_int  is_free;	/* 0 == free; 1 == occupied with data */
	pthread_rwlock_t	rw;
};

struct p_closing_thread
{	
	FILE *fptr;
	atomic_int has_file;
	pthread_t closing_thread;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
};

struct demod_thread_state
{
	struct mixer_state mixers[MAX_NUM_CHANNELS];
	struct demod_state demod_states[MAX_NUM_CHANNELS];

	pthread_t thread;
	int channel_count;
	struct demod_input_buffer	buffers[MAX_CIRCULAR_BUFFERS];
	int	  num_circular_buffers;
	int	  buffer_write_idx;
	int	  buffer_read_idx;

	FILE    * faudio[MAX_NUM_CHANNELS];
	FILE    * fmpx[MAX_NUM_CHANNELS];
	const char *audio_pipe_pattern;
	const char *mpx_pipe_pattern;
	const char *audio_file_pattern;
	const char *mpx_file_pattern;
	int audio_write_type;	/* 0: nothing, 1: file, 2: pipe */
	int mpx_write_type;
	struct p_closing_thread audio_p_closing_thread[MAX_NUM_CHANNELS]; /* specific for pipe closing */
	struct p_closing_thread mpx_p_closing_thread[MAX_NUM_CHANNELS]; /* specific for pipe closing */

	int mpx_split_duration;		/* split file or pipe in seconds */
	int audio_split_duration;

	int mpx_limit_duration;	/* limit per split in seconds */
	int audio_limit_duration;

	int32_t freqs[MAX_NUM_CHANNELS];
	uint32_t center_freq;

	pthread_cond_t ready;
	pthread_mutex_t ready_m;
	pthread_rwlock_t rw;
};

struct dongle_state
{
	rtlsdr_dev_t *dev;
	int	  dev_index;
	uint64_t freq;
	uint32_t rate;
	uint32_t bandwidth;
	int	  gain;
	uint32_t buf_len;
	int	  ppm_error;
	int	  direct_sampling;
	int	  mute;
	struct demod_thread_state *demod_target;

	int	  dc_block_raw;	/* flag: activate dc_block_raw_filter() */
	int	  rdc_avg[2];		/* state for dc_block_raw_filter() */
	int	  rdc_block_const;	/* parameter for dc_block_raw_filter() */
};

/* multiple of these, eventually */
struct dongle_state dongle;
struct demod_thread_state dm_thr;

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
		"\t	fm or nbfm or nfm, wbfm or wfm, mwfm, raw or iq, am, usb, lsb\n"
		"\t	wbfm == -M fm -s 170k -A fast -r 32k -E deemp\n"
		"\t	mwfm == -M fm -s 171k -r 21375\n"
		"\t	raw mode outputs 2x16 bit IQ pairs\n"
		"\t[-m minimum_capture_rate Hz (default: 1m, min=900k, max=3.2m)]\n"
		"\t[-s sample_rate (default: 24k)]\n"
		"\t[-t [x:|a:]split duration in seconds to split files (default: off)]\n"
		"\t     x:|a:   split the mpx or audio signal. without that prefix, both are split\n"
		"\t[-l [x:|a:]limit duration in seconds from (split) begin (default: off)]\n"
		"\t[-a audio pipe command or file pattern (default: file) with substitutions similar to strftime(), ie.\n"
		"\t\t\"p:%s\"\n"
		"\t\t\"a:%s\"\n"
		"\t\t\"p:\" uses the default pipe pattern, \"f:\" uses the default file pattern, \"n\" deactivates audio output.\n"
		"\t[-x mpx signal pipe command or file pattern (default: off)\n"
		"\t\tthe default pipe command pattern: \"p:%s\"\n"
		"\t\tthe default filename pattern: \"f:%s\"\n"
		"\t\t  substitutions, additional to strftime(): %%f for frequency, %%kf for frequency in kHz,\n"
		"\t\t  %%sf for samplerate, %%cf for channel number, and %%# for milliseconds of time\n"
		"\t[-r resample_rate (default: none / same as -s)]\n"
		"\t[-d device_index or serial (default: 0)]\n"
		"\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]\n"
		"\t[-D direct_sampling_mode (default: 0, 1 = I, 2 = Q, 3 = I below threshold, 4 = Q below threshold)]\n"
		"\t[-D direct_sampling_threshold_frequency (default: 0 use tuner specific frequency threshold for 3 and 4)]\n"
		"\t[-g tuner_gain (default: automatic)]\n"
		"\t[-w tuner_bandwidth in Hz (default: automatic)]\n"
		"\t[-W length of single buffer in units of 512 samples (default: 32 was 256)]\n"
		"\t[-n number of circular input buffers (default: 4, max: 32)]\n"
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
		/* "\t[-H write wave Header to file (default: off)]\n" */
		"Experimental options:\n"
		"\t[-F fir_size (default: off)]\n"
		"\t	enables low-leakage downsample filter\n"
		"\t	size can be 0 or 9.  0 has bad roll off\n"
		"\t[-A std/fast/lut/ale choose atan math (default: std)]\n"
		"\n"
		, DEFAULT_AUDIO_PIPE_PATTERN, DEFAULT_AUDIO_FILE_PATTERN
		, DEFAULT_MPX_PIPE_PATTERN, DEFAULT_MPX_FILE_PATTERN
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


int strfchannel(char *s, int max, const char * format, const int chno, const uint32_t freq, const unsigned rate, const int milli, const int rep_esc)
{
	/* rep_esc == 0: %% is NOT replaced for later call to strftime() */
	/* replaced strings:
	 * %f  => frequency in Hz
	 * %kf => frequency in kHz
	 * %cf => channel number: 0 ..
	 * %sf => sample frequency = rate
	 * %#  => ms
	 */
	int r, dest_pos = 0;
	const int fmt_len = strlen(format);
	const char escape = '%';

	for (int fmt_pos = 0; fmt_pos < fmt_len; ) {
		if (dest_pos >= max -1)
			break;
		if (format[fmt_pos] == escape) {
			if (format[fmt_pos+1] == escape) {
				if (rep_esc) {
					s[dest_pos++] = format[fmt_pos++];
				}
				else {
					/* rep_esc == 0: %% is NOT replaced for later call to strftime() */
					s[dest_pos++] = format[fmt_pos++];
					s[dest_pos++] = format[fmt_pos++];
				}
			}
			else if (format[fmt_pos+1] == '#') {
				/* %# : ms */
				r = snprintf(&s[dest_pos], max - dest_pos, "%03d", milli);
				dest_pos += r;
				fmt_pos += 2;
			}
			else if (format[fmt_pos+1] == 'f') {
				/* %f : frequency in Hz */
				r = snprintf(&s[dest_pos], max - dest_pos, "%u", (unsigned)freq);
				dest_pos += r;
				fmt_pos += 2;
			}
			else if (format[fmt_pos+1] == 'k' && format[fmt_pos+2] == 'f') {
				/* %kf : frequency in kHz */
				r = snprintf(&s[dest_pos], max - dest_pos, "%u", (unsigned)(freq / 1000U));
				dest_pos += r;
				fmt_pos += 3;
			}
			else if (format[fmt_pos+1] == 'c' && format[fmt_pos+2] == 'f') {
				/* %cf : channel number: 0 .. */
				r = snprintf(&s[dest_pos], max - dest_pos, "%d", chno);
				dest_pos += r;
				fmt_pos += 3;
			}
			else if (format[fmt_pos+1] == 's' && format[fmt_pos+2] == 'f') {
				/* %sf : sample frequency = rate */
				r = snprintf(&s[dest_pos], max - dest_pos, "%u", (unsigned)rate);
				dest_pos += r;
				fmt_pos += 3;
			}
			else {
				s[dest_pos++] = format[fmt_pos++];
			}
		}
		else {
			s[dest_pos++] = format[fmt_pos++];
		}
	}
	s[dest_pos++] = 0;
	if (max > 0)
		s[max -1] = 0;
	return dest_pos;
}

static FILE * open_pipe(const char *command, const char * st) {
	FILE *f = popen(command, "w");
	if (!f) {
		fprintf(stderr, "Error: Could not open %s pipe for '%s' !\n", st, command);
		return f;
	}
	fprintf(stdout, "%s pipe opened: '%s'\n", st, command);
	return f;
}

static FILE * create_iq_file(const char* filename, const char * st) {
	FILE * f = fopen(filename, "wb");
	if(!f) {
		fprintf(stderr, "Error: Could not create %s file '%s' !\n", st, filename);
		return f;
	}

	fprintf(stdout, "%s file created: %s\n", st, filename);
	return f;
}

static void *close_pipe_fn(void* arg) {
	struct p_closing_thread *ct = arg;
	while (!do_exit) {
		safe_cond_wait(&ct->ready, &ct->ready_m);
		if(!ct->has_file){
			fprintf(stderr, "Error, has file\n");
			ct->has_file = 0;
			continue;
		}
		if (ct->fptr){
			fflush(ct->fptr);
			pclose(ct->fptr);
		}
		ct->has_file = 0;
	}
	return 0;
}

void close_all_channel_outputs(struct demod_thread_state *s, int close_mpx, int close_audio, int use_thread)
{
	for(int ch = 0; ch < s->channel_count; ++ch) {
		FILE *f = s->faudio[ch];
		if (f && close_audio) {
			if (s->audio_write_type == 2) {
				if(use_thread == 1) {
					if(s->audio_p_closing_thread[ch].has_file == 0) {
						fprintf(stderr, "error, pclose has not finished\n");
					}
					s->audio_p_closing_thread[ch].fptr = f;
					s->audio_p_closing_thread[ch].has_file = 1;
					safe_cond_signal(&s->audio_p_closing_thread[ch].ready, &s->audio_p_closing_thread[ch].ready_m);
				} else {
					fflush(f);
					pclose(f);	
				}
			} else if (s->audio_write_type == 1) {
				fflush(f);
				fclose(f);
			}
			s->faudio[ch] = NULL;
		}

		f = s->fmpx[ch];
		if (f && close_mpx) {
			if (s->mpx_write_type == 2)
				if(use_thread == 1) {
					s->mpx_p_closing_thread[ch].fptr = f;
					safe_cond_signal(&s->mpx_p_closing_thread[ch].ready, &s->mpx_p_closing_thread[ch].ready_m);
				} else {
					fflush(f);
					pclose(f);	
				}
			else if (s->mpx_write_type == 1)
			{
				fflush(f);
				fclose(f);			
			}
			s->fmpx[ch] = NULL;
		}
	}
}

void open_all_mpx_channel_outputs(struct demod_thread_state *s, const unsigned mpx_rate, const struct tm *tm, const int milli)
{
	static char fnTmp[1024];
	static char fnTim[1024];

	for(int ch = 0; ch < s->channel_count; ++ch) {
		uint32_t freq = s->center_freq + s->freqs[ch];

		if (s->mpx_write_type == 2) {
			strfchannel(fnTmp, ARRAY_LEN(fnTmp), s->mpx_pipe_pattern, ch, freq, mpx_rate, milli, 0);
			strftime(fnTim, ARRAY_LEN(fnTim), fnTmp, tm);
			s->fmpx[ch] = open_pipe(fnTim, "mpx");
		} else if (s->mpx_write_type == 1) {
			strfchannel(fnTmp, ARRAY_LEN(fnTmp), s->mpx_file_pattern, ch, freq, mpx_rate, milli, 0);
			strftime(fnTim, ARRAY_LEN(fnTim), fnTmp, tm);
			s->fmpx[ch] = create_iq_file(fnTim, "mpx");
		}
	}
}

void open_all_audio_channel_outputs(struct demod_thread_state *s, const unsigned audio_rate, const struct tm *tm, const int milli)
{
	static char fnTmp[1024];
	static char fnTim[1024];

	for(int ch = 0; ch < s->channel_count; ++ch) {
		uint32_t freq = s->center_freq + s->freqs[ch];

		if (s->audio_write_type == 2) {
			strfchannel(fnTmp, ARRAY_LEN(fnTmp), s->audio_pipe_pattern, ch, freq, audio_rate, milli, 0);
			strftime(fnTim, ARRAY_LEN(fnTim), fnTmp, tm);
			s->faudio[ch] = open_pipe(fnTim, "audio");
		} else if (s->audio_write_type == 1) {
			strfchannel(fnTmp, ARRAY_LEN(fnTmp), s->audio_file_pattern, ch, freq, audio_rate, milli, 0);
			strftime(fnTim, ARRAY_LEN(fnTim), fnTmp, tm);
			s->faudio[ch] = create_iq_file(fnTim, "audio");
		}
	}
}


void full_demod(struct demod_state *d, FILE *f_mpx, FILE *f_audio)
{
	int nwritten;

	downsample_input(d);

	d->mode_demod(d);  /* lowpassed -> result */
	if (d->mode_demod == &raw_demod) {
		return;
	}

	if (f_mpx) {
		nwritten = (int)fwrite(d->result, 2, d->result_len, f_mpx);
		if (nwritten != d->result_len)
			fprintf(stderr, "error writing %d mpx samples .. result %d\n", d->result_len, nwritten);
	}

	if (!f_audio) {
		/* no need for audio? */
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

	nwritten = (int)fwrite(d->result, 2, d->result_len, f_audio);
	if (nwritten != d->result_len)
		fprintf(stderr, "error writing %d audio samples .. result %d\n", d->result_len, nwritten);
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct dongle_state *s = ctx;
	struct demod_thread_state *mds = s->demod_target;
	struct demod_input_buffer *buffer;
	int16_t *buf16;
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

	write_idx = mds->buffer_write_idx;
	mds->buffer_write_idx = (mds->buffer_write_idx + 1) % mds->num_circular_buffers;
	buffer = &mds->buffers[write_idx];
	pthread_rwlock_wrlock(&buffer->rw);	/* lock before writing into demod_thread_state.lowpassed */
	if (!buffer->is_free) {
		do_exit = 1;
		fprintf(stderr, "* * * Overflow of circular input buffers, exiting! * * *\n");
		rtlsdr_cancel_async(dongle.dev);
		pthread_rwlock_unlock(&buffer->rw);
		return;
	}

	buf16 = buffer->lowpassed;

	/* 1st: convert to 16 bit - to allow easier calculation of DC */
	for (i=0; i<(int)len; i++) {
		buf16[i] = ( (int16_t)buf[i] - 127 );
	}
	/* 2nd: do DC filtering BEFORE up-mixing */
	if (s->dc_block_raw) {
		dc_block_raw_filter(buf16, (int)len, s->rdc_avg, s->rdc_block_const);
	}

	buffer->lp_len = len;

	buffer->is_free = 0;
	pthread_rwlock_unlock(&buffer->rw);
	safe_cond_signal(&mds->ready, &mds->ready_m);
}

static void *multi_demod_thread_fn(void *arg)
{
	struct demod_thread_state *mds = arg;

	struct timeval t1_mpx, t1_audio, t2;
	struct timeval timestamp;
	double diff_ms_mpx, diff_ms_audio;
	int ch, read_idx;
	int init_open = 1;
	int mpx_is_limited = 1;
	int audio_is_limited = 1;
	const unsigned mpx_rate = mds->demod_states[0].rate_out;
	const unsigned audio_rate = mds->demod_states[0].rate_out2 ? (unsigned)mds->demod_states[0].rate_out2 : mpx_rate;
	struct demod_input_buffer *buffer;

	gettimeofday(&t1_mpx, NULL);
	t1_audio = t1_mpx;
	while (!do_exit) {
		safe_cond_wait(&mds->ready, &mds->ready_m);

		gettimeofday(&t2, NULL);
		diff_ms_mpx    = (t2.tv_sec  - t1_mpx.tv_sec) * 1000.0;
		diff_ms_mpx   += (t2.tv_usec - t1_mpx.tv_usec) / 1000.0;
		diff_ms_audio  = (t2.tv_sec  - t1_audio.tv_sec) * 1000.0;
		diff_ms_audio += (t2.tv_usec - t1_audio.tv_usec) / 1000.0;

		/* check limits first. else, files are immediately closed again, cause diff_ms still > .. */
		if (unlikely(!mpx_is_limited && mds->mpx_limit_duration > 0 && diff_ms_mpx > 1000.0 * mds->mpx_limit_duration)) {
			mpx_is_limited = 1;
			close_all_channel_outputs(mds, 1, 0, 1);	/* close mpx files or pipes */
			if (verbosity)
				fprintf(stderr, "closing mpx stream, cause of limit\n");
		}
		if (unlikely(!audio_is_limited && mds->audio_limit_duration > 0 && diff_ms_audio > 1000.0 * mds->audio_limit_duration)) {
			audio_is_limited = 1;
			close_all_channel_outputs(mds, 0, 1, 1);	/* close audio files or pipes */
			if (verbosity)
				fprintf(stderr, "closing audio stream, cause of limit\n");
		}

		if (unlikely(init_open || (mds->mpx_split_duration > 0 && diff_ms_mpx > 1000.0 * mds->mpx_split_duration))) {
			time_t current_time = time(NULL);
			struct tm *tm = localtime(&current_time);
			const int milli = (int)(t2.tv_usec/1000);
			close_all_channel_outputs(mds, 1, 0, 1);	/* close mpx files or pipes */
			open_all_mpx_channel_outputs(mds, mpx_rate, tm, milli);
			mpx_is_limited = 0;
			t1_mpx = t2;
		}
		if (unlikely(init_open || (mds->audio_split_duration > 0 && diff_ms_audio > 1000.0 * mds->audio_split_duration))) {
			time_t current_time = time(NULL);
			struct tm *tm = localtime(&current_time);
			const int milli = (int)(t2.tv_usec/1000);
			close_all_channel_outputs(mds, 0, 1, 1);	/* close audio files or pipes */
			open_all_audio_channel_outputs(mds, audio_rate, tm, milli);
			audio_is_limited = 0;
			t1_audio = t2;
		}
		init_open = 0;

		read_idx = mds->buffer_read_idx;
		mds->buffer_read_idx = (mds->buffer_read_idx + 1) % mds->num_circular_buffers;
		buffer = &mds->buffers[read_idx];
		pthread_rwlock_wrlock(&buffer->rw);	/* lock before reading into demod_thread_state.lowpassed */

		for (ch = 0; ch < mds->channel_count; ++ch) {
			mds->demod_states[ch].lp_len = buffer->lp_len;
			mixer_apply(&mds->mixers[ch], buffer->lp_len, buffer->lowpassed, mds->demod_states[ch].lowpassed);
		}

		buffer->is_free = 1;	/* buffer can be written again */
		/* we only need to lock the lowpassed buffer of demod_thread_state */
		pthread_rwlock_unlock(&buffer->rw);

		for (ch = 0; ch < mds->channel_count; ++ch) {
			full_demod(&mds->demod_states[ch], mds->fmpx[ch], mds->faudio[ch]);
		}

		if (do_exit)
			break;

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
	struct demod_state *config = &dt->demod_states[0];
	config->downsample = (MinCaptureRate / config->rate_in) + 1;
	if (config->downsample_passes) {
		config->downsample_passes = (int)log2(config->downsample) + 1;
		if (config->downsample_passes > MAXIMUM_DOWNSAMPLE_PASSES) {
			fprintf(stderr, "downsample_passes = %d exceeds it's limit. setting to %d\n", config->downsample, MAXIMUM_DOWNSAMPLE_PASSES);
			config->downsample_passes = MAXIMUM_DOWNSAMPLE_PASSES;
		}
		config->downsample = 1 << config->downsample_passes;
	}
	if (verbosity >= 2) {
		fprintf(stderr, "downsample_passes = %d (= # of fifth_order() iterations), downsample = %d\n", config->downsample_passes, config->downsample );
	}
	capture_rate = config->downsample * config->rate_in;
	if (capture_rate > 3200U*1000U) {
		fprintf(stderr, "Error: Capture rate of %u Hz exceedds 3200k!\n", (unsigned)capture_rate);
		return 1;
	}
	else if (capture_rate > 2400U*1000U) {
		fprintf(stderr, "Warning: Capture rate of %u Hz is too big (exceeds 2400k) for continous transfer!\n", (unsigned)capture_rate);
	}
	if (verbosity >= 2)
		fprintf(stderr, "capture_rate = dm->downsample * dm->rate_in = %d * %d = %d\n", config->downsample, config->rate_in, capture_rate );
	config->output_scale = (1<<15) / (128 * config->downsample);
	if (config->output_scale < 1) {
		config->output_scale = 1;}
	if (config->mode_demod == &fm_demod) {
		config->output_scale = 1;}
	d->freq = freq;
	d->rate = capture_rate;
	if (verbosity >= 2)
		fprintf(stderr, "optimal_settings(freq = %f MHz) delivers freq %f MHz, rate %.0f\n", freq * 1E-6, d->freq * 1E-6, (double)d->rate );
	return 0;
}

static int controller_fn(struct demod_thread_state *s)
{
	int i, r;
	int32_t dongle_rate, nyq_min, nyq_max;
	struct demod_state *demod_config = &dm_thr.demod_states[0];

	r = optimal_settings(s->center_freq, demod_config->rate_in);
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
	for (i = 0; i < s->channel_count; ++i) {
		fprintf(stderr, " freq %lu will be recording soon \n", dongle.freq+s->freqs[i]);
		if (s->freqs[i] <= -nyq_max || s->freqs[i] >= nyq_max) {
			fprintf(stderr, "Warning: frequency for channel %d (=%d Hz) is out of Nyquist band!\n", i, (int)s->freqs[i]);
			fprintf(stderr, "  nyquist band is from %d .. %d Hz\n", -nyq_max, nyq_max);
		}
		mixer_init(&dm_thr.mixers[i], s->freqs[i], dongle.rate);
		/* distribute demod setting from 1st (=config) to all */
		demod_copy_fields(&dm_thr.demod_states[i], demod_config);
		/* allocate memory per channel */
		demod_init(&dm_thr.demod_states[i], 0, 1);
	}
	fprintf(stderr, "Multichannel will demodulate %d channels.\n", dm_thr.channel_count);

	fprintf(stderr, "Oversampling input by: %ix.\n", demod_config->downsample);
	fprintf(stderr, "Buffer size: %u Bytes == %u quadrature samples == %0.2fms\n",
		(unsigned)dongle.buf_len,
		(unsigned)dongle.buf_len / 2,
		1000 * 0.5 * (float)dongle.buf_len / (float)dongle.rate);

	/* Set the sample rate */
	if (verbosity)
		fprintf(stderr, "verbose_set_sample_rate(%.0f Hz)\n", (double)dongle.rate);
	verbose_set_sample_rate(dongle.dev, dongle.rate);
	fprintf(stderr, "Output at %u Hz.\n", demod_config->rate_in/demod_config->post_downsample);

	return 0;
}

void frequency_range(struct demod_thread_state *s, char *arg)
{
	char *start, *stop, *step;
	int32_t i;
	start = arg;
	stop = strchr(start, ':') + 1;
	stop[-1] = '\0';
	step = strchr(stop, ':') + 1;
	step[-1] = '\0';
	for(i=(int32_t)atofs(start); i<=(int32_t)atofs(stop); i+=(int32_t)atofs(step))
	{
		s->freqs[s->channel_count] = i;
		s->channel_count++;
		if (s->channel_count >= MAX_NUM_CHANNELS) {
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

void init_demods()
{
	struct demod_state *conf = &dm_thr.demod_states[0];
	for(int i=0; i<dm_thr.channel_count; i++) {
		struct demod_state *s = &dm_thr.demod_states[i];
		demod_copy_fields(s, conf);
		demod_init(s, 0, 1);
	}
}

void demod_thread_state_init(struct demod_thread_state *s)
{
	for (int ch = 0; ch < MAX_NUM_CHANNELS; ++ch) {
		demod_init(&s->demod_states[ch], 1, 0);
		s->faudio[ch] = NULL;
		s->fmpx[ch] = NULL;
	}

	s->num_circular_buffers = 4;
	s->buffer_write_idx = 0;
	s->buffer_read_idx = 0;
	s->mpx_split_duration = -1;
	s->audio_split_duration = -1;
	s->mpx_limit_duration = 0;
	s->audio_limit_duration = 0;
	s->channel_count = -1;

	for (int i = 0; i < MAX_CIRCULAR_BUFFERS; i++) {
		pthread_rwlock_init(&s->buffers[i].rw, NULL);
		s->buffers[i].is_free = 1;
		s->buffers[i].lowpassed = NULL;
	}

	s->mpx_pipe_pattern = DEFAULT_MPX_PIPE_PATTERN;
	s->mpx_file_pattern = DEFAULT_MPX_FILE_PATTERN;
	s->audio_pipe_pattern = DEFAULT_AUDIO_PIPE_PATTERN;
	s->audio_file_pattern = DEFAULT_AUDIO_FILE_PATTERN;

	s->audio_write_type = DEFAULT_AUDIO_WRITE_TYPE;
	s->mpx_write_type = DEFAULT_MPX_WRITE_TYPE;

	s->center_freq = 100000000;

	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
}

void demod_thread_init_ring_buffers(struct demod_thread_state *s, const int buflen)
{
	for (int i = 0; i < s->num_circular_buffers; i++) {
		s->buffers[i].lowpassed = (int16_t*)malloc( sizeof(int16_t) * buflen );
	}
}

void multi_demod_init_fptrs(struct demod_thread_state *s)
{
	if (s->audio_write_type == 2 && !s->audio_pipe_pattern) {
		fprintf(stderr, "Error: command pattern for audio is missing! Exiting..\n");
		exit(0);
	}

	if (s->mpx_write_type == 2 && !s->mpx_pipe_pattern) {
		fprintf(stderr, "Error: command pattern for mpx is missing! Exiting..\n");
		exit(0);
	}
}

void demod_thread_cleanup(struct demod_thread_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);

	close_all_channel_outputs(s, 1, 1, 0);

	for(int i=0; i<s->channel_count; i++) {
		demod_cleanup(&s->demod_states[i]);

		pthread_cond_destroy(&s->audio_p_closing_thread[i].ready);
		pthread_cond_destroy(&s->mpx_p_closing_thread[i].ready);
		pthread_mutex_destroy(&s->audio_p_closing_thread[i].ready_m);
		pthread_mutex_destroy(&s->mpx_p_closing_thread[i].ready_m);
	}

	for (int k=0; k < MAX_CIRCULAR_BUFFERS; ++k) {
		if (s->buffers[k].lowpassed) {
			free(s->buffers[k].lowpassed);
			s->buffers[k].lowpassed = NULL;
		}
	}
}

void sanity_checks(void)
{
	if (dm_thr.channel_count <= 0) {
		fprintf(stderr, "Please specify a center frequency (RF) and one more relative frequency\n");
		exit(1);
	}
	else if (dm_thr.channel_count == 1) {
		fprintf(stderr, "Warning: With only one channel, prefer 'rtl_fm', which will have better performance.\n");
	}

	if (dm_thr.channel_count >= MAX_NUM_CHANNELS) {
		fprintf(stderr, "Too many channels, maximum %i.\n", MAX_NUM_CHANNELS);
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
	int enable_biastee = 0;
	const char * rtlOpts = NULL;
	struct demod_state *demod = NULL;
	enum rtlsdr_ds_mode ds_mode = RTLSDR_DS_IQ;
	uint32_t ds_temp, ds_threshold = 0;
	int timeConstant = 75; /* default: U.S. 75 uS */
	int rtlagc = 0;
	dongle_init(&dongle);
	demod_thread_state_init(&dm_thr);
	demod = &dm_thr.demod_states[0];

	while ((opt = getopt(argc, argv, "d:f:g:m:s:a:x:t:l:r:p:R:E:O:F:A:M:hTq:c:w:W:n:D:v")) != -1) {
		switch (opt) {
		case 'd':
			dongle.dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			if (dm_thr.channel_count >= MAX_NUM_CHANNELS) {
				break;}
			if (strchr(optarg, ':')) {
				if ( dm_thr.channel_count == -1 ) {
					fprintf(stderr, "error: 1st frequency parameter must be single frequency\n");
					exit(1);
				}
				frequency_range(&dm_thr, optarg);
			}
			else
			{
				if ( dm_thr.channel_count >= 0 )
					dm_thr.freqs[dm_thr.channel_count] = (int32_t)atofs(optarg);
				else
					dm_thr.center_freq = (int32_t)atofs(optarg);
				dm_thr.channel_count++;
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
			demod->rate_out2 = (int)atofs(optarg);
			break;
		case 't':
			if (!strncmp(optarg, "x:", 2)) {
				dm_thr.mpx_split_duration = atoi(&optarg[2]);
			} else if (!strncmp(optarg, "a:", 2)) {
				dm_thr.audio_split_duration = atoi(&optarg[2]);
			} else {
				dm_thr.audio_split_duration = atoi(optarg);
				dm_thr.mpx_split_duration = dm_thr.audio_split_duration;
			}
			break;
		case 'l':
			if (!strncmp(optarg, "x:", 2)) {
				dm_thr.mpx_limit_duration = atoi(&optarg[2]);
			} else if (!strncmp(optarg, "a:", 2)) {
				dm_thr.audio_limit_duration = atoi(&optarg[2]);
			} else {
				dm_thr.audio_limit_duration = atoi(optarg);
				dm_thr.mpx_limit_duration = dm_thr.audio_limit_duration;
			}
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
		case 'a':
			if (!strcmp(optarg, "n") || !strcmp(optarg, "N") || !strcmp(optarg, "%")) {
				dm_thr.audio_write_type = 0;
			} else if (!strncmp(optarg, "p:", 2)) {
				if (optarg[2])
					dm_thr.audio_pipe_pattern = &optarg[2];
				else
					fprintf(stderr, "activating audio pipe output - keeping previous pattern: %s\n", dm_thr.audio_pipe_pattern);
				dm_thr.audio_write_type = 2;
			} else if (!strncmp(optarg, "f:", 2)) {
				if (optarg[2])
					dm_thr.audio_file_pattern = &optarg[2];
				else
					fprintf(stderr, "activating audio file output - keeping previous pattern: %s\n", dm_thr.audio_file_pattern);
				dm_thr.audio_write_type = 1;
			} else {
				dm_thr.audio_file_pattern = optarg;
				dm_thr.audio_write_type = 1;
			}
			break;
		case 'x':
			if (!strcmp(optarg, "n") || !strcmp(optarg, "N") || !strcmp(optarg, "%")) {
				dm_thr.mpx_write_type = 0;
			} else if (!strncmp(optarg, "p:", 2)) {
				if (optarg[2])
					dm_thr.mpx_pipe_pattern = &optarg[2];
				else
					fprintf(stderr, "activating mpx pipe output - keeping previous pattern: %s\n", dm_thr.mpx_pipe_pattern);
				dm_thr.mpx_write_type = 2;
			} else if (!strncmp(optarg, "f:", 2)) {
				if (optarg[2])
					dm_thr.mpx_file_pattern = &optarg[2];
				else
					fprintf(stderr, "activating mpx file output - keeping previous pattern: %s\n", dm_thr.mpx_file_pattern);
				dm_thr.mpx_write_type = 1;
			} else {
				dm_thr.audio_file_pattern = optarg;
				dm_thr.mpx_write_type = 1;
			}
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
				demod->custom_atan = 1;
				demod->deemph = 1;
			}
			if (strcmp("mwfm", optarg) == 0) {
				demod->mode_demod = &fm_demod;
				demod->rate_in = 171000;
				demod->rate_out = 171000;
				demod->rate_out2 = 21375;	/* = 171000 / 8 */
				/* atan_lut_init();
				demod->custom_atan = 2; */
			}
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
		case 'n':
			dm_thr.num_circular_buffers = atoi(optarg);
			if (dm_thr.num_circular_buffers < 2) {
				fprintf(stderr, "Warning: lower limit for option -n is 2\n");
				dm_thr.num_circular_buffers = 2;    /* minimum 2 buffers */
			}
			if (dm_thr.num_circular_buffers > MAX_CIRCULAR_BUFFERS) {
				fprintf(stderr, "Warning: limit circular buffers from option -n to %d\n", MAX_CIRCULAR_BUFFERS);
				dm_thr.num_circular_buffers = MAX_CIRCULAR_BUFFERS;
			}
			break;
		case 'h':
		case '?':
		default:
			usage();
			break;
		}
	}

	if (demod->deemph) {
		double tc = (double)timeConstant * 1e-6;
		demod->deemph_a = (int)round(1.0/((1.0-exp(-1.0/(demod->rate_out * tc)))));
		if (verbosity)
			fprintf(stderr, "using wbfm deemphasis filter with time constant %d us\n", timeConstant );
	}

	/* demod_thread_init_ring_buffers(&dm_thr, MAXIMUM_BUF_LENGTH); */
	demod_thread_init_ring_buffers(&dm_thr, dongle.buf_len);
	multi_demod_init_fptrs(&dm_thr);
	init_demods();

	if (verbosity)
		fprintf(stderr, "verbosity set to %d\n", verbosity);

	/* quadruple sample_rate to limit to Δθ to ±π/2 */
	demod->rate_in *= demod->post_downsample;

	sanity_checks();

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
		verbose_list_bandwidths(dongle.dev);

	if (rtlOpts) {
		rtlsdr_set_opt_string(dongle.dev, rtlOpts, verbosity);
	}

	/* r = rtlsdr_set_testmode(dongle.dev, 1); */

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dongle.dev);

	controller_fn(&dm_thr);
	if (r) {
		rtlsdr_close(dongle.dev);
		return 1;
	}
	pthread_create(&dm_thr.thread, NULL, multi_demod_thread_fn, (void *)(&dm_thr));

	/* open pipe closing threads if need */
	for (int ch = 0; ch < dm_thr.channel_count; ++ch) {
		if (dm_thr.audio_write_type == 2 && dm_thr.audio_split_duration > 0) {
			dm_thr.audio_p_closing_thread[ch].fptr = NULL;
			pthread_create(&dm_thr.audio_p_closing_thread[ch].closing_thread, NULL, close_pipe_fn, (&dm_thr.audio_p_closing_thread[ch]));	
		} 
		if (dm_thr.mpx_write_type == 2 && dm_thr.mpx_split_duration > 0) {
			dm_thr.mpx_p_closing_thread[ch].fptr = NULL;
			pthread_create(&dm_thr.mpx_p_closing_thread[ch].closing_thread, NULL, close_pipe_fn, (&dm_thr.mpx_p_closing_thread[ch]));
		}
	}
	

	rtlsdr_read_async(dongle.dev, rtlsdr_callback, &dongle, 0, dongle.buf_len);

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");}
	else {
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);}

	rtlsdr_cancel_async(dongle.dev);
	safe_cond_signal(&dm_thr.ready, &dm_thr.ready_m);
	pthread_join(dm_thr.thread, NULL);

	/* dongle_cleanup(&dongle); */
	demod_thread_cleanup(&dm_thr);

	rtlsdr_close(dongle.dev);
	return r >= 0 ? r : -r;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
