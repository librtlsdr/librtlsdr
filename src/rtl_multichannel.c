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
#define MAX_CIRCULAR_BUFFERS    128

#define SLOTS_PER_CHANNEL       2


#define DEFAULT_MPX_PIPE_PATTERN	"redsea -r %sf >%F_%H-%M-%S_%#_ch-%cf_rds.txt"
#define DEFAULT_MPX_FILE_PATTERN	"%F_%H-%M-%S_%#_ch-%cf_mpx.raw"
/* prefer .ogg over .mp3: mp3 encoding produces some milliseconds (approx 80 ms) at start of every new file! ogg encoded files look gapless */
#define AUDIO_MP3_PIPE_PATTERN		"ffmpeg -f s16le -ar %sf -ac 1 -i pipe: -loglevel quiet %F_%H-%M-%S_%#_ch-%cf_audio.mp3"
#define AUDIO_OGG_PIPE_PATTERN		"oggenc -r --raw-endianness 0 -B 16 -C 1 -R %sf -q 2 -Q -o %F_%H-%M-%S_%#_ch-%cf_audio.ogg -"
#define DEFAULT_AUDIO_FILE_PATTERN	"%F_%H-%M-%S_%#_ch-%cf_audio.raw"

#define likely(x)	__builtin_expect(!!(x), 1)
#define unlikely(x)	__builtin_expect(!!(x), 0)
#define ARRAY_LEN(x)	(sizeof(x)/sizeof(x[0]))

enum WriteType {
	WriteNONE = 0,
	WriteFILE = 1,
	WritePIPE = 2
};

/* type: 0: nothing, 1: file, 2: pipe */
#define DEFAULT_AUD_WRITE_TYPE		WriteFILE
#define DEFAULT_MPX_WRITE_TYPE		WriteNONE

static int MinCaptureRate = 1000000;

static volatile int do_exit = 0;
static volatile int do_exit_close = 0;
static int verbosity = 0;
static const char * dongleid = "?";
static unsigned on_overflow = 0;		/* 0: (q)uit;  1: (d)iscard all buffers;  2: (i)gnore/discard single buffer */
static unsigned num_detected_overflows = 0;
static atomic_uint num_discarded_buffers = ATOMIC_VAR_INIT(0);
static unsigned num_consec_overflows = 0;
static struct timeval	tv_overflow_start;


time_t stop_time;
int duration = 0;

struct demod_input_buffer
{
	int16_t * lowpassed; /* input and decimated quadrature I/Q sample-pairs */
	int	  lp_len;		/* number of valid samples in lowpassed[] - NOT quadrature I/Q sample-pairs! */
	int	  was_overflow;	/* flag, if there was an overflow */
	atomic_int  is_free;	/* 1 == free; 0 == occupied with data */
	pthread_rwlock_t	rw;
	struct timeval		tv;	/* timestamp of reception */
};

struct file_thread_data
{
	FILE *fptr[SLOTS_PER_CHANNEL];
	atomic_int to_close[SLOTS_PER_CHANNEL];	/* => close to run */
	atomic_int is_closing[SLOTS_PER_CHANNEL];	/* => close still running (in thread) */
	atomic_int * p_sum_to_close;
	int open_idx;		/* fptr[] idx to use for [p]open() */
	pthread_t closing_thread;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
	int is_mpx;
	int ch;
	unsigned fno;
};

struct demod_thread_state
{
	struct mixer_state mixers[MAX_NUM_CHANNELS];
	struct demod_state demod_states[MAX_NUM_CHANNELS];

	pthread_t thread;
	int channel_count;
	struct demod_input_buffer	buffers[MAX_CIRCULAR_BUFFERS];
	int	  num_circular_buffers;
	int	  dummyPadA[128 / sizeof(int)];		/* pad, to get onto next cache line! */
	int	  buffer_write_idx;
	int	  dummyPadB[128 / sizeof(int)];
	int	  buffer_read_idx;
	int	  dummyPadC[128 / sizeof(int)];
	atomic_uint buffer_rcv_counter;	/* buffers received from dongle in callback */
	int	  dummyPadD[128 / sizeof(int)];
	atomic_uint buffer_proc_counter;	/* processed buffers */
	int	  dummyPadE[128 / sizeof(int)];

	const char *aud_pipe_pattern;
	const char *mpx_pipe_pattern;
	const char *aud_file_pattern;
	const char *mpx_file_pattern;
	enum WriteType aud_write_type;
	int aud_frame_modulo;
	int aud_frame_size;	/* e.g. 4608 (=multiple of 512 and 1152) for mp3 */
	enum WriteType mpx_write_type;
	struct file_thread_data aud_files[MAX_NUM_CHANNELS]; /* specific for audio */
	struct file_thread_data mpx_files[MAX_NUM_CHANNELS]; /* specific for mpx */
	FILE * actual_aud_files[MAX_NUM_CHANNELS];
	FILE * actual_mpx_files[MAX_NUM_CHANNELS];

	atomic_int sum_aud_to_close;
	atomic_int sum_mpx_to_close;

	int mpx_split_duration;		/* split file or pipe in seconds */
	int aud_split_duration;

	int mpx_limit_duration;	/* limit per split in seconds */
	int aud_limit_duration;

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

void usage(int verbosity)
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
		"\t[-t [x:|a:|af:] split duration in seconds to split files (default: off)]\n"
		"\t     x:|a:   split the mpx or audio signal. without that prefix, both are split\n"
		"\t     af:     split audio at multiples of size samples, e.g. 4608 for mp3. size must be multiple of 512\n"
		"\t  prepending 'timeout' command (linux) to the pipe command pattern might be interesting.\n"
		"\t[-l [x:|a:]limit duration in seconds from (split) begin (default: off)]\n"
		"\t[-a audio pipe command or file pattern (default: file) with substitutions similar to strftime(), ie.\n"
		"\t\t\"p:%s\"\n"
		"\t\t\"a:%s\"\n"
		"\t\t\"p:\" uses the default pipe pattern, \"f:\" uses the default file pattern, \"n\" deactivates audio output.\n"
		"\t[-x mpx signal pipe command or file pattern (default: off)\n"
		"\t\tthe default pipe command pattern: \"p:%s\"\n"
		"\t\tthe default filename pattern: \"f:%s\"\n"
		"\t\t  substitutions, additional to strftime(): %%f for frequency, %%kf for frequency in kHz,\n"
		"\t\t  %%sf for samplerate, %%cf for channel number, %%nf for file no and %%# for milliseconds of time\n"
		"\t[-o on_overflow (default: (q)uit), other options: (d)iscard all buffers, (i)gnore and discard single buffer\n"
		"\t\t  discard does close/reopen the files or pipes\n"
		"\t[-r resample_rate (default: none / same as -s)]\n"
		"\t[-d device_index or serial (default: 0 , -1 or '-' for stdin or f:filename)]\n"
		"%s"
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
		"\t[-q dc_avg_factor for option rdc (default: 9)]\n"
		/* "\t[-H write wave Header to file (default: off)]\n" */
		"Experimental options:\n"
		"\t[-F fir_size (default: off)]\n"
		"\t	enables low-leakage downsample filter\n"
		"\t	size can be 0 or 9.  0 has bad roll off\n"
		"\t[-A std/fast/lut/ale choose atan math (default: std)]\n"
		"\n"
		, AUDIO_OGG_PIPE_PATTERN, DEFAULT_AUDIO_FILE_PATTERN
		, DEFAULT_MPX_PIPE_PATTERN, DEFAULT_MPX_FILE_PATTERN
		, rtlsdr_get_opt_help(verbosity) );
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "dongle %s: Signal caught, exiting!\n", dongleid);
		do_exit = 1;
		if (dongle.dev)
			rtlsdr_cancel_async(dongle.dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "dongle %s: Signal caught, exiting!\n", dongleid);
	do_exit = 1;
	if (dongle.dev)
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


int strfchannel(char *s, int max, const char * format, const int chno, unsigned * fno, const uint32_t freq, const unsigned rate, const int milli, const int rep_esc)
{
	/* rep_esc == 0: %% is NOT replaced for later call to strftime() */
	/* replaced strings:
	 * %f  => frequency in Hz
	 * %kf => frequency in kHz
	 * %cf => channel number: 0 ..
	 * %sf => sample frequency = rate
	 * %nf => file number
	 * %#  => ms
	 */
	int r, dest_pos = 0;
	const int fmt_len = strlen(format);
	const char escape = '%';

	++(*fno);
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
			else if (format[fmt_pos+1] == 'n' && format[fmt_pos+2] == 'f') {
				/* %nf : file number */
				r = snprintf(&s[dest_pos], max - dest_pos, "%u", *fno);
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

static FILE * open_pipe(const char *command, const char * st, int ch, int slot, unsigned fno) {
	FILE *f = popen(command, "w");
	if (!f) {
		fprintf(stderr, "dongle %s: Error: Could not open %u.th %s-channel %d slot %d for '%s' !\n", dongleid, fno, st, ch, slot, command);
		return f;
	}
	if (verbosity) {
		fprintf(stdout, "dongle %s: %u.th %s-channel %d slot %d opened: '%s'\n", dongleid, fno, st, ch, slot, command);
		fflush(stdout);
	}
	return f;
}

static FILE * create_iq_file(const char* filename, const char * st, int ch, int slot, unsigned fno) {
	FILE * f = fopen(filename, "wb");
	if(!f) {
		fprintf(stderr, "dongle %s: Error: Could not create %u.th %s-channel %d slot %d file '%s' !\n", dongleid, fno, st, ch, slot, filename);
		return f;
	}
	if (verbosity) {
		fprintf(stdout, "dongle %s: %u.th %s-channel %d slot %d created: %s\n", dongleid, fno, st, ch, slot, filename);
		fflush(stdout);
	}
	return f;
}

static void *close_pipe_fn(void* arg) {
	struct file_thread_data *ct = arg;
	FILE *f = NULL;
	while (!do_exit_close) {
		int worked = 1;
		safe_cond_wait(&ct->ready, &ct->ready_m);
		while (!do_exit_close || worked) {
			/* close (multiple) files per signaled condition */
			worked = 0;
			for (int close_idx = 0; close_idx < SLOTS_PER_CHANNEL; ++close_idx) {
				int old_is_closing = atomic_fetch_add(&ct->is_closing[close_idx], 1);
				if(old_is_closing) {
					/* revert, cause already closing */
					atomic_fetch_sub(&ct->is_closing[close_idx], 1);
					if (verbosity >= 2)
						fprintf(stderr, "pclose()-thread of %s-channel %d slot %d: skipping.\n", ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx);
					continue;
				}

				if (atomic_load(&ct->to_close[close_idx])) {
					int left;
					f = ct->fptr[close_idx];
					if (f) {
						time_t tstart = time(NULL);
						int retcode, err_no;
						worked = 1;
						if (verbosity >= 2)
							fprintf(stderr, "starting pclose() of %s-channel %d slot %d ..\n", ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx);
						fflush(f);
						retcode = pclose(f);
						err_no = errno;
						left = atomic_fetch_sub(ct->p_sum_to_close, 1) - 1;
						if (verbosity >= 2 || retcode == -1) {
							char errmsg[256];
							int tdur = (int)(time(NULL) - tstart);
							if (retcode == -1)
							{
#if (_POSIX_C_SOURCE >= 200112L) || defined(_GNU_SOURCE)
								strerror_r(err_no, errmsg, 255);
#else
								strncpy(errmsg, strerror(err_no), 255);
#endif
								fprintf(stderr, "finished pclose() of %s-channel %d slot %d in %u seconds with errno %d. left to close: %d%s\nerror text for %d: %s\n",
									ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx, tdur, err_no, left, left?"\n":" *\n", err_no, errmsg);
							}
							else
								fprintf(stderr, "finished pclose() of %s-channel %d slot %d in %u seconds. left to close: %d%s", ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx, tdur, left, left?"\n":" *\n");
						}
					}
					else {
						left = atomic_fetch_sub(ct->p_sum_to_close, 1) - 1;
						fprintf(stderr, "skipped pclose() of %s-channel %d slot %d: already closed. left to close: %d%s", ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx, left, left?"\n":" *\n");
					}
					ct->fptr[close_idx] = NULL;
					atomic_fetch_sub(&ct->to_close[close_idx], 1);
				}
				atomic_fetch_sub(&ct->is_closing[close_idx], 1);
			}
			if (!worked)
				break;
		}
	}

	/* finally close everything */
	for (int close_idx = 0; close_idx < SLOTS_PER_CHANNEL; ++close_idx) {
		f = ct->fptr[close_idx];
		if (f) {
			/* 0: nothing, 1: file, 2: pipe */
			time_t tstart = time(NULL);
			int retcode, err_no;
			if (verbosity >= 2)
				fprintf(stderr, "starting final pclose() of %s-channel %d slot %d ..\n",  ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx);
			fflush(f);
			retcode = pclose(f);
			err_no = errno;
			if (verbosity >= 2 || retcode == -1) {
				char errmsg[256];
				int tdur = (int)(time(NULL) - tstart);
				if (retcode == -1) {
#if (_POSIX_C_SOURCE >= 200112L) || defined(_GNU_SOURCE)
					strerror_r(err_no, errmsg, 255);
#else
					strncpy(errmsg, strerror(err_no), 255);
#endif
					fprintf(stderr, "finished final pclose() of %s-channel %d slot %d in %u seconds with errno %d: %s\n", ct->is_mpx ? "mpx" : "audio",
						ct->ch, close_idx, tdur, err_no, errmsg);
				}
				else
					fprintf(stderr, "finished final pclose() of %s-channel %d slot %d in %u seconds\n", ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx, tdur);
			}
		}
		else {
			fprintf(stderr, "final pclose() for %s-channel %d slot %d : already closed.\n", ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx);
		}
		ct->fptr[close_idx] = NULL;
	}

	return 0;
}

void close_all_channel_outputs(struct demod_thread_state *s, int close_mpx, int close_aud, int use_thread)
{
	const char * outstr = (close_mpx && close_aud) ? "mpx and audio" : ( close_mpx ? "mpx" : (close_aud ? "audio" : "???") );
	if (verbosity)
		fprintf(stderr, "dongle %s: closing all %s outputs %s utilizing threads\n", dongleid, outstr, (use_thread ? "with" : "without") );
	if (close_aud)
		s->aud_frame_modulo = 0;
	for(int ch = 0; ch < s->channel_count; ++ch) {
		if (close_aud) {
			struct file_thread_data *ct = &(s->aud_files[ch]);
			const int close_idx = ct->open_idx;
			FILE *f = ct->fptr[close_idx];
			ct->open_idx = (ct->open_idx + 1) % SLOTS_PER_CHANNEL;
			s->actual_aud_files[ch] = NULL;
			if (s->aud_write_type == WritePIPE) {
				if(use_thread && f) {
					int old_to_close = atomic_fetch_add(&ct->to_close[close_idx], 1);
					if(old_to_close) {
						atomic_fetch_sub(&ct->to_close[close_idx], 1);	/* revert */
						fprintf(stderr, "dongle %s: Error: pclose() already/still running for audio-channel %d slot %d\n", dongleid, ch, close_idx);
					}
					else {
						atomic_fetch_add(ct->p_sum_to_close, 1);
						safe_cond_signal(&ct->ready, &ct->ready_m);
					}
				} else if (f) {
					int retcode, err_no;
					fflush(f);
					retcode = pclose(f);
					err_no = errno;
					ct->fptr[close_idx] = NULL;
					if (retcode == -1) {
						char errmsg[256];
#if (_POSIX_C_SOURCE >= 200112L) || defined(_GNU_SOURCE)
						strerror_r(err_no, errmsg, 255);
#else
						strncpy(errmsg, strerror(err_no), 255);
#endif
								fprintf(stderr, "finished synchronous pclose() of %s-channel %d slot %d with errno %d: %s\n",
									ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx, err_no, errmsg);
					}
				}
			}
			else if (s->aud_write_type == WriteFILE && f) {
				fflush(f);
				fclose(f);
				ct->fptr[close_idx] = NULL;
			}
		}

		if (close_mpx) {
			struct file_thread_data *ct = &(s->mpx_files[ch]);
			const int close_idx = ct->open_idx;
			FILE *f = ct->fptr[close_idx];
			ct->open_idx = (ct->open_idx + 1) % SLOTS_PER_CHANNEL;
			s->actual_mpx_files[ch] = NULL;
			if (s->mpx_write_type == WritePIPE) {
				if(use_thread && f) {
					int old_to_close = atomic_fetch_add(&ct->to_close[close_idx], 1);
					if(old_to_close) {
						atomic_fetch_sub(&ct->to_close[close_idx], 1);	/* revert */
						fprintf(stderr, "dongle %s: Error: close() not finished for mpx-channel %d slot %d\n", dongleid, ch, close_idx);
					}
					else {
						atomic_fetch_add(ct->p_sum_to_close, 1);
						safe_cond_signal(&ct->ready, &ct->ready_m);
					}
				} else if (f) {
					int retcode, err_no;
					fflush(f);
					retcode = pclose(f);
					err_no = errno;
					ct->fptr[close_idx] = NULL;
					if (retcode == -1) {
						char errmsg[256];
#if (_POSIX_C_SOURCE >= 200112L) || defined(_GNU_SOURCE)
						strerror_r(err_no, errmsg, 255);
#else
						strncpy(errmsg, strerror(err_no), 255);
#endif
						fprintf(stderr, "finished synchronous pclose() of %s-channel %d slot %d with errno %d: %s\n",
							ct->is_mpx ? "mpx" : "audio", ct->ch, close_idx, err_no, errmsg);
					}
				}
			}
			else if (s->mpx_write_type == WriteFILE && f) {
				fflush(f);
				fclose(f);
				ct->fptr[close_idx] = NULL;
			}
		}
	}
}

void open_all_mpx_channel_outputs(struct demod_thread_state *s, const unsigned mpx_rate, const struct tm *tm, const int milli)
{
	static char fnTmp[1024];
	static char fnTim[1024];

	if (verbosity)
		fprintf(stderr, "dongle %s: opening all %d outputs for mpx stream\n", dongleid, s->channel_count);
	for(int ch = 0; ch < s->channel_count; ++ch) {
		uint32_t freq = s->center_freq + s->freqs[ch];
		struct file_thread_data *ct = &(s->mpx_files[ch]);
		const int open_idx = ct->open_idx;
		s->actual_mpx_files[ch] = NULL;
		if (atomic_load(&ct->to_close[open_idx])) {
			fprintf(stderr, "dongle %s: Error: close() not finished for mpx-channel %d slot %d. Skipping [p]open()!\n", dongleid, ch, open_idx);
			continue;
		}
		if (s->mpx_write_type == WritePIPE) {
			strfchannel(fnTmp, ARRAY_LEN(fnTmp), s->mpx_pipe_pattern, ch, &ct->fno, freq, mpx_rate, milli, 0);
			strftime(fnTim, ARRAY_LEN(fnTim), fnTmp, tm);
			ct->fptr[open_idx] = open_pipe(fnTim, "mpx", ch, open_idx, ct->fno);
			s->actual_mpx_files[ch] = ct->fptr[open_idx];
		} else if (s->mpx_write_type == WriteFILE) {
			strfchannel(fnTmp, ARRAY_LEN(fnTmp), s->mpx_file_pattern, ch, &ct->fno, freq, mpx_rate, milli, 0);
			strftime(fnTim, ARRAY_LEN(fnTim), fnTmp, tm);
			ct->fptr[open_idx] = create_iq_file(fnTim, "mpx", ch, open_idx, ct->fno);
			s->actual_mpx_files[ch] = ct->fptr[open_idx];
		}
	}
}

void open_all_audio_channel_outputs(struct demod_thread_state *s, const unsigned aud_rate, const struct tm *tm, const int milli)
{
	static char fnTmp[1024];
	static char fnTim[1024];

	if (verbosity)
		fprintf(stderr, "dongle %s: opening all %d outputs for audio stream\n", dongleid, s->channel_count);
	s->aud_frame_modulo = 0;
	for(int ch = 0; ch < s->channel_count; ++ch) {
		uint32_t freq = s->center_freq + s->freqs[ch];
		struct file_thread_data *ct = &(s->aud_files[ch]);
		const int open_idx = ct->open_idx;
		s->actual_aud_files[ch] = NULL;
		if (atomic_load(&ct->to_close[open_idx])) {
			fprintf(stderr, "dongle %s: Error: close() not finished for audio-channel %d slot %d. Skipping [p]open()!\n", dongleid, ch, open_idx);
			continue;
		}
		if (s->aud_write_type == WritePIPE) {
			strfchannel(fnTmp, ARRAY_LEN(fnTmp), s->aud_pipe_pattern, ch, &ct->fno, freq, aud_rate, milli, 0);
			strftime(fnTim, ARRAY_LEN(fnTim), fnTmp, tm);
			ct->fptr[open_idx] = open_pipe(fnTim, "audio", ch, open_idx, ct->fno);
			s->actual_aud_files[ch] = ct->fptr[open_idx];
		} else if (s->aud_write_type == WriteFILE) {
			strfchannel(fnTmp, ARRAY_LEN(fnTmp), s->aud_file_pattern, ch, &ct->fno, freq, aud_rate, milli, 0);
			strftime(fnTim, ARRAY_LEN(fnTim), fnTmp, tm);
			ct->fptr[open_idx] = create_iq_file(fnTim, "audio", ch, open_idx, ct->fno);
			s->actual_aud_files[ch] = ct->fptr[open_idx];
		}
	}
}


int full_demod(struct demod_state *d, FILE *f_mpx, FILE *f_aud, int *aud_frame_modulo, int aud_frame_size, int channel, int mpx_slot, int aud_slot)
{
	int nwritten;
	int errFlags = 0;	/* return is bitmask of error flags: value 1: mpx error; 2: audio error */

	downsample_input(d);

	d->mode_demod(d);  /* lowpassed -> result */
	if (d->mode_demod == &raw_demod) {
		return errFlags;
	}

	if (f_mpx) {
		nwritten = (int)fwrite(d->result, 2, d->result_len, f_mpx);
		if (nwritten != d->result_len) {
			fprintf(stderr, "dongle %s: error writing %d samples to mpx-channel %d slot %d .. result %d\n", dongleid, d->result_len, channel, mpx_slot, nwritten);
			errFlags = errFlags | 1;
		}
	}

	if (!f_aud) {
		/* no need for audio? */
		return errFlags;
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

	nwritten = (int)fwrite(d->result, 2, d->result_len, f_aud);
	if (aud_frame_modulo && aud_frame_size) {
		/* we want to split - only at multiples of 1152 samples, the frame size of an mp3
		 * that is lcm(512, 1152) = 4608 == 9 x 512
		 */
		*aud_frame_modulo = (*aud_frame_modulo + d->result_len) % aud_frame_size;
	}
	if (nwritten != d->result_len) {
		fprintf(stderr, "dongle %s: error writing %d samples to audio-channel %d slot %d .. result %d\n", dongleid, d->result_len, channel, aud_slot, nwritten);
		errFlags = errFlags | 2;
	}
	return errFlags;
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct dongle_state *s = ctx;
	struct demod_thread_state *mds = s->demod_target;
	struct demod_input_buffer *buffer;
	int16_t *buf16;
	int i, write_idx;

	if (do_exit || !ctx) {
		return;
	}

	atomic_fetch_add(&mds->buffer_rcv_counter, 1);
	write_idx = mds->buffer_write_idx;
	buffer = &mds->buffers[write_idx];
	pthread_rwlock_wrlock(&buffer->rw);	/* lock before writing into demod_thread_state.lowpassed */
	if (!atomic_load(&buffer->is_free)) {
		pthread_rwlock_unlock(&buffer->rw);
		if (!num_consec_overflows)
			gettimeofday(&tv_overflow_start, NULL);
		++num_detected_overflows;
		++num_consec_overflows;
		atomic_fetch_add(&num_discarded_buffers, 1);	/* this block is definitely discarded */
		if (!on_overflow) {
			unsigned rcv_counter = atomic_load(&mds->buffer_rcv_counter);
			unsigned proc_counter = atomic_load(&mds->buffer_proc_counter);
			do_exit = 1;
			if (dongle.dev)
				rtlsdr_cancel_async(dongle.dev);
			fprintf(stderr, "\n\n*** dongle %s: Overflow of circular input buffers, exiting! ***\n", dongleid);
			fprintf(stderr, "dongle %s: Overflow happened after having read %u buffers from dongle.\n", dongleid, rcv_counter);
			fprintf(stderr, "dongle %s: Overflow happened after having processed %u buffers.\n", dongleid, proc_counter);
			fprintf(stderr, "dongle %s: There are %u unprocessed buffers\n\n", dongleid, rcv_counter - proc_counter );
		}
		return;
	}

	/* increment with a free writable buffer */
	mds->buffer_write_idx = (mds->buffer_write_idx + 1) % mds->num_circular_buffers;

	/* getting timestamp here, leads to more accurate timestamp - compared to multi_demod_thread_fn() */
	gettimeofday(&buffer->tv, NULL);

	if (duration > 0 && buffer->tv.tv_sec >= stop_time) {
		do_exit = 1;
		fprintf(stderr, "dongle %s: Time expired, exiting!\n", dongleid);
		if (dongle.dev)
			rtlsdr_cancel_async(dongle.dev);
		pthread_rwlock_unlock(&buffer->rw);
		return;
	}

	buffer->was_overflow = 0;	/* assume so */
	if (num_consec_overflows) {
		if (on_overflow == 1) {
			fprintf(stderr, "*** dongle %s: +%u overflows of circular input buffers, %u total: discard all buffers ***\n", dongleid, num_consec_overflows, num_detected_overflows);
			buffer->was_overflow = 1;	/* report overflows with the buffer contents */
		}
		else {
			char acTime[128];
			struct tm *tm = localtime(&tv_overflow_start.tv_sec);
			const int milli = (int)(tv_overflow_start.tv_usec/1000);
			strftime(acTime, ARRAY_LEN(acTime), "%F %T", tm);
			fprintf(stderr, "*** dongle %s: %s.%03d: +%u overflows of circular input buffers, %u total: ignore/discard single buffer ***\n",
				dongleid, acTime, milli, num_consec_overflows, num_detected_overflows);
		}

		num_consec_overflows = 0;
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

	atomic_store(&buffer->is_free, 0);
	pthread_rwlock_unlock(&buffer->rw);
	safe_cond_signal(&mds->ready, &mds->ready_m);
}


int read_from_file(FILE * f) {
	size_t nmemb = dongle.buf_len / ( 2 * sizeof(uint8_t) );
	unsigned char *buf = (unsigned char *)malloc(nmemb * sizeof(uint16_t));
	struct demod_thread_state *mds = dongle.demod_target;
#ifdef _WIN32
	if (f == stdin)
		_setmode(_fileno(f), _O_BINARY);
#endif

	while (!do_exit && !feof(f)) {
		/* @todo: check/wait for a free buffer? */
		/* assume input is rate limited with pv -L */
		size_t rd = fread(buf, sizeof(uint16_t), nmemb, f);
		if (rd != nmemb)
			return 1;
		while (!atomic_load(&mds->buffers[mds->buffer_write_idx].is_free))
			usleep(1000);
		rtlsdr_callback(buf, 2 *nmemb, &dongle);
	}
	free(buf);
	return 0;
}


static void *multi_demod_thread_fn(void *arg)
{
	struct demod_thread_state *mds = arg;

	struct timeval t1_mpx, t1_aud;
	double diff_ms_mpx, diff_ms_aud;
	int ch, read_idx;
	int init_open = 1;
	int mpx_is_limited = 1;
	int aud_is_limited = 1;
	unsigned num_remove_buffers = 0;
	const unsigned mpx_rate = mds->demod_states[0].rate_out;
	const unsigned aud_rate = mds->demod_states[0].rate_out2 ? (unsigned)mds->demod_states[0].rate_out2 : mpx_rate;
	struct demod_input_buffer *buffer;

	gettimeofday(&t1_mpx, NULL);
	t1_aud = t1_mpx;
	while (!do_exit) {
		safe_cond_wait(&mds->ready, &mds->ready_m);
		while (!do_exit) {
			/* process (multiple) blocks per signaled condition */
			read_idx = mds->buffer_read_idx;
			buffer = &mds->buffers[read_idx];
			if (atomic_load(&buffer->is_free))
				break;
			mds->buffer_read_idx = (mds->buffer_read_idx + 1) % mds->num_circular_buffers;
			pthread_rwlock_wrlock(&buffer->rw);	/* lock before reading into demod_thread_state.lowpassed */

			if (num_remove_buffers || buffer->was_overflow) {
				if (buffer->was_overflow) {
					num_remove_buffers = mds->num_circular_buffers;	/* discard all buffers */
					init_open = 1;	/* reopen streams directly after overflow? */
				}
				atomic_store(&buffer->is_free, 1);	/* buffer can be written again */
				pthread_rwlock_unlock(&buffer->rw);	/* unlock buffer as early as possible */

				atomic_fetch_add(&num_discarded_buffers, 1);
				--num_remove_buffers;
				if (!num_remove_buffers && verbosity) {
					fprintf(stderr, "dongle %s: discarded %u buffers, cause of overflows\n", dongleid, atomic_load(&num_discarded_buffers));
				}
				if (!mpx_is_limited || !aud_is_limited) {
					close_all_channel_outputs(mds, 1, 1, 1);	/* close mpx and audio files or pipes */
					mpx_is_limited = 1;
					aud_is_limited = 1;
					if (verbosity)
						fprintf(stderr, "dongle %s: closing mpx and audio streams, cause of overflow\n", dongleid);
				}
				continue;
			}

			diff_ms_mpx  = (buffer->tv.tv_sec  - t1_mpx.tv_sec) * 1000.0;
			diff_ms_mpx += (buffer->tv.tv_usec - t1_mpx.tv_usec) / 1000.0;
			diff_ms_aud  = (buffer->tv.tv_sec  - t1_aud.tv_sec) * 1000.0;
			diff_ms_aud += (buffer->tv.tv_usec - t1_aud.tv_usec) / 1000.0;

			/* check limits first. else, files are immediately closed again, cause diff_ms still > .. */
			if (unlikely(!mpx_is_limited && mds->mpx_limit_duration > 0 && diff_ms_mpx > 1000.0 * mds->mpx_limit_duration)) {
				mpx_is_limited = 1;
				close_all_channel_outputs(mds, 1, 0, 1);	/* close mpx files or pipes */
				if (verbosity)
					fprintf(stderr, "dongle %s: closing mpx stream, cause of limit\n", dongleid);
			}
			if (unlikely(!aud_is_limited && mds->aud_limit_duration > 0 && diff_ms_aud > 1000.0 * mds->aud_limit_duration)) {
				aud_is_limited = 1;
				close_all_channel_outputs(mds, 0, 1, 1);	/* close audio files or pipes */
				if (verbosity)
					fprintf(stderr, "dongle %s: closing audio stream, cause of limit\n", dongleid);
			}

			if (unlikely(init_open || (mds->mpx_split_duration > 0 && diff_ms_mpx > 1000.0 * mds->mpx_split_duration))) {
				struct tm *tm = localtime(&buffer->tv.tv_sec);
				const int milli = (int)(buffer->tv.tv_usec/1000);
				close_all_channel_outputs(mds, 1, 0, 1);	/* close mpx files or pipes */
				open_all_mpx_channel_outputs(mds, mpx_rate, tm, milli);
				mpx_is_limited = 0;
				t1_mpx = buffer->tv;
			}
			if (unlikely(init_open || (mds->aud_split_duration > 0 && mds->aud_frame_modulo == 0 && diff_ms_aud > 1000.0 * mds->aud_split_duration))) {
				struct tm *tm = localtime(&buffer->tv.tv_sec);
				const int milli = (int)(buffer->tv.tv_usec/1000);
				close_all_channel_outputs(mds, 0, 1, 1);	/* close audio files or pipes */
				open_all_audio_channel_outputs(mds, aud_rate, tm, milli);
				aud_is_limited = 0;
				t1_aud = buffer->tv;
			}
			init_open = 0;

			for (ch = 0; ch < mds->channel_count; ++ch) {
				mds->demod_states[ch].lp_len = buffer->lp_len;
				mixer_apply(&mds->mixers[ch], buffer->lp_len, buffer->lowpassed, mds->demod_states[ch].lowpassed);
			}

			atomic_fetch_add(&mds->buffer_proc_counter, 1);
			atomic_store(&buffer->is_free, 1);	/* buffer can be written again */
			/* we only need to lock the lowpassed buffer of demod_thread_state */
			pthread_rwlock_unlock(&buffer->rw);

			for (ch = 0; ch < mds->channel_count; ++ch) {
				int * p_frame_modulo = (!ch) ? &(mds->aud_frame_modulo) : NULL;
				int errFlags = full_demod(
					&mds->demod_states[ch],
					mds->actual_mpx_files[ch],	/* mds->mpx_files[ch].fptr[ mds->mpx_files[ch].open_idx ], */
					mds->actual_aud_files[ch],	/* mds->aud_files[ch].fptr[ mds->aud_files[ch].open_idx ], */
					p_frame_modulo, mds->aud_frame_size,
					ch, mds->mpx_files[ch].open_idx, mds->aud_files[ch].open_idx
					);
				/* stop writing to mpx or audio file with first error */
				if (errFlags & 1)
					mds->actual_mpx_files[ch] = NULL;
				if (errFlags & 2)
					mds->actual_aud_files[ch] = NULL;
			}
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
	struct demod_state *config = &dt->demod_states[0];
	config->downsample = (MinCaptureRate / config->rate_in) + 1;
	if (config->downsample_passes) {
		config->downsample_passes = (int)( ceil(log2(config->downsample)) + 0.1);
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

	if (dongle.dev) {
		if (dongle.direct_sampling) {
			verbose_direct_sampling(dongle.dev, 1);}

		/* Set the frequency */
		if (verbosity) {
			fprintf(stderr, "dongle %s: verbose_set_frequency(%f MHz)\n", dongleid, dongle.freq * 1E-6);
		}
		verbose_set_frequency(dongle.dev, dongle.freq);
	}

	dongle_rate = dongle.rate;
	nyq_max = dongle_rate /2;
	for (i = 0; i < s->channel_count; ++i) {
		fprintf(stderr, "  dongle %s: channel %d: freq %lu will be recording soon \n", dongleid, i, dongle.freq+s->freqs[i]);
		if (s->freqs[i] <= -nyq_max || s->freqs[i] >= nyq_max) {
			fprintf(stderr, "dongle %s: Warning: frequency for channel %d (=%d Hz) is out of Nyquist band!\n", dongleid, i, (int)s->freqs[i]);
			fprintf(stderr, "  dongle %s: nyquist band is from %d .. %d Hz\n", dongleid, -nyq_max, nyq_max);
		}
		mixer_init(&dm_thr.mixers[i], s->freqs[i], dongle.rate);
		/* distribute demod setting from 1st (=config) to all */
		demod_copy_fields(&dm_thr.demod_states[i], demod_config);
		/* allocate memory per channel */
		demod_init(&dm_thr.demod_states[i], 0, 1);
	}
	fprintf(stderr, "dongle %s: Multichannel will demodulate %d channels.\n", dongleid, dm_thr.channel_count);

	fprintf(stderr, "Oversampling input by: %ix.\n", demod_config->downsample);
	fprintf(stderr, "Buffer size: %u Bytes == %u quadrature samples == %0.2fms\n",
		(unsigned)dongle.buf_len,
		(unsigned)dongle.buf_len / 2,
		1000 * 0.5 * (float)dongle.buf_len / (float)dongle.rate);

	if (dongle.dev) {
		/* Set the sample rate */
		if (verbosity)
			fprintf(stderr, "verbose_set_sample_rate(%.0f Hz)\n", (double)dongle.rate);
		verbose_set_sample_rate(dongle.dev, dongle.rate);
	}
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
	s->sum_aud_to_close = ATOMIC_VAR_INIT(0);
	s->sum_mpx_to_close = ATOMIC_VAR_INIT(0);

	for (int ch = 0; ch < MAX_NUM_CHANNELS; ++ch) {
		int slot;
		demod_init(&s->demod_states[ch], 1, 0);
		for (slot = 0; slot < SLOTS_PER_CHANNEL; ++slot) {
			s->aud_files[ch].fptr[slot] = NULL;
			s->aud_files[ch].to_close[slot] = ATOMIC_VAR_INIT(0);
			s->aud_files[ch].is_closing[slot] = ATOMIC_VAR_INIT(0);
		}
		s->aud_files[ch].p_sum_to_close = &s->sum_aud_to_close;
		s->aud_files[ch].open_idx = 0;
		s->aud_files[ch].is_mpx = 0;
		s->aud_files[ch].ch = ch;
		s->aud_files[ch].fno = 0;
		pthread_cond_init(&s->aud_files[ch].ready, NULL);
		pthread_mutex_init(&s->aud_files[ch].ready_m, NULL);

		for (slot = 0; slot < SLOTS_PER_CHANNEL; ++slot) {
			s->mpx_files[ch].fptr[slot] = NULL;
			s->mpx_files[ch].to_close[slot] = ATOMIC_VAR_INIT(0);
			s->mpx_files[ch].is_closing[slot] = ATOMIC_VAR_INIT(0);
		}
		s->mpx_files[ch].p_sum_to_close = &s->sum_mpx_to_close;
		s->mpx_files[ch].open_idx = 0;
		s->mpx_files[ch].is_mpx = 1;
		s->mpx_files[ch].ch = ch;
		s->mpx_files[ch].fno = 0;
		pthread_cond_init(&s->mpx_files[ch].ready, NULL);
		pthread_mutex_init(&s->mpx_files[ch].ready_m, NULL);

		s->actual_aud_files[ch] = NULL;
		s->actual_mpx_files[ch] = NULL;
	}

	s->num_circular_buffers = 4;
	s->buffer_write_idx = 0;
	s->buffer_read_idx = 0;
	s->buffer_rcv_counter = ATOMIC_VAR_INIT(0);
	s->buffer_proc_counter = ATOMIC_VAR_INIT(0);

	s->mpx_split_duration = -1;
	s->aud_split_duration = -1;
	s->mpx_limit_duration = 0;
	s->aud_limit_duration = 0;
	s->channel_count = -1;

	for (int i = 0; i < MAX_CIRCULAR_BUFFERS; i++) {
		pthread_rwlock_init(&s->buffers[i].rw, NULL);
		s->buffers[i].is_free = ATOMIC_VAR_INIT(1);
		s->buffers[i].lowpassed = NULL;
	}

	s->mpx_pipe_pattern = DEFAULT_MPX_PIPE_PATTERN;
	s->mpx_file_pattern = DEFAULT_MPX_FILE_PATTERN;
	s->aud_pipe_pattern = AUDIO_OGG_PIPE_PATTERN;
	s->aud_file_pattern = DEFAULT_AUDIO_FILE_PATTERN;

	s->aud_write_type = DEFAULT_AUD_WRITE_TYPE;
	s->mpx_write_type = DEFAULT_MPX_WRITE_TYPE;
	s->aud_frame_modulo = 0;
	s->aud_frame_size = 0;

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
	if (s->aud_write_type == WritePIPE && !s->aud_pipe_pattern) {
		fprintf(stderr, "Error: command pattern for audio is missing! Exiting..\n");
		exit(0);
	}

	if (s->mpx_write_type == WritePIPE && !s->mpx_pipe_pattern) {
		fprintf(stderr, "Error: command pattern for mpx is missing! Exiting..\n");
		exit(0);
	}
}

void demod_thread_cleanup(struct demod_thread_state *s)
{
	int i;

	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);

	close_all_channel_outputs(s, 1, 1, 0);

	for(i=0; i<s->channel_count; i++) {
		demod_cleanup(&s->demod_states[i]);
	}

	for (int i = 0; i < MAX_NUM_CHANNELS; ++i) {
		pthread_cond_destroy(&s->aud_files[i].ready);
		pthread_cond_destroy(&s->mpx_files[i].ready);
		pthread_mutex_destroy(&s->aud_files[i].ready_m);
		pthread_mutex_destroy(&s->mpx_files[i].ready_m);
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
	FILE * dev_file = NULL;
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

	while ((opt = getopt(argc, argv, "d:f:g:m:s:a:x:t:l:r:p:R:E:O:o:F:A:M:hTq:c:w:W:n:D:v")) != -1) {
		switch (opt) {
		case 'd':
			if (strlen(optarg) >= 3 && !strncmp("f:", optarg, 2)) {
				dev_file = fopen(&optarg[2], "rb");
				dongleid = "file";
			}
			else if (!strcmp("-", optarg) || !strcmp("-1", optarg)) {
				dev_file = stdin;
				dongleid = "stdin";
			}
			else {
				dongle.dev_index = verbose_device_search(optarg);
				dongleid = optarg;
			}
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
				dm_thr.aud_split_duration = atoi(&optarg[2]);
			} else if (!strncmp(optarg, "af:", 3)) {
				dm_thr.aud_frame_size = atoi(&optarg[3]);
				if ((dm_thr.aud_frame_size % 512) != 0) {
					dm_thr.aud_frame_size = dm_thr.aud_frame_size & (~511);
					fprintf(stderr, "split frame size needs to be multiple of 512. corrected to %d\n", dm_thr.aud_frame_size);
				}
			} else {
				dm_thr.aud_split_duration = atoi(optarg);
				dm_thr.mpx_split_duration = dm_thr.aud_split_duration;
			}
			break;
		case 'l':
			if (!strncmp(optarg, "x:", 2)) {
				dm_thr.mpx_limit_duration = atoi(&optarg[2]);
			} else if (!strncmp(optarg, "a:", 2)) {
				dm_thr.aud_limit_duration = atoi(&optarg[2]);
			} else {
				dm_thr.aud_limit_duration = atoi(optarg);
				dm_thr.mpx_limit_duration = dm_thr.aud_limit_duration;
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
		case 'o':
			if (!strcmp("q", optarg) || !strcmp("quit", optarg)) {
				on_overflow = 0;
			} else if (!strcmp("d", optarg) || !strcmp("discard", optarg)) {
				on_overflow = 1;
			} else if (!strcmp("i", optarg) || !strcmp("ign", optarg) || !strcmp("ignore", optarg)) {
				on_overflow = 2;
			} else {
				fprintf(stderr, "Warning: unknown argument for option '-o' on overflow: %s\n", optarg);
			}
			break;
		case 'q':
			dongle.rdc_block_const = atoi(optarg);
			break;
		case 'a':
			if (!strcmp(optarg, "n") || !strcmp(optarg, "N") || !strcmp(optarg, "%")) {
				dm_thr.aud_write_type = WriteNONE;
			} else if (!strcmp(optarg, "p:ogg")) {
				dm_thr.aud_pipe_pattern = AUDIO_OGG_PIPE_PATTERN;
				dm_thr.aud_write_type = WritePIPE;
			} else if (!strcmp(optarg, "p:mp3")) {
				dm_thr.aud_pipe_pattern = AUDIO_MP3_PIPE_PATTERN;
				dm_thr.aud_write_type = WritePIPE;
			} else if (!strncmp(optarg, "p:", 2)) {
				if (optarg[2])
					dm_thr.aud_pipe_pattern = &optarg[2];
				else
					fprintf(stderr, "activating audio pipe output - keeping previous pattern: %s\n", dm_thr.aud_pipe_pattern);
				dm_thr.aud_write_type = WritePIPE;
			} else if (!strncmp(optarg, "f:", 2)) {
				if (optarg[2])
					dm_thr.aud_file_pattern = &optarg[2];
				else
					fprintf(stderr, "activating audio file output - keeping previous pattern: %s\n", dm_thr.aud_file_pattern);
				dm_thr.aud_write_type = WriteFILE;
			} else {
				dm_thr.aud_file_pattern = optarg;
				dm_thr.aud_write_type = WriteFILE;
			}
			break;
		case 'x':
			if (!strcmp(optarg, "n") || !strcmp(optarg, "N") || !strcmp(optarg, "%")) {
				dm_thr.mpx_write_type = WriteNONE;
			} else if (!strncmp(optarg, "p:", 2)) {
				if (optarg[2])
					dm_thr.mpx_pipe_pattern = &optarg[2];
				else
					fprintf(stderr, "activating mpx pipe output - keeping previous pattern: %s\n", dm_thr.mpx_pipe_pattern);
				dm_thr.mpx_write_type = WritePIPE;
			} else if (!strncmp(optarg, "f:", 2)) {
				if (optarg[2])
					dm_thr.mpx_file_pattern = &optarg[2];
				else
					fprintf(stderr, "activating mpx file output - keeping previous pattern: %s\n", dm_thr.mpx_file_pattern);
				dm_thr.mpx_write_type = WriteFILE;
			} else {
				dm_thr.aud_file_pattern = optarg;
				dm_thr.mpx_write_type = WriteFILE;
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
			usage(verbosity);
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
		dongleid = "0";
	}

	if (dongle.dev_index < 0 && !dev_file) {
		exit(1);
	}

	if (!dev_file) {
		r = rtlsdr_open(&dongle.dev, (uint32_t)dongle.dev_index);
		if (r < 0) {
			fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dongle.dev_index);
			exit(1);
		}
	}
	else
	{
		dongle.dev = NULL;
		r = 0;
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

	if (dongle.dev) {

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

		if (r) {
			rtlsdr_close(dongle.dev);
			return 1;
		}
	}

	controller_fn(&dm_thr);

	/* open pipe closing threads if need */
	for (int ch = 0; ch < dm_thr.channel_count; ++ch) {
		if (dm_thr.aud_write_type == WritePIPE && dm_thr.aud_split_duration > 0) {
			pthread_create(&dm_thr.aud_files[ch].closing_thread, NULL, close_pipe_fn, (&dm_thr.aud_files[ch]));
		}

		if (dm_thr.mpx_write_type == WritePIPE && dm_thr.mpx_split_duration > 0) {
			pthread_create(&dm_thr.mpx_files[ch].closing_thread, NULL, close_pipe_fn, (&dm_thr.mpx_files[ch]));
		}
	}

	pthread_create(&dm_thr.thread, NULL, multi_demod_thread_fn, (void *)(&dm_thr));

	if (dongle.dev)
		rtlsdr_read_async(dongle.dev, rtlsdr_callback, &dongle, 0, dongle.buf_len);
	else
		r = read_from_file(dev_file);

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");
	}
	else if (dev_file) {
		fprintf(stderr, "\nEnd of input file or stream...\n");
	}
	else {
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);
	}

	if (dongle.dev)
		rtlsdr_cancel_async(dongle.dev);

	do_exit = 1;
	safe_cond_signal(&dm_thr.ready, &dm_thr.ready_m);
	if (verbosity)
		fprintf(stderr, "wait for demod thread to finish ..\n");
	pthread_join(dm_thr.thread, NULL);

	/* close pipes and the closing threads */
	{
		do_exit_close = 1;
		/* trigger cleanup in all threads */
		for(int ch = 0; ch < dm_thr.channel_count; ++ch) {
			if (dm_thr.aud_write_type == WritePIPE && dm_thr.aud_split_duration > 0) {
				if (verbosity)
					fprintf(stderr, "wait for audio channel %d's pclose() thread to finish ..\n", ch);
				safe_cond_signal(&dm_thr.aud_files[ch].ready, &dm_thr.aud_files[ch].ready_m);
			}
			if (dm_thr.mpx_write_type == WritePIPE && dm_thr.mpx_split_duration > 0) {
				if (verbosity)
					fprintf(stderr, "wait for mpx channel %d's pclose() thread to finish ..\n", ch);
				safe_cond_signal(&dm_thr.mpx_files[ch].ready, &dm_thr.mpx_files[ch].ready_m);
			}
		}
		/* wait until finished */
		for (int ch = 0; ch < dm_thr.channel_count; ++ch) {
			if (dm_thr.aud_write_type == WritePIPE && dm_thr.aud_split_duration > 0)
				pthread_join(dm_thr.aud_files[ch].closing_thread, NULL);
			if (dm_thr.mpx_write_type == WritePIPE && dm_thr.mpx_split_duration > 0)
				pthread_join(dm_thr.mpx_files[ch].closing_thread, NULL);
		}
	}

	/* dongle_cleanup(&dongle); */
	if (verbosity)
		fprintf(stderr, "wait for demod_thread cleanup ..\n");
	demod_thread_cleanup(&dm_thr);

	if (dongle.dev) {
		if (verbosity)
			fprintf(stderr, "closing dongle and exit\n");
		rtlsdr_close(dongle.dev);
	}

	if (num_detected_overflows || atomic_load(&num_discarded_buffers) || verbosity)
		fprintf(stderr, "dongle %s: detected %u overflows, discarded %u of %u received buffers in total\n",
			dongleid, num_detected_overflows, atomic_load(&num_discarded_buffers), atomic_load(&dm_thr.buffer_rcv_counter));

	return r >= 0 ? r : -r;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
