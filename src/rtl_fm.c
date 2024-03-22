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
#define DEFAULT_BUFFER_DUMP		4096

#define FREQUENCIES_LIMIT		1024

static int BufferDump = DEFAULT_BUFFER_DUMP;
static int OutputToStdout = 1;
static int MinCaptureRate = 1000000;

static volatile int do_exit = 0;
static int verbosity = 0;
static const char * dongleid = "?";
static int printLevels = 0;
static int printLevelNo = 1;
static int levelMax = 0;
static int levelMaxMax = 0;
static double levelSum = 0.0;
static int32_t prev_if_band_center_freq = 0;

static WaveWriteState waveWrState;


enum trigExpr { crit_IN =0, crit_OUT, crit_LT, crit_GT };
char * aCritStr[] = { "in", "out", "<", ">" };

time_t stop_time;
int duration = 0;


struct demod_thread_state
{
	pthread_t		thread;
	pthread_rwlock_t	rw;
	pthread_cond_t		ready;
	pthread_mutex_t	ready_m;
	struct output_state	*output_target;
	struct cmd_state	*cmd;

	struct demod_state demod;
};

struct cmd_state
{
	const char * filename;
	FILE * file;
	int lineNo;
	char acLine[4096];
	int checkADCmax;
	int checkADCrms;
	uint64_t prevFreq;
	uint64_t freq;
	int prevGain;
	uint32_t prevBandwidth;
	int gain;
	enum trigExpr trigCrit;
	double refLevel;
	double refLevelTol;
	int numMeas;
	int numBlockTrigger;
	char * command;
	char * args;
	double levelSum;
	int numSummed;
	int omitFirstFreqLevels;
	int waitTrigger[FREQUENCIES_LIMIT];
	int statNumLevels[FREQUENCIES_LIMIT];
	uint64_t statFreq[FREQUENCIES_LIMIT];
	double statSumLevels[FREQUENCIES_LIMIT];
	float statMinLevel[FREQUENCIES_LIMIT];
	float statMaxLevel[FREQUENCIES_LIMIT];
};

struct dongle_state
{
	rtlsdr_dev_t *dev;
	int	  dev_index;
	uint64_t userFreq;
	uint64_t freq;
	uint32_t rate;
	uint32_t bandwidth;
	int	  bccorner;  /* -1 for low band corner, 0 for band center, +1 for high band corner */
	int	  gain;
	int16_t  buf16[MAXIMUM_BUF_LENGTH];
	uint32_t buf_len;
	int	  ppm_error;
	int	  offset_tuning;
	int	  direct_sampling;
	int	  mute;
	struct demod_thread_state *demod_target;
	double samplePowSum;
	int samplePowCount;
	unsigned char sampleMax;

	int	  dc_block_raw;	/* flag: activate dc_block_raw_filter() */
	int	  rdc_avg[2];		/* state for dc_block_raw_filter() */
	int	  rdc_block_const;	/* parameter for dc_block_raw_filter() */
};

struct output_state
{
	pthread_t thread;
	FILE	 *file;
	char	 *filename;
	char	 *tempfilename;
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int	  result_len;
	int	  rate;
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
};

struct controller_state
{
	pthread_t thread;
	uint32_t freqs[FREQUENCIES_LIMIT];
	int	  freq_len;
	int	  freq_now;
	int	  edge;
	int	  wb_mode;
	pthread_cond_t hop;
	pthread_mutex_t hop_m;
	struct cmd_state *cmd;
};

/* multiple of these, eventually */
struct dongle_state dongle;
struct demod_thread_state dm_thr;
struct output_state output;
struct controller_state controller;
struct cmd_state cmd;


void usage(int verbosity)
{
	fprintf(stderr,
		"rtl_fm, a simple demodulator for RTL2832 based SDR-receivers\n"
		"rtl_fm  version %d.%d %s (%s)\n"
		"rtl-sdr library %d.%d %s\n\n",
		APP_VER_MAJOR, APP_VER_MINOR, APP_VER_ID, __DATE__,
		rtlsdr_get_version() >>16, rtlsdr_get_version() & 0xFFFF,
		rtlsdr_get_ver_id() );
	fprintf(stderr,
		"Usage:\trtl_fm -f freq [-options] [filename]\n"
		"\t-f frequency_to_tune_to [Hz]\n"
		"\t	use multiple -f for scanning (requires squelch)\n"
		"\t	ranges supported, -f 118M:137M:25k\n"
		"\t[-C command_filename: command file with comma seperated values (.csv). sets modulation 'raw']\n"
		"\t\tcommand file contains lines with: freq,gain,trig-crit,trig_level,trig_tolerance,#meas,#blocks,trigger_command,arguments\n"
		"\t\t with trig_crit one of 'in', 'out', 'lt' or 'gt'\n"
		"\t[-B num_samples at capture rate: remove that many samples at capture_rate after changing frequency (default: 4096)]\n"
		"\t[-m minimum_capture_rate Hz (default: 1m, min=900k, max=3.2m)]\n"
		"\t[-v increase verbosity (default: 0)]\n"
		"\t[-M modulation (default: fm)]\n"
		"\t	fm or nbfm or nfm, wbfm or wfm, raw or iq, am, ook, usb, lsb\n"
		"\t	wbfm == -M fm -s 170k -o 4 -A fast -r 32k -l 0 -E deemp\n"
		"\t	ook == -M am -E adc,  ook == -M am without adc option\n"
		"\t	raw mode (==iq) outputs 2x16 bit I/Q pairs\n"
		"\t[-s sample_rate (default: 24k)]\n"
		"\t[-d device_index or serial (default: 0 , -1 or '-' for stdin)]\n"
		"%s"
		"\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]\n"
		"\t[-D direct_sampling_mode (default: 0, 1 = I, 2 = Q, 3 = I below threshold, 4 = Q below threshold)]\n"
		"\t[-D direct_sampling_threshold_frequency (default: 0 use tuner specific frequency threshold for 3 and 4)]\n"
		"\t[-g tuner_gain (default: automatic)]\n"
		"\t[-w tuner_bandwidth in Hz (default: automatic)]\n"
		"\t[-W length of single buffer in units of 512 samples (default: 32 was 256)]\n"
		"\t[-l squelch_level (default: 0/off)]\n"
		"\t[-L N  prints levels every N calculations]\n"
		"\t	output are comma separated values (csv):\n"
		"\t	avg rms since last output, max rms since last output, overall max rms, squelch (paramed), rms, rms level, avg rms level\n"
		"\t[-c de-emphasis_time_constant in us for wbfm. 'us' or 'eu' for 75/50 us (default: us)]\n"
#if 0
		"\t	for fm squelch is inverted\n"
#endif
		"\t[-o oversampling (default: 1, 4 recommended)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-R run_seconds] specify number of seconds to run\n"
		"\t[-E enable_option (default: none)]\n"
		"\t	use multiple -E to enable multiple options\n"
		"\t	edge:   enable lower edge tuning\n"
		"\t	rdc:    enable dc blocking filter on raw I/Q data at capture rate\n"
		"\t	adc:    enable dc blocking filter on demodulated audio\n"
		"\t	dc:     same as adc\n"
		"\t	rtlagc: enable rtl2832's digital agc (default: off)\n"
		"\t	agc:    same as rtlagc\n"
		"\t	deemp:  enable de-emphasis filter\n"
		"\t	direct: enable direct sampling (bypasses tuner, uses rtl2832 xtal)\n"
		"\t	offset: enable offset tuning (only e4000 tuner)\n"
		"\t	bcc:    use tuner bandwidths center as band center (default)\n"
		"\t	bclo:   use tuner bandwidths low  corner as band center\n"
		"\t	bchi:   use tuner bandwidths high corner as band center\n"
		"\t[-q dc_avg_factor for option rdc (default: 9)]\n"
		"\t[-n disables demodulation output to stdout/file]\n"
		"\t[-H write wave Header to file (default: off)]\n"
		"\t	limitation: only 1st tuned frequency will be written into the header!\n"
		"\tfilename ('-' means stdout)\n"
		"\t	omitting the filename also uses stdout\n\n"
		"Experimental options:\n"
		"\t[-r resample_rate (default: none / same as -s)]\n"
		"\t[-t squelch_delay (default: 10)]\n"
		"\t	+values will mute/scan, -values will exit\n"
		"\t[-F fir_size (default: off)]\n"
		"\t	enables low-leakage downsample filter\n"
		"\t	size can be 0 or 9.  0 has bad roll off\n"
		"\t[-A std/fast/lut/ale choose atan math (default: std)]\n"
#if 0
		"\t[-C clip_path (default: off)\n"
		"\t (create time stamped raw clips, requires squelch)\n"
		"\t (path must have '\%s' and will expand to date_time_freq)\n"
		"\t[-H hop_fifo (default: off)\n"
		"\t (fifo will contain the active frequency)\n"
#endif
		"\n"
		"Produces signed 16 bit ints, use Sox or aplay to hear them.\n"
		"\trtl_fm ... | play -t raw -r 24k -es -b 16 -c 1 -V1 -\n"
		"\t		   | aplay -r 24000 -f S16_LE -t raw -c 1\n"
		"\t  -M wbfm  | play -r 32k ... \n"
		"\t  -s 22050 | multimon -t raw /dev/stdin\n\n"
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
	fprintf(stderr, "dongle %s: Signal %d caught, exiting!\n", dongleid, signum);
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


static char * trim(char * s) {
	char *p = s;
	int l = strlen(p);

	while(isspace(p[l - 1])) p[--l] = 0;
	while(*p && isspace(*p)) ++p;

	return p;
}


static void cmd_init(struct cmd_state *c)
{
	int k;
	c->filename = NULL;
	c->file = NULL;
	c->lineNo = 1;
	c->checkADCmax = 0;
	c->checkADCrms = 0;
	c->prevFreq = -1;
	c->prevGain = -200;
	c->prevBandwidth = -1;
	c->freq = 0;
	c->gain = 0;
	c->trigCrit = crit_IN;
	c->refLevel = 0.0;
	c->refLevelTol = 0.0;
	c->numMeas = 0;
	c->numBlockTrigger = 0;
	c->command = NULL;
	c->args = NULL;
	c->levelSum = 0.0;
	c->numSummed = 0;
	c->omitFirstFreqLevels = 3;
	for (k = 0; k < FREQUENCIES_LIMIT; k++) {
		c->waitTrigger[k] = 0;
		c->statNumLevels[k] = 0;
		c->statFreq[k] = 0;
		c->statSumLevels[k] = 0.0;
		c->statMinLevel[k] = 0.0F;
		c->statMaxLevel[k] = 0.0F;
	}
}

static int toNextCmdLine(struct cmd_state *c)
{
	const char * delim = ",";
	char * pLine = NULL;
	char * pCmdFreq = NULL;
	char * pCmdGain = NULL;
	char * pCmdTrigCrit = NULL;
	char * pCmdLevel = NULL;
	char * pCmdTol = NULL;
	char * pCmdNumMeas = NULL;
	char * pCmdNumBlockTrigger = NULL;
	int numValidLines = 1;  /* assume valid lines */
	while (1) {
		if (c->file && feof(c->file)) {
			if (!numValidLines) {
				fprintf(stderr, "error: command file '%s' does not contain any valid lines!\n", c->filename);
				return 0;
			}
			fclose(c->file);
			c->file = NULL;
		}
		if (!c->file) {
			c->file = fopen(c->filename, "r");
			numValidLines = 0;
			c->lineNo = 0;
		}
		if (!c->file)
			return 0;
		pLine = fgets(c->acLine, 4096, c->file);
		if (!pLine) {
			continue;
		}
		c->lineNo++;
		pLine = trim(c->acLine);
		if (pLine[0]=='#' || pLine[0]==0)
			continue;  /* detect comment lines and empty lines */

		pCmdFreq = strtok(pLine, delim);
		if (!pCmdFreq) { fprintf(stderr, "error parsing frequency in line %d of command file!\n", c->lineNo); continue; }
		pCmdFreq = trim(pCmdFreq);
		/* check keywords */
		if (!strcmp(pCmdFreq, "adc") || !strcmp(pCmdFreq, "adcmax")) {
			c->checkADCmax = 1;
			continue;
		}
		else if (!strcmp(pCmdFreq, "adcrms")) {
			c->checkADCrms = 1;
			continue;
		}
		c->freq = (uint64_t)atofs(pCmdFreq);

		pCmdGain = strtok(NULL, delim);
		if (!pCmdGain) { fprintf(stderr, "error parsing gain in line %d of command file!\n", c->lineNo); continue; }
		pCmdGain = trim(pCmdGain);
		if (!strcmp(pCmdGain,"auto") || !strcmp(pCmdGain,"a"))
			c->gain = AUTO_GAIN;
		else
			c->gain = (int)(atof(pCmdGain) * 10);

		pCmdTrigCrit = strtok(NULL, delim);
		if (!pCmdTrigCrit) { fprintf(stderr, "error parsing expr in line %d of command file!\n", c->lineNo); continue; }
		pCmdTrigCrit = trim(pCmdTrigCrit);
		if (!strcmp(pCmdTrigCrit,"in"))			c->trigCrit = crit_IN;
		else if (!strcmp(pCmdTrigCrit,"=="))	c->trigCrit = crit_IN;
		else if (!strcmp(pCmdTrigCrit,"out"))	c->trigCrit = crit_OUT;
		else if (!strcmp(pCmdTrigCrit,"!="))	c->trigCrit = crit_OUT;
		else if (!strcmp(pCmdTrigCrit,"<>"))	c->trigCrit = crit_OUT;
		else if (!strcmp(pCmdTrigCrit,"lt"))	c->trigCrit = crit_LT;
		else if (!strcmp(pCmdTrigCrit,"<"))		c->trigCrit = crit_LT;
		else if (!strcmp(pCmdTrigCrit,"gt"))	c->trigCrit = crit_GT;
		else if (!strcmp(pCmdTrigCrit,">"))		c->trigCrit = crit_GT;
		else { fprintf(stderr, "error parsing expr in line %d of command file!\n", c->lineNo); continue; }

		pCmdLevel = strtok(NULL, delim);
		if (!pCmdLevel) { fprintf(stderr, "error parsing level in line %d of command file!\n", c->lineNo); continue; }
		c->refLevel = atof(trim(pCmdLevel));

		pCmdTol = strtok(NULL, delim);
		if (!pCmdTol) { fprintf(stderr, "error parsing tolerance in line %d of command file!\n", c->lineNo); continue; }
		c->refLevelTol = atof(trim(pCmdTol));

		pCmdNumMeas = strtok(NULL, delim);
		if (!pCmdNumMeas) { fprintf(stderr, "error parsing #measurements in line %d of command file!\n", c->lineNo); continue; }
		c->numMeas = atoi(trim(pCmdNumMeas));
		if (c->numMeas <= 0) { fprintf(stderr, "warning: fixed #measurements from %d to 10 in line %d of command file!\n", c->numMeas, c->lineNo); c->numMeas=10; }

		pCmdNumBlockTrigger = strtok(NULL, delim);
		if (!pCmdNumBlockTrigger) { fprintf(stderr, "error parsing #blockTrigger in line %d of command file!\n", c->lineNo); continue; }
		c->numBlockTrigger = atoi(trim(pCmdNumBlockTrigger));

		c->command = strtok(NULL, delim);
		/* no check: allow empty string. just trim it */
		if (c->command)
			c->command = trim(c->command);

		c->args = strtok(NULL, delim);
		/* no check: allow empty string. just trim it */
		if (c->args)
			c->args = trim(c->args);

		if (verbosity >= 2)
			fprintf(stderr, "read from cmd file: freq %.3f kHz, gain %0.1f dB, level %s {%.1f +/- %.1f}, cmd '%s %s'\n",
				c->freq /1000.0, c->gain /10.0, 
				aCritStr[c->trigCrit], c->refLevel, c->refLevelTol,
				(c->command ? c->command : "%"), (c->args ? c->args : "") );

		numValidLines++;
		return 1;
	}

	return 0;
}

static int testTrigCrit(struct cmd_state *c, double level)
{
	switch(c->trigCrit)
	{
	case crit_IN:	return ( c->refLevel-c->refLevelTol <= level && level <= c->refLevel+c->refLevelTol );
	case crit_OUT:	return ( c->refLevel-c->refLevelTol > level || level > c->refLevel+c->refLevelTol );
	case crit_LT:	return ( level < c->refLevel-c->refLevelTol );
	case crit_GT:	return ( level > c->refLevel+c->refLevelTol );
	}
	return 0;
}

static void checkTriggerCommand(struct cmd_state *c, unsigned char adcSampleMax, double powerSum, int powerCount )
{
	char acRepFreq[32], acRepGain[32], acRepMLevel[32], acRepRefLevel[32], acRepRefTolerance[32];
	char * execSearchStrings[7] = { "!freq!", "!gain!", "!mlevel!", "!crit!", "!reflevel!", "!reftol!", NULL };
	char * execReplaceStrings[7] = { acRepFreq, acRepGain, acRepMLevel, NULL, acRepRefLevel, acRepRefTolerance, NULL };
	double triggerLevel;
	double adcRms = 0.0;
	int k, triggerCommand = 0;
	int adcMax = (int)adcSampleMax - 127;
	char adcText[128];

	if (c->numSummed != c->numMeas)
		return;

	if (c->omitFirstFreqLevels) {
		/* workaround: measured levels of first controlled frequency looks wrong! */
		c->omitFirstFreqLevels--;
		return;
	}

	/* decrease all counters */
	for ( k = 0; k < FREQUENCIES_LIMIT; k++ ) {
		if ( c->waitTrigger[k] > 0 ) {
			c->waitTrigger[k] -= c->numMeas;
			if ( c->waitTrigger[k] < 0 )
				c->waitTrigger[k] = 0;
		}
	}
	triggerLevel = 20.0 * log10( 1E-10 + c->levelSum / c->numSummed );
	triggerCommand = testTrigCrit(c, triggerLevel);

	/* update statistics */
	if ( c->lineNo < FREQUENCIES_LIMIT ) {
		if ( c->statNumLevels[c->lineNo] == 0 ) {
			++c->statNumLevels[c->lineNo];
			c->statFreq[c->lineNo] = c->freq;
			c->statSumLevels[c->lineNo] = triggerLevel;
			c->statMinLevel[c->lineNo] = (float)triggerLevel;
			c->statMaxLevel[c->lineNo] = (float)triggerLevel;
		} else if ( c->statFreq[c->lineNo] == c->freq ) {
			++c->statNumLevels[c->lineNo];
			c->statSumLevels[c->lineNo] += triggerLevel;
			if ( c->statMinLevel[c->lineNo] > (float)triggerLevel )
				c->statMinLevel[c->lineNo] = (float)triggerLevel;
			if ( c->statMaxLevel[c->lineNo] < (float)triggerLevel )
				c->statMaxLevel[c->lineNo] = (float)triggerLevel;
		}
	}

	adcText[0] = 0;
	if (c->checkADCmax && c->checkADCrms) {
		adcRms = (powerCount >0) ? sqrt( powerSum / powerCount ) : -1.0;
		sprintf(adcText, "adc max %3d%s rms %5.1f ", adcMax, (adcMax>=64 ? (adcMax>=120 ? "!!" : "! " ) : "  "), adcRms );
	}
	else if (c->checkADCmax)
		sprintf(adcText, "adc max %3d%s ", adcMax, (adcMax>=64 ? (adcMax>=120 ? "!!" : "! " ) : "  ") );
	else if (c->checkADCrms) {
		adcRms = (powerCount >0) ? sqrt( powerSum / powerCount ) : -1.0;
		sprintf(adcText, "adc rms %5.1f ", adcRms );
	}

	if ( c->lineNo < FREQUENCIES_LIMIT && c->waitTrigger[c->lineNo] <= 0 ) {
			c->waitTrigger[c->lineNo] = triggerCommand ? c->numBlockTrigger : 0;
			if (verbosity)
				fprintf(stderr, "%.3f kHz: gain %4.1f + level %4.1f dB %s=> %s\n",
					(double)c->freq /1000.0, 0.1*c->gain, triggerLevel, adcText,
					(triggerCommand ? "activates trigger" : "does not trigger") );
			if (triggerCommand && c->command && c->command[0]) {
				fprintf(stderr, "command to trigger is '%s %s'\n", c->command, c->args);
				/* prepare search/replace of special parameters for command arguments */
				snprintf(acRepFreq, 32, "%.0f", (double)c->freq);
				snprintf(acRepGain, 32, "%d", c->gain);
				snprintf(acRepMLevel, 32, "%d", (int)(0.5 + triggerLevel*10.0) );
				execReplaceStrings[3] = aCritStr[c->trigCrit];
				snprintf(acRepRefLevel, 32, "%d", (int)(0.5 + c->refLevel*10.0) );
				snprintf(acRepRefTolerance, 32, "%d", (int)(0.5 + c->refLevelTol*10.0) );
				executeInBackground( c->command, c->args, execSearchStrings, execReplaceStrings );
			}
	} else if (verbosity) {
		fprintf(stderr, "%.3f kHz: gain %4.1f + level %4.1f dB %s=> %s, blocks for %d\n",
			(double)c->freq /1000.0, 0.1*c->gain, triggerLevel, adcText, (triggerCommand ? "would trigger" : "does not trigger"),
			(c->lineNo < FREQUENCIES_LIMIT ? c->waitTrigger[c->lineNo] : -1 ) );
	}
	c->numSummed++;
}


void full_demod(struct demod_thread_state *dt)
{
	struct cmd_state *c = dt->cmd;
	struct demod_state *d = &dt->demod;
	double freqK, avgRms, rmsLevel, avgRmsLevel;
	int sr = 0;
	static int printBlockLen = 1;

	downsample_input(d);

	/* power squelch */
	if (d->squelch_level) {
		sr = rms(d->lowpassed, d->lp_len, 1, d->omit_dc_fix);
		if (sr >= 0) {
			if (sr < d->squelch_level) {
				d->squelch_hits++;
				for (int i=0; i<d->lp_len; i++) {
					d->lowpassed[i] = 0;
				}
			} else {
				d->squelch_hits = 0;}
		}
	}

	if (printLevels) {
		if (!sr)
			sr = rms(d->lowpassed, d->lp_len, 1, d->omit_dc_fix);
		--printLevelNo;
		if (printLevels && sr >= 0) {
			levelSum += sr;
			if (levelMax < sr)		levelMax = sr;
			if (levelMaxMax < sr)	levelMaxMax = sr;
			if  (!printLevelNo) {
				printLevelNo = printLevels;
				freqK = dongle.userFreq /1000.0;
				avgRms = levelSum / printLevels;
				rmsLevel = 20.0 * log10( 1E-10 + sr );
				avgRmsLevel = 20.0 * log10( 1E-10 + avgRms );
				fprintf(stderr, "%.3f kHz, %.1f avg rms, %d max rms, %d max max rms, %d squelch rms, %d rms, %.1f dB rms level, %.2f dB avg rms level\n",
					freqK, avgRms, levelMax, levelMaxMax, d->squelch_level, sr, rmsLevel, avgRmsLevel );
				levelMax = 0;
				levelSum = 0;
			}
		}
	}

	if (c->filename) {
		if (!sr)
			sr = rms(d->lowpassed, d->lp_len, 1, d->omit_dc_fix);
		if ( printBlockLen && verbosity ) {
			fprintf(stderr, "block length for rms after decimation is %d samples\n", d->lp_len);
			if ( d->lp_len < 128 )
				fprintf(stderr, "\n  WARNING: increase block length with option -W\n\n");
			--printBlockLen;
		}
		if (!c->numSummed)
			c->levelSum = 0;
		if (c->numSummed < c->numMeas && sr >= 0) {
			c->levelSum += sr;
			c->numSummed++;
		}
	}

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

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct dongle_state *s = ctx;
	struct demod_thread_state *dt = s->demod_target;
	struct cmd_state *c = dt->cmd;
	int16_t *buf16;
	int i, muteLen = s->mute;
	unsigned char sampleMax;
	uint32_t sampleP, samplePowSum = 0.0;
	int samplePowCount = 0, step = 2;
	time_t rawtime;

	if (do_exit || !ctx) {
		return;
	}

	time(&rawtime);
	if (duration > 0 && rawtime >= stop_time) {
		do_exit = 1;
		fprintf(stderr, "Time expired, exiting!\n");
		rtlsdr_cancel_async(dongle.dev);
	}
	if (s->mute) {
		if(muteLen > (int)len)
			muteLen = len;
		s->mute -= muteLen;  /* we may need to mute multiple blocks */
		if(!c->filename) {
			for (i=0; i<muteLen; i++)
				buf[i] = 127;
		}
		/* reset adc max and power */
		s->samplePowSum = 0.0;
		s->samplePowCount = 0;
		s->sampleMax = 0;
	}
	/* OR all samples to allow checking overflow
	 * - before conversion to 16 bit and before DC filtering.
	 * we only get bitmask of positive samples (after -127) but that won't matter */
	if (c->checkADCmax ) {
		sampleMax = s->sampleMax;
		for (i=0; i<(int)len; i++) {
			if ( buf[i] > sampleMax )
				sampleMax = buf[i];
		}
		s->sampleMax = sampleMax;
	}
	if (c->checkADCrms ) {
		while ( (int)len >= 16384 * step )
			step += 2;
		for (i=0; i<(int)len; i+= step) {
			sampleP  = ( (int)buf[i]   -127 ) * ( (int)buf[i]   -127 );  /* I^2 */
			sampleP += ( (int)buf[i+1] -127 ) * ( (int)buf[i+1] -127 );  /* Q^2 */
			samplePowSum += sampleP;
			++samplePowCount;
		}
		s->samplePowSum += (double)samplePowSum / samplePowCount;
		s->samplePowCount += 1;
	}

	buf16 = s->buf16;

	/* 1st: convert to 16 bit - to allow easier calculation of DC */
	for (i=0; i<(int)len; i++) {
		buf16[i] = ( (int16_t)buf[i] - 127 );
	}
	/* 2nd: do DC filtering BEFORE up-mixing */
	if (s->dc_block_raw) {
		dc_block_raw_filter(buf16, (int)len, s->rdc_avg, s->rdc_block_const);
	}
	if (muteLen && c->filename)
		return;	/* "mute" after the dc_block_raw_filter(), giving it time to remove the new DC */
	/* 3rd: down-mixing */
	if (!s->offset_tuning) {
		rotate16_neg90(s->buf16, (int)len);
	}

	pthread_rwlock_wrlock(&dt->rw);
	memcpy(dt->demod.lowpassed, s->buf16, 2*len);
	dt->demod.lp_len = len;
	pthread_rwlock_unlock(&dt->rw);
	safe_cond_signal(&dt->ready, &dt->ready_m);
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
		/* while (!atomic_load(&mds->buffers[mds->buffer_write_idx].is_free))
			usleep(1000);
		*/
		rtlsdr_callback(buf, 2 *nmemb, &dongle);
	}
	free(buf);
	return 0;
}


static void *demod_thread_fn(void *arg)
{
	struct demod_thread_state *dt = arg;
	struct demod_state *d = &dt->demod;
	struct output_state *o = dt->output_target;
	struct cmd_state *c = dt->cmd;
	while (!do_exit) {
		safe_cond_wait(&dt->ready, &dt->ready_m);
		pthread_rwlock_wrlock(&dt->rw);
		full_demod(dt);
		pthread_rwlock_unlock(&dt->rw);

		if (d->squelch_level && d->squelch_hits > d->conseq_squelch) {
			d->squelch_hits = d->conseq_squelch + 1;  /* hair trigger */
			safe_cond_signal(&controller.hop, &controller.hop_m);
			continue;
		}

		if (do_exit)
			break;

		if (c->filename && c->numSummed >= c->numMeas) {
			checkTriggerCommand(c, dongle.sampleMax, dongle.samplePowSum, dongle.samplePowCount);

			safe_cond_signal(&controller.hop, &controller.hop_m);
			continue;
		}

		if (OutputToStdout) {
			pthread_rwlock_wrlock(&o->rw);
			memcpy(o->result, d->result, 2*d->result_len);
			o->result_len = d->result_len;
			pthread_rwlock_unlock(&o->rw);
			safe_cond_signal(&o->ready, &o->ready_m);
		}
	}
	return 0;
}

static void *output_thread_fn(void *arg)
{
	struct output_state *s = arg;
	if (!waveWrState.waveHdrStarted) {
		while (!do_exit) {
			/* use timedwait and pad out under runs */
			safe_cond_wait(&s->ready, &s->ready_m);
			pthread_rwlock_rdlock(&s->rw);
			fwrite(s->result, 2, s->result_len, s->file);
			pthread_rwlock_unlock(&s->rw);
		}
	} else {
		while (!do_exit) {
			/* use timedwait and pad out under runs */
			safe_cond_wait(&s->ready, &s->ready_m);
			pthread_rwlock_rdlock(&s->rw);
			/* distinguish for endianness: wave requires little endian */
			waveWriteSamples(&waveWrState, s->file, s->result, s->result_len, 0);
			pthread_rwlock_unlock(&s->rw);
		}
	}
	return 0;
}

static int optimal_settings(uint64_t freq, uint32_t rate)
{
	/* giant ball of hacks
	 * seems unable to do a single pass, 2:1
	 */
	uint64_t capture_freq;
	uint32_t capture_rate;
	struct dongle_state *d = &dongle;
	struct demod_thread_state *dt = &dm_thr;
	struct demod_state *dm = &dt->demod;
	struct controller_state *cs = &controller;
	dm->downsample = (MinCaptureRate / dm->rate_in) + 1;
	if (dm->downsample_passes) {
		dm->downsample_passes = (int)( ceil(log2(dm->downsample)) + 0.1);
		if (dm->downsample_passes > MAXIMUM_DOWNSAMPLE_PASSES) {
			fprintf(stderr, "downsample_passes = %d exceeds it's limit. setting to %d\n", dm->downsample, MAXIMUM_DOWNSAMPLE_PASSES);
			dm->downsample_passes = MAXIMUM_DOWNSAMPLE_PASSES;
		}
		dm->downsample = 1 << dm->downsample_passes;
	}
	if (verbosity >= 2) {
		fprintf(stderr, "downsample_passes = %d (= # of fifth_order() iterations), downsample = %d\n", dm->downsample_passes, dm->downsample );
	}
	capture_freq = freq;
	capture_rate = dm->downsample * dm->rate_in;
	if (verbosity >= 2)
		fprintf(stderr, "capture_rate = downsample * rate_in = %d * %d = %d\n", dm->downsample, dm->rate_in, capture_rate );
	if (capture_rate > 3200U*1000U) {
		fprintf(stderr, "Error: Capture rate of %u Hz exceedds 3200k!\n", (unsigned)capture_rate);
		return 1;
	}
	else if (capture_rate > 2400U*1000U) {
		fprintf(stderr, "Warning: Capture rate of %u Hz is too big (exceeds 2400k) for continous transfer!\n", (unsigned)capture_rate);
	}
	if (!d->offset_tuning) {
		capture_freq = freq - capture_rate/4;
		if (verbosity >= 2)
			fprintf(stderr, "optimal_settings(freq = %f MHz): capture_freq = freq - capture_rate/4 = %f MHz\n", freq * 1E-6, capture_freq * 1E-6 );
	}
	capture_freq += cs->edge * dm->rate_in / 2;
	if (verbosity >= 2)
		fprintf(stderr, "optimal_settings(freq = %f MHz): capture_freq +=  cs->edge * dm->rate_in / 2 = %d * %d / 2 = %f MHz\n", freq * 1E-6, cs->edge, dm->rate_in, capture_freq * 1E-6 );
	dm->output_scale = (1<<15) / (128 * dm->downsample);
	if (dm->output_scale < 1) {
		dm->output_scale = 1;}
	if (verbosity >= 2)
		fprintf(stderr, "output_scale = %d (used for AM/USB/LSB demodulation)\n", dm->output_scale);
	if (dm->mode_demod == &fm_demod) {
		dm->output_scale = 1;}
	d->userFreq = freq;
	d->freq = capture_freq;
	d->rate = capture_rate;
	if (verbosity >= 2)
		fprintf(stderr, "optimal_settings(freq = %f MHz) delivers freq %f MHz, rate %.0f\n", freq * 1E-6, d->freq * 1E-6, (double)d->rate );
	return 0;
}

static void *controller_thread_fn(void *arg)
{
	/* thoughts for multiple dongles
	 * might be no good using a controller thread if retune/rate blocks
	 */
	int i, r, execWaitHop = 1;
	int32_t if_band_center_freq = 0;
	struct controller_state *s = arg;
	struct cmd_state *c = s->cmd;
	struct demod_state *demod = &dm_thr.demod;

	if (s->wb_mode) {
		if (verbosity)
			fprintf(stderr, "wbfm: adding 16000 Hz to every input frequency\n");
		for (i=0; i < s->freq_len; i++) {
			s->freqs[i] += 16000;}
	}

	/* set up primary channel */
	if (c->filename) {
		dongle.mute = dongle.rate; /* over a second - until parametrized the dongle */
		toNextCmdLine(c);
		/*fprintf(stderr, "\nswitched to next command line. new freq %u\n", c->freq);*/
		s->freqs[0] = c->freq;
		execWaitHop = 0;
	}

	optimal_settings(s->freqs[0], demod->rate_in);
	if (dongle.direct_sampling) {
		verbose_direct_sampling(dongle.dev, 1);}
	if (dongle.offset_tuning) {
		verbose_offset_tuning(dongle.dev);}

	/* Set the frequency */
	if (verbosity) {
		fprintf(stderr, "verbose_set_frequency(%f MHz)\n", dongle.userFreq * 1E-6);
		if (!dongle.offset_tuning)
			fprintf(stderr, "  frequency is away from parametrized one, to avoid negative impact from dc\n");
	}
	verbose_set_frequency(dongle.dev, dongle.freq);
	fprintf(stderr, "Oversampling input by: %ix.\n", demod->downsample);
	fprintf(stderr, "Oversampling output by: %ix.\n", demod->post_downsample);
	fprintf(stderr, "Buffer size: %u Bytes == %u quadrature samples == %0.2fms\n",
		(unsigned)dongle.buf_len,
		(unsigned)dongle.buf_len / 2,
		1000 * 0.5 * (float)dongle.buf_len / (float)dongle.rate);

	/* Set the sample rate */
	if (verbosity)
		fprintf(stderr, "verbose_set_sample_rate(%.0f Hz)\n", (double)dongle.rate);
	verbose_set_sample_rate(dongle.dev, dongle.rate);
	fprintf(stderr, "Output at %u Hz.\n", demod->rate_in/demod->post_downsample);

	if ( dongle.bandwidth ) {
		if_band_center_freq = dongle.userFreq - dongle.freq;
		if (dongle.bccorner < 0)
			if_band_center_freq += ( dongle.bandwidth - demod->rate_out ) / 2;
		else if (dongle.bccorner > 0)
			if_band_center_freq -= ( dongle.bandwidth - demod->rate_out ) / 2;

		if ( prev_if_band_center_freq != if_band_center_freq ) {
			r = rtlsdr_set_tuner_band_center(dongle.dev, if_band_center_freq );
			if (r)
				fprintf(stderr, "WARNING: Failed to set band center.\n");
			else {
				prev_if_band_center_freq = if_band_center_freq;
				if (verbosity)
					fprintf(stderr, "rtlsdr_set_tuner_band_center(%.0f Hz) successful\n", (double)if_band_center_freq);
			}
		}
	}

	while (!do_exit) {
		if (execWaitHop)
			safe_cond_wait(&s->hop, &s->hop_m);
		execWaitHop = 1;  /* execute following safe_cond_wait()'s */
		/* fprintf(stderr, "\nreceived hop condition\n"); */
		if (s->freq_len <= 1 && !c->filename) {
			continue;}
		if (!c->filename) {
			/* hacky hopping */
			s->freq_now = (s->freq_now + 1) % s->freq_len;
			optimal_settings(s->freqs[s->freq_now], demod->rate_in);
			rtlsdr_set_center_freq64(dongle.dev, dongle.freq);
			if ( dongle.bandwidth ) {
				if_band_center_freq = dongle.userFreq - dongle.freq;
				if ( prev_if_band_center_freq != if_band_center_freq ) {
					r = rtlsdr_set_tuner_band_center(dongle.dev, if_band_center_freq );
					if (r)
						fprintf(stderr, "WARNING: Failed to set band center.\n");
					else {
						prev_if_band_center_freq = if_band_center_freq;
						if (verbosity)
							fprintf(stderr, "rtlsdr_set_tuner_band_center(%.0f Hz) successful\n", (double)if_band_center_freq);
					}
				}
			}
			dongle.mute = DEFAULT_BUFFER_DUMP;
		} else {
			dongle.mute = 2 * dongle.rate; /* over a second - until parametrized the dongle */
			c->numSummed = 0;

			toNextCmdLine(c);

			optimal_settings(c->freq, demod->rate_in);
			/* 1- set center frequency */
			if (c->prevFreq != dongle.freq) {
				rtlsdr_set_center_freq64(dongle.dev, dongle.freq);
				c->prevFreq = dongle.freq;
			}
			/* 2- Set the tuner gain */
			if (c->prevGain != c->gain) {
				if (c->gain == AUTO_GAIN) {
					r = rtlsdr_set_tuner_gain_mode(dongle.dev, 0);
					if (r != 0)
						fprintf(stderr, "WARNING: Failed to set automatic tuner gain.\n");
					else
						c->prevGain = c->gain;
				} else {
					c->gain = nearest_gain(dongle.dev, c->gain);
					r = rtlsdr_set_tuner_gain_mode(dongle.dev, 1);
					if (r < 0)
						fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
					else {
						r = rtlsdr_set_tuner_gain(dongle.dev, c->gain);
						if (r != 0)
							fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
						else
							c->prevGain = c->gain;
					}
				}
			}
			/* 3- Set tuner bandwidth */
			if (c->prevBandwidth != dongle.bandwidth) {
				r = rtlsdr_set_tuner_bandwidth(dongle.dev, dongle.bandwidth);
				if (r < 0)
					fprintf(stderr, "WARNING: Failed to set bandwidth.\n");
				else
					c->prevBandwidth = dongle.bandwidth;
			}
			/*  */
			if ( dongle.bandwidth ) {
				if_band_center_freq = dongle.userFreq - dongle.freq;
				if ( prev_if_band_center_freq != if_band_center_freq ) {
					r = rtlsdr_set_tuner_band_center(dongle.dev, if_band_center_freq );
					if (r)
						fprintf(stderr, "WARNING: Failed to set band center.\n");
					else {
						prev_if_band_center_freq = if_band_center_freq;
						if (verbosity)
							fprintf(stderr, "rtlsdr_set_tuner_band_center(%.0f Hz) successful\n", (double)if_band_center_freq);
					}
				}
			}
			/* 4- Set ADC samplerate *
			r = rtlsdr_set_sample_rate(dongle.dev, dongle.rate);
			if (r < 0)
				fprintf(stderr, "WARNING: Failed to set sample rate.\n");
			*/

			c->levelSum = 0;
			c->numSummed = 0;
			/* reset DC filters */
			demod->adc_avg = 0;
			dongle.rdc_avg[0] = 0;
			dongle.rdc_avg[1] = 0;
			dongle.mute = BufferDump;
			/* reset adc max and power */
			dongle.samplePowSum = 0.0;
			dongle.samplePowCount = 0;
			dongle.sampleMax = 0;
		}

	}
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
		s->freqs[s->freq_len] = (uint32_t)i;
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
	s->offset_tuning = 0;
	s->demod_target = &dm_thr;
	s->samplePowSum = 0.0;
	s->samplePowCount = 0;
	s->sampleMax = 0;
	s->bandwidth = 0;
	s->bccorner = 0;
	s->buf_len = 32 * 512;  /* see rtl_tcp */

	s->dc_block_raw = 0;
	s->rdc_avg[0] = 0;
	s->rdc_avg[1] = 0;
	s->rdc_block_const = 9;
}

void demod_thread_init(struct demod_thread_state *s, struct output_state *output, struct cmd_state *cmd)
{
	demod_init(&s->demod, 1, 1);
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
	s->output_target = output;
	s->cmd = cmd;
}

void demod_thread_cleanup(struct demod_thread_state *s)
{
	demod_cleanup(&s->demod);
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void output_init(struct output_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
}

void output_cleanup(struct output_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void controller_init(struct controller_state *s)
{
	s->freqs[0] = 100000000;
	s->freq_len = 0;
	s->edge = 0;
	s->wb_mode = 0;
	pthread_cond_init(&s->hop, NULL);
	pthread_mutex_init(&s->hop_m, NULL);
	s->cmd = &cmd;
}

void controller_cleanup(struct controller_state *s)
{
	pthread_cond_destroy(&s->hop);
	pthread_mutex_destroy(&s->hop_m);
}

void sanity_checks(void)
{
	if (controller.freq_len == 0) {
		fprintf(stderr, "Please specify a frequency.\n");
		exit(1);
	}

	if (controller.freq_len >= FREQUENCIES_LIMIT) {
		fprintf(stderr, "Too many channels, maximum %i.\n", FREQUENCIES_LIMIT);
		exit(1);
	}

	if (controller.freq_len > 1 && dm_thr.demod.squelch_level == 0) {
		fprintf(stderr, "Please specify a squelch level.  Required for scanning multiple frequencies.\n");
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
	FILE * dev_file = NULL;
	int enable_biastee = 0;
	const char * rtlOpts = NULL;
	enum rtlsdr_ds_mode ds_mode = RTLSDR_DS_IQ;
	uint32_t ds_temp, ds_threshold = 0;
	int timeConstant = 75; /* default: U.S. 75 uS */
	int rtlagc = 0;
	struct demod_state *demod = &dm_thr.demod;

	initWaveWriteState(&waveWrState);
	dongle_init(&dongle);
	demod_thread_init(&dm_thr, &output, &cmd);
	output_init(&output);
	controller_init(&controller);
	cmd_init(&cmd);

	while ((opt = getopt(argc, argv, "d:f:g:s:b:l:o:t:r:p:R:E:O:F:A:M:hTC:B:m:L:q:c:w:W:D:nHv")) != -1) {
		switch (opt) {
		case 'd':
			if (!strcmp("-", optarg) || !strcmp("-1", optarg)) {
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
			if (controller.freq_len >= FREQUENCIES_LIMIT) {
				break;}
			if (strchr(optarg, ':'))
				{frequency_range(&controller, optarg);}
			else
			{
				controller.freqs[controller.freq_len] = (uint32_t)atofs(optarg);
				controller.freq_len++;
			}
			break;
		case 'C':
			cmd.filename = optarg;
			demod->mode_demod = &raw_demod;
			break;
		case 'm':
			MinCaptureRate = (int)atofs(optarg);
			break;
		case 'B':
			BufferDump = atoi(optarg);
			break;
		case 'n':
			OutputToStdout = 0;
			break;
		case 'g':
			dongle.gain = (int)(atof(optarg) * 10);
			break;
		case 'l':
			demod->squelch_level = (int)atof(optarg);
			break;
		case 'L':
			printLevels = (int)atof(optarg);
			break;
		case 's':
			demod->rate_in = (uint32_t)atofs(optarg);
			demod->rate_out = (uint32_t)atofs(optarg);
			break;
		case 'r':
			output.rate = (int)atofs(optarg);
			demod->rate_out2 = (int)atofs(optarg);
			break;
		case 'o':
			fprintf(stderr, "Warning: -o is very buggy\n");
			demod->post_downsample = (int)atof(optarg);
			if (demod->post_downsample < 1 || demod->post_downsample > MAXIMUM_OVERSAMPLE) {
				fprintf(stderr, "Oversample must be between 1 and %i\n", MAXIMUM_OVERSAMPLE);}
			break;
		case 't':
			demod->conseq_squelch = (int)atof(optarg);
			if (demod->conseq_squelch < 0) {
				demod->conseq_squelch = -demod->conseq_squelch;
				demod->terminate_on_squelch = 1;
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
			if (strcmp("edge",  optarg) == 0) {
				controller.edge = 1;}
			if (strcmp("dc", optarg) == 0 || strcmp("adc", optarg) == 0) {
				demod->dc_block_audio = 1;}
			if (strcmp("rdc", optarg) == 0) {
				dongle.dc_block_raw = demod->omit_dc_fix = 1;}
			if (strcmp("deemp",  optarg) == 0) {
				demod->deemph = 1;}
			if (strcmp("direct",  optarg) == 0) {
				dongle.direct_sampling = 1;}
			if (strcmp("offset",  optarg) == 0) {
				dongle.offset_tuning = 1;}
			if (strcmp("rtlagc", optarg) == 0 || strcmp("agc", optarg) == 0) {
				rtlagc = 1;}
			if (strcmp("bclo", optarg) == 0 || strcmp("bcL", optarg) == 0 || strcmp("bcl", optarg) == 0) {
				dongle.bccorner = -1; }
			if (strcmp("bcc", optarg) == 0 || strcmp("bcC", optarg) == 0) {
				dongle.bccorner = 0; }
			if (strcmp("bchi", optarg) == 0 || strcmp("bcH", optarg) == 0 || strcmp("bch", optarg) == 0) {
				dongle.bccorner = 1; }
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
				demod->mode_demod = &am_demod;
				demod->dc_block_audio = 1;	/* remove the DC */
			}
			if (strcmp("ook",  optarg) == 0) {
				demod->mode_demod = &am_demod;
				demod->dc_block_audio = 0;
			}
			if (strcmp("usb", optarg) == 0) {
				demod->mode_demod = &usb_demod;}
			if (strcmp("lsb", optarg) == 0) {
				demod->mode_demod = &lsb_demod;}
			if (strcmp("wbfm",  optarg) == 0 || strcmp("wfm",  optarg) == 0) {
				controller.wb_mode = 1;
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
			usage(verbosity);
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

	if (controller.freq_len > 1) {
		demod->terminate_on_squelch = 0;}

	if (optind < argc) {
		output.filename = argv[optind];
	} else {
		output.filename = "-";
	}

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

	if (demod->deemph) {
		double tc = (double)timeConstant * 1e-6;
		demod->deemph_a = (int)round(1.0/((1.0-exp(-1.0/(demod->rate_out * tc)))));
		if (verbosity)
			fprintf(stderr, "using wbfm deemphasis filter with time constant %d us\n", timeConstant );
	}

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
				waveWriteHeader(&waveWrState, srate, f, 16, nChan, output.file);
			}
		}
	}

	pthread_create(&controller.thread, NULL, controller_thread_fn, (void *)(&controller));
	usleep(1000000); /* it looks, that startup of dongle level takes some time at startup! */
	pthread_create(&output.thread, NULL, output_thread_fn, (void *)(&output));
	pthread_create(&dm_thr.thread, NULL, demod_thread_fn, (void *)(&dm_thr));

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
	pthread_join(dm_thr.thread, NULL);
	safe_cond_signal(&output.ready, &output.ready_m);
	pthread_join(output.thread, NULL);
	safe_cond_signal(&controller.hop, &controller.hop_m);
	pthread_join(controller.thread, NULL);

	/* dongle_cleanup(&dongle); */
	demod_thread_cleanup(&dm_thr);
	output_cleanup(&output);
	controller_cleanup(&controller);

	if (cmd.filename) {
		int k;
		/* output scan statistics */
		for (k = 0; k < FREQUENCIES_LIMIT; k++) {
			if (cmd.statNumLevels[k] > 0)
				fprintf(stderr, "%.0f, %.1f, %.2f, %.1f\n", (double)(cmd.statFreq[k]), cmd.statMinLevel[k], cmd.statSumLevels[k] / cmd.statNumLevels[k], cmd.statMaxLevel[k] );
		}
	}

	if (output.file != stdout) {
		if (writeWav) {
			int r;
			waveFinalizeHeader(&waveWrState, output.file);
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

	if (dongle.dev) {
		if (verbosity)
			fprintf(stderr, "closing dongle and exit\n");
		rtlsdr_close(dongle.dev);
	}

	return r >= 0 ? r : -r;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
