/*
 * Copyright (C) 2019 by Hayati Ayguen <h_ayguen@web.de>
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

#include "wavewrite.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

#ifndef _WIN32
#include <unistd.h>
#include <sys/time.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include <process.h>
#define _USE_MATH_DEFINES
#endif

#include <math.h>


#ifdef _WIN32
int gettimeofday(struct timeval *tv, void* ignored)
{
	FILETIME ft;
	unsigned __int64 tmp = 0;
	if (NULL != tv) {
		GetSystemTimeAsFileTime(&ft);
		tmp |= ft.dwHighDateTime;
		tmp <<= 32;
		tmp |= ft.dwLowDateTime;
		tmp /= 10;
#ifdef _MSC_VER
		tmp -= 11644473600000000Ui64;
#else
		tmp -= 11644473600000000ULL;
#endif
		tv->tv_sec = (long)(tmp / 1000000UL);
		tv->tv_usec = (long)(tmp % 1000000UL);
	}
	return 0;
}
#endif

static void waveSetCurrTime(Wind_SystemTime *p)
{
	struct timeval tv;
	struct tm t;

	gettimeofday(&tv, NULL);
	p->wMilliseconds = tv.tv_usec / 1000;

#ifdef _WIN32
	t = *gmtime(&tv.tv_sec);
#else
	gmtime_r(&tv.tv_sec, &t);
#endif

	p->wYear = t.tm_year + 1900;	/* 1601 through 30827 */
	p->wMonth = t.tm_mon + 1;		/* 1..12 */
	p->wDayOfWeek = t.tm_wday;		/* 0 .. 6: 0 == Sunday, .., 6 == Saturday */
	p->wDay = t.tm_mday;			/* 1 .. 31 */
	p->wHour = t.tm_hour;			/* 0 .. 23 */
	p->wMinute = t.tm_min;			/* 0 .. 59 */
	p->wSecond = t.tm_sec;			/* 0 .. 59 */
}

static void waveSetStartTimeInt(time_t tim, double fraction, Wind_SystemTime *p)
{
	struct tm t = *gmtime( &tim );
	p->wYear = t.tm_year + 1900;	/* 1601 through 30827 */
	p->wMonth = t.tm_mon + 1;		/* 1..12 */
	p->wDayOfWeek = t.tm_wday;		/* 0 .. 6: 0 == Sunday, .., 6 == Saturday */
	p->wDay = t.tm_mday;			/* 1 .. 31 */
	p->wHour = t.tm_hour;			/* 0 .. 23 */
	p->wMinute = t.tm_min;			/* 0 .. 59 */
	p->wSecond = t.tm_sec;			/* 0 .. 59 */
	p->wMilliseconds = (int)( fraction * 1000.0 );
	if (p->wMilliseconds >= 1000)
		p->wMilliseconds = 999;
}

void waveSetStartTime(WaveWriteState *state, time_t tim, double fraction)
{
	waveSetStartTimeInt(tim, fraction, &state->waveHdr.a.StartTime );
	state->waveHdr.a.StopTime = state->waveHdr.a.StartTime;		/* to fix */
}


void wavePrepareHeader(WaveWriteState *state, unsigned samplerate, unsigned freq, int bitsPerSample, int numChannels)
{
	int	bytesPerSample = bitsPerSample / 8;
	int bytesPerFrame = bytesPerSample * numChannels;

	memcpy( state->waveHdr.r.hdr.ID, "RIFF", 4 );
	state->waveHdr.r.hdr.size = sizeof(waveFileHeader) - 8;		/* to fix */
	memcpy( state->waveHdr.r.waveID, "WAVE", 4 );

	memcpy( state->waveHdr.f.hdr.ID, "fmt ", 4 );
	state->waveHdr.f.hdr.size = 16;
	state->waveHdr.f.wFormatTag = 1;					/* PCM */
	state->waveHdr.f.nChannels = numChannels;		/* I and Q channels */
	state->waveHdr.f.nSamplesPerSec = samplerate;
	state->waveHdr.f.nAvgBytesPerSec = samplerate * bytesPerFrame;
	state->waveHdr.f.nBlockAlign = state->waveHdr.f.nChannels;
	state->waveHdr.f.nBitsPerSample = bitsPerSample;

	memcpy( state->waveHdr.a.hdr.ID, "auxi", 4 );
	state->waveHdr.a.hdr.size = 2 * sizeof(Wind_SystemTime) + 9 * sizeof(int32_t);  /* = 2 * 16 + 9 * 4 = 68 */
	waveSetCurrTime( &state->waveHdr.a.StartTime );
	state->waveHdr.a.StopTime = state->waveHdr.a.StartTime;		/* to fix */
	state->waveHdr.a.centerFreq = freq;
	state->waveHdr.a.ADsamplerate = samplerate;
	state->waveHdr.a.IFFrequency = 0;
	state->waveHdr.a.Bandwidth = 0;
	state->waveHdr.a.IQOffset = 0;
	state->waveHdr.a.Unused2 = 0;
	state->waveHdr.a.Unused3 = 0;
	state->waveHdr.a.Unused4 = 0;
	state->waveHdr.a.Unused5 = 0;

	memcpy( state->waveHdr.d.hdr.ID, "data", 4 );
	state->waveHdr.d.hdr.size = 0;		/* to fix later */
	state->waveDataSize = 0;
}


void initWaveWriteState(WaveWriteState * state)
{
	state->waveHdrStarted = 0;
	state->waveDataSize = 0;
}

void waveWriteHeader(WaveWriteState *state, unsigned samplerate, unsigned freq, int bitsPerSample, int numChannels, FILE * f)
{
	if (f != stdout) {
		assert( !state->waveHdrStarted );
		wavePrepareHeader(state, samplerate, freq, bitsPerSample, numChannels);
		fwrite(&state->waveHdr, sizeof(waveFileHeader), 1, f);
		state->waveHdrStarted = 1;
	}
}

int  waveWriteSamples(WaveWriteState *state, FILE* f,  const void * vpData, size_t numSamples, int needCleanData)
{
	size_t nw;
	switch (state->waveHdr.f.nBitsPerSample)
	{
	case 0:
	default:
		return 1;
	case 8:
		/* no endian conversion needed for single bytes */
		nw = fwrite(vpData, sizeof(uint8_t), numSamples, f);
		state->waveDataSize += sizeof(uint8_t) * numSamples;
		return (nw == numSamples) ? 0 : 1;
	case 16:
		/* TODO: endian conversion needed */
		nw = fwrite(vpData, sizeof(int16_t), numSamples, f);
		state->waveDataSize += sizeof(int16_t) * numSamples;
		if ( needCleanData )
		{
			/* TODO: convert back endianness */
		}
		return (nw == numSamples) ? 0 : 1;
	}
}

int  waveWriteFrames(WaveWriteState *state, FILE* f,  const void * vpData, size_t numFrames, int needCleanData)
{
	size_t nw;
	switch (state->waveHdr.f.nBitsPerSample)
	{
	case 0:
	default:
		return 1;
	case 8:
		/* no endian conversion needed for single bytes */
		nw = fwrite(vpData, state->waveHdr.f.nChannels * sizeof(uint8_t), numFrames, f);
		state->waveDataSize += state->waveHdr.f.nChannels * sizeof(uint8_t) * numFrames;
		return (nw == numFrames) ? 0 : 1;
	case 16:
		/* TODO: endian conversion needed */
		nw = fwrite(vpData, state->waveHdr.f.nChannels * sizeof(int16_t), numFrames, f);
		state->waveDataSize += state->waveHdr.f.nChannels * sizeof(int16_t) * numFrames;
		if ( needCleanData )
		{
			/* TODO: convert back endianness */
		}
		return (nw == numFrames) ? 0 : 1;
	}
}


int  waveFinalizeHeader(WaveWriteState *state, FILE * f)
{
	if (f != stdout) {
		assert( state->waveHdrStarted );
		waveSetCurrTime( &state->waveHdr.a.StopTime );
		state->waveHdr.d.hdr.size = state->waveDataSize;
		state->waveHdr.r.hdr.size += state->waveDataSize;
		/* fprintf(stderr, "waveFinalizeHeader(): datasize = %d\n", waveHdr.dataSize); */
		state->waveHdrStarted = 0;
		if ( fseek(f, 0, SEEK_SET) )
			return 1;
		if ( 1 != fwrite(&state->waveHdr, sizeof(waveFileHeader), 1, f) )
			return 1;
		/* fprintf(stderr, "waveFinalizeHeader(): success writing header\n"); */
		return 0;
	}
	return 1;
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
