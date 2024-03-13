/*
 * Copyright (c) 2015 by
 Arturo Guadalupi <a.guadalupi@arduino.cc>
 Angelo Scialabba <a.scialabba@arduino.cc>
 Claudio Indellicati <c.indellicati@arduino.cc> <bitron.it@gmail.com>

 * Audio library for Arduino Zero.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef AUDIOZERO_H
#define AUDIOZERO_H

#include <Arduino.h>

// #include "Print.h"

class AudioZeroSource {
public:
    virtual int seek(size_t offset) = 0;
    virtual int read(void *buf, size_t count) = 0;
    virtual int available() = 0;
};

class AudioZeroClass{
public:

	AudioZeroClass() {};
	void begin();
	int prepare(AudioZeroSource &);
	void play() ;
	void end();
	unsigned int getNumSamples();
    unsigned int getSampleRate();

	/* Call a function every n_samples */
	void set_callback(void (*func)(void), uint32_t n_samples);

private:
	void dacConfigure(void);
	void tcConfigure(uint32_t sampleRate);
	bool tcIsSyncing(void);
	void tcStartCounter(void);
	void tcReset(void);
	void tcEnable(void);
	void tcDisable(void);
};

extern AudioZeroClass AudioZero;
#endif
