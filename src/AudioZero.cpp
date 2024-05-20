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

#include "AudioZero.h"
#include "wav.h"

#define NEUTRAL_SOUND 512 // = (2^10 / 2)
#define NUMBER_OF_SAMPLES 512 // Number of samples to read in block

/* Global variables */
volatile uint32_t __samples_callback;
volatile uint32_t __samples_next_callback;
volatile bool __callback_due;
void (*__callback)(void);

volatile int16_t __buffer1[NUMBER_OF_SAMPLES];
volatile int16_t __buffer2[NUMBER_OF_SAMPLES];

volatile int16_t *__front_buffer = __buffer1;
volatile int16_t *__back_buffer = __buffer2;

volatile size_t __buffer1_len = 0;
volatile size_t __buffer2_len = 0;

volatile size_t *__front_buffer_len = &__buffer1_len;
volatile size_t *__back_buffer_len = &__buffer2_len;

volatile bool __buffer_swap_required = false;
volatile size_t __front_buffer_pos = 0;

riff_header __RIFFHeader; // RIFF header
wav_header __WavHeader; // Wav header
data_header __DataHeader; // Wav header
unsigned int __TotalSamples;

AudioZeroSource *__toPlay;
bool __TcConfigured = false;

void AudioZeroClass::begin() {
    __TotalSamples = 0;

    /*Set callback to none */
    __samples_callback = 0;
    __samples_next_callback = 0;
    __callback = NULL;

    /*Modules configuration */
    dacConfigure();
}

void AudioZeroClass::end() {
    if (__TcConfigured) {
        tcDisable();
        tcReset();
    }
    analogWrite(A0, NEUTRAL_SOUND);
}


unsigned int AudioZeroClass::getNumSamples() {
    return __TotalSamples;
}

unsigned int AudioZeroClass::getSampleRate() {
    return __WavHeader.sample_rate;
}

int AudioZeroClass::prepare(AudioZeroSource& toPlay){
    __toPlay = &toPlay;

    // Read header + first buffer
    int available = __toPlay->available();

    if (available > 0) {
        // READ RIFF Header
        __toPlay->read(&__RIFFHeader, RIFF_HEADER_SIZE);

        // Read WAV Header
        __toPlay->read(&__WavHeader, WAV_HEADER_SIZE);

        tcConfigure(__WavHeader.sample_rate);
        __toPlay->seek(RIFF_HEADER_SIZE + __WavHeader.data_header_length + 8);

        __toPlay->read(&__DataHeader, DATA_HEADER_SIZE);
        __TotalSamples = __DataHeader.data_length / sizeof(uint16_t);

        __buffer_swap_required = true;

        tcStartCounter(); //start the interruptions
        return 0;
    } else {
        return -1; // File is empty
    }
}

void AudioZeroClass::play() {

    while (!__toPlay->abort() && __toPlay->available()) {

        // Read our file into our back buffer
        int bytes_read = __toPlay->read((void*) __back_buffer, NUMBER_OF_SAMPLES * sizeof(uint16_t));
        *__back_buffer_len = bytes_read / sizeof(uint16_t);

        if (*__back_buffer_len == 0)
            break; // EOF

        // Wait for the currently playing buffer
        while (!__buffer_swap_required)
        {
            if (__callback_due) {
                __callback_due = false;
                if (__callback)
                    __callback();
            }
        }

        // Disable interrupts
        noInterrupts();

        // Swap the buffers
        volatile int16_t *temp_buffer = __front_buffer;
        __front_buffer = __back_buffer;
        __back_buffer = temp_buffer;

        volatile size_t *temp_buffer_len = __front_buffer_len;
        __front_buffer_len = __back_buffer_len;
        __back_buffer_len = temp_buffer_len;

        // Reset the front buffer position
        __front_buffer_pos = 0;

        // Clear the buffer swap required flag
        __buffer_swap_required = false;

        // Re-enable interrupts
        interrupts();

    }
    
    // Wait for the final buffer completion
    while (!__buffer_swap_required)
    {
        if (__callback_due) {
            __callback_due = false;
            if (__callback)
                __callback();
        }
    }
}

/**
 * Configures the DAC in event triggered mode.
 *
 * Configures the DAC to use the module's default configuration, with output
 * channel mode configured for event triggered conversions.
 */
void AudioZeroClass::dacConfigure(void) {
    analogWriteResolution(10);
    analogWrite(A0, NEUTRAL_SOUND);
}

/**
 * Configures the TC to generate output events at the sample frequency.
 *
 * Configures the TC in Frequency Generation mode, with an event output once
 * each time the audio sample frequency period expires.
 */
 void AudioZeroClass::tcConfigure(uint32_t sampleRate) {
    __TcConfigured = true;
    // Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
    while (GCLK->STATUS.bit.SYNCBUSY);

    tcReset();

    // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

    // Set TC5 mode as match frequency
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

    TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
    while (tcIsSyncing());

    // Configure interrupt request
    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);

    // Enable the TC5 interrupt request
    TC5->COUNT16.INTENSET.bit.MC0 = 1;
    while (tcIsSyncing());
}

bool AudioZeroClass::tcIsSyncing() {
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void AudioZeroClass::tcStartCounter(){
    // Enable TC
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (tcIsSyncing());
}

void AudioZeroClass::tcReset() {
    // Reset TCx
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (tcIsSyncing());
    while (TC5->COUNT16.CTRLA.bit.SWRST);
}

void AudioZeroClass::tcDisable() {
    // Disable TC5
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (tcIsSyncing());
}

void AudioZeroClass::set_callback(void (*func)(void), uint32_t n_samples) {
    __callback = func;
    __samples_next_callback = n_samples;
    __samples_callback = n_samples;
}

AudioZeroClass AudioZero;

#ifdef __cplusplus
extern "C" {
#endif

void Audio_Handler (void) {

    // Clear our interrupt flag
    TC5->COUNT16.INTFLAG.bit.MC0 = 1;

    // Process the front buffer
    if (__buffer_swap_required)
        return;

    uint16_t data = (uint16_t) __front_buffer[__front_buffer_pos++];
    data >>= 6; // Convert from 16 bit to 10 bit
    data += 512; // Center the data
    analogWrite(A0, data);

    if (__front_buffer_pos >= *__front_buffer_len)
        __buffer_swap_required = true;
    
    if (__samples_callback != 0) {
        __samples_next_callback--;
        if (__samples_next_callback == 0 ) {
            __callback_due = true;
            __samples_next_callback = __samples_callback;
        }
    }
}

void TC5_Handler (void) __attribute__ ((alias("Audio_Handler")));

#ifdef __cplusplus
}
#endif
