/*
 * RcOutput.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: Sasawat
 */

#include "RcOutput.hpp"
#include "MaavMath.hpp"

using MaavMath::map;

RcOutput::RcOutput(const Maav::Pair<uint32_t, uint32_t> *channelProfiles,
        uint8_t size)
        : numChan(size)
{
    profiles = (Pair<uint32_t, uint32_t>*) malloc(size * sizeof(Pair<uint32_t, uint32_t>));
    memcpy(profiles, channelProfiles, size * sizeof(Pair<uint32_t, uint32_t>));
}

RcOutput::~RcOutput()
{
    free(profiles);
}

uint32_t RcOutput::pulse(float pulse, uint8_t channel)
{
    if (channel > numChan) return -1.0;
    if(pulse < 1.0f || pulse > 2.0f) return -1.0;
    return map(pulse, 1.0f, 2.0f, profiles[channel].first, profiles[channel].second);
}

uint32_t RcOutput::dutyCycle(float duty, uint8_t channel)
{
    if (channel > numChan) return -1.0;
    if(duty < 0.0f || duty > 1.0f) return -1.0;
    return map(raw, 0.0f, 1.0f, profiles[channel].first, profiles[channel].second);
}
