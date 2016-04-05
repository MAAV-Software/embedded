#include <stdint.h>
#include <cstdlib>
#include <cstring>
#include "MaavMath.hpp"
#include "Pair.hpp"
#include "RcController.hpp"

using Maav::Pair;
using MaavMath::map;
using std::memcpy;
using std::malloc;
using std::free;

RcController::RcController(const Pair<uint32_t, uint32_t> *channelProfiles, 
                           uint8_t size) 
    : numChan(size)
{
    profiles = (Pair<uint32_t, uint32_t>*) malloc(size * sizeof(Pair<uint32_t, uint32_t>));
    memcpy(profiles, channelProfiles, size * sizeof(Pair<uint32_t, uint32_t>));
}

RcController::~RcController()
{
    free(profiles);
}

float RcController::pulse(uint32_t raw, uint8_t channel)
{
    if (channel > numChan) return -1.0;
    return map(raw, profiles[channel].first, profiles[channel].second, 1.0f, 2.0f);
}

float RcController::dutyCycle(uint32_t raw, uint8_t channel)
{
    if (channel > numChan) return -1.0;
    return map(raw, profiles[channel].first, profiles[channel].second, 0.0f, 1.0f);
}
