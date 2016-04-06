/*
 * RcProfiles.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: Sasawat
 */

#include "RcProfiles.hpp"
#include "Pair.hpp"

using namespace Maav;

//Note that for the Futaba these values were calibrated with
//Thrust and Roll manually flipped to match the HiTec directions
const Maav::Pair<uint32_t, uint32_t> Maav::futaba[6] =
{
        {88500, 156000}, //Channel 1
        {88500, 156000}, //Channel 2
        {88500, 156000}, //Channel 3
        {88500, 156000}, //Channel 4
        {88500, 156000}, //Channel 5
        {88500, 156000} //Channel 6
};

const uint8_t Maav::futabaNumch = 6;

const Maav::Pair<uint32_t, uint32_t> Maav::hitec[5] =
{
        {86000, 155000}, //Channel 1
        {85000, 125000}, //Channel 2
        {70000, 155000}, //Channel 3
        {85000, 154000}, //Channel 4
        {65000, 160000} //Channel 5
};

const uint8_t Maav::hitecNumch = 5;

const Maav::Pair<uint32_t, uint32_t> Maav::dji[4] =
{
        {86000, 155000}, //Channel 1
        {85000, 125000}, //Channel 2
        {70000, 155000}, //Channel 3
        {85000, 154000} //Channel 4
};
const uint8_t Maav::djiNumch = 4;
