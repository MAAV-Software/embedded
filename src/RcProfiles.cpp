/*
 * RcProfiles.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: Sasawat
 */

#include "RcProfiles.hpp"
#include "Pair.hpp"

namespace Maav
{

//Note that for the Futaba these values were calibrated with
//Thrust and Roll manually flipped to match the HiTec directions
const Pair<uint32_t, uint32_t> futaba[6] =
{
        Pair<uint32_t, uint32_t>(80000, 161000), //Channel 1
        Pair<uint32_t, uint32_t>(83500, 161000), //Channel 2
        Pair<uint32_t, uint32_t>(83500, 161000), //Channel 3
        Pair<uint32_t, uint32_t>(83500, 161000), //Channel 4
        Pair<uint32_t, uint32_t>(83500, 161000), //Channel 5
        Pair<uint32_t, uint32_t>(83500, 161000) //Channel 6
};

const uint8_t futabaNumch = 6;

const Pair<uint32_t, uint32_t> hitec[5] =
{
        Pair<uint32_t, uint32_t>(86000, 155000), //Channel 1
        Pair<uint32_t, uint32_t>(85000, 125000), //Channel 2
        Pair<uint32_t, uint32_t>(70000, 155000), //Channel 3
        Pair<uint32_t, uint32_t>(85000, 154000), //Channel 4
        Pair<uint32_t, uint32_t>(65000, 160000) //Channel 5
};

const uint8_t hitecNumch = 5;

const Pair<uint32_t, uint32_t> dji[4] =
{
		Pair<uint32_t, uint32_t>(81000, 160000), //Channel 1
        Pair<uint32_t, uint32_t>(80000, 130000), //Channel 2
        Pair<uint32_t, uint32_t>(65000, 160000), //Channel 3
        Pair<uint32_t, uint32_t>(80000, 159000) //Channel 4
};

const uint8_t djiNumch = 4;

}
