/*
 * RcProfiles.hpp
 *
 *  Created on: Apr 5, 2016
 *      Author: Sasawat
 */

#ifndef RCPROFILES_HPP_
#define RCPROFILES_HPP_

#include "RcController.hpp"
#include "Pair.hpp"

namespace Maav
{

extern const Pair<uint32_t, uint32_t> futaba[];
extern const uint8_t futabaNumch;

extern const Pair<uint32_t, uint32_t> hitec[];
extern const uint8_t hitecNumch;

extern const Pair<uint32_t, uint32_t> dji[];
extern const uint8_t djiNumch;

}

#endif /* RCPROFILES_HPP_ */
