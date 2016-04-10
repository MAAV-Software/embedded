/*
 * Switch.hpp
 *
 *  Created on: Apr 9, 2016
 *      Author: Sasawat
 */

#ifndef SWITCH_HPP_
#define SWITCH_HPP_

#include "general.h"
// Struct for lighted 3-position switch states
class ThreeSwitch
{
    uint32_t periph;
    uint32_t portBase;
    uint8_t  pinNum     : 4;
    uint8_t  readState  : 1;
    uint8_t  driveState : 1;
    void read();
    void drive(uint8_t value);
public:
    ThreeSwitch(uint32_t periph, uint32_t base, uint32_t pin);
    void update();
    uint8_t getReadValue() const { return readState; }
};

#endif /* SWITCH_HPP_ */
