
/**
 * @brief The function to call when something goes horribly wrong
 * @author unknown
 *
 * @details **update details**
 */
#ifndef APESHIT_HPP
#define APESHIT_HPP

#include <stdint.h>

#include "LED.h"

#define APESHIT_CYCLE_TIME_DEFAULT 300000

void goApeshit();

void goApeshit(uint32_t);

#endif //APESHIT_HPP
