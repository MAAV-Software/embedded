/*
 * RcOutput.hpp
 *
 *  Created on: Apr 5, 2016
 *      Author: Sasawat
 */

#ifndef RCOUTPUT_HPP_
#define RCOUTPUT_HPP_


#include <stdint.h>
#include "Pair.hpp"

/**
 * @brief Represents an RcOutput
 * @details Contains the calibration profile for a specific RC Output and
 *          allows conversion of the raw signals measured through the servoIn
 *          module to map to the standard 1ms to 2ms pulse range.
 */
class RcOutput
{
public:
    /**
     * @brief RcOutput Constructor
     * @details Constructs a RcOutput with the given calibration profile for
     *          each channel of the Output.
     * @param channelProfiles   Array of pairs of min and max calibrated values
     *                          for each channel. Order of profiles corresponds
     *                          to the order of channels in the physical RC
     *                          Output. Each Pair in the array has the min
     *                          and the max values as the first and second
     *                          memebers of the struct, respectively.
     * @param size              Number of channels (i.e. length of
     *                          channelProfiles array)
     */
    RcOutput(const Maav::Pair<uint32_t, uint32_t> *channelProfiles,
                 uint8_t size);

    ~RcOutput();

    /**
     * @brief Calculates the calibrated ppm for a given pulse
     * @details Returns the ppm in ticks for the given pulse input.
     *          The ppm will be in the range given in the pair.
     *          Returns -1 on error.
     * @param pulse     pulse will in the standard 1-2ms range.
     * @param channel   Channel index (0-based index)
     */
    uint32_t pulse(float pulse, uint8_t channel);

    /**
     * @brief Calculates the calibrated ppm for a given duty cycle
     * @details Returns the ppm in ticks for the given duty cycle input.
     *          The ppm will be in the range given in the pair.
     *          Returns -1 on error.
     * @param duty      Duty cycle from 0.0f to 1.0f
     * @param channel   Channel index (0-based index)
     */
    uint32_t dutyCycle(float duty, uint8_t channel);

private:
    Maav::Pair<uint32_t, uint32_t> *profiles; /**< array of channel calibration profiles */
    uint8_t numChan; /**< number of channels for this RC Output */
};




#endif /* RCOUTPUT_HPP_ */
