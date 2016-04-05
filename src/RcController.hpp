#ifndef RCCONTROLLER_HPP_
#define RCCONTROLLER_HPP_

#include <stdint.h>
#include "Pair.hpp"

/**
 * @brief Represents an RcController
 * @details Contains the calibration profile for a specific RC Controller and 
 *          allows conversion of the raw signals measured through the servoIn
 *          module to map to the standard 1ms to 2ms pulse range. 
 */
class RcController
{
public:
    /**
     * @brief RcController Constructor
     * @details Constructs a RcController with the given calibration profile for
     *          each channel of the controller.
     * @param channelProfiles   Array of pairs of min and max calibrated values
     *                          for each channel. Order of profiles corresponds
     *                          to the order of channels in the physical RC 
     *                          controller. Each Pair in the array has the min
     *                          and the max values as the first and second 
     *                          memebers of the struct, respectively.
     * @param size              Number of channels (i.e. length of
     *                          channelProfiles array)
     */
    RcController(const Maav::Pair<uint32_t, uint32_t> *channelProfiles, 
                 uint8_t size);
   
    ~RcController();

    /**
     * @brief Calculates the calibrated PWM pulse output
     * @details Returns the pulse in ms for the given raw input and channel.
     *          The pulse will be in the standard 1-2ms range. Returns -1 on 
     *          error.
     * @param raw       Raw signal input (from servoIn)
     * @param channel   Channel index (0-based index)
     */
    float pulse(uint32_t raw, uint8_t channel);

    /**
     * @brief Calculates the calibrated PWM duty cycle ouput
     * @details Returns the duty cycle for the given raw input and channel.
     *          Returns -1 on error.
     * @param raw       Raw signal input (from servoIn)
     * @param channel   Channel index (0-based index)
     */
    float dutyCycle(uint32_t raw, uint8_t channel);
    
private:
    Maav::Pair<uint32_t, uint32_t> *profiles; /**< array of channel calibration profiles */
    uint8_t numChan; /**< number of channels for this RC controller */
};

#endif /* RCCONTROLLER_HPP_ */
