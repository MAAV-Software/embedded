#ifndef MOVINGAVG_HPP_
#define MOVINGAVG_HPP_

#include <stdint.h>

/**
 * @brief Moving Averge Filter
 * @details Implements a simple moving averge filter for a set sample size
 */
class MovingAvg
{
public:
    /**
     * @brief MovingAvg Constructor
     *
     * @details Constructs a Moving Average Filter with the specifed sample size
     * 
     * @param size number of samples to average
     */
    MovingAvg(uint16_t size);

    /**
     * @brief MovingAvg Destructor
     *
     * @details frees memory (allocated on construction using calloc) for filter
     */
    ~MovingAvg();

    /**
     * @brief run
     *
     * @details Runs the filter and returns the response for this iteration.
     *          It will overwrite the oldest sample if the internal buffer of 
     *          "size" samples fills up.
     *
     * @param input new input to use in this iteration of the filter
     */
    float run(const float input);

    /**
     * @brief State
     *
     * @details Returns the most recent filter response/state
     */
    float state() const;
        
private:
    uint16_t size_;   /**< @brief sample size for the filter */
    float *buf_;    /**< @brief buffer of samples average */
    float state_;   /**< @brief current state/response of the filter */
    uint16_t idx_;    /**< @brief index of current write head for buf */
};

#endif /* MOVINGAVG_HPP_ */
