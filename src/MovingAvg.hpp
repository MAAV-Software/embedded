#ifndef MOVINGAVG_HPP_
#define MOVINGAVG_HPP_

class MovingAvg
{
public:
    MovingAvg(size_t n);
    ~MovingAvg();

    float run(const float input);
    float state() const;
        
private:
    size_t size_;
    float *buf_;
    float state_;
    size_t idx_;
};

#endif /* MOVINGAVG_HPP_ */
