#include <cstdlib>
#include "MovingAvg.hpp"

using std::calloc;

MovingAvg::MovingAvg(size_t size)
    : size_(size), buf_((float*)calloc(size, sizeof(float))), 
      state_(0), idx_(0) {}

MovingAvg::~MovingAvg()
{
    free(buf_);
}

float MovingAvg::run(const float input)
{
    buf_[idx_++] = input;
    if (idx_ >= size_) idx_ = 0;
    
    float sum = 0.0;
    for (size_t i = 0; i < size_; ++i) sum += buf_[i];
  
    state_ = sum / (float)size_;
        
    return state_;
}

float MovingAvg::state() const
{
    return state_;
}
