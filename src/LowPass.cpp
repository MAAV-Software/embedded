#include "LowPass.hpp"

LowPass::LowPass() : alpha(0), state(0) {}

LowPass::LowPass(const float a, const float s) : alpha(a), state(s) {}

LowPass::LowPass(const float a) : alpha(a), state(0) {}

void LowPass::run(const float input)
{
	state = (alpha * input) + ((1.0f - alpha) * state);
}

float LowPass::getState() const 
{ 
	return state; 
}
