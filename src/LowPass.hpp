#ifndef LOWPASS_HPP_
#define LOWPASS_HPP_

class LowPass
{
public:
	LowPass();
	LowPass(const float alpha, const float state);
	LowPass(const float alpha);
	
	void run(const float input);
	float getState() const;
	
private:
	float alpha;
	float state;
};

#endif /* LowPass.hpp */
