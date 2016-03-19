#ifndef IMURUNNABLE_HPP_
#define IMURUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class ImuRunnable : public Runnable
{
public:
	ImuRunnable(ProgramState *pState);
	void run();
	void AccelCalib();
	void CaptureGyroBias();
	
private:
//	Imu *imu;
	ProgramState *state;
	float ReverseBytes(const uint8_t *raw, const unsigned int i);

};

#endif /* IMURUNNABLE_HPP_ */
