#ifndef IMURUNNABLE_HPP_
#define IMURUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class ImuRunnable : public Runnable
{
public:
	ImuRunnable(ProgramState *pState);
	void run();
	
private:
	ProgramState *state;
    uint32_t imuTime; 
    
    void AccelCalib();
	void CaptureGyroBias();
};

#endif /* IMURUNNABLE_HPP_ */
