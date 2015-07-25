#ifndef CTRLLOGS_HPP_
#define CTRLLOGS_HPP_

#include <stdint.h>

struct PidLog
{
	float setpt;
	float kp;
	float ki;
	float kd;
	uint8_t flags;

	PidLog() : setpt(0), kp(0), ki(0), kd(0), flags(0) {}
};

struct VehicleLog
{
	float xUval;
	float yUval;
	float zUval;
	float xFilt;
	float yFilt;
	float zFilt;
	float xdotFilt;
	float ydotFilt;
	float zdotFilt;
	float rollFilt;
	float pitchFilt;
	float yawFilt;

	VehicleLog() : xUval(0), yUval(0), zUval(0), xFilt(0), yFilt(0), zFilt(0),
				   xdotFilt(0), ydotFilt(0), zdotFilt(0), rollFilt(0),
				   pitchFilt(0), yawFilt(0) {}
};

/* Note that these are minimalist log objects.
   
   The setpt of a rate PID is the output of a value PID. 
   
   The uval output of a Dof is the output of a rate PID.

   The rate output of the Yaw value PID is the DJI rate.
   
   Dof log info is captured within Pid (setpts) and Vehicle log (uvals/rates
   through Dof::getUval() and Dof::getRate()).
   
   The DJI values (including Yaw Value PID/Y Dof output) can be obtained using 
   Vehicle::getDjiVals().

   Thus, for the purpose of accessing controller information that is not 
   available from the public interface of the Vehicle, Dof, and Pid classes, 
   these two structs above are sufficient.
*/

#endif /* CtrlLogs.hpp */
