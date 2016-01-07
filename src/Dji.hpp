#ifndef DJI_HPP_
#define DJI_HPP_

typedef struct _Dji
{
	float roll;		// roll value for the DJI
	float pitch;	// pitch value for the DJI
	float yawRate;	// yaw rate value for the DJI
	float thrust;	// Body-frame thrust (Force_Z) value for the DJI
} Dji;

#endif /* Dji.hpp */
