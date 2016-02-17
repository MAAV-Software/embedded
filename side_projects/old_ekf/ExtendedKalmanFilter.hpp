#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

/* temporary hack to let us use our own implementation of a subset of arm_math.h */
#ifdef LINUX
#include "cmeigen.hpp"
#else
#include "arm_math.h"
#endif

#include <stdint.h>

#define NUM_UPDATE_SLOTS 2

/**
 * Implementation of Discrete-Continous Extended Kalman Filter from
 * Beard's Quadrotor Notes
 *
 * Vector/Matrix size reference:
 * State vector, x, is (n, 1)
 * Control Input, u, is (u, 1)
 * Sensor Input, y, is (s, 1)
 * Error Covariance Matrix, P, is (n, n)
 * System Jacobian, A, is (n, n)
 * System Covariance, Q, is (n, n)
 * Sensor Jacobian, C, is (s, n)
 * Sensor Covariance, R, is (s, s)
 */
class ExtendedKalmanFilter 
{
public:
	/**
	 * @brief Initializes a Kalman filter that has stateSize number of states to keep track of
	 * @param stateSize number of states filter will estimate
	 * @param initialState data for initialState (ctor will copy this data)
	 * @param initialErrorCov data for initial error covariance (ctor will copy this data)
	 */
	ExtendedKalmanFilter(const uint16_t stateSize, const float* initialState,
						 const float* initialErrorCov);

	~ExtendedKalmanFilter();

	/**
	 * @brief sets the function pointers and system covariance
	 * matrix for a predict
	 * @param controlInputSize size of control input
	 * @param deltaState function that will take in (in order)
	 * current state, control input, and populate a delta State matrix
	 * @param getJacobian function that will take in (in order)
	 * current state, control input, and populate the system jacobian
	 * @param covariance system covariance matrix (copied internally)
	 */
	void setPredictFunc(const uint16_t controlInputSize,
						void (*deltaState)(const arm_matrix_instance_f32*,
										   const arm_matrix_instance_f32*, 
										   const float,
										   const float,
										   const float,
										   const float,
										   const float,
										   const float,
										   const float,
										   arm_matrix_instance_f32*),
						void (*getJacobian)(const arm_matrix_instance_f32*,
											const arm_matrix_instance_f32*, 
											arm_matrix_instance_f32*),
						const arm_matrix_instance_f32* covariance);

	/**
	 * @brief sets the function poitners and system covariance matrix
	 * for an update. Multiple update types can be set
	 * @param id an identifier for the update that will be set
	 * can be between 0 and NUM_UPDATE_SLOTS-1
	 * @param sensorSize number of elements of sensor input
	 * @param predictSensor function that will take in (in order)
	 * current state, and populate the predicted sensor output
	 * @param getJacobian function that will take in (in order)
	 * current state, and populate the sensor jacobian
	 * @param covariance sensor covariance matrix (copied internally)
	 */
	void setUpdateFunc(const uint16_t id, const uint16_t sensorSize,
					   void (*predictSensor)(const arm_matrix_instance_f32*,
											 arm_matrix_instance_f32*),
					   void (*getJacobian)(const arm_matrix_instance_f32*,
										   arm_matrix_instance_f32*),
					   const arm_matrix_instance_f32* covariance);

	/**
	 * @brief predicts the state after a delta time
	 * @param deltaTime time past since last predict or update
	 * @param controlInput control input going into system
	 */
	void predict(const float deltaTime,
				 const float mass,
			  	 const float sinR,
				 const float cosR,
				 const float sinP,
				 const float cosP,
				 const float sinY,
				 const float cosY,
				 const arm_matrix_instance_f32* controlInput);

	/**
	 * @brief updates the state with a sensor measurement
	 * @param deltaTime time past since last predict or update
	 * @param id identifier for which update functions to use
	 * @param sensorMeasurement sensor measurement going into update function
	 */
	void update(const float deltaTime, const uint16_t id,
				const arm_matrix_instance_f32* sensorMeasurement);
	
	// Returns the state vector of the filter
	arm_matrix_instance_f32 getState() const { return _state; }

private:
	// General State Variables
	arm_matrix_instance_f32 _state;
	arm_matrix_instance_f32 _P;
	uint16_t _stateSize; // for easy access
	uint16_t _controlInputSize; // TODO: do we want to save this?

	// Prediction Variables
	void (*_deltaState)(const arm_matrix_instance_f32*,
						const arm_matrix_instance_f32*, 
						const float,
						const float,
						const float,
						const float,
						const float,
						const float,
						const float,
						arm_matrix_instance_f32*);
	void (*_getJacobian)(const arm_matrix_instance_f32*,
						 const arm_matrix_instance_f32*,
						 arm_matrix_instance_f32*);
	arm_matrix_instance_f32 _systemJacobian; // n by n matrix
	arm_matrix_instance_f32 _systemCovariance;

	// Update Variables
	struct updateStruct 
	{
		uint16_t sensorSize;
		void (*predictSensor)(const arm_matrix_instance_f32*,
							  arm_matrix_instance_f32*);
		void (*getJacobian)(const arm_matrix_instance_f32*,
							arm_matrix_instance_f32*);
		arm_matrix_instance_f32 jacobian; // s by n matrix
		arm_matrix_instance_f32 covMatrix; // s by s matrix

		// temporary variables
		arm_matrix_instance_f32 ns_0; // n by s matrix
		arm_matrix_instance_f32 ns_1; // n by s matrix
		arm_matrix_instance_f32 ss_0; // s by s matrix
		arm_matrix_instance_f32 ss_1; // s by s matrix
		arm_matrix_instance_f32 s1_0; // s by 1 matrix
	};
	updateStruct _updateArr[NUM_UPDATE_SLOTS];

	// temporary variables used during predict and/or update
	arm_matrix_instance_f32 _n1_0; // n by 1 matrix
	arm_matrix_instance_f32 _nn_0; // n by n matrix
	arm_matrix_instance_f32 _nn_1; // n by n matrix

	arm_matrix_instance_f32 _nn_identity; // n by n identity matrix
};

#endif /* EXTENDED_KALMAN_FILTER_HPP */
