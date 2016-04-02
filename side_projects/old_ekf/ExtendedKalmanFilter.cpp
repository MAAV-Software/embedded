#include "kalman/ExtendedKalmanFilter.hpp"
#include <cstdlib>
#include <cstring>
#include <cassert>

#ifdef LINUX
#include <iostream>
#endif

using namespace std;

// TODO: check in arm_mat_mult_f32 if the destination can
// be the same as a source (and inverse as well)

// easy func for initializing a matrix with uninitialized data
inline static void MAT_INIT(arm_matrix_instance_f32* mat, size_t rows, size_t cols) 
{
	//arm_mat_init_f32(mat, (uint16_t)rows, (uint16_t)cols, (float*)malloc(sizeof(float) * rows * cols));
	arm_mat_init_f32(mat, (uint16_t)rows, (uint16_t)cols, (float*)calloc((rows * cols), sizeof(float)));
}

ExtendedKalmanFilter::ExtendedKalmanFilter(const uint16_t stateSize, 
										   const float* initialState, 
										   const float* initialErrorCov) 
	: _stateSize(stateSize), _controlInputSize(0), _deltaState(NULL), 
	  _getJacobian(NULL) 
{
	// initializing state matrices
	size_t stateMatSize = stateSize * stateSize * sizeof(float);
	MAT_INIT(&_state, stateSize, 1); // init X
	memcpy(_state.pData, initialState, stateSize * sizeof(float));
	MAT_INIT(&_P, stateSize, stateSize); // init P
	memcpy(_P.pData, initialErrorCov, stateMatSize);

	// initializing predict related matrices
	MAT_INIT(&_systemJacobian, stateSize, stateSize); // allocate A
	MAT_INIT(&_systemCovariance, stateSize, stateSize); // allocate Q

	// initializing temporary matrices
	MAT_INIT(&_n1_0, stateSize, 1);
	//MAT_INIT(&_nn_0, stateSize, 1); // this needed to be 6x6!!!
	MAT_INIT(&_nn_0, stateSize, stateSize);
	//MAT_INIT(&_nn_1, stateSize, 1); // this also needed to be 6x6
	MAT_INIT(&_nn_1, stateSize, stateSize);

	// initializing identity matrix
	MAT_INIT(&_nn_identity, stateSize, stateSize);
	for (uint16_t i = 0; i < (stateSize * stateSize); ++i) 
	{
		if ((i % (stateSize + 1)) == 0) 
		{
			_nn_identity.pData[i] = 1;
		} 
		else 
		{
			_nn_identity.pData[i] = 0;
		}
	}

	for (uint16_t i = 0; i < NUM_UPDATE_SLOTS; ++i) 
	{
		_updateArr[i].sensorSize = 0;
		_updateArr[i].predictSensor = NULL;
		_updateArr[i].getJacobian = NULL;
		_updateArr[i].jacobian.pData = NULL;
		_updateArr[i].covMatrix.pData = NULL;
		_updateArr[i].ns_0.pData = NULL;
		_updateArr[i].ns_1.pData = NULL;
		_updateArr[i].ss_0.pData = NULL;
		_updateArr[i].ss_1.pData = NULL;
		_updateArr[i].s1_0.pData = NULL;
	}
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() 
{
	free(_state.pData);
	free(_P.pData);
	free(_systemJacobian.pData);
	free(_systemCovariance.pData);
	free(_n1_0.pData);
	free(_nn_0.pData);
	free(_nn_1.pData);
	free(_nn_identity.pData);

	for (uint16_t i = 0; i < NUM_UPDATE_SLOTS; ++i) 
	{
		if (_updateArr[i].sensorSize != 0) 
		{
			if (_updateArr[i].jacobian.pData  != NULL) free(_updateArr[i].jacobian.pData);
			if (_updateArr[i].covMatrix.pData != NULL) free(_updateArr[i].covMatrix.pData);
			if (_updateArr[i].ns_0.pData      != NULL) free(_updateArr[i].ns_0.pData); // invalid next size (fast)
			if (_updateArr[i].ns_1.pData	  != NULL) free(_updateArr[i].ns_1.pData); // double free or corruption (out)
			if (_updateArr[i].ss_0.pData	  != NULL) free(_updateArr[i].ss_0.pData);
			if (_updateArr[i].ss_1.pData      != NULL) free(_updateArr[i].ss_1.pData);
			if (_updateArr[i].s1_0.pData      != NULL) free(_updateArr[i].s1_0.pData);
		}
	}	
}

void ExtendedKalmanFilter::setPredictFunc(const uint16_t controlInputSize,
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
										    const arm_matrix_instance_f32* covariance)
{
	_deltaState = deltaState;
	_getJacobian = getJacobian;

	assert(covariance->numRows == _stateSize);
	assert(covariance->numCols == _stateSize);

	size_t covMatSize = _stateSize * _stateSize * sizeof(float);
	memcpy(_systemCovariance.pData, covariance->pData, covMatSize); // init Q with given covariance

	_controlInputSize = controlInputSize;
}

void ExtendedKalmanFilter::setUpdateFunc(const uint16_t id, 
										 const uint16_t sensorSize,
										 void (*predictSensor)(const arm_matrix_instance_f32*, arm_matrix_instance_f32*),
										 void (*getJacobian)(const arm_matrix_instance_f32*, arm_matrix_instance_f32*),
										 const arm_matrix_instance_f32* covariance) 
{
	assert(sensorSize != 0);
	assert(covariance->numRows == sensorSize);
	assert(covariance->numCols == sensorSize);

	// if sensor size is different or its the first time access
	// delete old memory (if it exists) and allocate correct size
	if (sensorSize != _updateArr[id].sensorSize) 
	{
		if (_updateArr[id].sensorSize != 0) 
		{
			// there is old data here of a different size
			if (_updateArr[id].jacobian.pData != NULL)
			{
				free(_updateArr[id].jacobian.pData); 	
				_updateArr[id].jacobian.pData = NULL;
			}
			if (_updateArr[id].covMatrix.pData != NULL)
			{
				free(_updateArr[id].covMatrix.pData);	
				_updateArr[id].covMatrix.pData = NULL;
			}
			if (_updateArr[id].ns_0.pData != NULL)
			{
				free(_updateArr[id].ns_0.pData); 
				_updateArr[id].ns_0.pData = NULL;
			}
			if (_updateArr[id].ns_1.pData != NULL)
			{
				free(_updateArr[id].ns_1.pData);
				_updateArr[id].ns_1.pData = NULL;
			}
			if (_updateArr[id].ss_0.pData != NULL)
			{
				free(_updateArr[id].ss_0.pData);
				_updateArr[id].ss_0.pData = NULL;
			}	
			if (_updateArr[id].ss_1.pData != NULL)
			{
				free(_updateArr[id].ss_1.pData);
				_updateArr[id].ss_1.pData = NULL;
			}
			if (_updateArr[id].s1_0.pData != NULL)
			{
				free(_updateArr[id].s1_0.pData);
				_updateArr[id].s1_0.pData = NULL;
			}
		}
		_updateArr[id].sensorSize = sensorSize;

		MAT_INIT(&_updateArr[id].jacobian, sensorSize, _stateSize); // 3x6
		MAT_INIT(&_updateArr[id].covMatrix, sensorSize, sensorSize); // 3x3
		MAT_INIT(&_updateArr[id].ns_0, _stateSize, sensorSize); // 6x3
		MAT_INIT(&_updateArr[id].ns_1, _stateSize, sensorSize); // 6x3
		MAT_INIT(&_updateArr[id].ss_0, sensorSize, sensorSize); // 3x3
		MAT_INIT(&_updateArr[id].ss_1, sensorSize, sensorSize); // 3x3
		MAT_INIT(&_updateArr[id].s1_0, sensorSize, 1); 			// 3x1
	}

	_updateArr[id].predictSensor = predictSensor;
	_updateArr[id].getJacobian = getJacobian;
	memcpy(_updateArr[id].covMatrix.pData, covariance->pData, 
		   sensorSize * sensorSize * sizeof(float)); // init R with given covariance
}

void ExtendedKalmanFilter::predict(const float deltaTime,
								   const float mass,
								   const float sinR,
								   const float cosR,
								   const float sinP,
								   const float cosP,
								   const float sinY,
								   const float cosY,
								   const arm_matrix_instance_f32* controlInput) 
{
	assert(controlInput->numRows == _controlInputSize);
	assert(controlInput->numCols == 1);

	int err;	

	// x = x + deltaTime * f(x, u)
	_deltaState(&_state, controlInput, mass, sinR, cosR, sinP, cosP, sinY, cosY, &_n1_0);
	err = arm_mat_scale_f32(&_n1_0, deltaTime, &_n1_0);
#ifdef LINUX
	cout << "dt * f(x,u): " << err << endl;
#endif
	
	err = arm_mat_add_f32(&_state, &_n1_0, &_state);
#ifdef LINUX
	cout << "x = x + dt*f(x,u): " << err << endl;
#endif

	// P = P + deltaTime * (A * P + P * A' + Q)
	_getJacobian(&_state, controlInput, &_systemJacobian);
	err = arm_mat_trans_f32(&_systemJacobian, &_nn_0); 			// _nn_0 = A^T
#ifdef LINUX
	cout << "A': " << err << endl;
#endif
	
	err = arm_mat_mult_f32(&_P, &_nn_0, &_nn_1); 					// _nn_1 = PA^T
#ifdef LINUX
	cout << "PA': " << err << endl;
#endif
	
	err = arm_mat_mult_f32(&_systemJacobian, &_P, &_nn_0); 		// _nn_0 = AP
#ifdef LINUX
	cout << "AP: " << err << endl;
#endif
	
	err = arm_mat_add_f32(&_nn_0, &_nn_1, &_nn_0);  				// _nn_0 = AP + PA^T
#ifdef LINUX
	cout << "AP + PA': " << err << endl;
#endif
	
	err = arm_mat_add_f32(&_nn_0, &_systemCovariance, &_nn_0); 	// _nn_0 = AP + PA^T + Q
#ifdef LINUX
	cout << "AP + PA' + Q: " << err << endl;
#endif
	
	err = arm_mat_scale_f32(&_nn_0, deltaTime, &_nn_0);			// _nn_0 = dt * (AP + PA^T + Q)
#ifdef LINUX
	cout << "dt * (AP + PA' + Q): " << err << endl;
#endif
	
	err = arm_mat_add_f32(&_P, &_nn_0, &_P);						// P = P + dt * (AP + PA^T + Q)
#ifdef LINUX
	cout << "add to P: " << err << endl;
#endif
}

void ExtendedKalmanFilter::update(const float deltaTime, const uint16_t id,
								  const arm_matrix_instance_f32* sensorMeasurement) 
{
	assert(sensorMeasurement->numRows == _updateArr[id].sensorSize);
	assert(sensorMeasurement->numCols == 1);
	
	int err;

	arm_matrix_instance_f32& jacobian = _updateArr[id].jacobian;
	arm_matrix_instance_f32& covMatrix = _updateArr[id].covMatrix;
	arm_matrix_instance_f32& ns_0 = _updateArr[id].ns_0;
	arm_matrix_instance_f32& ns_1 = _updateArr[id].ns_1;
	arm_matrix_instance_f32& ss_0 = _updateArr[id].ss_0;
	arm_matrix_instance_f32& ss_1 = _updateArr[id].ss_1;
	arm_matrix_instance_f32& s1_0 = _updateArr[id].s1_0;

	// P * C'
	_updateArr[id].getJacobian(&_state, &jacobian); // fill jacobian C (3x6)

#ifdef LINUX
	cout << "Before C':\t" << "Dim(C) = " << jacobian.numRows << "x" << jacobian.numCols << endl;
	for (uint16_t i = 0; i < jacobian.numRows; ++i)
	{
		for (uint16_t j = 0; j < jacobian.numCols; ++j) 
			cout << jacobian.pData[jacobian.numCols * i + j] << " ";
		
		cout << endl;
	}
	cout << "NS_0:\tDim(ns_0) = " << ns_0.numRows << "x" << ns_0.numCols << endl;
	for (uint16_t i = 0; i < ns_0.numRows; ++i)
	{
		for (uint16_t j = 0; j < ns_0.numCols; ++j) 
			cout << ns_0.pData[ns_0.numCols * i + j] << " ";
		
		cout << endl;
	}
#endif
	err = arm_mat_trans_f32(&jacobian, &ns_0);			// ns_0 = C' (6x3)
#ifdef LINUX
	cout << "After C': " << err << endl;
	cout << "Dim(C'): " << ns_0.numRows << "x" << ns_0.numCols << endl;
	
	for (uint16_t i = 0; i < ns_0.numRows; ++i)
	{
		for (uint16_t j = 0; j < ns_0.numCols; ++j) 
			cout << ns_0.pData[ns_0.numCols * i + j] << " ";
		
		cout << endl;
	}
#endif

	err = arm_mat_mult_f32(&_P, &ns_0, &ns_1);			// ns_1 = PC' (6x3) 
#ifdef LINUX
	cout << "PC': " << err << endl;
	for (uint16_t i = 0; i < ns_1.numRows; ++i)
	{
		for (uint16_t j = 0; j < ns_1.numCols; ++j) 
			cout << ns_1.pData[ns_1.numCols * i + j] << " ";
		
		cout << endl;
	}
#endif
	
	// C * P * C'
	err = arm_mat_mult_f32(&jacobian, &ns_1, &ss_0);	// ss_0 = C * PC' (3x3)
#ifdef LINUX
	cout << "CPC': " << err << endl;
#endif	

	// inv(R + (C * P * C'))
	err = arm_mat_add_f32(&ss_0, &covMatrix, &ss_0);	// ss_0 = R + CPC' (3x3)
#ifdef LINUX
	cout << "R + CPC': " << err << endl;
#endif

	err = arm_mat_inverse_f32(&ss_0, &ss_1);	// ss_1 = ss_0^-1 = (R + CPC')^-1 (3x3)
#ifdef LINUX
	cout << "inverse: " << err << endl;
#endif

	// L = P * C' * inv(R + (C*P*C'))
	err = arm_mat_mult_f32(&ns_1, &ss_1, &ns_0);	// ns_0 = ns_1 + ss_1 = L (6x3)
#ifdef LINUX
	cout << "L: " << err << endl;
	for (uint16_t i = 0; i < ns_0.numRows; ++i)
	{
		for (uint16_t j = 0; j < ns_0.numCols; ++j) 
			cout << ns_0.pData[ns_0.numCols * i + j] << " ";
		
		cout << endl;
	}
	cout << endl;
#endif

	// x = x + L * (y - c(x))
	_updateArr[id].predictSensor(&_state, &s1_0);	// s1_0 = x
	err = arm_mat_sub_f32(sensorMeasurement, &s1_0, &s1_0); // s1_0 = y - s1_0
#ifdef LINUX
	cout << "y - s1_0: " << err << endl;
#endif

	err = arm_mat_mult_f32(&ns_0, &s1_0, &_n1_0); // n1_0 = ns_0 * s1_0
#ifdef LINUX
	cout << "L * (y - c(x)): " << err << endl;
#endif
	
	err = arm_mat_add_f32(&_state, &_n1_0, &_state); // state = x + n1_0
#ifdef LINUX
	cout << "Updated X: " << err << endl;
#endif

	// P = (I - L * C) * P
#ifdef LINUX
	cout << "size of nn_0: " << _nn_0.numRows << " x " << _nn_0.numCols << endl;
#endif
	err = arm_mat_mult_f32(&ns_0, &jacobian, &_nn_0); // nn_0 = ns_0 * C = LC
#ifdef LINUX
	cout << "size of C: " << jacobian.numRows << " x " << jacobian.numCols << endl;
	cout << "size of L: " << ns_0.numRows << " x " << ns_0.numCols << endl;
	cout << "LC: " << err << endl;
#endif

	err = arm_mat_sub_f32(&_nn_identity, &_nn_0, &_nn_1); // nn_1 = I - LC
#ifdef LINUX
	cout << "I-LC: " << err << endl;
#endif
	
	err = arm_mat_mult_f32(&_nn_1, &_P, &_nn_0); // nn_0 = nn_1 * P = (I - LC)P
#ifdef LINUX
	cout << "(I-LC)P: " << err << endl;
#endif
	
	memcpy(_P.pData, _nn_0.pData, sizeof(float) * _stateSize * _stateSize);
#ifdef LINUX
	cout << "memcpy into P to update: " << endl;
#endif
}