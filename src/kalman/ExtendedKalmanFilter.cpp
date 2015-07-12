#include "kalman/ExtendedKalmanFilter.hpp"
#include <stdlib.h>
#include <assert.h>

// TODO: check in arm_mat_mult_f32 if the destination can
// be the same as a source (and inverse as well)

// easy func for initializing a matrix with uninitialized data
inline static void MAT_INIT(arm_matrix_instance_f32* mat,
	size_t rows, size_t cols) {
	arm_mat_init_f32(mat, rows, cols,
		(float*)malloc(sizeof(float) * rows * cols));
}

ExtendedKalmanFilter::ExtendedKalmanFilter(uint16_t stateSize,
	float* initialState, float* initialErrorCov) :
	_stateSize(stateSize), _controlInputSize(0),
	_deltaState(NULL), _getJacobian(NULL) {

	// initializing state matrices
	size_t stateMatSize = stateSize * stateSize * sizeof(float);
	MAT_INIT(&_state, stateSize, 1);
	memcpy(_state.pData, initialState, stateSize * sizeof(float));
	MAT_INIT(&_P, stateSize, stateSize);
	memcpy(_P.pData, initialErrorCov, stateMatSize);

	// initializing predict related matrices
	MAT_INIT(&_systemJacobian, stateSize, stateSize);
	MAT_INIT(&_systemCovariance, stateSize, stateSize);

	// initializing temporary matrices
	MAT_INIT(&_n1_0, stateSize, 1);
	MAT_INIT(&_nn_0, stateSize, 1);
	MAT_INIT(&_nn_1, stateSize, 1);

	// initializing identity matrix
	MAT_INIT(&_nn_identity, stateSize, stateSize);
	for (uint16_t i = 0; i < stateSize * stateSize; i++) {
		if (i % (stateSize + 1) == 0) {
			_nn_identity.pData[i] = 1;
		} else {
			_nn_identity.pData[i] = 0;
		}
	}

	for (uint16_t i = 0; i < NUM_UPDATE_SLOTS; i++) {
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

ExtendedKalmanFilter::~ExtendedKalmanFilter() {
	free(_state.pData);
	free(_P.pData);
	free(_systemJacobian.pData);
	free(_systemCovariance.pData);
	free(_n1_0.pData);
	free(_nn_0.pData);
	free(_nn_1.pData);
	free(_nn_identity.pData);

	for (uint16_t i = 0; i < NUM_UPDATE_SLOTS; i++) {
		if (_updateArr[i].sensorSize != 0) {
			free(_updateArr[i].jacobian.pData);
			free(_updateArr[i].covMatrix.pData);
			free(_updateArr[i].ns_0.pData);
			free(_updateArr[i].ns_1.pData);
			free(_updateArr[i].ss_0.pData);
			free(_updateArr[i].ss_1.pData);
			free(_updateArr[i].s1_0.pData);
		}
	}
}

void ExtendedKalmanFilter::setPredictFunc(uint16_t controlInputSize,
	void (*deltaState)(const arm_matrix_instance_f32*,
		const arm_matrix_instance_f32*, arm_matrix_instance_f32*),
	void (*getJacobian)(const arm_matrix_instance_f32*,
		const arm_matrix_instance_f32*, arm_matrix_instance_f32*),
	const arm_matrix_instance_f32* covariance) {
	_deltaState = deltaState;
	_getJacobian = getJacobian;

	assert(covariance->numRows == _stateSize);
	assert(covariance->numCols == _stateSize);

	size_t covMatSize = _stateSize * _stateSize * sizeof(float);
	memcpy(_systemCovariance.pData, covariance->pData, covMatSize);

	_controlInputSize = controlInputSize;
}

void ExtendedKalmanFilter::setUpdateFunc(uint16_t id, uint16_t sensorSize,
	void (*predictSensor)(const arm_matrix_instance_f32*,
		arm_matrix_instance_f32*),
	void (*getJacobian)(const arm_matrix_instance_f32*,
		arm_matrix_instance_f32*),
	const arm_matrix_instance_f32* covariance) {

	assert(sensorSize != 0);
	assert(covariance->numRows == sensorSize);
	assert(covariance->numCols == sensorSize);

	// if sensor size is different or its the first time access
	// delete old memory (if it exists) and allocate correct size
	if (sensorSize != _updateArr[id].sensorSize) {
		if (_updateArr[id].sensorSize != 0) {
			// there is old data here of a different size
			free(_updateArr[id].jacobian.pData);
			free(_updateArr[id].covMatrix.pData);
			free(_updateArr[id].ns_0.pData);
			free(_updateArr[id].ns_1.pData);
			free(_updateArr[id].ss_0.pData);
			free(_updateArr[id].ss_1.pData);
			free(_updateArr[id].s1_0.pData);
		}
		_updateArr[id].sensorSize = sensorSize;

		MAT_INIT(&_updateArr[id].jacobian, sensorSize, _stateSize);
		MAT_INIT(&_updateArr[id].covMatrix, sensorSize, sensorSize);
		MAT_INIT(&_updateArr[id].ns_0, _stateSize, sensorSize);
		MAT_INIT(&_updateArr[id].ns_1, _stateSize, sensorSize);
		MAT_INIT(&_updateArr[id].ss_0, sensorSize, sensorSize);
		MAT_INIT(&_updateArr[id].ss_1, sensorSize, sensorSize);
		MAT_INIT(&_updateArr[id].s1_0, sensorSize, 1);
	}

	_updateArr[id].predictSensor = predictSensor;
	_updateArr[id].getJacobian = getJacobian;
	memcpy(_updateArr[id].covMatrix.pData, covariance->pData,
		sensorSize * sensorSize * sizeof(float));
}

void ExtendedKalmanFilter::predict(float deltaTime,
	const arm_matrix_instance_f32* controlInput) {

	assert(controlInput->numRows == _controlInputSize);
	assert(controlInput->numCols == 1);

	// x = x + deltaTime * f(x, u)
	_deltaState(&_state, controlInput, &_n1_0);
	arm_mat_scale_f32(&_n1_0, deltaTime, &_n1_0);
	arm_mat_add_f32(&_state, &_n1_0, &_state);

	// P = P + deltaTime * (A * P + P * A' + Q)
	_getJacobian(&_state, controlInput, &_systemJacobian);
	arm_mat_trans_f32(&_systemJacobian, &_nn_0);
	arm_mat_mult_f32(&_P, &_nn_0, &_nn_1);
	arm_mat_mult_f32(&_P, &_systemJacobian, &_nn_0);
	arm_mat_add_f32(&_nn_0, &_nn_1, &_nn_0);
	arm_mat_add_f32(&_nn_0, &_systemCovariance, &_nn_0);
	arm_mat_scale_f32(&_nn_0, deltaTime, &_nn_0);
	arm_mat_add_f32(&_P, &_nn_0, &_P);
}

void ExtendedKalmanFilter::update(float deltaTime, const uint16_t id,
	const arm_matrix_instance_f32* sensorMeasurement) {

	assert(sensorMeasurement->numRows == _updateArr[id].sensorSize);
	assert(sensorMeasurement->numCols == 1);

	arm_matrix_instance_f32& jacobian = _updateArr[id].jacobian;
	arm_matrix_instance_f32& covMatrix = _updateArr[id].covMatrix;
	arm_matrix_instance_f32& ns_0 = _updateArr[id].ns_0;
	arm_matrix_instance_f32& ns_1 = _updateArr[id].ns_1;
	arm_matrix_instance_f32& ss_0 = _updateArr[id].ss_0;
	arm_matrix_instance_f32& ss_1 = _updateArr[id].ss_1;
	arm_matrix_instance_f32& s1_0 = _updateArr[id].s1_0;

	// P * C'
	_updateArr[id].getJacobian(&_state, &jacobian);
	arm_mat_trans_f32(&jacobian, &ns_0);
	arm_mat_mult_f32(&_P, &ns_0, &ns_1);

	// C * P * C'
	arm_mat_mult_f32(&jacobian, &ns_1, &ss_0);

	// inv(R + C * P * C')
	arm_mat_add_f32(&ss_0, &covMatrix, &ss_0);
	arm_mat_inverse_f32(&ss_0, &ss_1);

	// L = P * C' * inv(R + C*P*C')
	arm_mat_mult_f32(&ns_1, &ss_1, &ns_0);

	// x = x + L * (y - c(x))
	_updateArr[id].predictSensor(&_state, &s1_0);
	arm_mat_sub_f32(sensorMeasurement, &s1_0, &s1_0);
	arm_mat_mult_f32(&ns_0, &s1_0, &_n1_0);
	arm_mat_add_f32(&_state, &_n1_0, &_state);

	// P = (I - L * C) * P
	arm_mat_mult_f32(&ns_0, &jacobian, &_nn_0);
	arm_mat_sub_f32(&_nn_identity, &_nn_0, &_nn_1);
	arm_mat_mult_f32(&_nn_1, &_P, &_nn_0);
	memcpy(_P.pData, _nn_0.pData, sizeof(float) * _stateSize * _stateSize);
}
