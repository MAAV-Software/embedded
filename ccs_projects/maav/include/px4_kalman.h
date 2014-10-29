#ifndef PX4_KALMAN_H_
#define PX4_KALMAN_H_

// These are what you tune. This is your system noise model.
// It is a measure of confidence in your prediction
// I sometimes like to do this based on time.
// For example Qxdot = k * dt
#define SQ(X) ((X)*(X))

// define PI just in case
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define COS225 -0.70710678118
#define SIN225 -0.70710678118


typedef struct kalman_t
{
    double x, xdot, y, ydot, z, zdot, yaw, yawdot;
    double P11, P12, P22, P33, P34, P44, P55, P56, P66, P77, P78, P88;

    double dt;
    double xdot_meas;
    double ydot_meas;
    double yawdot_meas;
    double z_meas;
    double px4_quality;
    long   last_sonar_time;
    long   last_utime;

    volatile bool xyyaw_corr_ready;
    volatile bool z_corr_ready;

} kalman_t;



//kalman_t* kalman_create(void);
void kalman_create(kalman_t*);

void      kalman_update(kalman_t *k, long utime);

void      kalman_predict(kalman_t *k,
                         double Qx, double Qxdot,
                         double Qy, double Qydot,
                         double Qz, double Qzdot,
                         double Qyaw, double Qyawdot,
                         double dt);

void      kalman_process_data(kalman_t *k, int fcx, int fcy, int z, int gyro_z,
							  int gyro_r, int quality, int sonar_time,
							  long utime);

void      kalman_correct_xyyawdot(kalman_t *k,
                                  double Rxdot,
                                  double Rydot,
                                  double Ryawdot);

void      kalman_correct_z(kalman_t *k, double Rz);

void      kalman_print_graph(FILE *fp, kalman_t *k, long utime);

void      kalman_destroy(kalman_t *filter);

#endif /* PX4_KALMAN_H_ */
