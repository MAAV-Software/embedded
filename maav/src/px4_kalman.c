#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "px4_kalman.h"

// uncomment the following line if you are running this on Linux
// for generating kalman plots in matlab
//#define LINUX

/* Global Kalman Filter Matrix Entries */
double Kqx      =      0.01;
double Kqxdot   =      1.00;
double Kqy      =      0.01;
double Kqydot   =      1.00;
double Kqz      =      0.01;
double Kqzdot   =      0.30;
double Kqyaw    =      0.01;
double Kqyawdot =  SQ(M_PI/7);

double Rxydot_max 	= SQ(7.0 / 3.0);
double Rxydot_min 	= SQ(4.0 / 3.0);
double Rz_max 		= SQ(10.0 / 3.0);
double Rz_min 		= SQ(0.5 / 3.0);
double Ryawdot 		= SQ(0.35 / 3.0);

const double Z_MAX_RETURN = 2.5;
const double Z_MIN_RETURN = 0.3;

#ifdef LINUX
bool getData(kalman_t *filter)
{
}

void kalman_print_state(char *buff, int buff_size, kalman_t *filter)
{
    int size = 0;
    size += snprintf(buff + size, buff_size - size, "x      = % lf   P = % lf  % lf\n", filter->x,      filter->P11, filter->P12);
    size += snprintf(buff + size, buff_size - size, "xdot   = % lf       % lf  % lf\n", filter->xdot,   filter->P12, filter->P22);
    size += snprintf(buff + size, buff_size - size, "y      = % lf   P = % lf  % lf\n", filter->y,      filter->P33, filter->P34);
    size += snprintf(buff + size, buff_size - size, "ydot   = % lf       % lf  % lf\n", filter->ydot,   filter->P34, filter->P44);
    size += snprintf(buff + size, buff_size - size, "z      = % lf   P = % lf  % lf\n", filter->z,      filter->P55, filter->P56);
    size += snprintf(buff + size, buff_size - size, "zdot   = % lf       % lf  % lf\n", filter->zdot,   filter->P56, filter->P66);
    size += snprintf(buff + size, buff_size - size, "yaw    = % lf   P = % lf  % lf\n", filter->yaw,    filter->P77, filter->P78);
    size += snprintf(buff + size, buff_size - size, "yawdot = % lf       % lf  % lf\n", filter->yawdot, filter->P78, filter->P88);
}

void kalman_printf_state(kalman_t *filter)
{
#define buff_size 2000
    char buff[buff_size];
    kalman_print_state(buff, buff_size, filter);
    printf("%s", buff);
#undef buff_size
}

int main()
{
	// edit the file paths for the input file (tiva log) and output (kalman_out)
    //FILE *fp = fopen("/home/jonathan/Downloads/kalman1.log", "r");
    FILE *fp = fopen("/home/jonathan/Downloads/up_down.txt", "r");
    //FILE *fp = fopen("/home/jonathan/Downloads/horizontal_box.txt", "r");
    //FILE *fp = fopen("/home/jonathan/Downloads/Static.txt", "r");
    FILE *fout = fopen("/tmp/kalman_out.log", "w");
    if(fp == NULL)
    {
        fprintf(stderr, "Unable to open input file\n");
        exit(1);
    }
    if(fout == NULL)
    {
        fprintf(stderr, "Unable to open output file\n");
        exit(1);
    }

    kalman_t *filter = kalman_create();

    int intnull;
    float floatnull;

    int fc, fcx, fcy, q, gz, gr, st, gd;

    while(fscanf(fp, "%d, %d, %d, %d, %d, %d, %d, %d, "
                     "%d, %d, %d, %d, "
                     "%f, %f, %f, %f, %f, "
                     "%f, %f, %f, %f\n",
                     &fc, &intnull, &intnull, &fcx, &fcy, &q, &intnull,
                     &intnull, &gz, &gr, &st, &gd,
                     &floatnull, &floatnull, &floatnull, &floatnull, &floatnull,
                     &floatnull, &floatnull, &floatnull, &floatnull) != EOF)
    {
        static int n = 0;
        kalman_process_data(filter, fcx, fcy, gd, gz, gr, q, st, fc*1e4);
        kalman_update(filter, fc*1e4);
        kalman_print_graph(fout, filter, fc*1e4);
        //if(n == 5)
            //exit(1);
        n++;
    }

    fclose(fp);
    fclose(fout);
    kalman_destroy(filter);
}
#endif



//kalman_t* kalman_create(void)
void kalman_create(kalman_t *ret)
{
//    kalman_t *ret = (kalman_t*) malloc(sizeof(kalman_t));

    ret->x = ret->xdot = ret->y = ret->ydot = ret->z = ret->zdot = ret->yaw = ret->yawdot = 0;

    ret->P11 = 1;
    ret->P12 = 0;
    ret->P22 = 1;
    ret->P33 = 1;
    ret->P34 = 0;
    ret->P44 = 1;
    ret->P55 = 1;
    ret->P56 = 0;
    ret->P66 = 1;
    ret->P77 = 1;
    ret->P78 = 0;
    ret->P88 = 1;

    ret->dt              = 0;
    ret->xdot_meas       = 0;
    ret->ydot_meas       = 0;
    ret->yawdot_meas     = 0;
    ret->z_meas          = 0;
    ret->px4_quality     = 0;
    ret->last_sonar_time = 0;
    ret->last_utime      = 0;

    ret->xyyaw_corr_ready = false;
    ret->z_corr_ready = false;

//    return ret;
}

void kalman_update(kalman_t *k, long utime)
{
    // This is to handle the case where we havent done a
    // process data in between two kalman_updates
    if(k->xyyaw_corr_ready == false)
    {
        k->dt = (utime - k->last_utime) / 1e6;
        k->last_utime = utime;
    }

    if(k->dt > 0.001)
    {
        kalman_predict(k,
                       k->dt * Kqx,
                       k->dt * Kqxdot,
                       k->dt * Kqy,
                       k->dt * Kqydot,
                       k->dt * Kqz,
                       k->dt * Kqzdot,
                       k->dt * Kqyaw,
                       k->dt * Kqyawdot,
                       k->dt);
#ifdef LINUX
        printf("****************************\n");
        printf("After the predict\n");
        printf("****************************\n");
        kalman_printf_state(k);
#endif

        if(k->xyyaw_corr_ready)
        {
            double Rxydot = SQ((Rxydot_max - (Rxydot_max - Rxydot_min) * k->px4_quality)/3);
            kalman_correct_xyyawdot(k,
                                    Rxydot,
                                    Rxydot,
                                    Ryawdot);
#ifdef LINUX
            printf("****************************\n");
            printf("After the x, y, yaw correct\n");
            printf("****************************\n");
            kalman_printf_state(k);
#endif
        }

        if(k->z_corr_ready)
        {
            double tempRz;
            if(( k->z_meas < k->z - sqrt(k->P55)*3) || (k->z_meas > k->z + sqrt(k->P55)*3))
            {
                tempRz = Rz_max;
            }
            else
            {
                tempRz = Rz_min;
            }
            kalman_correct_z(k, tempRz);
#ifdef LINUX
            printf("****************************\n");
            printf("After the z correct\n");
            printf("****************************\n");
            kalman_printf_state(k);
#endif
        }
#ifdef LINUX
        printf("\n\n");
#endif
    }
}

// dt here is the time since the last predict or correct
void kalman_predict(kalman_t *k,
                    double Qx, double Qxdot,
                    double Qy, double Qydot,
                    double Qz, double Qzdot,
                    double Qyaw, double Qyawdot,
                    double dt)
{
    k->x      += dt*k->xdot;
  //k->xdot    = k->xdot;
    k->y      += dt*k->ydot;
  //k->ydot    = k->ydot;
    k->z      += dt*k->zdot;
  //k->zdot    = k->zdot;
    k->yaw    += dt*k->yawdot;
  //k->yawdot  = k->yawdot;

    double dt2 = dt * dt;

    double tmp[12];

    tmp[0]  = k->P11 + Qx + 2 * k->P12 * dt + k->P22 * dt2 + Qxdot * dt2;
    tmp[1]  = k->P12 + k->P22 * dt + Qxdot * dt;
    tmp[2]  = k->P22 + Qxdot;
    tmp[3]  = k->P33 + Qy + 2 * k->P34 * dt + k->P44 * dt2 + Qydot * dt2;
    tmp[4]  = k->P34 + k->P44 * dt + Qydot * dt;
    tmp[5]  = k->P44 + Qydot;
    tmp[6]  = k->P55 + Qz + 2 * k->P56 * dt + k->P66 * dt2 + Qzdot * dt2;
    tmp[7]  = k->P56 + k->P66 * dt + Qzdot * dt;
    tmp[8]  = k->P66 + Qzdot;
    tmp[9]  = k->P77 + Qyaw + 2 * k->P78 * dt + k->P88 * dt2 + Qyawdot * dt2;
    tmp[10] = k->P78 + k->P88 * dt + Qyawdot * dt;
    tmp[11] = k->P88 + Qyawdot;

    k->P11 = tmp[0];
    k->P12 = tmp[1];
    k->P22 = tmp[2];
    k->P33 = tmp[3];
    k->P34 = tmp[4];
    k->P44 = tmp[5];
    k->P55 = tmp[6];
    k->P56 = tmp[7];
    k->P66 = tmp[8];
    k->P77 = tmp[9];
    k->P78 = tmp[10];
    k->P88 = tmp[11];

}

void kalman_process_data(kalman_t *k, int fcx, int fcy, int z, int gyro_z,
						 int gyro_r, int quality, int sonar_time, long utime) {
    if(k->last_utime == 0){
        k->last_utime = utime;
        return;
    }

    k->dt          = (utime - k->last_utime) / 1e6;
//    k->xdot_meas   = fcx / 1000.0;
//    k->ydot_meas   = fcy / 1000.0;
    k->xdot_meas   = ( COS225 * (double)fcx / 1000.0)
    			   - ( SIN225 * (double)fcy / 1000.0);
    k->ydot_meas   = ( COS225 * (double)fcx / 1000.0)
    			   + ( SIN225 * (double)fcy / 1000.0);

    k->yawdot_meas = (double)gyro_z / (double)(1 << 15) * (double)gyro_r;
    k->z_meas      = (double)z / 1000.0;
    k->px4_quality = ((double)quality) / 255.0;
    k->last_utime = utime;

#ifdef LINUX
    printf("****************************\n");
    printf("Measurements taken:\n");
    printf("****************************\n");
    printf("xdot meas   = %f\n", k->xdot_meas);
    printf("ydot meas   = %f\n", k->ydot_meas);
    printf("z meas      = %f\n", k->z_meas);
    printf("yawdot meas = %f\n", k->yawdot_meas);
    printf("quality     = %f\n", k->px4_quality);
    printf("dt          = %f\n", k->dt);
#endif

    k->xyyaw_corr_ready = true;

    if(sonar_time < k->last_sonar_time)
    {
        if(k->z_meas < Z_MAX_RETURN && k->z_meas > Z_MIN_RETURN)
            k->z_corr_ready = true;
    }
    k->last_sonar_time = sonar_time;
}

void kalman_correct_xyyawdot(kalman_t *k,
                             double Rxdot,
                             double Rydot,
                             double Ryawdot)
{
    double tmp[9];

    tmp[0] = k->x - (k->P12*(k->xdot - k->xdot_meas))/(k->P22 + Rxdot);
    tmp[1] = (k->P22*k->xdot_meas + Rxdot*k->xdot)/(k->P22 + Rxdot);
    tmp[2] = k->y - (k->P34*(k->ydot - k->ydot_meas))/(k->P44 + Rydot);
    tmp[3] = (k->P44*k->ydot_meas + Rydot*k->ydot)/(k->P44 + Rydot);
    tmp[4] = k->yaw - (k->P78*(k->yawdot - k->yawdot_meas))/(k->P88 + Ryawdot);
    tmp[5] = (k->P88*k->yawdot_meas + Ryawdot*k->yawdot)/(k->P88 + Ryawdot);

    k->x      = tmp[0];
    k->xdot   = tmp[1];
    k->y      = tmp[2];
    k->ydot   = tmp[3];
    k->yaw    = tmp[4];
    k->yawdot = tmp[5];

    tmp[0] = - SQ(k->P12)/(k->P22 + Rxdot) + k->P11;
    tmp[1] = (k->P12*Rxdot)/(k->P22 + Rxdot);
    tmp[2] = (k->P22*Rxdot)/(k->P22 + Rxdot);
    tmp[3] = - SQ(k->P34)/(k->P44 + Rydot) + k->P33;
    tmp[4] = (k->P34*Rydot)/(k->P44 + Rydot);
    tmp[5] = (k->P44*Rydot)/(k->P44 + Rydot);
    tmp[6] = - SQ(k->P78)/(k->P88 + Ryawdot) + k->P77;
    tmp[7] = (k->P78*Ryawdot)/(k->P88 + Ryawdot);
    tmp[8] = (k->P88*Ryawdot)/(k->P88 + Ryawdot);

    k->P11 = tmp[0];
    k->P12 = tmp[1];
    k->P22 = tmp[2];
    k->P33 = tmp[3];
    k->P34 = tmp[4];
    k->P44 = tmp[5];
    k->P77 = tmp[6];
    k->P78 = tmp[7];
    k->P88 = tmp[8];

    k->xyyaw_corr_ready = false;
}

void kalman_correct_z(kalman_t *k, double Rz)
{
    double tmp[3];

    tmp[0] = (k->P55*k->z_meas + Rz*k->z)/(k->P55 + Rz);
    tmp[1] = k->zdot - (k->P56*(k->z - k->z_meas))/(k->P55 + Rz);

    k->z    = tmp[0];
    k->zdot = tmp[1];

    tmp[0] = (k->P55*Rz)/(k->P55 + Rz);
    tmp[1] = (k->P56*Rz)/(k->P55 + Rz);
    tmp[2] = - SQ(k->P56)/(k->P55 + Rz) + k->P66;

    k->P55 = tmp[0];
    k->P56 = tmp[1];
    k->P66 = tmp[2];

    k->z_corr_ready = false;
}

void kalman_print_graph(FILE *fp, kalman_t *k, long utime)
{
    fprintf(fp, "%ld, %f, %f, %f, %f, %f, "
                 "%f, %f, %f, %f, %f, %f, %f, %f,"
                 "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                utime, k->xdot_meas, k->ydot_meas, k->z_meas, k->yawdot_meas,
                k->px4_quality, k->x, k->xdot, k->y, k->ydot, k->z, k->zdot,
                k->yaw, k->yawdot, k->P11, k->P12, k->P22, k->P33, k->P34,
                k->P44, k->P55, k->P56, k->P66, k->P77, k->P78, k->P88);
}

void kalman_destroy(kalman_t *filter)
{
    free(filter);
}
