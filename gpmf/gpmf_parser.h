#ifndef  GPMF_PARSER_H
#define   GPMF_PARSER_H

typedef struct accelerometer
{
  float sampling_rate;   
  int num_samples;
  float * t;
  float * x;
  float * y;
  float * z;
  //for intialization
  float x_bias;
  float y_bias;
  float z_bias;
} Accelerometer_t;

typedef struct gyro
{
  float sampling_rate;
  int   num_samples;
  float * t;
  float * x;
  float * y;
  float * z;
  //for intialization
  float x_bias;
  float y_bias;
  float z_bias;
} Gyro_t;


typedef struct imu
{
 Accelerometer_t accl;
 Gyro_t          gyro;
} IMU_t;


//returns 1 if an error occured otherwise it does memory allocation on the imu data (mallocs)
//the data can be read like this after the function is called:
//if(ExtractIMU(filename, &imu) != 0)
//{
//  printf("An error occured in the parser \n");
//  return 1;
//}
//Lets check out some answers
//for(int i = 0; i < 20; i++)
//{
//  printf("Accl[%i] = %f,%f,%f,%f \n",i, imu.accl.t[i],imu.accl.z[i],imu.accl.x[i],imu.accl.y[i]);
//  printf("Gyro[%i] = %f,%f,%f,%f \n",i, imu.gyro.t[i],imu.gyro.z[i],imu.gyro.x[i],imu.gyro.y[i]);
//}
int  ExtractIMU(char *basename, IMU_t * imu);

//this function deallocates the memory that was allocated by ExtractIMU
void DeAllocateImu(IMU_t * imu);


#endif
