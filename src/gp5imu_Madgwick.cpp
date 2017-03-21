#include "types.h"
#include <stdio.h>
#include <math.h>
#include <iostream>

namespace imua
{
//*********************************************************************************************************

  float q0,q1,q2,q3, beta;
//  float roll, pitch, yaw;

void MadgwickAHRSupdateIMU(const IMU & imu, float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  float sampleFreq = imu.gyro.samplingRate;


	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
	  //		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		recipNorm = 1.0/sqrtf(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		//		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		recipNorm = 1.0/sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	//	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	//	printf("recipNorm=%f, %f\n", recipNorm, 1.0f/sqrtf((q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)));
	recipNorm = 1.0f/sqrtf((q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3));
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

}

//extract Euler angles and assume accelerometer is sampled half the gyro rate
int  getEulerAngles(const IMU & imu, Euler & euler)
{

  euler.num_samples = imu.gyro.size; //num_samples;
  euler.sampling_rate = imu.gyro.samplingRate;
  euler.t     = new float[euler.num_samples];    if( euler.t     == nullptr) { std::cout << "could not allocate memory and will exit\n"; return 1;}
  euler.roll  = new float[euler.num_samples];    if( euler.roll  == nullptr) { std::cout << "could not allocate memory and will exit\n"; return 1;}
  euler.pitch = new float[euler.num_samples];    if( euler.pitch == nullptr) { std::cout << "could not allocate memory and will exit\n"; return 1;}
  euler.yaw   = new float[euler.num_samples];    if( euler.yaw   == nullptr) { std::cout << "could not allocate memory and will exit\n"; return 1;}

//might have to flush it a bit
//MadgwickAHRSupdateIMU(0,0,0,ax,ay,az);

//CALIBRATE ???????????????????

  //initialize the orinetation
    q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; //x,y,z,w
    //q0 =0.0f, q1=0.0f, q2=0.0f, q3=1.0f; //180% about the z-axis start

    //beta = 0.05;//5;//0.1;//0.05f;//05f;
    beta = 0.25;//5;//0.1;//0.05f;//05f;

    float gx,gy,gz,ax,ay,az;
    int acc_count = 0;

int do_euler_init = 0;

//---------------------------------------------------------------------------------------------------
 if(do_euler_init)
 {
   for(int i = 0; i < 400*2; i++) //do a 2 second init
   {

     gx = imu.gyro.x[i];
     gy = imu.gyro.y[i];
     gz = imu.gyro.z[i];

     //this is half sampling_rate of gyro so every other
     if( (i & 1) == 0)
     {

       //we need this in terms of G's
       ax = imu.accl.x[acc_count]/ 9.81f;
       ay = imu.accl.y[acc_count]/ 9.81f;
       az = imu.accl.z[acc_count]/ 9.81f;
       acc_count++;
     }

     MadgwickAHRSupdateIMU(imu, gx, gy, gz, ax, ay, az);
   }

   euler.roll_init       = (-asin(2.0f * (q1 * q3 - q0 * q2)))*(57.2958);
   euler.pitch_init      = (atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3))*(57.2958);
   euler.yaw_init        = (atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3))*(57.2958);
}


//---------------------------------------------------------------------------------------------------------
   acc_count = 0;
   for (int i = 0; i < euler.num_samples; i++)
   {
      gx = imu.gyro.x[i];
      gy = imu.gyro.y[i];
      gz = imu.gyro.z[i];

      //this is half sampling_rate of gyro so every other
      if( (i & 1) == 0)
      {

        //we need this in terms of G's
        ax = imu.accl.x[acc_count]/ 9.81f;
        ay = imu.accl.y[acc_count]/ 9.81f;
        az = imu.accl.z[acc_count]/ 9.81f;
        acc_count++;
      }

      MadgwickAHRSupdateIMU(imu, gx, gy, gz, ax, ay, az);

       euler.t[i] = imu.gyro.t[i];
       //this is radians and convert (180/PI) = 57.29 ... to degrees
       euler.roll[i]       = (-asin(2.0f * (q1 * q3 - q0 * q2)))*(57.2958);
       euler.pitch[i]        = (atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3))*(57.2958);

      float y = 2.0f * (q1 * q2 + q0 * q3);
      float x = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
      euler.yaw[i]         = atan2(y,x)*(57.2958);

  //the yaw is keep positive
      if(euler.yaw[i] < 0)
        euler.yaw[i]  += 360.0;

       if(do_euler_init)
       {
         euler.roll[i]  -= euler.roll_init;
         euler.pitch[i] -= euler.pitch_init;
         euler.yaw[i]   -= euler.yaw_init;
       }

    }

    //if(writeBinaryFiles != 0)
    // writeToFile(euler.yaw,euler.num_samples, (char *)"_yaw.bin");

return 0;

}

#if 0
void toEulerianAngle() //double& pitch, double& roll, double& yaw)
{

//these are in degrees
  roll       = (-asin(2.0f * (q1 * q3 - q0 * q2)))*(57.2958);
  pitch      = (atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3))*(57.2958);
  yaw        = (atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3))*(57.2958);

}
#endif

}//namespace
