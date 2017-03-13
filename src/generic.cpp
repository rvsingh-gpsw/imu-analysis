#include "generic.h"

#include <iostream>
#include <cmath>

namespace imua
{
  namespace generic
  {

    bool detectJumps(const IMU & imu,
                    std::vector<Detection> & detections,
                    const float gforceThreshold,
                    const float hangetimeThreshold)
    {

      // std::cout << "In the jump detector \n";
      int num_samples = imu.accl.size;
      // std::cout << "num_samples = " <<  num_samples << std::endl;

      //Get the norm of the acceleration
      float * gforce          = new float[num_samples];    if( gforce         == nullptr) { std::cout << "could not allocate memory and will exit\n"; return false;}
      float * gforce_lowpass  = new float[num_samples];    if( gforce_lowpass == nullptr) { std::cout << "could not allocate memory and will exit\n"; return false;}    //bigger than it needs to be

      //get the norm of acceleration
      for(int i = 0; i < num_samples; i++)
      {
        double norm  =  sqrt(imu.accl.x[i]*imu.accl.x[i] + imu.accl.y[i]*imu.accl.y[i] + imu.accl.z[i]*imu.accl.z[i]);
        gforce[i]  =  norm;
      }

      //put the gforce through a low pass filter
      float weight = 0.1;
      float sample = 9.8;
      float average_gforce = 0;
      for(int i = 0; i < num_samples; i++)
      {
        sample = weight*gforce[i] + (1-weight)*sample;
        gforce_lowpass[i] = sample;
         average_gforce += sample;
      }

      average_gforce /= num_samples;
      // printf("The average gforce is %f\n", average_gforce);

      //get approximately how many samples hangtime is
      float threshold_samples = hangetimeThreshold*imu.accl.samplingRate;
      int   state    = 0;
      int   current = 0;
      int   count   = 0;
      if( gforce_lowpass[0] < gforceThreshold) {state = 1; count++;}
      for(int i = 1; i < num_samples; i++)
      {
        if( gforce_lowpass[i] < gforceThreshold)
        current = 1;  //true
        else
        current = 0;

        //do some logic now, if current is below threshold this marks the end of a run
        if( (state == 1) && (current == 0))
        {
          if(count > threshold_samples)
          {

            // Tune the jump !!!
            //OK, we should find where we start the jump and really end the jump
            //we should find the gravity crossings and label them as start and finish
            int start_idx = i-count;
            int   end_idx = i;


       //------------------THIS IS TUNING THE JUMP FOR TAKE OF AND LANDING

        float start_jump_thresh = average_gforce;
        float end_jump_thresh   = average_gforce;

        while(gforce_lowpass[start_idx] < start_jump_thresh )
        start_idx--;
        while(gforce_lowpass[end_idx] < end_jump_thresh )
        end_idx++;

            //-------- get the start and end times and make a highlight
            float jump_start =  imu.accl.t[start_idx];
            float jump_end   =  imu.accl.t[end_idx];

            int minutes = (int)jump_start / 60;
            int seconds = (int)jump_start % 60;
            float height = 0.5*9.8*( ((jump_end-jump_start)/2)*((jump_end-jump_start)/2));

              // std::cout << "We had a Jump at " << jump_start << " =>" <<  minutes << ":" << seconds << " for " << (jump_end-jump_start) << " seconds" << " height = " << height*3.28084  << "ft" << std::endl;
              // high_light jump_highlight;
              // jump_highlight.in_time    =    jump_start*1000;
              // jump_highlight.out_time   =    jump_end*1000;
              // jump_highlight.event_type =    STR2FOURCC("JUMP");
              // highlights.push_back(jump_highlight);
              Detection detection(jump_start, jump_end, (jump_start+jump_end)/2, height, "jump");
              detections.push_back(detection);

            }
            count = 0;
          }
         else if (current == 1)
         {
           count++;
         }
         else; //no run

         //store for next interation
         state = current;

       }

      //dealocate the memory
       if(gforce)          free(gforce);
       if(gforce_lowpass)  free(gforce_lowpass);

      return true;
    }



  }
}
