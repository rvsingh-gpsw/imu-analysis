#include "surfing.h"
#include "generic.h"
#include <iostream>
#include <math.h>


namespace imua
{
  namespace surfing
  {

    bool detectSurfing(const IMU & imu, std::vector<Detection> & detections, float min_surf_time)
    {

          float average_speed = 15.0;  //just put in a number so the branch below is taken below if GPS in not enabled
          //int num_samples = imu.accl.num_samples;
          int num_samples = imu.accl.size;

          //Get the norm of the acceleration
           float * norm_acc  = new float[num_samples];    if( norm_acc == NULL) { std::cout << "could not allocate memory and will exit\n"; return false;}
           float * activity  = new float[num_samples];    if( activity == NULL) { std::cout << "could not allocate memory and will exit\n"; return false;}
           float * average   = new float[num_samples];    if( average == NULL) { std::cout << "could not allocate memory and will exit\n"; return false;}    //bigger than it needs to be


           //GForce         = getGForce(Accelerometer(:,1),Accelerometer(:,2),Accelerometer(:,3)); //--------------------------------------------------------
           //Activity       = getRunningAverage(abs(GForce-9.8), 0.1);
           for(int i = 0; i < num_samples; i++)
           {
              float norm  =  sqrt(imu.accl.x[i]*imu.accl.x[i] + imu.accl.y[i]*imu.accl.y[i] + imu.accl.z[i]*imu.accl.z[i]);
              norm_acc[i]  =  norm;
              activity[i] =  fabs(norm-9.8);
           }


          //put the activity through a low pass filter
          float weight = 0.1;
          float sample = 9.8;
          for(int i = 0; i < num_samples; i++)
          {
            sample = weight*activity[i] + (1-weight)*sample;
            activity[i] = sample;
          }

          //------------------------------------------------------------------------------------------------------------------------------------------------
          //get the maximum average from the 1 second window %----------------------------------------------------

         float window_size_time = 1;
         int   window_size = (int)(imu.accl.samplingRate*window_size_time );
         //int   num_windows =  num_samples/window_size;
         float max_average = 0;


         for(int i =0; i < num_samples; i++ )
         average[i] = 0;

         //OK to help out with plotting, we should make an average vector
         //Maybe do averaging properly ...
         max_average = 0;
          for(int i = (window_size/2); i < num_samples-(window_size/2); i++)
          {

              float sum = 0;
              for(int j = 0; j < window_size; j++)
               sum += activity[j+i - (window_size/2)];

               average[i] = sum/(float)(window_size);

               if(average[i] > max_average)
               max_average = average[i];

          }

            float threshold = max_average/8.0;
            if(threshold < 2.0)
            {
              //output += " Most likely no activity will be found ";
              std::cout << " Most likely no activity will be found\n";
              threshold = 2.0;
            }
            //output +=   std::string("The maximum average is ") + std::to_string(max_average) + " ";
            //output +=   std::string("The activity threshold is ") + std::to_string(threshold) + " ";
            std::cout << "The maximum average is " << max_average  << std::endl;
            std::cout << "The activity threshold is " << threshold << std::endl;

            //OK time to get run counts which means when the average activity is over the threshold
            int state = 0;
            int current = 0;;
            int count = 0;
            int surf_time_threshold = min_surf_time*imu.accl.samplingRate; //3 seconds
            if(average[0] > threshold) {state = 1; count++;}

            for(int i = 1; i < num_samples; i++)
            {

              if( average[i] > threshold)
              current = 1;  //true
              else
              current = 0;

              //do some logic now
            //end of a run
            if( (state == 1) && (current == 0))
            {
              if(count > surf_time_threshold)
              {

        #if 1
                 float surf_start = imu.accl.t[i-count];
                 float surf_end   = imu.accl.t[i];
        #else
                 float surf_start = imu.accl.t[i-count] + (window_size_time/2);
                 float surf_end   = imu.accl.t[i] + (window_size_time/2);
        #endif
                  double maxSpeed = 0;


                    //----------------------------------------------------------------------------------------------------------------
                    if(average_speed > 9) //default is 15mph if gps in not available so branch is taken
                    {
                      int minutes = (int)surf_start / 60;
                      int seconds = (int)surf_start % 60;
                      //std::cout << "We had a surf at " << surf_start << " =>" <<  minutes << ":" << seconds << " for " << (surf_end - surf_start) << " seconds" << std::endl;
                      Detection detection(surf_start, surf_end, "surfing");
                      detections.push_back(detection);
        						}
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

          //Need to fix this end case up ..... not sure and have to think of this a little more
          //can have an end case where the surf continued and the clip cut off
          if(current == 1)
          {
              if(count > surf_time_threshold)
              {
                //std::cout << "We had a surf at " << imu.accl.t[num_samples-count] << " for " << (imu.accl.t[num_samples-1]-imu.accl.t[num_samples-count]) << " seconds" << std::endl;
                Detection detection(imu.accl.t[num_samples-count],imu.accl.t[num_samples-1] ,"surfing");
                detections.push_back(detection);
             }
          }

        //make sure we free the memory
           if(norm_acc)  delete [] norm_acc;
           if(activity) delete [] activity;
           if(average) delete [] average;
           return 0;
    }

  }
}
