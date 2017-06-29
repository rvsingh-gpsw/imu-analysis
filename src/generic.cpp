#include "generic.h"
#include "tools.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

#include <fstream>


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
      float * gforce          = new float[num_samples];    if( gforce         == NULL) { std::cout << "could not allocate memory and will exit\n"; return false;}
      float * gforce_lowpass  = new float[num_samples];    if( gforce_lowpass == NULL) { std::cout << "could not allocate memory and will exit\n"; return false;}    //bigger than it needs to be

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

        while(gforce_lowpass[start_idx] < start_jump_thresh && start_idx>0)
          start_idx--;
        while(gforce_lowpass[end_idx] < end_jump_thresh && end_idx<num_samples-1)
          end_idx++;

            //-------- get the start and end times and make a highlight
            float jump_start =  imu.accl.t[start_idx];
            float jump_end   =  imu.accl.t[end_idx];

            // int minutes = (int)jump_start / 60;
            // int seconds = (int)jump_start % 60;
            float height = 0.5*9.8*( ((jump_end-jump_start)/2)*((jump_end-jump_start)/2));

              // std::cout << "We had a Jump at " << jump_start << " =>" <<  minutes << ":" << seconds << " for " << (jump_end-jump_start) << " seconds" << " height = " << height*3.28084  << "ft" << std::endl;
              // high_light jump_highlight;
              // jump_highlight.in_time    =    jump_start*1000;
              // jump_highlight.out_time   =    jump_end*1000;
              // jump_highlight.event_type =    STR2FOURCC("JUMP");
              // highlights.push_back(jump_highlight);
              if (detections.empty() || jump_start>detections.back().end) {
                Detection detection(jump_start, jump_end, height, "jump");
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

      //dealocate the memory
      delete[](gforce);
      delete[](gforce_lowpass);

      return true;
    }


    bool detectJumps2(const IMU & imu,
                    std::vector<Detection> & detections)
    {


      // Parameters
      const float threshold_norm_min = 4.f;
      const float threshold_norm_max = 9.f;
      const float threshold_dur_min  = 0.25f;

      // This part allocate memory and may throw an exception. Let's protect it with a try-catch.
      try {

        // Smooth the input data
        const int   size  = imu.accl.size;
        const float sigma = imu.accl.samplingRate * 0.02f;
        std::vector<float> x2(size);
        std::vector<float> y2(size);
        std::vector<float> z2(size);
        SmoothArrayBox(imu.accl.x, &x2[0], imu.accl.size, sigma);
        SmoothArrayBox(imu.accl.y, &y2[0], imu.accl.size, sigma);
        SmoothArrayBox(imu.accl.z, &z2[0], imu.accl.size, sigma);

        // Compute the norm
        std::vector<float> norm_smooth;
        ComputeNorm(&x2[0], &y2[0], &z2[0], imu.accl.size, norm_smooth);

        // // DEBUG
        // std::ofstream ofs;
        // ofs.open("/Users/vincent/Desktop/output.csv");
        // for (int i=0; i<size; i++)
        //   ofs << imu.accl.t[i] << ((i<size-1) ? "," : "\n");
        // for (int i=0; i<size; i++)
        //   ofs << imu.accl.x[i] << ((i<size-1) ? "," : "\n");
        // for (int i=0; i<size; i++)
        //   ofs << imu.accl.y[i] << ((i<size-1) ? "," : "\n");
        // for (int i=0; i<size; i++)
        //   ofs << imu.accl.z[i] << ((i<size-1) ? "," : "\n");
        // for (int i=0; i<size; i++)
        //   ofs << x2[i] << ((i<size-1) ? "," : "\n");
        // for (int i=0; i<size; i++)
        //   ofs << y2[i] << ((i<size-1) ? "," : "\n");
        // for (int i=0; i<size; i++)
        //   ofs << z2[i] << ((i<size-1) ? "," : "\n");
        // for (int i=0; i<size; i++)
        //   ofs << norm_smooth[i] << ((i<size-1) ? "," : "");
        // ofs.close();

         // Variables
        bool  dip     = false; // Detection in progress
        float start   = 0.f;
        float end     = 0.f;
        float val     = 0.f;
        // std::vector<Detection> array;

        // Go through the values
        for (int i=0; i<size; ++i) {

          // If the current point potentially belong to a jump
          if (norm_smooth[i]<=threshold_norm_max) {

            // If we are already detecting a jump, we update the potential detection.
            // Otherwise we create a new potential detection.
            if (dip) {
              end = imu.accl.t[i];
              val = std::min(val, norm_smooth[i]);
            }
            else {
              start = imu.accl.t[i];
              end   = imu.accl.t[i];
              val   = norm_smooth[i];
            }
            dip = true;
          }
          // The potential detetion is finished, let's add it to the array
          else if (dip) {

            // Add a detection 
            if (val<=threshold_norm_min && end-start>=threshold_dur_min) {
              detections.push_back(Detection(start, end, val, "jump2"));
            }

            // The detection is finished
            dip = false;
          }
        }

        // Handle if we still have a potential detection in progress
        if (dip && val<=threshold_norm_min && end-start>=threshold_dur_min) {
          detections.push_back(Detection(start, end, val, "jump2"));
        }

        // If no jumps has been detected, no need to filter them
        if (detections.empty())
          return true;

        // Find the minimum norm value and the maximum duration
        float min_val = 100.f;
        float max_dur = 0.f;
        for (int i=0; i<detections.size(); ++i) {
          min_val = std::min(min_val, detections[i].value);
          max_dur = std::max(max_dur, detections[i].end-detections[i].start);
        }


        // // Compute relative threshold for norm and duration
        // printf("Min val = %f\n", min_val);
        // printf("Max dur = %f\n", max_dur);
        // // const float threshold_norm_min_relative = (threshold_norm_min+min_val) / 2.f;
        // // const float threshold_dur_min_relative  = (threshold_dur_min+max_dur) / 2.f;

        // // Compute the threshold on the norm
        // // min_val = 0                 -> t_n = 3
        // // min_val = threshold_dur_min -> t_n = threshold_dur_min 
        // float t_n = (threshold_norm_min-4.f) * min_val / threshold_norm_min + 4.f;
        // t_n = std::max(4.f, std::min(threshold_norm_min,  t_n) );
        // t_n = threshold_norm_min;

        // // Compute the threshold on the duration
        // // max_dur = threshold_dur_min -> t_d = threshold_dur_min
        // // max_dur = 2                 -> t_d = 1
        // float t_d = (0.5f-threshold_dur_min) * max_dur / (2.f-threshold_dur_min) + threshold_dur_min / (2.f-threshold_dur_min);
        // t_d = std::max(threshold_dur_min, std::min(0.5f,  t_d) );

        // t_n = 3.f;
        // t_d = 0.33f;

        // printf("Threshold val = %f\n", t_n);
        // printf("Threshold dur = %f\n", t_d);

        // // We keep only good enough jumps
        // for (int i=0; i<array.size(); ++i) {
        //   Detection & d = array[i];
        //   // const float score_dur = (d.end-d.start) / 2.f; // dur = 0s -> score = 0, dur=2s -> score = 1
        //   // const float score_val = (9.81-d.value) / 9.81; // val = 9.81 -> score =0, val=0 -> score = 1
        //   // d.value = score_dur + score_val;
        //   detections.push_back(d);
        //   // if (detections.back().value>t_n || detections.back().end-detections.back().start<t_d)
        //   //   detections.back().description = "REJECTED";
        // }

      }
      catch (...) {
        std::cerr << "Exception caught in jump detector" << std::endl;
        return false;
      } 

      return true;
    }


    //----------------------------------------------------------------------------------------------
    void detectFlips(const IMU & imu,
                     const Euler & euler,
                     std::vector<Detection> & detections)
    {
      //detect flips
      //print off the angles for debug
      //float count = 0;
      int   jump_state = 0;

      for (int i = 0; i < euler.num_samples; i++)
      {

        #if 0
        if(euler.t[i] > count)
        {
        std::cout << "Time: " << euler.t[i] << " Roll: " << euler.roll[i] << " Pitch: " << euler.pitch[i] << " Yaw: " << euler.yaw[i] << std::endl;
        count += 0.5;
      }
      #endif
      //-------------------------------------------------------------------------------------------
      //report if we had a flip , just need one printf hence the report flag
      if(jump_state == 0)
      {
        if(fabs(euler.pitch[i]) > 140)
        {
          //std::cout<< "We have a FLIP at " <<  euler.t[i] << std::endl;
          Detection detection(euler.t[i],euler.t[i]+1 ,"FLIP");    //HACK
          detections.push_back(detection);
          jump_state = 1;
        }
      }
      //this is how we determine we are done the flip
      if(fabs(euler.pitch[i]) < 140)
      jump_state = 0;
    } //for
  }//function ------------------------------------------------------------------------------------------




  //default settings for mountain bike
  void detectCorners(const IMU & imu,
                     const Euler & euler,
                     std::vector<Detection> & detections,
                     const int secant_length,
                     const float threshold_spin_degrees,
                     const int threshold_samples)
  {

    detectSpins(imu, euler, detections,secant_length,threshold_spin_degrees,threshold_samples);

    //rewrite the description/lable
    for (size_t i=0; i<detections.size(); i++)
    {
      detections[i].description = "corner";
    }
  }

  //------------------------------------------------------------------------------------------------
  void detectSpins(const IMU & imu,
                   const Euler & euler,
                   std::vector<Detection> & detections,
                   const int secant_length,
                   const float threshold_spin_degrees,
                   const int spin_threshold_samples)
  {

    //int   secant_length          = 100;   //1/4 second
  //  float threshold_spin_degrees = 90;   //did we spin 90 degrees with 1/4 second?
    int   spin_state = 0;
    int   spin_current = 0;;
    int   spin_count = 0;
  //  int   spin_threshold_samples = 10;

    for (int i = 0; i < (euler.num_samples-secant_length); i++)
    {

      #if 0
           if(euler.t[i] > count)
           {
             std::cout << "Time: " << euler.t[i] << " Roll: " << euler.roll[i] << " Pitch: " << euler.pitch[i] << " Yaw: " << euler.yaw[i] << std::endl;
             count += 0.5;
           }
      #endif

        float diff = fabs(euler.yaw[i]-euler.yaw[i+secant_length]);
        // do the modular arithmetic
        if( diff >180.0f)
          diff = (360.0f - diff);

         if( diff > threshold_spin_degrees)
           spin_current = 1;
         else
           spin_current = 0;

         //OK, if we where in a spin and now we are not spinning, this is the end of the event
         if( (spin_state == 1) && (spin_current == 0))
         {
           if(spin_count > spin_threshold_samples)
           {
             float corner_start = euler.t[i-spin_count];
             float corner_end   = euler.t[i];
             //std::cout << "We had a SPIN at " << euler.t[i-spin_count] << std::endl;
//             Detection detection(euler.t[i-spin_count],euler.t[i-spin_count]+1 ,"SPIN");    //HACK
             Detection detection(euler.t[i],euler.t[i]+0.5 ,"SPIN");    //HACK
             detections.push_back(detection);
           }
           spin_count = 0;  //reset the spin count (number of samples in this spin)
         }
     else if (spin_current == 1)  //we are still spinning
     {
       spin_count++;
     }
     else; //not spinning

     //store for next interation
     spin_state = spin_current;
    } //if spin
  }//function

  } // namespace generic
} // namespace imua
