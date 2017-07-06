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

    bool detectJumps(const IMU              & imu,
                     std::vector<Detection> & detections,
                     const float              thresholdNorm,
                     const float              durationMin) {


      // Parameters
      const float threshold_norm_min = thresholdNorm;
      const float threshold_norm_max = 9.f;
      const float threshold_dur_min  = durationMin;


      // This part allocate memory and may throw an exception. Let's protect it with a try-catch.
      try {

        // Smooth the input data
        const int   size  = imu.accl.size;
        const float sigma = imu.accl.samplingRate * 0.02f;
        std::vector<float> x2(size);
        std::vector<float> y2(size);
        std::vector<float> z2(size);
        bool result = true;
        result &= SmoothArrayBoxFilter(imu.accl.x, imu.accl.size, &x2[0], sigma);
        result &= SmoothArrayBoxFilter(imu.accl.y, imu.accl.size, &y2[0], sigma);
        result &= SmoothArrayBoxFilter(imu.accl.z, imu.accl.size, &z2[0], sigma);

        // Exit if we cannot smooth the data
        if (!result)
          return false;

        // Compute the norm
        std::vector<float> norm_smooth(size);
        if (!ComputeNorm(&x2[0], &y2[0], &z2[0], imu.accl.size, &norm_smooth[0]))
          return false;

         // Variables
        bool  dip     = false; // Detection in progress
        float start   = 0.f;
        float end     = 0.f;
        float val     = 0.f;

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
