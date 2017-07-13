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
              detections.push_back(Detection(start, end, val, "jump"));
            }

            // The detection is finished
            dip = false;
          }
        }

        // Handle if we still have a potential detection in progress
        if (dip && val<=threshold_norm_min && end-start>=threshold_dur_min) {
          detections.push_back(Detection(start, end, val, "jump"));
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


    bool detectShakiness(const IMU              & imu,
                         std::vector<Detection> & detections,
                         const float              thresholdLow,
                         const float              thresholdHigh)
    {
      // Sanity check
      if (imu.gyro.size<=0 || imu.gyro.samplingRate<1.f) {
          return false;
      }

      // Constants
      const int   size           = imu.gyro.size;
      const float window_duration = 1.f; // seconds
      const int   window_size     = static_cast<int>(imu.gyro.samplingRate * window_duration);
      const int   window_nb       = std::ceil(static_cast<float>(size) / window_size);

      // Sanity check
      if (window_size<1 || window_nb<1) {
          return false;
      }

      // This part allocate memory and may throw an exception. Let's protect it with a try-catch.
      try {

          // Compute low frequencies
          std::vector<float> x_lf(size);
          std::vector<float> y_lf(size);
          std::vector<float> z_lf(size);
          SmoothArrayStream(imu.gyro.x, size, &x_lf[0], 0.02f);
          SmoothArrayStream(imu.gyro.y, size, &y_lf[0], 0.02f);
          SmoothArrayStream(imu.gyro.z, size, &z_lf[0], 0.02f);

          // Compute high frequencies
          std::vector<float> x_hf(size);
          std::vector<float> y_hf(size);
          std::vector<float> z_hf(size);
          for (int i=0; i<size; ++i) {
              x_hf[i] = imu.gyro.x[i] - x_lf[i];
              y_hf[i] = imu.gyro.y[i] - y_lf[i];
              z_hf[i] = imu.gyro.z[i] - z_lf[i];
          }

          // Compute the shakiness value for each window
          std::vector<int>   window_value(window_nb);
          std::vector<float> window_start(window_nb);
          std::vector<float> window_end(window_nb);
          for (int i=0; i<window_nb; ++i) {

              // Index where the chunk starts and ends
              const int start = i * window_size;
              const int end   = std::min(start+window_size-1, size-1);

              // Compute the variances
              float var_x  = 0;
              float var_y  = 0;
              float var_z  = 0;
              int   count  = 0;
              for (int j=start; j<=end; ++j) {
                  var_x  += x_hf[j] * x_hf[j];
                  var_y  += y_hf[j] * y_hf[j];
                  var_z  += z_hf[j] * z_hf[j];
                  count++;
              }
              var_x /= count;
              var_y /= count;
              var_z /= count;

              // Compute the norm of the standard deviation
              const float norm = std::sqrt(var_x + var_y + var_z);

              // Deduce the chunk value
              if (norm<=thresholdLow)
                  window_value[i] = 0; // Not shaky
              else if (norm<=thresholdHigh)
                  window_value[i] = 1; // Shaky medium
              else
                  window_value[i] = 2; // Shaky strong

              // Set when the chunk starts and ends
              window_start[i] = imu.gyro.t[start];
              window_end[i]   = imu.gyro.t[end];
          }

          // Now deduce detections
          int previous_val = 0;
          for (int i=0; i<window_nb; ++i)
          {
              // If the shakiness mode is the same as before, we update the detection's end
              const int current_val = window_value[i];
              if (current_val==previous_val) {
                  if (current_val>0) {
                      detections.back().end = window_end[i];
                  }
              }
              // We change the mode of detection
              else {
                  if (current_val==1) {
                      Detection detection(window_start[i], window_end[i], 1., "shaky medium");
                      detections.push_back(detection);
                  }
                  else if (current_val==2) {
                      Detection detection(window_start[i], window_end[i], 2., "shaky strong");
                      detections.push_back(detection);
                  }
              }

              // Update the previous detection for next chunk
              previous_val = current_val;
          }

      }
      catch (...) {
        std::cerr << "Exception caught in jump detector" << std::endl;
        return false;
      } 

      return true;
    }


    bool detectPans(const IMU & imu,
                    std::vector<Detection> & pans)
    {

        // Parameters
        const float Z_MIN        = 0.4f;
        const float Z_MAX        = 1.f;
        const float XY_MAX       = 0.2f;
        const float DURATION_MIN = 2.f;

        // To make code lighter
        const float * x = imu.gyro.x;
        const float * y = imu.gyro.y;
        const float * z = imu.gyro.z;
        const float * t = imu.gyro.t;

        // Variable for the detections
        enum DetectionType {none, left, right};
        DetectionType previous = none;
        float         start    = 0.f;
        float         end      = 0.f;

        // Go through all gyroscopic values
        for (int i=0; i<imu.gyro.size; ++i) {

            // Get the type of pan
            const bool is_left_pan  = z[i]>=Z_MIN  && z[i]<=Z_MAX;
            const bool is_right_pan = z[i]<=-Z_MIN && z[i]>=-Z_MAX;
            const bool is_not_noisy = std::max(std::abs(x[i]), std::abs(y[i])) <= XY_MAX;
            DetectionType current = none;
            if (is_left_pan && is_not_noisy)
                current = left;
            else if (is_right_pan && is_not_noisy)
                current = right;

            // We change the type of pan
            if (current!=previous) {

                // Write the previous pan if necessary
                if (previous!=none && end-start>=DURATION_MIN) {
                    const std::string description = (previous==left) ? "pan left" : "pan right";
                    const float       value       = (previous==left) ? 1 : 2;
                    pans.push_back(Detection(start, end, value, description));
                }

                // Create a new type of pan
                if (current!=none) {
                    start = t[i];
                    end   = t[i];
                }
            }
            // The previous pan continues, we need to update its ending (eventually)
            else if (current!=none) {
                end = t[i];
            }

            // Udate the type of the previous pan
            previous = current;
        }

        // Write the current pan if needed
        if (previous!=none && end-start>=DURATION_MIN) {
          const std::string description = (previous==left) ? "pan left" : "pan right";
          const float       value       = (previous==left) ? 1 : 2;
          pans.push_back(Detection(start, end, value, description));
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
