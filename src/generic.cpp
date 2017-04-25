#include "generic.h"
#include "tools.h"

#include <iostream>
#include <iomanip>
#include <sstream>
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

        while(gforce_lowpass[start_idx] < start_jump_thresh )
        start_idx--;
        while(gforce_lowpass[end_idx] < end_jump_thresh )
        end_idx++;

        std::cout << "start_idx  : " << start_idx << std::endl;
        std::cout << "end_idx    : " << end_idx << std::endl;
        std::cout << "jump_start : " << jump_start << std::endl;
        std::cout << "jump_end   : " << jump_end << std::endl << std::endl;

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
              Detection detection(jump_start, jump_end, height, "jump");
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
      delete[](gforce);
      delete[](gforce_lowpass);

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


    void detectShakiness(const IMU & imu,
                         std::vector<Detection> & detections,
                         const float thresholdLow,
                         const float thresholdHigh)
    {
      // Constants
      const float first = imu.gyro.t[0];
      const float last  = imu.gyro.t[imu.gyro.size-1];
      const float freq  = imu.gyro.size / (last-first);
      const float chunk_duration = 1.f; // seconds
      const int   chunk_size     = static_cast<int>(freq*chunk_duration);
      const int   chunk_nb       = std::ceil(static_cast<float>(imu.gyro.size) / chunk_size);

      // Compute low frequencies
      std::vector<float> x_lf;
      std::vector<float> y_lf;
      std::vector<float> z_lf;
      SmoothArray(imu.gyro.x, imu.gyro.size, x_lf, 0.02f);
      SmoothArray(imu.gyro.y, imu.gyro.size, y_lf, 0.02f);
      SmoothArray(imu.gyro.z, imu.gyro.size, z_lf, 0.02f);

      // Compute high frequencies
      std::vector<float> x_hf(imu.gyro.size);
      std::vector<float> y_hf(imu.gyro.size);
      std::vector<float> z_hf(imu.gyro.size);
      for (int i=0; i<imu.gyro.size; ++i)
      {
        x_hf[i] = imu.gyro.x[i] - x_lf[i];
        y_hf[i] = imu.gyro.y[i] - y_lf[i];
        z_hf[i] = imu.gyro.z[i] - z_lf[i];
      }

      // Compute the shakiness value for each chunk
      std::vector<int>   chunk_value(chunk_nb);
      std::vector<float> chunk_start(chunk_nb);
      std::vector<float> chunk_end(chunk_nb);
      for (int i=0; i<chunk_nb; ++i)
      {

        // Index where the chunk starts and ends
        const int start = i * chunk_size;
        const int end   = std::min(start+chunk_size-1, imu.gyro.size-1);

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

        // // Debug string
        // std::stringstream ss;
        // ss << std::fixed << std::setprecision(3);
        // ss << "variance ";
        // ss << "x= " << std::setw(6) << var_x << " ";
        // ss << "y= " << std::setw(6) << var_y << " ";
        // ss << "z= " << std::setw(6) << var_z << " ";
        // ss << "norm= " << std::setw(6) << norm;
        // Detection detection(imu.gyro.t[start], imu.gyro.t[end], ss.str());
        // detections.push_back(detection);

        // Deduce the chunk value
        if (norm<=thresholdLow)
          chunk_value[i] = 0;
        else if (norm<=thresholdHigh)
          chunk_value[i] = 1;
        else
          chunk_value[i] = 2;

        // Set when the chunk starts and ends
        chunk_start[i] = imu.gyro.t[start];
        chunk_end[i]   = imu.gyro.t[end];
      }

      // Now deduce detections
      int previous_val = 0;
      for (int i=0; i<chunk_nb; ++i)
      {
        // If the shakiness mode is the same as before, we update the detection's end
        const int current_val = chunk_value[i];
        if (current_val==previous_val) {
          if (current_val>0) {
            detections.back().end = chunk_end[i];
          }
        }
        // We change the mode of detection
        else {
          if (current_val==1) {
            Detection detection(chunk_start[i], chunk_end[i], 1., "shaky medium");
            detections.push_back(detection);
          }
          else if (current_val==2) {
            Detection detection(chunk_start[i], chunk_end[i], 2., "shaky strong");
            detections.push_back(detection);
          }
        }

        // Update the previous detection for next chunk
        previous_val = current_val;
      }
    }


    void detectPans(const IMU & imu,
                    std::vector<Detection> & leftPans,
                    std::vector<Detection> & rightPans)
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

      // Detection properties
      float start  = 0.f;
      float end    = 0.f;

      // Detect left pans
      bool dip = false;
      for (int i=0; i<imu.gyro.size; ++i) {
        if (z[i]>=Z_MIN && z[i]<=Z_MAX && std::abs(x[i])<XY_MAX && std::abs(y[i])<XY_MAX) {
          if (dip) {
            end = t[i];
          }
          else {
            dip   = true;
            start = t[i];
            end   = t[i];
          }
        }
        else {
          if (dip && end-start>=DURATION_MIN) {
            leftPans.push_back(Detection(start, end, "left pan"));
          }
          dip = false;
        }
      }
      if (dip && end-start>=DURATION_MIN) {
        leftPans.push_back(Detection(start, end, "left pan"));
      }

      // Detect right pans
      dip = false;
      for (int i=0; i<imu.gyro.size; ++i) {
        if (-z[i]>=Z_MIN && -z[i]<=Z_MAX && std::abs(x[i])<XY_MAX && std::abs(y[i])<XY_MAX) {
          if (dip) {
            end = t[i];
          }
          else {
            dip   = true;
            start = t[i];
            end   = t[i];
          }
        }
        else {
          if (dip && end-start>=DURATION_MIN) {
            rightPans.push_back(Detection(start, end, "right pan"));
          }
          dip = false;
        }
      }
      if (dip && end-start>=DURATION_MIN) {
        rightPans.push_back(Detection(start, end, "right pan"));
      }
    }


    void detectPans(const IMU & imu,
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

    }

  } // namespace generic
} // namespace imua
