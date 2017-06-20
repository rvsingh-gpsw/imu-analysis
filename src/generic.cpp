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
                    std::vector<Detection> & detections,
                    const float threshold,
                    const float durationMin)
    {

      // Constants
      const float threshold_max = 9.f;
      const float gap_max       = durationMin / 2.f;
      const int   size          = imu.accl.size;

      // Smooth the input data
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


      // std::vector<float>::iterator result = std::min_element(std::begin(norm_smooth), std::end(norm_smooth));
      // printf("Min = %f\n", *result);

      // float threshold_min = std::min(9.81f - (9.81f-*result)*0.9f, 5.f);
      float threshold_min = threshold;
      // printf("Threshold = %f\n", threshold_min);

      // // DEBUG
      // std::ofstream ofs;
      // ofs.open("/Users/vincent/Desktop/output.csv");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << imu.accl.t[i] << ((i<norm.size()-1) ? "," : "\n");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << imu.accl.x[i] << ((i<norm.size()-1) ? "," : "\n");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << imu.accl.y[i] << ((i<norm.size()-1) ? "," : "\n");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << imu.accl.z[i] << ((i<norm.size()-1) ? "," : "\n");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << norm[i] << ((i<norm.size()-1) ? "," : "\n");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << norm_smooth[i] << ((i<norm.size()-1) ? "," : "\n");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << x2[i] << ((i<norm.size()-1) ? "," : "\n");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << y2[i] << ((i<norm.size()-1) ? "," : "\n");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << z2[i] << ((i<norm.size()-1) ? "," : "\n");
      // for (int i=0; i<norm.size(); i++)
      //   ofs << norm_smooth2[i] << ((i<norm.size()-1) ? "," : "");
      // ofs.close();

      // DEBUG
      std::ofstream ofs;
      ofs.open("/Users/vincent/Desktop/output.csv");
      for (int i=0; i<size; i++)
        ofs << imu.accl.t[i] << ((i<size-1) ? "," : "\n");
      for (int i=0; i<size; i++)
        ofs << imu.accl.x[i] << ((i<size-1) ? "," : "\n");
      for (int i=0; i<size; i++)
        ofs << imu.accl.y[i] << ((i<size-1) ? "," : "\n");
      for (int i=0; i<size; i++)
        ofs << imu.accl.z[i] << ((i<size-1) ? "," : "\n");
      for (int i=0; i<size; i++)
        ofs << x2[i] << ((i<size-1) ? "," : "\n");
      for (int i=0; i<size; i++)
        ofs << y2[i] << ((i<size-1) ? "," : "\n");
      for (int i=0; i<size; i++)
        ofs << z2[i] << ((i<size-1) ? "," : "\n");
      for (int i=0; i<size; i++)
        ofs << norm_smooth[i] << ((i<size-1) ? "," : "");
      ofs.close();


      // Variables
      bool  dip     = false; // Detection in progress
      float start   = 0.f;
      float end     = 0.f;
      float min_val = 100.f;
      std::vector<Detection> array;

      int nb_fusions = 0;


      for (int i=0; i<imu.accl.size; ++i) {

        // If the current point potentially belong to a jump
        if (norm_smooth[i]<=threshold_max) {

          // printf("coucou!\n");

          // If we are already detecting a jump, we update the potential detection.
          // Otherwise we create a new potential detection.
          if (dip) {
            end     = imu.accl.t[i];
            min_val = std::min(min_val, norm_smooth[i]);
          }
          else {
            start   = imu.accl.t[i];
            end     = imu.accl.t[i];
            min_val = norm_smooth[i];
          }
          dip = true;
        }
        // The potential detetion is finished, let's add it to the array
        else if (dip) {

          // It needs to be a jump based on its minimum value
          if (min_val<threshold_min) {

            // The potential detection reach the detection threshold so it is a
            // detection. If it's close enough to the previous detection, we can fuse
            // both detections. If it's not close enough we create a new detection.
            if (!array.empty() && std::abs(start-array.back().end)<=gap_max) {
                printf("Gap = %f\n", start-array.back().end);
                array.back().end   = end;
                array.back().value = std::min(min_val, array.back().value);
                nb_fusions++;
            }
            else {
                array.push_back(Detection(start, end, min_val, "jump2"));
            }
          }

          // The detection is finished
          dip = false;
        }
      }

      // Handle if we still have a potential detection in progress
      if (dip && min_val<threshold_min) {
        if (!array.empty() && std::abs(start-array.back().end)<=gap_max) {
                printf("Gap = %f\n", start-array.back().end);
            array.back().end   = end;
            array.back().value = std::min(min_val, array.back().value);
            nb_fusions++;
        }
        else {
            array.push_back(Detection(start, end, min_val, "jump2"));
        }
      }

      // We keep only long enough detection
      for (int i=0; i<array.size(); ++i) {
        if (array[i].end-array[i].start>=durationMin) {
          detections.push_back(array[i]);
        }
      }

      printf("Nb fusions : %d\n", nb_fusions);

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
      // Sanity check
      if (imu.gyro.size<=0 || imu.gyro.samplingRate<1.f) {
        return;
      }

      // Constants
      const float chunk_duration = 1.f; // seconds
      const int   chunk_size     = static_cast<int>(imu.gyro.samplingRate * chunk_duration);
      const int   chunk_nb       = std::ceil(static_cast<float>(imu.gyro.size) / chunk_size);

      // Sanity check
      if (chunk_size<1 || chunk_nb<1) {
        return;
      }

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
