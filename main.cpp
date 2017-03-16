#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <iomanip>

#include <math.h>
#include <generic.h>
#include <snowboard.h>
#include <surfing.h>
#include <gp5imu_Madgwick.h>
#include <types.h>

extern "C" {
  #include "gpmf_parser.h"
}


/**
 * Parse the input parameters.
 */
bool parseParameters(int argc, char *argv[], std::string & vertical, std::string &path)
{
  // Check the number of parameters
  if (argc!=3)
  {
    return false;
  }

  // Get the vertical (lowercase string)
  vertical = argv[1];
  std::transform(vertical.begin(), vertical.end(), vertical.begin(), ::tolower);

  // Get the path to the video file
  path = argv[2];

  return true;
}


/**
 * Compare function for 2 detections.
 */
bool compare(const imua::Detection & a, const imua::Detection & b)
{
    return a.start < b.start;
}


/**
 * Display the detections.
 */
void displayDetections(const std::string & vertical, const std::vector<imua::Detection> & detections)
{
  for (size_t i=0; i<detections.size(); ++i)
  {
    const imua::Detection & det = detections[i];
    // std::cout << "  " << vertical << " " << det.description;
    // std::cout << " for " << det.end-det.start << " s";;
    // std::cout << " (" << det.start << " -> " << det.end << ")";
    // std::cout << " with value " << det.value << std::endl;

    std::cout << "  " << vertical << " " << det.description;
    std::cout << " for " << std::fixed;
    std::cout << std::setprecision(1) << std::setw(6) << det.end-det.start << " s : ";;
    std::cout << std::setprecision(1) << std::setw(6) << det.start << " -> ";
    std::cout << std::setprecision(1) << std::setw(6) << det.end << "";
    std::cout << std::endl;
  }
}


/**
 * Convert a time as a string (for subtitle files).
 */
std::string timeAsString(const float time)
{
    std::stringstream ss;

    float     t  = time;
    const int h  = static_cast<int>(t/3600.); t -= h * 3600.;
    const int m  = static_cast<int>(t/60.);   t -= m * 60.;
    const int s  = static_cast<int>(t);       t -= s;
    const int ms = static_cast<int>(t * 1000);

    ss << std::setfill('0') << std::setw(2) << h << ":";
    ss << std::setfill('0') << std::setw(2) << m << ":";
    ss << std::setfill('0') << std::setw(2) << s << ",";
    ss << std::setfill('0') << std::setw(3) << ms;

    return ss.str();
}


/**
 * Eport the detections as subtitles.
 */
void exportSubtitles(const std::string & path, const std::string & vertical, const std::vector<imua::Detection> & detections)
{
  std::ofstream srtFile;
  srtFile.open(path.c_str());
  if (srtFile.is_open())
  {
      for (int i=0; i<detections.size(); ++i)
      {
          if (detections[i].start>detections[i].end)
              std::cout << "Error: Invalid detection" << std::endl;

          srtFile << i+1 << std::endl;
          srtFile << timeAsString(detections[i].start) << " --> " << timeAsString(detections[i].end) << std::endl;
          srtFile << vertical << " : " << detections[i].description << " for " << detections[i].end-detections[i].start << " s" << std::endl;
          srtFile << std::endl;
      }
      srtFile.close();
  }
  else
  {
      std::cout << "Error : Cannot open the SRT file" << std::endl;
  }
}


/**
 * Extract the IMU data and correct the number of samples.
 */
bool ReadImuData(const std::string & path, IMU_t & imu)
{
    // Path as C string
    char *cstr = new char[path.length() + 1];
    strcpy(cstr, path.c_str());

    // Extract the IMU data using Kevin's code
    if (ExtractIMU(cstr, &imu) != 0)
    {
        return false;
    }

    // Find the last valid accelerometer index and update the number of samples
    int lastIdx = 0;
    for (int i=1; i<imu.accl.num_samples; ++i)
    {
        const float prev = imu.accl.t[i-1];
        const float curr = imu.accl.t[i];
        if (curr<=prev || curr>(prev+1.))
            break;
        lastIdx = i;
    }
    imu.accl.num_samples = lastIdx+1;

    // Find the last valid gyroscope index and update the number of samples
    lastIdx = 0;
    for (int i=1; i<imu.gyro.num_samples; ++i)
    {
        const float prev = imu.gyro.t[i-1];
        const float curr = imu.gyro.t[i];
        if (curr<=prev || curr>(prev+1.))
            break;
        lastIdx = i;
    }
    imu.gyro.num_samples = lastIdx+1;

    return true;
}


/**
 * Copy the IMU data from the IMU structure define in gpmf_parser.h into the imua::IMU
 * structure define in types.h. This way, the library is not responsible to deallocates
 * the memory contained in the float *. The client is responsible for it.
 * It allows also to not display the 'biais' variables. This is not optimal...
 */
void copyImuData(const IMU_t & imuIn, imua::IMU & imuOut)
{
  imuOut.gyro.samplingRate = imuIn.gyro.sampling_rate;
  imuOut.gyro.size         = imuIn.gyro.num_samples;
  imuOut.gyro.t            = imuIn.gyro.t;
  imuOut.gyro.x            = imuIn.gyro.x;
  imuOut.gyro.y            = imuIn.gyro.y;
  imuOut.gyro.z            = imuIn.gyro.z;

  imuOut.accl.samplingRate = imuIn.accl.sampling_rate;
  imuOut.accl.size         = imuIn.accl.num_samples;
  imuOut.accl.t            = imuIn.accl.t;
  imuOut.accl.x            = imuIn.accl.x;
  imuOut.accl.y            = imuIn.accl.y;
  imuOut.accl.z            = imuIn.accl.z;
}


int main(int argc, char *argv[])
{

  // Parse the parameters
  std::string vertical;
  std::string path;
  if (!parseParameters(argc, argv, vertical, path))
  {
    std::cerr << "Usage: ./detection vertical path_to_video.MP4" << std::endl;
    return EXIT_FAILURE;
  }

  // Access the IMU data
  IMU_t imu_c;
  if (!ReadImuData(path, imu_c))
  {
      std::cerr << "Error: Unable to get IMU data." << std::endl;
      return EXIT_FAILURE;
  }

  // Copy the IMU data in the lib structure
  imua::IMU imu;
  copyImuData(imu_c, imu);

  // Container that will contain all the detections
  std::vector<imua::Detection> detections;

  // Detect stuffs
  if (vertical=="generic")
  {
    std::vector<imua::Detection> jumps;
    imua::generic::detectJumps(imu, jumps);
    std::copy(jumps.begin(), jumps.end(), back_inserter(detections));
  }
  else if (vertical=="surfing")
  {
    std::vector<imua::Detection> surfs;
    imua::surfing::detectSurfing(imu, surfs, 5);
    std::copy(surfs.begin(), surfs.end(), back_inserter(detections));
  }
  else if (vertical=="euler")
  {
   Euler_t euler;
   getEulerAngles(imu, euler);   //need to deallocate memory

   //print off the angles for debug
   float count = 0;
   int   jump_state = 0;
   int   secant_length = 100;   //1/4 second
   float threshold_spin = 90;   //did we spin 90 degrees with 1/4 second?
   int   spin_state = 0;
   int   spin_current = 0;;
   int   spin_count = 0;
   int   spin_threshold_samples = 10;

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
          std::cout<< "We have a FLIP at " <<  euler.t[i] << std::endl;
          jump_state = 1;
        }
      }
      //this is how we determine we are done the flip
      if(fabs(euler.pitch[i]) < 140)
      jump_state = 0;
 }


     //---------------------------------------------------------------------------------------------
     //spin time, look at yaw
     //if( i < (euler.num_samples-secant_length))   //make sure we have enough samples
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

          if( diff > threshold_spin)
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
              std::cout << "We had a SPIN at " << euler.t[i-spin_count] << std::endl;
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

  //}//loop


  }
  else if (vertical=="snowboard")
  {
    std::vector<imua::Detection> jumps;
    imua::snowboard::detectJumps(imu, jumps);
    std::copy(jumps.begin(), jumps.end(), back_inserter(detections));

    std::vector<imua::Detection> threesixsty;
    imua::snowboard::detect360(imu, threesixsty);
    std::copy(threesixsty.begin(), threesixsty.end(), back_inserter(detections));
  }
  else
  {
    std::cerr << "Error: Invalid vertical." << std::endl;
    std::cerr << "List : generic, snowboard" << std::endl;
    return EXIT_FAILURE;
  }

  // Sort the detections
  std::sort(detections.begin(), detections.end(), compare);

  // Export the detection as subtitles
  std::string srtPath = path.substr(0, path.length()-3) + "srt";
  exportSubtitles(srtPath, vertical, detections);

  // // Display information and the detections
  // std::cout << "Vertical       : " << vertical << std::endl;
  // std::cout << "Video path     : " << path << std::endl;
  // std::cout << "Subtitles path : " << srtPath << std::endl;
  // std::cout << "Gyro #         : " << imu.gyro.size << std::endl;
  // std::cout << "Accelero #     : " << imu.accl.size << std::endl;
  // std::cout << "Detections #   : " << detections.size() << std::endl;
  displayDetections(vertical, detections);

  // Deallocate the memory
  DeAllocateImu(&imu_c);

  return EXIT_SUCCESS;
}
