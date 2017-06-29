#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <iomanip>
#include <math.h>

#include <types.h>
#include <generic.h>
#include <snowboarding.h>
#include <skateboarding.h>
#include <mountainbiking.h>
#include <surfing.h>
#include <gp5imu_Madgwick.h>

#include <cstring>

#include <chrono>

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
    // std::cout << "  " << vertical << "->" << det.description;
    // std::cout << " for " << std::fixed;
    // std::cout << std::setprecision(1) << std::setw(6) << det.end-det.start << " s : ";;
    // std::cout << std::setprecision(1) << std::setw(6) << det.start << " -> ";
    // std::cout << std::setprecision(1) << std::setw(6) << det.end << "";
    // std::cout << std::endl;


    // std::cout << std::setw(20) << std::left << vertical;
    // std::cout << std::setw(20) << det.description;
    // std::cout << " for " << std::fixed;
    // std::cout << std::setprecision(1) << std::setw(6) << det.end-det.start << " s : ";;
    // std::cout << std::setprecision(1) << std::setw(6) << det.start << " -> ";
    // std::cout << std::setprecision(1) << std::setw(6) << det.end << "";
    // std::cout << std::endl;

    // printf("%-15s %-10s start : %6.1f end : %6.1f duration : %6.1f\n", vertical.c_str(), det.description.c_str(), det.start, det.end, det.end-det.start);
    printf("%-15s %-10s %f %7.2f -> %7.2f = %7.2f seconds\n", vertical.c_str(), det.description.c_str(), det.value, det.start, det.end, det.end-det.start);
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
          srtFile << std::fixed;
          srtFile << "<font color=\"#00FF00\">";
          srtFile << detections[i].description;
          srtFile << " t= " << std::setprecision(2) << detections[i].end-detections[i].start << " s ";
          srtFile << " g= " << std::setprecision(2) << detections[i].value;
          srtFile << " r= " << (detections[i].end-detections[i].start) / detections[i].value;
          srtFile << "</font>" << std::endl;
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
bool readImuData(const std::string & path, IMU_t & imu)
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
  if (!readImuData(path, imu_c))
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
  else if (vertical=="snowboarding")
  {
    std::vector<imua::Detection> jumps;
    imua::snowboarding::detectJumps(imu, jumps);
    std::copy(jumps.begin(), jumps.end(), back_inserter(detections));
  }
  else if (vertical=="skateboarding")
  {
    std::vector<imua::Detection> jumps;
    imua::skateboarding::detectJumps(imu, jumps);
    std::copy(jumps.begin(), jumps.end(), back_inserter(detections));
  }
  else if (vertical=="mountainbiking")
  {
   imua::Euler euler;
   getEulerAngles(imu, euler, 1);   //need to deallocate memory

   std::vector<imua::Detection> jumps;
   imua::mountainbiking::detectJumps(imu, jumps);
   std::copy(jumps.begin(), jumps.end(), back_inserter(detections));

   std::vector<imua::Detection> flips;
   imua::mountainbiking::detectFlips(imu, euler, flips);
   std::copy(flips.begin(), flips.end(), back_inserter(detections));

   std::vector<imua::Detection> corners;
   imua::mountainbiking::detectCorners(imu, euler, corners);
   std::copy(corners.begin(), corners.end(), back_inserter(detections));

   //cheesy deallocate
   if(euler.t != NULL)     delete[](euler.t);
   if(euler.roll  != NULL) delete[](euler.roll);
   if(euler.pitch != NULL) delete[](euler.pitch);
   if(euler.yaw   != NULL) delete[](euler.yaw);

  }
  else if (vertical=="euler")
  {
   imua::Euler euler;
   getEulerAngles(imu, euler, 1);   //need to deallocate memory

   std::vector<imua::Detection> flips;
   imua::generic::detectFlips(imu, euler, flips);
   std::copy(flips.begin(), flips.end(), back_inserter(detections));

   std::vector<imua::Detection> spins;
   imua::generic::detectSpins(imu, euler, spins);
   std::copy(spins.begin(), spins.end(), back_inserter(detections));

   //cheesy deallocate
   if(euler.t != NULL)     delete[](euler.t);
   if(euler.roll  != NULL) delete[](euler.roll);
   if(euler.pitch != NULL) delete[](euler.pitch);
   if(euler.yaw   != NULL) delete[](euler.yaw);

  }
  else if (vertical=="vincent")
  {

    // std::vector<imua::Detection> jumps;
    // imua::generic::detectJumps(imu, jumps);
    // std::copy(jumps.begin(), jumps.end(), back_inserter(detections));

    std::vector<imua::Detection> jumps2;
    imua::generic::detectJumps2(imu, jumps2);
    std::copy(jumps2.begin(), jumps2.end(), back_inserter(detections));
  }
  else
  {
    std::cerr << "Error: Invalid vertical." << std::endl;
    std::cerr << "List : generic, snowboarding, surfing, mountainbiking, euler" << std::endl;
    return EXIT_FAILURE;
  }

  // Sort the detections
  std::sort(detections.begin(), detections.end(), compare);

  // Export the detection as subtitles
  std::string srtPath = path.substr(0, path.length()-3) + "srt";
  exportSubtitles(srtPath, vertical, detections);

  // Display information and the detections
  std::cout << "Vertical       : " << vertical << std::endl;
  std::cout << "Video path     : " << path << std::endl;
  std::cout << "Subtitles path : " << srtPath << std::endl;
  std::cout << "Gyro #         : " << imu.gyro.size << std::endl;
  std::cout << "Accelero #     : " << imu.accl.size << std::endl;
  std::cout << "Detections #   : " << detections.size() << std::endl;
  displayDetections(vertical, detections);

  // Deallocate the memory
  DeAllocateImu(&imu_c);

  return EXIT_SUCCESS;
}
