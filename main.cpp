#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <generic.h>
#include <snowboard.h>
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
bool compare(imua::Detection & a, imua::Detection & b)
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
    std::cout << "  " << vertical << " " << det.description;
    std::cout << " for " << det.end-det.start << " s (";
    std::cout << det.start << " -> " << det.end << ")" << std::endl;
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
bool ReadImuData(const std::string & path, imua::IMU & imu)
{
    // Path as C string
    char *cstr = new char[path.length() + 1];
    strcpy(cstr, path.c_str());

    // Extract the IMU data using Kevin's code
    IMU_t imu_c;
    if (ExtractIMU(cstr, &imu_c) != 0)
    {
        return false;
    }

    // Find the last valid accelerometer index and update the number of samples
    int lastIdx = 0;
    for (int i=1; i<imu_c.accl.num_samples; ++i)
    {
        const float prev = imu_c.accl.t[i-1];
        const float curr = imu_c.accl.t[i];
        if (curr<=prev || curr>(prev+1.))
            break;
        lastIdx = i;
    }
    imu_c.accl.num_samples = lastIdx+1;

    // Find the last valid gyroscope index and update the number of samples
    lastIdx = 0;
    for (int i=1; i<imu_c.gyro.num_samples; ++i)
    {
        const float prev = imu_c.gyro.t[i-1];
        const float curr = imu_c.gyro.t[i];
        if (curr<=prev || curr>(prev+1.))
            break;
        lastIdx = i;
    }
    imu_c.gyro.num_samples = lastIdx+1;

    // Copy one structure to the other one
    imu.gyro.samplingRate = imu_c.gyro.sampling_rate;
    imu.gyro.size         = imu_c.gyro.num_samples;
    imu.gyro.t            = imu_c.gyro.t;
    imu.gyro.x            = imu_c.gyro.x;
    imu.gyro.y            = imu_c.gyro.y;
    imu.gyro.z            = imu_c.gyro.z;
    imu.accl.samplingRate = imu_c.accl.sampling_rate;
    imu.accl.size         = imu_c.accl.num_samples;
    imu.accl.t            = imu_c.accl.t;
    imu.accl.x            = imu_c.accl.x;
    imu.accl.y            = imu_c.accl.y;
    imu.accl.z            = imu_c.accl.z;

    return true;
}


int main(int argc, char *argv[])
{

  // Parse the parameters
  std::string vertical;
  std::string path;
  if (!parseParameters(argc, argv, vertical, path))
  {
    std::cerr << "Usage: ./hello vertical path_to_video.MP4" << std::endl;
    return EXIT_FAILURE;
  }

  // Access the IMU data
  imua::IMU imu;
  if (!ReadImuData(path, imu))
  {
      std::cerr << "Error: Unable to get IMU data." << std::endl;
      return EXIT_FAILURE;
  }

  // Container that will contain all the detections
  std::vector<imua::Detection> detections;

  // Detect stuffs
  if (vertical=="generic")
  {
    // std::vector<imua::Detection> jumps;
    // imua::generic::detectJumps(jumps);

    std::vector<imua::Detection> jumps2;
    imua::generic::detectJumps(imu, jumps2);

    // std::copy(jumps.begin(), jumps.end(), back_inserter(detections));
    std::copy(jumps2.begin(), jumps2.end(), back_inserter(detections));
  }
  else if (vertical=="snowboard")
  {
    std::vector<imua::Detection> jumps;
    imua::snowboard::detectJumps(jumps);

    std::vector<imua::Detection> threesixsty;
    imua::snowboard::detect360(threesixsty);

    // Gather all detections
    std::copy(jumps.begin(), jumps.end(), back_inserter(detections));
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

  // Display information and the detections
  std::cout << "Vertical       : " << vertical << std::endl;
  std::cout << "Video path     : " << path << std::endl;
  std::cout << "Subtitles path : " << srtPath << std::endl;
  std::cout << "Gyro #         : " << imu.gyro.size << std::endl;
  std::cout << "Accelero #     : " << imu.accl.size << std::endl;
  std::cout << "Detections #   : " << detections.size() << std::endl;
  displayDetections(vertical, detections);

  return EXIT_SUCCESS;
}
