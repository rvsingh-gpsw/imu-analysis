#include <iostream>
#include <generic.h>
#include <snowboard.h>
#include <types.h>




/**
 * Print a detection.
 */
std::ostream& operator<<(std::ostream& stream, const imua::Detection& detection)
{
  stream << "Type   : " << detection.description << std::endl;
  stream << "Start  : " << detection.startTime << std::endl;
  stream << "Climax : " << detection.climaxTime << std::endl;
  stream << "End    : " << detection.endTime << std::endl;
  return stream;
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



int main(int argc, char *argv[])
{

  // Parse the parameters
  std::string vertical;
  std::string path;
  if (!parseParameters(argc, argv, vertical, path))
  {
    std::cerr << "Usage: ./hello path_to_video.MP4" << std::endl;
    return EXIT_FAILURE;
  }

  // Display
  std::cout << "Vertical : " << vertical << std::endl;
  std::cout << "Path     : " << path << std::endl;

  // Detect stuffs
  if (vertical=="generic")
  {
    std::vector<imua::Detection> jumps;
    imua::generic::detectJumps(0.33f, jumps);
  }
  else if (vertical=="snowboard")
  {
    std::vector<imua::Detection> jumps;
    imua::snowboard::detectJumps(jumps);
  }
  else
  {
    std::cerr << "Error: Invalid vertical" << std::endl;
    return EXIT_FAILURE;
  }

  // imua::Detection d;
  // std::cout << d << std::endl;

  return EXIT_SUCCESS;
}
