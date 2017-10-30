#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MoveToDesiredJoints.h"
#include <sstream>


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "MoveToDesiredJoints");

  std_msgs::Float64MultiArray desiredJoints;

  // Set the number of joints
  desiredJoints.data.resize(7);

  // Initialize desired joints
  for(int k = 0; k < 7; k++)
  {
    desiredJoints.data[k] = 0.0f;
  }

  // Check if desired angles are specified with the command line
  if(argc == 8)
  {
    for(int k = 0; k < 7; k++)
    {
      std::cout << atof(argv[k+1]) <<" ";
      desiredJoints.data[k] = atof(argv[k+1])*M_PI/180.0f;

    }
  }

  std::cout << "\n ";

  ros::NodeHandle n;
  float frequency = 100.0f;


  MoveToDesiredJoints moveToDesiredJoints(n,frequency);

  if (!moveToDesiredJoints.init()) 
  {
    return -1;
  }
  else
  {
  	moveToDesiredJoints.setDesiredJoints(desiredJoints);
    moveToDesiredJoints.run();
  }

  return 0;

}