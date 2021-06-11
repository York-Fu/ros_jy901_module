#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

#include <pthread.h>

#include "uartjy901.h"

void *recv_data_thread(void *ptr)
{
  while (ros::ok())
  {
    jy901_loop();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jy901Module_node");
  ros::NodeHandle n;

  std::vector<std::string> portStrVec{"/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyUSB1"};
  int ret = 0;
  for(uint8_t i = 0; i < portStrVec.size(); i++) 
  {
    ret = jy901_init(portStrVec[i].c_str());
    if (ret == -1)
    {
      ROS_ERROR("error: jy901_init %s failed!", portStrVec[i].c_str());
      jy901_close();
    }
  }
  if(ret == -1)
    exit(EXIT_FAILURE);

  pthread_t thread_recv_data;
  pthread_create(&thread_recv_data, NULL, recv_data_thread, NULL);

  ros::Publisher jy901Data_Pub = n.advertise<std_msgs::Float64MultiArray>("/jy901Module_node/jy901Data", 1);

  std_msgs::Float64MultiArray f64MArray;
  f64MArray.data.resize(9);

  ros::Rate loopRete(200);
  while (ros::ok())
  {
    for (uint16_t i = 0; i < 3; i++)
    {
      f64MArray.data[i] = jy901_getGyro(i);
      f64MArray.data[i + 3] = jy901_getAcc(i);
      f64MArray.data[i + 6] = jy901_getEuler(i);
    }
    jy901Data_Pub.publish(f64MArray);

    loopRete.sleep();
  }
  pthread_join(thread_recv_data, NULL);
  jy901_close();

  return 0;
}
