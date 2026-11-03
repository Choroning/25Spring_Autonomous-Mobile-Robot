// Node 2

// [CMakeLists.txt 파일에 아래 내용 추가]
// add_executable(controller src/controller.cpp)
// target_link_libraries(controller
//    ${catkin_LIBRARIES}
// )

#include "ros/ros.h"
#include "std_msgs/String.h"

void SensorCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("hello_dsc_check: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("hello_dsc_test", 1000, SensorCallback);
    ros::spin();

    return 0;
}