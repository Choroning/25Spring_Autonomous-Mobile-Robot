// Node 1

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <unistd.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hello_dsc");

    // ROS 시스템 사용을 위한 핸들 선언
    ros::NodeHandle n;
    int count = 0;

    ros::Publisher hello_dsc_pub = n.advertise<std_msgs::String>("hello_dsc_test",1000);

    while (ros::ok())
    {
        // 메시지 선언
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;

        // 메시지에 데이터 입력
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        // 퍼블리시
        // hello_dsc_test라는 토픽 이름으로 데이터 획득 가능
        hello_dsc_pub.publish(msg);

        count++;

        usleep(1000);
    }
    return 0;
}