#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "imu_uwb_fusion.h"

#include "imu_uwb_fusion/UwbMsg.h"


class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        // 퍼블리쉬 할 토픽 선언
        pub_ = n_.advertise<imu_uwb_fusion::UwbMsg>("/uwb_filtered", 1000);

        // 서브스크라이브 할 토픽 선언
        sub_ = n_.subscribe("/uwb", 1, &SubscribeAndPublish::callback, this);
    }

    void callback(const imu_uwb_fusion::UwbMsg& input)
    {
        imu_uwb_fusion::UwbMsg output;
        uwb_vec.push_back(input);
        output = input;
        
        sum_x += input.pos_x;
        sum_y += input.pos_y;
        sum_z += input.pos_z;

        if (cnt >= length)
        {
            sum_x -= uwb_vec[cnt - length].pos_x;
            sum_y -= uwb_vec[cnt - length].pos_y;
            sum_z -= uwb_vec[cnt - length].pos_z;

        }
        if (cnt >= length-1)
        {
            // cout << "Your SMA x: " << (sum_x / (double) length) << endl;
            output.pos_x = (sum_x / (double) length);
            // cout << "Your SMA y: " << (sum_y / (double) length) << endl;
            output.pos_y = (sum_y / (double) length);
            // cout << "Your SMA z: " << (sum_z / (double) length) << endl;
            output.pos_z = (sum_z / (double) length);

            if (hzNum == 3)
            {
                pub_.publish(output);
                hzNum = 0;
            }
        hzNum++;
        }
        cnt++;


    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    double length = 30;
    double sum_x, sum_y, sum_z = 0;
    int cnt = 0;
    std::vector<imu_uwb_fusion::UwbMsg> uwb_vec;
    int hzNum = 0;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscribe_and_publish");
    SubscribeAndPublish MyNode;
    ros::spin();
    return 0;
}