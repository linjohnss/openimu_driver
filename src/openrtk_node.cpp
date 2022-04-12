#include "driver.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "openrtk_node");
    ros::NodeHandle handler;
    ros::Publisher imu_pub = handler.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Rate loop_rate(100);
    serial_port_bringup(1);
    rtkDataPointer result;
    result = (rtkDataPointer) malloc(sizeof(rtkDataPointer *));

    while(ros::ok()) {
        int8_t *data = launch_driver_8(HEADER, PACKET_TYPE_RTK);
        if (data) {
            parse_data_rtk(&(*data), &(*result));
            sensor_msgs::Imu imu;
            float t = result->GPS_TimeOfWeek + (float)result->GPS_Week*604800;
            uint32_t sec = (uint32_t)t;
            uint32_t nsec = (t - (float)sec) * 1000000000;
            imu.header.stamp = ros::Time::now();
            // printf("%f\n", t);
            // imu.header.stamp = ros::Time(t);
            imu.header.frame_id = "base_link";
            imu.linear_acceleration.x = result->accx;
            imu.linear_acceleration.y = result->accy;
            imu.linear_acceleration.z = result->accz;
            imu.angular_velocity.x = result->gyrox;
            imu.angular_velocity.y = result->gyroy;
            imu.angular_velocity.z = result->gyroz;

            imu_pub.publish(imu);
            memset(result, 0, sizeof(result));
        }

        ros::spinOnce();
        loop_rate.sleep();
        // free(data);
    }
    free(result);
    return 0;
}