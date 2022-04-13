#include "driver.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "openrtk_node");
    ros::NodeHandle handler;
    ros::Publisher imu_pub = handler.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Rate loop_rate(1000);
    serial_port_bringup(1);
    rtkDataPointer result= (rtkDataPointer) malloc(sizeof(rtkDataPointer *));;
    uint8_t *data;
    while(ros::ok()) {
        data = launch_driver_8(HEADER, PACKET_TYPE_RTK);
        if (data) {
            parse_data_rtk(&(data[3]), &(*result));
            sensor_msgs::Imu imu;
            float t = static_cast<float>(result->GPS_TimeOfWeek);
            uint32_t sec = result->GPS_Week*604800 + result->GPS_TimeOfWeek/1000;
            uint32_t nsec = result->GPS_TimeOfWeek - result->GPS_TimeOfWeek/1000 *1000;
            imu.header.stamp = ros::Time(sec ,nsec);
            imu.header.frame_id = "base_link";
            imu.linear_acceleration.x = result->accx;
            imu.linear_acceleration.y = result->accy;
            imu.linear_acceleration.z = result->accz;
            imu.angular_velocity.x = result->gyrox;
            imu.angular_velocity.y = result->gyroy;
            imu.angular_velocity.z = result->gyroz;

            imu_pub.publish(imu);
            memset(result, 0, sizeof(result));
            // free(result);
            free(data);
            // free(result);
        }
        
        
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    puts("free");
    // free(data);
    free(result);
    return 0;
}