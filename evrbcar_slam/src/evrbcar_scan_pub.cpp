#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "../../robocar/src/evrbcar.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "evrbcar_scan_pub"); 
    ros::NodeHandle nh; 
    const char *server_address = "";
    unsigned short port = SCAN_UDP_PORT;
    struct sockaddr servSockAddr;
    int server_sock;
    int ioctlval = 0;
    int seq = 0;
 
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    current_time = ros::Time::now();

    ros::Rate r(30.0);

    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_th = 0.0;

    server_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    ioctlval = 1;
    ioctl(server_sock, FIONBIO, &ioctlval);
    sockaddr_init(server_address, port, &servSockAddr);
    if (bind(server_sock, &servSockAddr, sizeof(servSockAddr)) < 0) {
        perror("bind() failed.");
        exit(EXIT_FAILURE);
    }

    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("base_scan", 1000);

    while(nh.ok()){
        struct sockaddr_in clitSockAddr;
        unsigned int sockaddrLen = sizeof(clitSockAddr);
        char buffer[BUFSIZ];
        t_scan_data* scandat = (t_scan_data*)buffer;
        int i= 0;

        current_time = ros::Time::now();

        if (0 < recvfrom(server_sock, buffer, BUFSIZ, 0, (struct sockaddr *)&clitSockAddr, &sockaddrLen)){
            odom_x = DEG2RAD(scandat->odom[0]);
            odom_y = DEG2RAD(scandat->odom[1]);
            odom_th= DEG2RAD(scandat->odom[2]);

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = odom_x;
            odom_trans.transform.translation.y = odom_y;
            odom_trans.transform.translation.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th);
            odom_trans.transform.rotation = odom_quat;
            odom_broadcaster.sendTransform(odom_trans);

            if(scandat->scan){
                sensor_msgs::LaserScan msg;
                msg.ranges.resize(scandat->num);
                
                msg.header.seq = seq++;
                msg.header.stamp = current_time;
                msg.header.frame_id = "base_link";
                msg.angle_min = DEG2RAD(scandat->start_angle);
                msg.angle_max = DEG2RAD(scandat->end_angle);
                msg.angle_increment = DEG2RAD((scandat->end_angle + 360.0F - scandat->start_angle) / (scandat->num - 1));
                for(i = 0; i < scandat->num; i++){
                    msg.ranges[i] = scandat->range[i]; 
                }
                scan_pub.publish(msg);
            }
        } 

        ros::spinOnce();
        r.sleep();
    }
}
