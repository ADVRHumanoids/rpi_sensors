#include "video_decoder.h"

#include <ros/ros.h>

using namespace std::string_literals;

// to stream, use the following command (or similar)
// ffmpeg -f v4l2 -i /dev/video0 -vcodec libx264 -preset ultrafast -tune zerolatency -g 25 -b:v 900k -f h264 tcp://:8080?listen=1

int main(int argc, char **argv)
{
    // ros
    ros::init(argc, argv, "udp_cam_receiver");
    ros::NodeHandle nhpr("~"), nh;
    ros::Publisher pub = nhpr.advertise<sensor_msgs::Image>("image", 1);

    while(ros::ok()) try
    {
        // decoder
        // depending on the protocol, this might wait until headers are received (udp),
        // or try to initiate a connection (tcp)
        rpi_sensors::VideoDecoder decoder(nhpr.param("url", "tcp://10.240.23.65:8080"s),
                                          nhpr.param("format", "h264"s));

        using decode_ret_t = rpi_sensors::VideoDecoder::decode_ret_t;

        // loop
        sensor_msgs::Image img;

        // rate limit loop
        ros::Rate rate(1./100.);  // max 100 fps

        for(;;)
        {
            // wait for frame and decode it
            auto ret = decoder.decode(img);

            // got frame
            if(ret == decode_ret_t::got_frame)
            {
                img.header.frame_id = nhpr.param("frame_id", "cam"s);
                pub.publish(img);
            }

            // should exit
            if(ret == decode_ret_t::end_of_stream)
            {
                ROS_INFO("end of stream, exiting..");
                break;
            }

            rate.sleep();
        }

    }
    catch(std::exception& e)
    {
        ROS_ERROR("got exception: %s", e.what());
        ros::Duration(1.0).sleep();
    }
}
