#include "udp_socket.h"
#include <iostream>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>

int main(int argc, char **argv)
{
    // register ros node
    ros::init(argc, argv, "udp_cam_receiver");
    ros::NodeHandle nh("~");

    // udp client endpoint
    UdpClientRaw udp;

    // remote address
    int remote_port = nh.param("remote_port", 8080);
    std::string remote_addr;
    if(!nh.getParam("remote_addr", remote_addr))
    {
        ROS_ERROR("remote_port parameter is required");
        exit(1);
    }

    // bind to local address/port
    if(!udp.init(remote_addr, remote_port))
    {
        ROS_ERROR("could not initialize udp client");
        exit(1);
    }

    // set timeout for read operations
    udp.set_timeout(1.0);

    // allocate a buffer
    const int buffer_size = 65536;
    uint8_t buffer[buffer_size];

    // allocate libav data struct
    SwsContext* g_sws = 0;

    // detect decoders, create h264 decoder
    avcodec_register_all();
    av_log_set_level(AV_LOG_QUIET);

    AVCodec *decoder = avcodec_find_decoder(AV_CODEC_ID_H264);
    if(!decoder)
    {
        throw std::runtime_error("h264 decoding not supported in this build of ffmpeg");
    }

    AVCodecContext *g_codec = avcodec_alloc_context3(decoder);
    g_codec->flags |= AV_CODEC_FLAG_LOW_DELAY;
    g_codec->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
    g_codec->thread_type = 0;

    if(avcodec_open2(g_codec, decoder, 0) != 0)
        throw std::runtime_error("Could not open decoder");

    // ros stuff
    ros::Publisher g_pub = nh.advertise<sensor_msgs::Image>("image", 1);

    // start loop
    int msg_size = 0;
    for(;;)
    {
        // blocking receive
        msg_size = udp.receive(buffer, buffer_size);

        // server does not reply within 1 sec
        if(msg_size == -1)
        {
            // try to reach server
            bool srv_msg_recv = false;
            while(!srv_msg_recv)
            {
                ROS_INFO_THROTTLE(1.0,
                                  "contacting server at %s:%d..",
                                  remote_addr.c_str(), remote_port);
                // send a dummy msg for now
                uint32_t msg_cli_to_srv = 0xdeadbeef;
                udp.try_send(reinterpret_cast<uint8_t*>(&msg_cli_to_srv),
                             sizeof(msg_cli_to_srv));

                // try to receive from server
                msg_size = udp.try_receive(buffer, buffer_size);
                srv_msg_recv = msg_size > 0;
            }
        }

        // consume buffer
        for(;;)
        {
            int maybe_msg_size = udp.try_receive(buffer, buffer_size);

            if(maybe_msg_size > 0)
            {
                msg_size = maybe_msg_size;
            }
            else
            {
                break;
            }
        }

        // debug print: msg size and source address
        std::cout << "msg recv: size = "
                  << msg_size << " from "
                  << remote_addr << ":" << remote_port << " \n";

        // convert received data to avpacket
        AVPacket packet;
        av_init_packet(&packet);
        packet.data = buffer;
        packet.size = msg_size;
        packet.pts = AV_NOPTS_VALUE;
        packet.dts = AV_NOPTS_VALUE;

        // try to decode it to a frame
        AVFrame frame;
        memset(&frame, 0, sizeof(frame));

        int gotPicture;
        if(avcodec_decode_video2(g_codec, &frame, &gotPicture, &packet) < 0)
        {
            // failure
            continue;
        }

        // success
        if(gotPicture)
        {
            g_sws = sws_getCachedContext(
                        g_sws,
                        frame.width, frame.height, AV_PIX_FMT_YUV420P,
                        frame.width, frame.height, AV_PIX_FMT_RGB24,
                        0, 0, 0, 0
                        );

            sensor_msgs::ImagePtr img(new sensor_msgs::Image);

            img->encoding = "rgb8";
            img->data.resize(frame.width * frame.height * 3);
            img->step = frame.width * 3;
            img->width = frame.width;
            img->height = frame.height;
            img->header.frame_id = "cam";
            img->header.stamp = ros::Time::now(); // FIXME

            uint8_t* destData[1] = {img->data.data()};
            int linesize[1] = {(int)img->step};

            sws_scale(g_sws, frame.data, frame.linesize, 0, frame.height,
                      destData, linesize);

            g_pub.publish(img);
        }
    }

}
