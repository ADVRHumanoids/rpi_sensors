#ifndef VIDEO_DECODER_H
#define VIDEO_DECODER_H

#include <string>
#include <memory>
#include <sensor_msgs/Image.h>

namespace rpi_sensors {

class VideoDecoder
{

public:

    VideoDecoder(std::string url, std::string format);

    enum class decode_ret_t
    {
        got_frame = 0,
        no_frame,
        end_of_stream
    };

    decode_ret_t decode(sensor_msgs::Image& img);

    ~VideoDecoder();

private:

    class Impl;
    std::unique_ptr<Impl> i;

};


}

#endif // VIDEO_DECODER_H
