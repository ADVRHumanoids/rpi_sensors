#include "video_decoder.h"

extern "C"
{
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>

#include <libavutil/imgutils.h>
#include "libavcodec/avcodec.h"
#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/mathematics.h"
#include "libavutil/samplefmt.h"
#include "libavformat/avformat.h"
}

using namespace rpi_sensors;

class VideoDecoder::Impl
{

public:

    Impl(std::string url, std::string format);

    decode_ret_t decode(sensor_msgs::Image& img);

    ~Impl();

private:

    AVFormatContext *fmt_ctx = NULL;
    AVCodecContext *video_dec_ctx = NULL, *audio_dec_ctx;
    AVStream *video_stream = NULL, *audio_stream = NULL;
    const char *src_filename;
    int video_dst_bufsize;
    int video_stream_idx = -1, audio_stream_idx = -1;
    AVFrame *frame = NULL;
    int video_frame_count = 0;
    SwsContext *sws_ctx = 0;

    AVCodecContext *c = NULL;
    AVPacket avpkt;

    int open_codec_context(int *stream_idx, AVFormatContext *fmt_ctx, enum AVMediaType type);



};

VideoDecoder::VideoDecoder(std::string url, std::string format)
{
    i = std::make_unique<Impl>(url, format);
}

VideoDecoder::decode_ret_t VideoDecoder::decode(sensor_msgs::Image &img)
{
    return i->decode(img);
}

VideoDecoder::~VideoDecoder()
{

}

VideoDecoder::Impl::Impl(std::string url, std::string format)
{
    // set input url
    src_filename = url.c_str();

    // open input stream, and allocate format context
    AVInputFormat * fmt = av_find_input_format(format.c_str());
    if (avformat_open_input(&fmt_ctx, src_filename, fmt, NULL) < 0)
    {
        throw std::runtime_error("could not open input at " + url);
    }

    printf("avformat_open_input SUCCESS \n");

    // retrieve stream information
    if(avformat_find_stream_info(fmt_ctx, NULL) < 0)
    {
        throw std::runtime_error("could not find stream information");
    }

    printf("avformat_find_stream_info SUCCESS \n");

    // open video context
    if(open_codec_context(&video_stream_idx, fmt_ctx, AVMEDIA_TYPE_VIDEO) < 0)
    {
        throw std::runtime_error("open_codec_context failed");
    }

    av_init_packet(&avpkt);

    frame = av_frame_alloc();
}

VideoDecoder::decode_ret_t VideoDecoder::Impl::decode(sensor_msgs::Image &img)
{
    if(av_read_frame(fmt_ctx, &avpkt) < 0)
    {
        return decode_ret_t::end_of_stream;
    }

    if (avpkt.size == 0)
    {
        return decode_ret_t::end_of_stream;
    }

    // This function might fail because of parameter set packets, just ignore and continue
    int ret = avcodec_send_packet(video_dec_ctx, &avpkt);
    if(ret < 0)
    {
        fprintf(stderr, "avcodec_send_packet ret < 0\n");
        return decode_ret_t::no_frame;
    }

    // Receive the uncompressed frame back
    ret = avcodec_receive_frame(video_dec_ctx, frame);
    if(ret < 0)
    {
        // Sometimes we cannot get a new frame, continue in this case
        if(ret == AVERROR(EAGAIN))
        {
            return decode_ret_t::no_frame;
        }

        fprintf(stderr, "avcodec_receive_frame ret < 0\n");
        return decode_ret_t::end_of_stream;
    }

    // Calculate output buffer requirements
    uint32_t image_buffer_size = av_image_get_buffer_size(AVPixelFormat(frame->format),
                                                          frame->width,
                                                          frame->height,
                                                          1);
    // unused
    static_cast<void>(image_buffer_size);

    // save to input image
    img.encoding = "rgb8";
    img.data.resize(frame->width * frame->height * 3);
    img.step = frame->width * 3;
    img.width = frame->width;
    img.height = frame->height;
    img.header.frame_id = "cam";
    img.header.stamp = ros::Time::now(); // FIXME

    // scale and convert yuv -> rgb
    uint8_t* destData[1] = {img.data.data()};
    int linesize[1] = {(int)img.step};

    sws_ctx = sws_getCachedContext(
                sws_ctx,
                frame->width, frame->height, AVPixelFormat(frame->format),
                frame->width, frame->height, AV_PIX_FMT_RGB24,
                0, 0, 0, 0
                );

    sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height,
              destData, linesize);

    return decode_ret_t::got_frame;

}

VideoDecoder::Impl::~Impl()
{
    avcodec_close(video_dec_ctx);
    av_free(video_dec_ctx);
    av_frame_free(&frame);
}


int VideoDecoder::Impl::open_codec_context(int *stream_idx,
                                           AVFormatContext *fmt_ctx,
                                           AVMediaType type)
{

    int ret = av_find_best_stream(fmt_ctx, type, -1, -1, NULL, 0);

    if(ret < 0)
    {
        fprintf(stderr, "Could not find %s stream in input file '%s'\n",
                av_get_media_type_string(type), src_filename);
        return ret;
    }
    else
    {
        // get stream
        *stream_idx = ret;
        AVStream *st = fmt_ctx->streams[*stream_idx];

        // find decoder for the stream
        AVCodecParameters *dec_param = st->codecpar;
        AVCodec *dec = avcodec_find_decoder(dec_param->codec_id);
        if(!dec)
        {
            fprintf(stderr, "Failed to find %s codec\n",
                    av_get_media_type_string(type));
            return ret;
        }

        printf("avcodec_find_decoder SUCCESS \n");

        // allocate context for decoder
        video_dec_ctx = avcodec_alloc_context3(dec);
        avcodec_parameters_to_context(video_dec_ctx, dec_param);

        if((ret = avcodec_open2(video_dec_ctx, dec, NULL)) < 0)
        {
            fprintf(stderr, "Failed to open %s codec\n",
                    av_get_media_type_string(type));
            return ret;
        }

        printf("avcodec_open2 SUCCESS \n");
    }
    return 0;
}
