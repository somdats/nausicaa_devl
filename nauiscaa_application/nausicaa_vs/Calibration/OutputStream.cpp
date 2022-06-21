#include"ffmpegException.h"
#include"OutputStream.h"

#if VIDEO_STREAM


using namespace ffmpegCodec;


OutputStream::OutputStream(const char* formatName, const char* outputIPPort, const char*codecName) {

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 9, 100)
    av_register_all();
#endif
    avformat_network_init();
    int ret;
    std::string errbuf;
    AVCodecID codecID;
    if (std::string(outputIPPort).find(std::string("rtsp")) != std::string::npos)
    {
        outFormat = *av_guess_format("rtsp", NULL, NULL);
        ret = avformat_alloc_output_context2(&formatCtx, &outFormat, "rtsp", outputIPPort);
        codecID = outFormat.video_codec;
    }
    if (std::string(outputIPPort).find(std::string("udp")) != std::string::npos)
    {
        ret = avformat_alloc_output_context2(&formatCtx, nullptr, formatName, nullptr);
        codecID = AV_CODEC_ID_H264;
    }

  /*  if (std::string(formatName) == "" || std::string(codecName) == "mpeg")
        ret = avformat_alloc_output_context2(&formatCtx, nullptr, "avi", nullptr);*/

    if (ret < 0)
    {
       
       char *err = av_make_error_string((char*)(errbuf.c_str()), 124, std::abs(ret));
       std::cout << "Error:" << std::string(err) << std::endl;
       throw FFmpegException("avformat allocation incorrect- error parsing encoder type!");
    }


 /*   if (std::string(formatName) == "" || std::string(codecName) == "mpeg")
        ret = avformat_alloc_output_context2(&formatCtx, nullptr, "avi", nullptr);
    else
        ret = avformat_alloc_output_context2(&formatCtx, nullptr, formatName, nullptr);

    if (ret < 0)
    {
        throw FFmpegException("avformat allocation incorrect- error parsing encoder type!");
    }*/

    //AVCodecID codecID = formatCtx->oformat->video_codec;
   

    printf("Output context format:%d\n", formatCtx->oformat->video_codec);

  /*  if (ret < 0)
    {
        throw FFmpegException("Could not allocate output format context!");
        
    }*/

    if (!(formatCtx->oformat->flags & AVFMT_NOFILE))
    {
        int ret = avio_open2(&formatCtx->pb, outputIPPort, AVIO_FLAG_WRITE, nullptr, nullptr);
        if (ret < 0)
        {
            char* err = av_make_error_string((char*)(errbuf.c_str()), 124, std::abs(ret));
            std::cout << "Error:" << err << std::endl;
            const char* proto;
            proto = avio_find_protocol_name(outputIPPort);
            int proto_len = proto ? strlen(proto) : 0;
            if (proto_len > 0)
                std::cout << std::string(proto) << std::endl;

            throw FFmpegException("Could not open output IO context!");
           
        }
    }
   

    outCodec = const_cast<AVCodec*>(avcodec_find_encoder(codecID));
    outStream = avformat_new_stream(formatCtx, outCodec);
 

    /// initialze numFrame;
    numFrame = 0;
}

OutputStream::~OutputStream()
{
    CleanUp();
}

void OutputStream::CleanUp()
{
    avio_close(formatCtx->pb);
    avformat_free_context(formatCtx);

 }

AVStream* OutputStream ::getStream() {
    return outStream;
 }

AVFormatContext* OutputStream::getFormatContext() {
    return formatCtx;
}

void OutputStream::WriteHeader() {
    int ret = avformat_write_header(formatCtx, nullptr);
    if (ret < 0)
    {
        throw FFmpegException("Could not write header!" );
      
    }
}

size_t OutputStream::getNumFrames() const {

    return numFrame;

}

void OutputStream::writeTrailer()
{
    av_write_trailer(formatCtx);
}

void OutputStream::setExtras(uint8_t* extraData, int extraDataSize)
{
    outStream->codecpar->extradata = extraData;
    outStream->codecpar->extradata_size = extraDataSize;
}

void OutputStream::createStream( FrameProcessor& frame, AVCodecContext& cdcCntx, cv::Mat& buffData)
{
    //cv::Mat data = cv::imread("D:/CamImages/rectified_streamed_output_04052022.jpg");
  /*  cv::VideoCapture cap("udpsrc port=5000 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! decodebin ! videoconvert !  appsink",
        cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "VideoCapture-1 not opened" << std::endl;
        exit(-1);
    }*/
    /*while (true) {
        */
        frame.frameData(buffData, cdcCntx,*formatCtx, *outStream);
        ++numFrame;
        //std::cout << "Frame Number:" << numFrame << std::endl;
   /* }*/
}

AVRational OutputStream::getTimeBase() {
    return outStream->time_base;
}

void OutputStream::setStreamFrameRate(const AVRational& framerate) {
    outStream->avg_frame_rate = framerate;
}
#endif