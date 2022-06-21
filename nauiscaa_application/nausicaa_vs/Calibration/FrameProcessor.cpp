

#include"ffmpegException.h"
#include"FrameProcessor.h"

#if VIDEO_STREAM

using namespace ffmpegCodec;


FrameProcessor::FrameProcessor(const AVCodecContext& codecCtx, int w, int h) {

	width = w;
	height = h;
	pts = 0;
	initializeFrame(codecCtx);
	initializeSampleScaler(codecCtx);

}

FrameProcessor::FrameProcessor()
{

}

void FrameProcessor::resetDimension(int w, int h) {

	width = w;
	height = h;

}

FrameProcessor::~FrameProcessor() {
	clean();
}


void FrameProcessor::initializeFrame(const AVCodecContext& codecCtx) {

	frame = av_frame_alloc();
	int ret;
	if (!frame)
	{
		clean();
		throw FFmpegException("Error allocating a video frame\n");
	}
	
	frame->format = codecCtx.pix_fmt;
	frame->width = codecCtx.width;
	frame->height = codecCtx.height;

   ret = av_frame_get_buffer(frame, 32);

	if (ret < 0) {
		throw FFmpegException("Could not allocate the video frame data\n");
		
	}
}

void FrameProcessor::initializeSampleScaler(const AVCodecContext& codecCtx)
{
	swsContext = sws_getContext(width, height, AV_PIX_FMT_BGR24, width, height, codecCtx.pix_fmt,
		SWS_BICUBIC, nullptr, nullptr, nullptr);

	if (!swsContext)
	{
		throw FFmpegException("Could not initialize sample scaler!");
		exit(1);
	}
}

void FrameProcessor::clean() {

	if (frame != nullptr)
	{
		av_frame_free(&frame);
		frame = nullptr;
	}
	if (swsContext != nullptr)
	{
		sws_freeContext(swsContext);
		swsContext = nullptr;
	}
}

int FrameProcessor::getWidth() {
	return width;
}

int FrameProcessor::getHeight() {
	return height;
}

void FrameProcessor::writeFrame( AVCodecContext& codec_ctx,  AVFormatContext& fmt_ctx) {
	AVPacket pkt = { 0 };
	av_new_packet(&pkt, 0);
	

	int ret = avcodec_send_frame(&codec_ctx, frame);
	if (ret < 0)
	{
		throw FFmpegException("Error sending frame to codec context!");
		exit(1);
	}

	ret = avcodec_receive_packet(&codec_ctx, &pkt);
	if (ret < 0)
	{
		throw FFmpegException("Error receiving packet from codec context!");
		exit(1);
	}
	//printf("Packet size = %d", pkt.size);
	/* Write the compressed frame to the media file. */
	 //av_packet_rescale_ts(&pkt, codec_ctx.time_base, outputStream.time_base);
	//pkt.stream_index = outputStream.index;
	av_interleaved_write_frame(&fmt_ctx, &pkt);
	av_packet_unref(&pkt);
}

void FrameProcessor::frameData(const cv::Mat& data, AVCodecContext& codecCtx, AVFormatContext& fmt_ctx, AVStream& stream) {

	const int stride[] = { static_cast<int>(data.step[0]) };
	sws_scale(swsContext, &data.data, stride, 0, data.rows, frame->data, frame->linesize);
	
	//frame->pts = pts;
	frame->pts+= av_rescale_q(1, codecCtx.time_base, stream.time_base);
	writeFrame(codecCtx, fmt_ctx);

}

void FrameProcessor::TimeBase(AVRational timebase) {
	streamTimeBase.num = timebase.num;
	streamTimeBase.den = timebase.den;
}

void FrameProcessor::StreamInformation(const AVStream& stream)
{
	outputStream = stream;
}
#endif