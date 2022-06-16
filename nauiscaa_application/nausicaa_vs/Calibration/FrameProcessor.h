#pragma once

#include"Logger.h"
#include"defines.h"

#include <opencv2/opencv.hpp>
#if VIDEO_STREAM
#include"Config.h"

namespace ffmpegCodec
{
	class FrameProcessor {
	public:
		FrameProcessor(const AVCodecContext& codecCtx, int w, int h);
		FrameProcessor();
		~FrameProcessor();
		int getWidth();
		int getHeight();
		void writeFrame(AVCodecContext& codec_ctx, AVFormatContext& fmt_ctx);
		void frameData(const cv::Mat& data, AVCodecContext& codecCtx, AVFormatContext& fmt_ctx, AVStream& stream); // replace it with actual rawdata
		void TimeBase(AVRational timebase); // /* rescale output packet timestamp values from codec to stream timebase */
		void StreamInformation(const AVStream& stream);
		void resetDimension(int w, int h);
		void initializeFrame(const AVCodecContext& codecCtx);
		void initializeSampleScaler(const AVCodecContext& codecCtx);
		

	protected:
	private:
		AVFrame* frame = nullptr;
	    SwsContext* swsContext = nullptr;
		int width;
		int height;
		AVRational streamTimeBase;
		AVStream outputStream;
		void clean();
		int64_t pts;
	};
}
#endif
