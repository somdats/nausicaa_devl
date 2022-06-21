#pragma once

#include"defines.h"


#if VIDEO_STREAM
#include"Config.h"
#include"FrameProcessor.h"

namespace ffmpegCodec
{
	class OutputStream {
	public:
		OutputStream(const char* formatName, const char* outputIPPort,  const char*codecName);  // default  format name : "H264"
		~OutputStream();
		AVStream* getStream();
		AVFormatContext* getFormatContext();
		void WriteHeader(); //uses AVFormatContext
		void createStream( FrameProcessor& frame, AVCodecContext& cdcCntx, cv::Mat &buffData);
		size_t getNumFrames() const; 
		void writeTrailer();
		void setExtras(uint8_t* extraData, int extraDataSize);
		AVRational getTimeBase();
		void setStreamFrameRate(const AVRational& framerate);
		
		
	protected:
		AVOutputFormat outFormat;
		AVStream* outStream = nullptr;
		AVFormatContext* formatCtx = nullptr;
		AVCodec* outCodec = nullptr;


	private:
		bool endOfStream = false;
		size_t numFrame;

		void CleanUp();
		


	};
}
#endif
