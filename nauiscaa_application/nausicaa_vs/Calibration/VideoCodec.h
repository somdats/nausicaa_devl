#pragma once

#include"defines.h"

#include<string>
#if VIDEO_STREAM

#include"Config.h"



namespace ffmpegCodec {



	class VideoCodec {

	public:
		VideoCodec(const char* codecName, std::string url);
		VideoCodec(AVCodecID codecId);
		~VideoCodec();
		void setCodecParameter(int width, int height, int fps, int bitrate,int flags, AVPixelFormat format = AV_PIX_FMT_YUV420P);
		AVDictionary* setDict(std::string codec_profile);
		AVCodecContext* getCodecContext();
		void InitializeCodecStream(AVStream& stream, std::string presetVal = "slow", std::string codecProfile = "high444");

	private:
		void CleanUp();

		void LoadContext(AVCodec* codec);

		bool opened = false;
		AVCodecID avCodec;
		std::string protocol;

	protected:
		AVCodecContext* codecContext = nullptr;
		AVDictionary* dict = nullptr;


	};
}
#endif
