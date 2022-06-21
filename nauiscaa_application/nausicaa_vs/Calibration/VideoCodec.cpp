
#include"VideoCodec.h"
#include"ffmpegException.h"



#if VIDEO_STREAM


using namespace ffmpegCodec;

VideoCodec::VideoCodec(const char* codecName)
{

	// get encoder typename and convert to appropriatde AVCodec 
	AVCodecID codecID;
	if (std::string(codecName) == "h264")
		codecID = AV_CODEC_ID_H264;
	else if (std::string(codecName) == "mpeg")
		codecID = AV_CODEC_ID_MPEG4;
	AVCodec *codec = const_cast<AVCodec*>(avcodec_find_encoder(codecID));

	if (!codec)
	{
		throw FFmpegException("Codec " + std::string(codecName) + " not found");
	}
	avCodec = codec->id;

	LoadContext(codec);
}

VideoCodec::VideoCodec(AVCodecID codecId)
{
	// change position
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 9, 100)
	av_register_all();
#endif
	avformat_network_init();
	AVCodec* codec = const_cast<AVCodec*>(avcodec_find_encoder(codecId));
	if (!codec)
	{
		throw FFmpegException("Codec with id " + std::to_string((int)codecId) + " not found");
	}
	avCodec = codecId;
	LoadContext(codec);
}

VideoCodec::~VideoCodec()
{
	CleanUp();
}

void VideoCodec::CleanUp()
{
	if (codecContext != nullptr)
	{
		avcodec_free_context(&codecContext);
		avcodec_close(codecContext);
	}
	if (dict != NULL)
	{
		delete dict;
		dict = NULL;
	}

}

void VideoCodec::LoadContext(AVCodec* codec)
{
	codecContext = avcodec_alloc_context3(codec);
	if (!codecContext)
	{
		CleanUp();
		throw FFmpegException("Could not allocate video codec context for codec " + std::string(codec->name));
	}

	// copy the type
	//codecContext->codec_type = codec->type;

}

void VideoCodec::setCodecParameter(int width, int height, int fps, int bitrate, int flags, AVPixelFormat format)
{
	const AVRational dst_fps = { fps, 1 };
	codecContext->width = width;
	codecContext->height = height;
	codecContext->bit_rate = bitrate;
	codecContext->codec_tag = 0;
	codecContext->codec_id = avCodec;
	codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
	codecContext->gop_size = 12;
	codecContext->pix_fmt = format; // AV_PIX_FMT_YUV420P;
	codecContext->framerate = dst_fps;
	codecContext->time_base = av_inv_q(dst_fps);

	//av_opt_set(codecContext->priv_data, "crf", TCHAR_TO_ANSI(*H264Crf), 0);
	
	if (flags & AVFMT_GLOBALHEADER)
	{
		codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
	}
 }

AVDictionary* VideoCodec::setDict(std::string codec_profile)
{
	av_dict_set(&dict, "profile", codec_profile.c_str(), 0);
	av_dict_set(&dict, "preset", "slow", 0); // "superfast"
	av_dict_set(&dict, "tune", "zerolatency", 0);
	return dict;
 }

AVCodecContext* VideoCodec:: getCodecContext()
{
	return codecContext;
}

void VideoCodec::InitializeCodecStream(AVStream& stream, std::string codecProfile, std::string presetVal)
{
	int ret = avcodec_parameters_from_context(stream.codecpar, codecContext);
	if (ret < 0)
	{
		throw FFmpegException("Could not initialize stream codec parameters!");
		
	}

	// open video encoder
	AVCodec* codec = const_cast<AVCodec*>(avcodec_find_encoder(avCodec));
	if (avCodec == 27)  // H264 codec
	{
		av_dict_set(&dict, "profile", codecProfile.c_str(), 0);
		av_dict_set(&dict, "preset", presetVal.c_str(), 0); // "superfast"
		av_dict_set(&dict, "tune", "zerolatency", 0);


		ret = avcodec_open2(codecContext, codec, &dict);
	}
	else
		ret = avcodec_open2(codecContext, codec, NULL);
	if (ret < 0)
	{
		throw FFmpegException("Could not open video encoder!");
	}
	
}
#endif