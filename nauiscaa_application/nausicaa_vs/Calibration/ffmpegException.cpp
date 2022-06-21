#include "ffmpegException.h"


using namespace std;

#if VIDEO_STREAM
FFmpegException::FFmpegException(string error)
{
	exception(error.c_str());
}

FFmpegException::FFmpegException(string error, int returnValue)
{
	exception(
		(error + ": " + av_make_error_string(this->error, AV_ERROR_MAX_STRING_SIZE, returnValue)).c_str()
	);

}
#endif
