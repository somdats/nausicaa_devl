
#include <exception>
#include<string>
#include"Config.h"
#include"defines.h"
#if VIDEO_STREAM
class FFmpegException : std::exception
{

public:

	FFmpegException(std::string error);

	FFmpegException(std::string error, int returnValue);

	virtual char const* what() const
	{
		return std::exception::what();
	}


private:

	char error[AV_ERROR_MAX_STRING_SIZE];
};
#endif