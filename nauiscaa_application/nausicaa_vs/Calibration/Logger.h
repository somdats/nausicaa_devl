#pragma once

#include <chrono>
#include <iomanip> // put_time
#include <fstream>
#include <sstream> // stringstream
#include<experimental/filesystem>

#include <opencv2/imgproc.hpp>
#include<opencv2/imgcodecs.hpp>

#include"velodyne_reader.h"
#include"defines.h"
namespace fs = std::experimental::filesystem;



#define EPSILON_DIFF_IMG 0.03
#define EPSILON_DIFF_PC 0.03

extern  std::string DUMP_FOLDER_PATH;


extern  std::string IMG_EXT;
extern  std::string PC_EXT;
extern int CameraCount;
typedef  std::vector< std::pair<std::string, std::string>>vecPair;




namespace logger
{

	bool isExistDirectory(std::string DirPath);
	bool saveImages(std::string dirPath, std::string timeStamp, const cv::Mat& data, std::string camID = "");
	bool savePointCloud(int currLidar, std::string timeStamp, std::string dirPath, const PacketDecoder::HDLFrame latestFrame);
	bool savePointCloudASCII(std::string dirName, const PacketDecoder::HDLFrame latestFrame);
	bool savePointCloudBinary(int currLidar, std::string timeStamp, std::string dirPath, const PacketDecoder::HDLFrame latestFrame);
	bool getTimeStamp(std::chrono::system_clock::time_point& timePrev, std::string& time, bool pc = true);
	void LoadPointCloudBinary(std::string fileName, PacketDecoder::HDLFrame& latestFrame);
	vecPair readConfigFile(std::string configFile);
}


