#pragma once

#include <chrono>
#include <iomanip> // put_time
#include <fstream>
#include <sstream> // stringstream
#include<experimental/filesystem>

#include <opencv2/imgproc.hpp>
#include<opencv2/imgcodecs.hpp>

#include"velodyne_reader.h"

namespace fs = std::experimental::filesystem;



#ifndef VIDEO_STREAM
#define VIDEO_STREAM 0
#endif

#ifndef SAVE_IMG
#define SAVE_IMG 1
#endif

#ifndef SAVE_PC
#define SAVE_PC 0
#endif

#define EPSILON_DIFF_IMG 0.03
#define EPSILON_DIFF_PC 0.03

extern  std::string DUMP_FOLDER_PATH;


extern  std::string IMG_EXT;
extern  std::string PC_EXT;
extern int CameraCount;



namespace logger
{

	bool isExistDirectory(std::string DirPath);
	bool saveImages(std::string dirPath, std::string timeStamp, const cv::Mat& data, std::string camID = "");
	bool savePointCloud(int currLidar, std::string timeStamp, std::string dirPath, const PacketDecoder::HDLFrame latestFrame);
	bool savePointCloudASCII(std::string dirName, const PacketDecoder::HDLFrame latestFrame);
	bool savePointCloudBinary(int currLidar, std::string timeStamp, std::string dirPath, const PacketDecoder::HDLFrame latestFrame);
	bool getTimeStamp(std::chrono::system_clock::time_point& timePrev, std::string& time, bool pc = true);
	void LoadPointCloudBinary(std::string fileName, PacketDecoder::HDLFrame& latestFrame);
}


