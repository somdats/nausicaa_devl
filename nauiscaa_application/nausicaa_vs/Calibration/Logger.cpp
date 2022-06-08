#pragma once

#include<iostream>

#include "Logger.h"


std::string DUMP_FOLDER_PATH  = "C:\\Users\\Fabio Ganovelli\\Documents\\GitHub\\nausicaa_devl\\fakeinput\\PointClouds";


std::string IMG_EXT = "jpeg";
std::string PC_EXT = "txt";



bool logger::isExistDirectory(std::string DirPath) {

	fs::path p = DirPath;
	bool exist = false;
	if (fs::exists(p))
		exist = true;
	return exist;

}


bool logger:: saveImages(std::string dirPath, std::string timeStamp, const cv::Mat& data, std::string camID) {

	std::string imageDir = "";
	// check if directory exist!!!
	if (logger::isExistDirectory(dirPath))
	{
		if (!camID.empty())
		{
			imageDir = dirPath + "Images/" + camID;
			if (!logger::isExistDirectory(imageDir))
				fs::create_directories(imageDir);
		}
		else
		{
			imageDir = dirPath + "Images";

			if (!logger::isExistDirectory(imageDir))
				fs::create_directory(imageDir);
		}
	}
	else {
		std::cout << "root path does not exist:" << dirPath << std::endl;
		return false;
	}

	std::string fileName = imageDir + "/" + timeStamp + "." + IMG_EXT;
	if (!data.empty())
	{
		cv::imwrite(fileName, data);
		//std::cout << " Writing image with time stamps:" << timeStamp << std::endl;
	}

	return true;
}

bool logger::savePointCloud(int currLidar, std::string timeStamp, std::string dirPath, const PacketDecoder::HDLFrame latestFrame) {

	if (latestFrame.x.empty()) {

		//std::cout << "lidar Frame empty:" << std::endl;
		return false;

	}

	std::string pcDir = "";
	// check if directory exist!!!
	if (logger::isExistDirectory(dirPath))
	{
		pcDir = dirPath + "PointClouds";
		if (!logger::isExistDirectory(pcDir))
			fs::create_directory(pcDir);
	}
	else
	{
		std::cout << "root path does not exist:" << dirPath << std::endl;
		return false;
	}

	std::string fileName = pcDir + "/" + timeStamp + "_" + std::to_string(currLidar) + "." + PC_EXT;

	FILE* fo = fopen(fileName.c_str(), "wb");
	for (unsigned int i = 0; i < latestFrame.x.size(); ++i)
		fprintf(fo, "%f %f %f %d %d %hu %f %u\n", latestFrame.x[i], latestFrame.y[i], latestFrame.z[i], latestFrame.intensity[i],
			latestFrame.laser_id[i], latestFrame.azimuth[i], latestFrame.distance[i],
			latestFrame.ms_from_top_of_hour[i]);
	fclose(fo);

	return true;
}

bool logger::getTimeStamp(std::chrono::system_clock::time_point& timePrev, std::string& time, bool pc)
{
	auto currTime = std::chrono::system_clock::now();
	auto UTC = std::chrono::duration_cast<std::chrono::milliseconds>(currTime.time_since_epoch()).count();
	// OR
	//auto in_time_t = std::chrono::system_clock::to_time_t(now);
	//std::stringstream datetime;
	//datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");

	double extime = std::chrono::duration_cast<
		std::chrono::duration<double, std::milli>>(currTime - timePrev).count();
	extime = extime / double(1000);

	if (pc)
	{
		if (extime >= EPSILON_DIFF_PC)
		{
			timePrev = currTime;
			time = std::to_string(UTC);
			return true;
		}
	}
	else 
	{
		if (extime >= EPSILON_DIFF_IMG)
		{
			timePrev = currTime;
			time = std::to_string(UTC);
			return true;
		}
	}
	time = "";
	return false;
}

