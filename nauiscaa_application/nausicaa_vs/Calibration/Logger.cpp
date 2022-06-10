#pragma once

#include<iostream>

#include "Logger.h"


std::string DUMP_FOLDER_PATH;


std::string IMG_EXT = "jpeg";
std::string PC_EXT = "txt";
std::string PC_BIN = "bin";



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
		pcDir = dirPath + "PointClouds" + "/" + std::to_string(currLidar);
		if (!logger::isExistDirectory(pcDir))
			fs::create_directories(pcDir);
	}
	else
	{
		std::cout << "root path does not exist:" << dirPath << std::endl;
		return false;
	}

	std::string fileName = pcDir + "/" + timeStamp + /*"_" + std::to_string(currLidar) +*/ "." + PC_EXT;

	FILE* fo = fopen(fileName.c_str(), "wb");
	for (unsigned int i = 0; i < latestFrame.x.size(); ++i)
		fprintf(fo, "%f %f %f %d %d %hu %f %u\n", latestFrame.x[i], latestFrame.y[i], latestFrame.z[i], latestFrame.intensity[i],
			latestFrame.laser_id[i], latestFrame.azimuth[i], latestFrame.distance[i],
			latestFrame.ms_from_top_of_hour[i]);
	fclose(fo);

	return true;
}

bool logger:: savePointCloudASCII(std::string dirName, const PacketDecoder::HDLFrame latestFrame) {
	if (latestFrame.x.empty()) {

		//std::cout << "lidar Frame empty:" << std::endl;
		return false;

	}
	// check if directory exist!!!
	if (!logger::isExistDirectory(dirName))
	{
		std::cout << "root path does not exist:" << dirName << std::endl;
		return false;
	}


	std::string fileName = dirName + "/" + "test_pc." + PC_EXT;

	FILE* fo = fopen(fileName.c_str(), "wb");
	for (unsigned int i = 0; i < latestFrame.x.size(); ++i)
		fprintf(fo, "%f %f %f %d %d %hu %f %u\n", latestFrame.x[i], latestFrame.y[i], latestFrame.z[i], latestFrame.intensity[i],
			latestFrame.laser_id[i], latestFrame.azimuth[i], latestFrame.distance[i],
			latestFrame.ms_from_top_of_hour[i]);
	fclose(fo);

	return true;
}

bool logger::savePointCloudBinary(int currLidar, std::string timeStamp, std::string dirPath, const PacketDecoder::HDLFrame latestFrame) {

	if (latestFrame.x.empty()) {

		//std::cout << "lidar Frame empty:" << std::endl;
		return false;

	}

	std::string pcDir = "";
	// check if directory exist!!!
	if (logger::isExistDirectory(dirPath))
	{
		pcDir = dirPath + "PointClouds" + "/" + std::to_string(currLidar);
		if (!logger::isExistDirectory(pcDir))
			fs::create_directories(pcDir);
	}
	else
	{
		std::cout << "root path does not exist:" << dirPath << std::endl;
		return false;
	}

	std::string fileName = pcDir + "/" + timeStamp + /*"_" + std::to_string(currLidar) +*/ "." + PC_BIN;

	FILE* fo = fopen(fileName.c_str(), "wb");
	/*for (unsigned int i = 0; i < latestFrame.x.size(); ++i)
	{*/
	    uint32_t num_points = (uint32_t)latestFrame.x.size();
	    fwrite(&num_points, sizeof(uint32_t), 1, fo);
		fwrite(latestFrame.x.data(), sizeof(latestFrame.x[0]), num_points, fo);
		fwrite(latestFrame.y.data(), sizeof(latestFrame.y[0]), num_points, fo);
		fwrite(latestFrame.z.data(), sizeof(latestFrame.z[0]), num_points, fo);
		fwrite(latestFrame.intensity.data(), sizeof(latestFrame.intensity[0]), num_points, fo);
		fwrite(latestFrame.laser_id.data(), sizeof(latestFrame.laser_id[0]), num_points, fo);
		fwrite(latestFrame.azimuth.data(), sizeof(latestFrame.azimuth[0]), num_points, fo); //2
		fwrite(latestFrame.distance.data(), sizeof(latestFrame.distance[0]), num_points, fo);  //8
		fwrite(latestFrame.ms_from_top_of_hour.data(), sizeof(latestFrame.ms_from_top_of_hour[0]), num_points, fo);
	/*}*/
	
	fclose(fo);

	return true;
}

void logger:: LoadPointCloudBinary(std::string fileName, PacketDecoder::HDLFrame& latestFrame)
{
	FILE* fo;
	fo = fopen(fileName.c_str(), "rb");
	if (!fo)
	{
		std::cout << "cannot-open file:" << std::endl;
		exit(1);
	}
	uint32_t num_points;
	fread(&num_points, sizeof(uint32_t), 1, fo);

	latestFrame.x.resize(num_points);
	fread(latestFrame.x.data(), sizeof(std::vector<double>::value_type), latestFrame.x.size(), fo);

	latestFrame.y.resize(num_points);
	fread(latestFrame.y.data(), sizeof(std::vector<double>::value_type), latestFrame.y.size(), fo);

	latestFrame.z.resize(num_points);
	fread(latestFrame.z.data(), sizeof(std::vector<double>::value_type), latestFrame.z.size(), fo);

	latestFrame.intensity.resize(num_points);
	fread(latestFrame.intensity.data(), sizeof(std::vector<uchar>::value_type), latestFrame.intensity.size(), fo);

	latestFrame.laser_id.resize(num_points);
	fread(latestFrame.laser_id.data(), sizeof(std::vector<uchar>::value_type), latestFrame.laser_id.size(), fo);

	latestFrame.azimuth.resize(num_points);
	fread(latestFrame.azimuth.data(), sizeof(std::vector<unsigned short>::value_type), latestFrame.azimuth.size(), fo); //2

	latestFrame.distance.resize(num_points);
	fread(latestFrame.distance.data(), sizeof(std::vector<double>::value_type), latestFrame.distance.size(), fo);  //8

	latestFrame.ms_from_top_of_hour.resize(num_points);
	fread(latestFrame.ms_from_top_of_hour.data(), sizeof(std::vector<unsigned int>::value_type), latestFrame.ms_from_top_of_hour.size(), fo);
	fclose(fo);
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

