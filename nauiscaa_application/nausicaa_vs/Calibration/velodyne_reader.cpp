#include "velodyne_reader.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include<deque>

#include"Logger.h"
#include <direct.h>
#include<experimental/filesystem>
#include <chrono>
#include <thread>

#if SAVE_PC
std::chrono::system_clock::time_point timeLidar;
FILE* flid1 = nullptr;
FILE*  flid2 = nullptr;
#endif


using namespace std;







 

void Lidar::init(uint port, std::string path_correction_file) {
#ifdef FAKE_INPUT
    std::string toFolder = DUMP_FOLDER_PATH + "\\PointClouds\\" + std::to_string(port) + "\\";
   std::string timestamps = toFolder +"timestamps.txt";
   FILE* ft = fopen(timestamps.c_str(), "r");
   while (!feof(ft)) {
       char time_alfanumeric[20];
       fscanf(ft, "%s", time_alfanumeric);
       std::string ta = std::string(time_alfanumeric);
       std::string ms = ta.substr(ta.length() - 9, 6); // take the milliseconds
       timed_pointclouds.push_back(std::make_pair(atoi(ms.c_str()), toFolder+ta+".bin"));
   }
   fclose(ft);

   std::sort(timed_pointclouds.begin(), timed_pointclouds.end());
#else
    driver.InitPacketDriver(port);
    decoder.SetCorrectionsFile(path_correction_file.c_str());
#endif
    lidarPort = port;

#if SAVE_PC
    timeLidar = std::chrono::system_clock::now();
#endif
}

using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

void Lidar::start_reading(){
#ifndef FAKE_INPUT
    std::string* data = new std::string();
    unsigned int* dataLength = new unsigned int();
    std::deque<PacketDecoder::HDLFrame> frames;
#endif

    reading = true;

#ifdef FAKE_INPUT
    int start_time = clock();
    int offset = timed_pointclouds[0].first;
    int i = 0;
#endif

#if SAVE_PC
    if (logger::isExistDirectory(DUMP_FOLDER_PATH))
    {
        std::string pcDir = DUMP_FOLDER_PATH + "PointClouds";
        if (!logger::isExistDirectory(pcDir))
            fs::create_directory(pcDir);

        std:: string lidarDir = pcDir + "/" + std::to_string(lidarPort);
        if (!logger::isExistDirectory(lidarDir))
            fs::create_directory(lidarDir);

        std::string timeStampFileName = lidarDir + "/" +  "timestamps.txt";
        if (fs::exists(timeStampFileName))
            fs::remove(timeStampFileName);

        if (lidarPort == 2368 && std::string(timeStampFileName).find(std::to_string(2368)) != std::string::npos)
            flid1 = fopen(timeStampFileName.c_str(), "wb");
        if(lidarPort == 2369 && std::string(timeStampFileName).find(std::to_string(2369)) != std::string::npos)
            flid2 = fopen(timeStampFileName.c_str(), "wb");

    }
    else
    {
        std::cout << "root path does not exist:" << DUMP_FOLDER_PATH << std::endl;
      
    }
#endif // SAVE_PC


    while(reading){

#ifdef FAKE_INPUT
        if (clock() - start_time > timed_pointclouds[i].first-offset) {
            std::this_thread::sleep_for(10ms);
            latest_frame_mutex.lock();
            logger::LoadPointCloudBinary(timed_pointclouds[i].second, latest_frame);
            latest_frame_mutex.unlock();
            ++i;
            if (i == timed_pointclouds.size()) {
                start_time = clock();
                i = 0;
            }
        }
#else
        driver.GetPacket(data, dataLength);
        decoder.DecodePacket(data, dataLength);

        frames = decoder.GetFrames();
        latest_frame_mutex.lock();
        decoder.GetLatestFrame(&latest_frame);   
        latest_frame_mutex.unlock();
#endif
#if SAVE_PC
        std::string l1Cam;
        bool stat = logger::getTimeStamp(timeLidar, l1Cam);
        if (stat)
        {
            logger::savePointCloudBinary(lidarPort, l1Cam, DUMP_FOLDER_PATH, latest_frame);
            if (lidarPort == 2368)
                fprintf(flid1, "%s\n", l1Cam.c_str());
            if (lidarPort == 2369)
                fprintf(flid2, "%s\n", l1Cam.c_str());
        }
#endif
     
    }
}

void Lidar::stop_reading(){
    latest_frame_mutex.lock();
    reading = false;
#if SAVE_PC
    fclose(flid1);
    fclose(flid2);
#endif // SAVE_PC
    latest_frame_mutex.unlock();
}

