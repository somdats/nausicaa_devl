#include "velodyne_reader.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include<deque>

#include"Logger.h"
#include <direct.h>
#include<filesystem>
#include <chrono>
#include <thread>

 
std::chrono::system_clock::time_point timeLidar;
FILE* flid1 = nullptr;
FILE*  flid2 = nullptr;
 


using namespace std;


//#undef SCENE_REPLAY
//#undef FAKE_IMAGE




 

void Lidar::init(uint port, std::string path_correction_file) {
    if (SCENE_REPLAY) {
        std::string toFolder = DUMP_FOLDER_PATH + "\\PointClouds\\" + std::to_string(port) + "\\";
        std::string timestamps = toFolder + "timestamps.txt";
        FILE* ft = fopen(timestamps.c_str(), "r");
        while (!feof(ft)) {
            char time_alfanumeric[20];
            fscanf(ft, "%s", time_alfanumeric);
            std::string ta = std::string(time_alfanumeric);
            timed_pointclouds.push_back(std::make_pair(std::stoull(ta), toFolder + ta + ".bin"));
        }
        fclose(ft);

        std::sort(timed_pointclouds.begin(), timed_pointclouds.end());
    }
    else 
    {
        driver.InitPacketDriver(port);
        decoder.SetCorrectionsFile(path_correction_file.c_str());
    }
    lidarPort = port;

if(SAVE_PC)
    timeLidar = std::chrono::system_clock::now();
}

using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

void Lidar::start_reading() {
    int first_i;
    int i = 0;
    std::string* data = new std::string();
    unsigned int* dataLength = new unsigned int();
    std::deque<PacketDecoder::HDLFrame> frames;

    if (SCENE_REPLAY) {
        while (timed_pointclouds[i].first < start_time)++i;
        first_i = i;
    }

    reading = true;


    if (SAVE_PC){
        if (logger::isExistDirectory(DUMP_FOLDER_PATH))
        {
            std::string pcDir = DUMP_FOLDER_PATH + "/PointClouds/";
            if (!logger::isExistDirectory(pcDir))
                fs::create_directory(pcDir);

            std::string lidarDir = pcDir + std::to_string(lidarPort);
            if (!logger::isExistDirectory(lidarDir))
                fs::create_directory(lidarDir);

            std::string timeStampFileName = lidarDir + "/" + "timestamps.txt";
            if (fs::exists(timeStampFileName))
                fs::remove(timeStampFileName);

            if (lidarPort == 2368 && std::string(timeStampFileName).find(std::to_string(2368)) != std::string::npos)
                flid1 = fopen(timeStampFileName.c_str(), "wb");
            if (lidarPort == 2369 && std::string(timeStampFileName).find(std::to_string(2369)) != std::string::npos)
                flid2 = fopen(timeStampFileName.c_str(), "wb");

        }
        else
        {
            std::cout << "root path does not exist:" << DUMP_FOLDER_PATH << std::endl;

        }
    }

    while(reading){

        if (SCENE_REPLAY) {
            unsigned long long delta = timed_pointclouds[i % timed_pointclouds.size()].first - start_time;
            unsigned long long delta1 = clock() - restart_time + partial_time;

            int ii = first_i;

            if (time_running) {
                while ((ii < timed_pointclouds.size()) && virtual_time > timed_pointclouds[ii].first - start_time) ++ii;
                if (ii < timed_pointclouds.size())
                    if (ii != i)
                    {
                        i = ii;
                        std::this_thread::sleep_for(10ms);
                        latest_frame_mutex.lock();
                        logger::LoadPointCloudBinary(timed_pointclouds[i].second, latest_frame);
                        latest_frame_mutex.unlock();
                    }
            }

        }
        else
        {
            driver.GetPacket(data, dataLength);
            decoder.DecodePacket(data, dataLength);

            frames = decoder.GetFrames();
            latest_frame_mutex.lock();
            decoder.GetLatestFrame(&latest_frame);
            latest_frame_mutex.unlock();
        }
        if (SAVE_PC) {
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
        }
     
    }
}

void Lidar::stop_reading(){
    latest_frame_mutex.lock();
    reading = false;
    if (SAVE_PC) {
        fclose(flid1);
        fclose(flid2);
    }
    latest_frame_mutex.unlock();
}

