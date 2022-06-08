#include "velodyne_reader.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include<deque>

#include"Logger.h"
#include <direct.h>
#include <filesystem>
#include <chrono>
#include <thread>

#if SAVE_PC
std::chrono::system_clock::time_point timeLidar;
#endif


using namespace std;





#ifdef FAKE_INPUT
void Lidar::read_from_file(std::string filepath) {
     FILE* ld = fopen(filepath.c_str(), "r");

     latest_frame.x.clear();
     latest_frame.y.clear();
     latest_frame.z.clear();

     latest_frame.intensity.clear();
     latest_frame.laser_id.clear();
     latest_frame.azimuth.clear();
     latest_frame.distance.clear();

     while (!feof(ld)) {
        float x,y,z,distance;
        unsigned char intensity, laser_id;
        unsigned short azimuth;
        unsigned int  ms_from_top_of_hour;

        fscanf(ld, "%f %f %f %hhu  %hhu %hu %f %d", &x, &y, &z, &intensity,  &laser_id,&azimuth ,&distance,&ms_from_top_of_hour);
        latest_frame.x.push_back(x);
        latest_frame.y.push_back(y);
        latest_frame.z.push_back(z);

        latest_frame.intensity.push_back(intensity);
        latest_frame.laser_id.push_back(laser_id);
        latest_frame.azimuth.push_back(azimuth);
        latest_frame.distance.push_back(distance);
    }
    fclose(ld);     
}
#endif
 

void Lidar::init(uint port, std::string path_correction_file) {
#ifdef FAKE_INPUT
   std::filesystem::directory_iterator ite(DUMP_FOLDER_PATH);
   for (const auto& file : ite) {
       std::string base_filename = file.path().string().substr(file.path().string().find_last_of("/\\") + 1);
       std::string time_part = base_filename.substr(0, base_filename.find_last_of("_"));
       std::string port_part = base_filename.substr(base_filename.find_last_of("_")+1, 4);
       if (std::to_string(port) == port_part)
       {
           std::string micsecs = time_part.substr(time_part.length() - 9, 9);
              
           timed_pointclouds.push_back(std::make_pair(atoi(micsecs.c_str()), DUMP_FOLDER_PATH+"\\"+base_filename));
       }
   }
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

    while(reading){

#ifdef FAKE_INPUT
        if (clock() - start_time > timed_pointclouds[i].first-offset) {
            std::this_thread::sleep_for(10ms);
            latest_frame_mutex.lock();
            int _ = clock();
            read_from_file(timed_pointclouds[i].second);
            printf("%d\n", clock() - _);
            latest_frame_mutex.unlock();
            ++i;
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
            logger::savePointCloud(lidarPort, l1Cam, DUMP_FOLDER_PATH, latest_frame);
#endif
     
    }
}

void Lidar::stop_reading(){
    latest_frame_mutex.lock();
    reading = false;
    latest_frame_mutex.unlock();
}

