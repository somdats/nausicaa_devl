#include "velodyne_reader.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include<deque>




using namespace std;

#ifdef FAKE_INPUT
#undef FAKE_INPUT

void Lidar::init(uint port, std::string path_correction_file){
    lidar_dump_name = std::string("1_") + std::to_string(port) + ".txt";
}


void Lidar::start_reading() {
    FILE* ld = fopen(lidar_dump_name.c_str(), "r");


    while (!feof(ld)) {
        float x,y,z,distance;
        unsigned char intensity, laser_id;
        unsigned short azimuth;
        unsigned int  ms_from_top_of_hour;

        fscanf(ld, "%f %f %f %hhu %hhu %hhu  %hhu %hu %f %d", &x, &y, &z, &intensity, &intensity, &intensity,  &laser_id,&azimuth ,&distance,&ms_from_top_of_hour);
        latest_frame.x.push_back(x);
        latest_frame.y.push_back(y);
        latest_frame.z.push_back(z);

        latest_frame.intensity.push_back(intensity);
        latest_frame.laser_id.push_back(laser_id);
        latest_frame.azimuth.push_back(azimuth);
        latest_frame.distance.push_back(distance);
    }
    fclose(ld);
    reading = true;
}

#else

void Lidar::init(uint port, std::string path_correction_file) {
    driver.InitPacketDriver(port);
    decoder.SetCorrectionsFile(path_correction_file.c_str());
}


void Lidar::start_reading(){
    std::string* data = new std::string();
    unsigned int* dataLength = new unsigned int();
    std::deque<PacketDecoder::HDLFrame> frames;

    reading = true;

    while(reading){

        driver.GetPacket(data, dataLength);
        decoder.DecodePacket(data, dataLength);

        frames = decoder.GetFrames();
        latest_frame_mutex.lock();
        decoder.GetLatestFrame(&latest_frame);
        latest_frame_mutex.unlock();
    }
}
#endif

void Lidar::stop_reading(){
    latest_frame_mutex.lock();
    reading = false;
    latest_frame_mutex.unlock();
}

