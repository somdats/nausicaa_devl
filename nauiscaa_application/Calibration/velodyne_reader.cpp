#include "velodyne_reader.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include<deque>



using namespace std;



void Lidar::init(uint port, std::string path_correction_file){
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

void Lidar::stop_reading(){
    latest_frame_mutex.lock();
    reading = false;
    latest_frame_mutex.unlock();
}

