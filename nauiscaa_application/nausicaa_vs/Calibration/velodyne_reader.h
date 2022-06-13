#ifndef VELODYNE_READER_H
#define VELODYNE_READER_H

#include "defines.h"

#include <PacketDriver.h>
#include <PacketDecoder.h>

#include <mutex>
#include <thread>

#ifndef uint
#define uint unsigned int
#endif


 
 
struct Lidar{
    PacketDriver driver;
    PacketDecoder decoder;

    PacketDecoder::HDLFrame latest_frame;
    std::mutex latest_frame_mutex;

    bool reading;
    int lidarPort;

    void init(uint port, std::string path_correction_file);

    void start_reading();
    void stop_reading();

#ifdef SCENE_REPLAY

    std::vector<std::pair<unsigned long long, std::string>> timed_pointclouds;
    std::string lidar_dump_name;
    void read_from_file(std::string filepath);
#endif

};
#endif // VELODYNE_READER_H
