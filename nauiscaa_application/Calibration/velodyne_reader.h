#ifndef VELODYNE_READER_H
#define VELODYNE_READER_H

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

    void init(uint port, std::string path_correction_file);

    void start_reading();
    void stop_reading();

};
#endif // VELODYNE_READER_H
