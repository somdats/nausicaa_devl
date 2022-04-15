#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "PacketDriver.h"
#include "PacketDecoder.h"
#include <boost/shared_ptr.hpp>
#include<deque>



#include <thread>

using namespace std;



void read_from_lidar( void*args[]){
    std::cout << "lidar thread "<<((std::string*)args[2])->c_str()<<endl;

    PacketDriver * driver =  (PacketDriver*) args[0];
    PacketDecoder * decoder=  (PacketDecoder*) args[1];;

    std::string* data = new std::string();
    unsigned int* dataLength = new unsigned int();
    std::deque<PacketDecoder::HDLFrame> frames;
    PacketDecoder::HDLFrame latest_frame;
    int n = 0;


    while (n<100) {
      std::cout << "get packed "<<((std::string*)args[2])->c_str()<<endl;
      driver->GetPacket(data, dataLength);
      decoder->DecodePacket(data, dataLength);
      frames = decoder->GetFrames();
      if (decoder->GetLatestFrame(&latest_frame)) {
        std::cout << "Number of points: " << latest_frame.x.size() << std::endl;
        FILE * fo =fopen((std::to_string(n )+"_"+ *(std::string*)args[2] +".txt").c_str(),"w");
        for(unsigned int i = 0; i < latest_frame.x.size();++i)
            fprintf(fo,"%f %f %f %d %d %d\n",latest_frame.x[i],latest_frame.y[i],latest_frame.z[i],latest_frame.intensity[i],latest_frame.intensity[i],latest_frame.intensity[i]);
        fclose(fo);
        ++n;
      }
    }
}


void*  args0[3];
void*  args1[3];

int main()
{
    PacketDriver driver_2368;
    driver_2368.InitPacketDriver(2368);
    PacketDecoder decoder_2368;
    decoder_2368.SetCorrectionsFile("D:/nausicaa_vs/nausicaa_vs/Calibration/32db.xml");

    PacketDriver driver_2369;
    driver_2369.InitPacketDriver(2369);
    PacketDecoder decoder_2369;
    decoder_2369.SetCorrectionsFile("D:/nausicaa_vs/nausicaa_vs/Calibration/32db.xml");


    std::string a("2368");
    std::string b("2369");

    args0[0] = (void*) &driver_2368;
    args0[1] = (void*) &decoder_2368;
    args0[2] = &a;

    args1[0] = (void*) &driver_2369;
    args1[1] = (void*) &decoder_2369;
    args1[2] = &b;

 //   read_from_lidar(args0);

//    std::cout << "main thread\n";
    //std::thread t0(&read_from_lidar,args0);
     std::thread t1(&read_from_lidar,args1);


    //t0.join();
     t1.join();

  return 0;
}
