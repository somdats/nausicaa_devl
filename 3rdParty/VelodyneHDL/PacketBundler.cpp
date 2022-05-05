// Velodyne HDL Packet Bundler
// Nick Rypkema (rypkema@mit.edu), MIT 2017
// shared library to bundle velodyne packets into enough for a single frame

#include <cmath>
#include <stdint.h>
#include <iostream>

#include "PacketBundler.h"

__declspec(dllexport) PacketBundler::PacketBundler()
{
  _max_num_of_bundles = 10;
  UnloadData();
}

__declspec(dllexport) PacketBundler::~PacketBundler()
{

}

__declspec(dllexport) void PacketBundler::SetMaxNumberOfBundles(unsigned int max_num_of_bundles)
{

  if (max_num_of_bundles <= 0) {
    return;
  } else {
    _max_num_of_bundles = max_num_of_bundles;
  }
  while (_bundles.size() >= _max_num_of_bundles) {
    _bundles.pop_front();
  }
}

__declspec(dllexport) void PacketBundler::BundlePacket(std::string* data, unsigned int* data_length)
{
  const unsigned char* data_char = reinterpret_cast<const unsigned char*>(data->c_str());
  BundleHDLPacket(const_cast<unsigned char*>(data_char), *data_length);
}

__declspec(dllexport) void PacketBundler::BundleHDLPacket(unsigned char *data, unsigned int data_length)
{
  if (data_length != 1206) {
    std::cout << "PacketBundler: Warning, data packet is not 1206 bytes" << std::endl;
    return;
  }

  HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket *>(data);

  for (int i = 0; i < HDL_FIRING_PER_PKT; ++i) {
    HDLFiringData firingData = dataPacket->firingData[i];

    if (firingData.rotationalPosition < _last_azimuth) {
      SplitBundle();
    }

    _last_azimuth = firingData.rotationalPosition;
  }

  _bundle->append(reinterpret_cast<const char*>(data), (size_t)data_length);
}

__declspec(dllexport) void PacketBundler::SplitBundle()
{
  if (_bundles.size() == _max_num_of_bundles-1) {
    _bundles.pop_front();
  }
  _bundles.push_back(*_bundle);
  delete _bundle;
  _bundle = new std::string();
}

__declspec(dllexport) void PacketBundler::UnloadData()
{
  _last_azimuth = 0;
  _bundle = new std::string();
  _bundles.clear();
}

__declspec(dllexport) std::deque<std::string> PacketBundler::GetBundles()
{
  return _bundles;
}

__declspec(dllexport) void PacketBundler::ClearBundles()
{
  _bundles.clear();
}

__declspec(dllexport) bool PacketBundler::GetLatestBundle(std::string* bundle, unsigned int* bundle_length)
{
  if (_bundles.size()) {
    *bundle = _bundles.back();
    *bundle_length = bundle->size();
    _bundles.clear();
    return(true);
  }
  return(false);
}
