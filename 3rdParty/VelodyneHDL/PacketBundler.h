// Velodyne HDL Packet Bundler
// Nick Rypkema (rypkema@mit.edu), MIT 2017
// shared library to bundle velodyne packets into enough for a single frame

#ifndef PACKET_BUNDLER_H_INCLUDED
#define PACKET_BUNDLER_H_INCLUDED

#include <string>
#include <deque>
#include "PacketDecoder.h"

class PacketBundler
{
public:
	__declspec(dllexport) PacketBundler();
	__declspec(dllexport) virtual ~PacketBundler();
	__declspec(dllexport) void SetMaxNumberOfBundles(unsigned int max_num_of_bundles);
	__declspec(dllexport) void BundlePacket(std::string* data, unsigned int* data_length);
	__declspec(dllexport) std::deque<std::string> GetBundles();
	__declspec(dllexport) void ClearBundles();
	__declspec(dllexport) bool GetLatestBundle(std::string* bundle, unsigned int* bundle_length);

protected:
	__declspec(dllexport) void UnloadData();
	__declspec(dllexport) void BundleHDLPacket(unsigned char *data, unsigned int data_length);
	__declspec(dllexport) void SplitBundle();

private:
  unsigned int _last_azimuth;
  unsigned int _max_num_of_bundles;
  std::string* _bundle;
  std::deque<std::string> _bundles;
};

#endif // PACKET_BUNDLER_H_INCLUDED
