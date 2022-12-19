#pragma once
#include "../DeviceCluster.h"
#include "ClientServer.h"
#include "node/nodeInternal.pb.h"

class RadarDFDevice;
class RadarDFDeviceCluster : public DeviceCluster
{
public:
    RadarDFDeviceCluster(const LANConfig& cfg);
    ~RadarDFDeviceCluster();

public:
    void refresh() override;

private:
    void addDevice(std::pair<const std::string, std::shared_ptr<RadarDFDevice>> radarDev);
    void eraseDevice(std::pair<const std::string, std::shared_ptr<RadarDFDevice>> radarDev);

private:
    std::mutex m_mutex;
    std::map<std::string, std::shared_ptr<RadarDFDevice>> m_devices;
};