#include "../pch.h"
#include "RadarDFDeviceCluster.h"
#include "RadarDFDevice.h"
#include "Logger.h"

using namespace std;

RadarDFDeviceCluster::RadarDFDeviceCluster(const LANConfig& cfg)
{
	PS_InitSSLib();
	for (auto devConf : cfg.cluster_radardf())
	{
		auto dev = make_shared<RadarDFDevice>(devConf);
		if (dev)
			m_devices[devConf.address().ip()] = dev;
	}
}

RadarDFDeviceCluster::~RadarDFDeviceCluster()
{
	PS_DeinitSSLib();
}

//刷新在线设备
void RadarDFDeviceCluster::refresh()
{
	lock_guard<mutex> lg(m_mutex);
	for (auto& radarDev : m_devices)
	{
		if (radarDev.second->isConnected())
			addDevice(radarDev);
		else
			eraseDevice(radarDev);
	}
}

void RadarDFDeviceCluster::addDevice(pair<const std::string, std::shared_ptr<RadarDFDevice>> radarDev)
{
	//该设备不在devices中则添加该设备
	if (devices.find(radarDev.second->id().value()) != devices.end())
		return;
	CLOG("RadarDevice {} Online", radarDev.first);
	radarDev.second->sendSelfDetectCmd();
	devices[radarDev.second->id().value()] = radarDev.second;
}

void RadarDFDeviceCluster::eraseDevice(pair<const std::string, std::shared_ptr<RadarDFDevice>> radarDev)
{
	//该设备在devices中则删除该设备
	if (devices.find(radarDev.second->id().value()) == devices.end())
		return;
	CLOG("RadarDevice {} Offline", radarDev.first);
	radarDev.second->setDeviceStatus(DeviceStatus::D_OFFLINE);
	devices.erase(radarDev.second->id().value());
}