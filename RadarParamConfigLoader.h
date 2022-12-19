#pragma once
#include "TargetMatch.h"
#include "pugixml.hpp"

class RadarParamConfigLoader
{
public:
	RadarParamConfigLoader(const std::wstring& filePath);
	~RadarParamConfigLoader(void) = default;
public:
	ZBSYB_RADAR_DIRECTION::ThresholdData loadThreshold(const uint32_t& deviceId);
	std::tuple<float, float, float> loadPosition(uint32_t deviceId);
	int loadTimezone();
	void writeTimeDiff(std::tuple<int32_t, int32_t, int32_t> timeDiff);
	std::tuple<int32_t,int32_t, int32_t> loadTimeDiff(uint32_t deviceId);

private:
	std::string loadIDCacheFileName(const uint32_t& deviceId);
	std::tuple<float, float, float> loadGPS(pugi::xml_node xmlIdentity,uint32_t deviceId);
	ZBSYB_RADAR_DIRECTION::ThresholdData loadIdentity(pugi::xml_node xmlIdentity);

private:
	const std::wstring filePath;
	std::map<int32_t, pugi::char_t*> m_device;
};