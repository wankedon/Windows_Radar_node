#include "../pch.h"
#include "StringConv.h"
#include "TargetMatch.h"
#include "RadarParamConfigLoader.h"
#include "Miscellaneous.h"
#include "Logger.h"

using namespace std;

RadarParamConfigLoader::RadarParamConfigLoader(const std::wstring& fp)
	:filePath(fp)
{
	m_device[404101] = L"设备404101";
	m_device[404102] = L"设备404102";
	m_device[404103] = L"设备404103";
}

ZBSYB_RADAR_DIRECTION::ThresholdData RadarParamConfigLoader::loadThreshold(const uint32_t& deviceId)
{
	pugi::xml_document doc;
	if (!doc.load_file(filePath.c_str()))
	{
		LOG("configuration load failed");
		return ZBSYB_RADAR_DIRECTION::ThresholdData{};
	}
	auto root = doc.child(L"配置");
	if (root == nullptr)
		return ZBSYB_RADAR_DIRECTION::ThresholdData{};
	auto xmlIdentity = root.child(L"阈值精度");
	ZBSYB_RADAR_DIRECTION::ThresholdData result;
	if (xmlIdentity)
		result = loadIdentity(xmlIdentity);
	result.cacheFileName = loadIDCacheFileName(deviceId);
	return result;
}

ZBSYB_RADAR_DIRECTION::ThresholdData RadarParamConfigLoader::loadIdentity(pugi::xml_node xmlIdentity)
{
	ZBSYB_RADAR_DIRECTION::ThresholdData data;
	auto angle = xmlIdentity.child(L"角度");
	if (angle)
		data.angle = angle.text().as_float();
	auto freq = xmlIdentity.child(L"频率");
	if (freq)
		data.rf = freq.text().as_float();
	auto multi = xmlIdentity.child(L"方差");
	if (multi)
		data.multi = multi.text().as_float();
	return data;
}


string RadarParamConfigLoader::loadIDCacheFileName(const uint32_t& deviceId)
{
	pugi::xml_document doc;
	if (!doc.load_file(filePath.c_str()))
	{
		LOG("configuration load failed");
		return string();
	}
	auto device = doc.child(L"配置").child(L"位置").child(m_device[deviceId]);
	if (device == nullptr)
		return string();
	auto fileName = device.child(L"目标缓存文件").text().as_string();
	return StrConvert::wstringToUTF8(fileName);
}

tuple<float,float,float> RadarParamConfigLoader::loadPosition(uint32_t deviceId)
{
	pugi::xml_document doc;
	if (!doc.load_file(filePath.c_str()))
	{
		LOG("configuration load failed");
		return make_tuple(0, 0, 0);
	}
	auto root = doc.child(L"配置");
	if (root == nullptr)
		return make_tuple(0, 0, 0);
	auto position = root.child(L"位置");
	if (!position.first_attribute().as_bool())
		return make_tuple(0, 0, 0);
	if (position)
		return loadGPS(position, deviceId);
}

int RadarParamConfigLoader::loadTimezone()
{
	pugi::xml_document doc;
	if (!doc.load_file(filePath.c_str()))
	{
		LOG("configuration load failed");
		return -1;
	}
	auto root = doc.child(L"配置");
	if (root)
		return root.child(L"时区").text().as_int();
}

tuple<float, float, float> RadarParamConfigLoader::loadGPS(pugi::xml_node xmlIdentity, uint32_t deviceId)
{
	float GPSlong = 0;
	float GPSlat = 0;
	float GPSheight = 0;
	pugi::xml_node device = xmlIdentity.child(m_device[deviceId]);
	if (device)
	{
		GPSlong = device.child(L"经度").text().as_float();
		GPSlat = device.child(L"纬度").text().as_float();
		GPSheight = device.child(L"高度").text().as_float();
	}
	return make_tuple(GPSlong, GPSlat, GPSheight);
}

void RadarParamConfigLoader::writeTimeDiff(tuple<int32_t, int32_t, int32_t> timeDiff)
{
	pugi::xml_document doc;
	if (!doc.load_file(filePath.c_str()))
		return;
	auto dev = doc.child(L"配置").child(L"位置").child(m_device[get<0>(timeDiff)]);
	if (!dev)
		return;
	if (dev.first_attribute().as_bool())
		return;
	pugi::char_t buf[128] = { 0 };
	swprintf_s(buf, L"%d", get<1>(timeDiff));
	dev.child(L"时差秒").first_child().set_value(buf);
	swprintf_s(buf, L"%d", get<2>(timeDiff));
	dev.child(L"时差毫秒").first_child().set_value(buf);
	doc.save_file(filePath.c_str());
}

tuple<int32_t,int32_t, int32_t> RadarParamConfigLoader::loadTimeDiff(uint32_t deviceId)
{
	pugi::xml_document doc;
	if (!doc.load_file(filePath.c_str()))
	{
		LOG("configuration load failed");
		return make_tuple(0, 0, 0);
	}
	auto dev = doc.child(L"配置").child(L"位置").child(m_device[deviceId]);
	if (!dev.first_attribute().as_bool())
		return make_tuple(0, 0, 0);
	return make_tuple(
		1,
		dev.child(L"时差秒").text().as_int(),
	    dev.child(L"时差毫秒").text().as_int());
}