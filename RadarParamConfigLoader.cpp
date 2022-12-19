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
	m_device[404101] = L"�豸404101";
	m_device[404102] = L"�豸404102";
	m_device[404103] = L"�豸404103";
}

ZBSYB_RADAR_DIRECTION::ThresholdData RadarParamConfigLoader::loadThreshold(const uint32_t& deviceId)
{
	pugi::xml_document doc;
	if (!doc.load_file(filePath.c_str()))
	{
		LOG("configuration load failed");
		return ZBSYB_RADAR_DIRECTION::ThresholdData{};
	}
	auto root = doc.child(L"����");
	if (root == nullptr)
		return ZBSYB_RADAR_DIRECTION::ThresholdData{};
	auto xmlIdentity = root.child(L"��ֵ����");
	ZBSYB_RADAR_DIRECTION::ThresholdData result;
	if (xmlIdentity)
		result = loadIdentity(xmlIdentity);
	result.cacheFileName = loadIDCacheFileName(deviceId);
	return result;
}

ZBSYB_RADAR_DIRECTION::ThresholdData RadarParamConfigLoader::loadIdentity(pugi::xml_node xmlIdentity)
{
	ZBSYB_RADAR_DIRECTION::ThresholdData data;
	auto angle = xmlIdentity.child(L"�Ƕ�");
	if (angle)
		data.angle = angle.text().as_float();
	auto freq = xmlIdentity.child(L"Ƶ��");
	if (freq)
		data.rf = freq.text().as_float();
	auto multi = xmlIdentity.child(L"����");
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
	auto device = doc.child(L"����").child(L"λ��").child(m_device[deviceId]);
	if (device == nullptr)
		return string();
	auto fileName = device.child(L"Ŀ�껺���ļ�").text().as_string();
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
	auto root = doc.child(L"����");
	if (root == nullptr)
		return make_tuple(0, 0, 0);
	auto position = root.child(L"λ��");
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
	auto root = doc.child(L"����");
	if (root)
		return root.child(L"ʱ��").text().as_int();
}

tuple<float, float, float> RadarParamConfigLoader::loadGPS(pugi::xml_node xmlIdentity, uint32_t deviceId)
{
	float GPSlong = 0;
	float GPSlat = 0;
	float GPSheight = 0;
	pugi::xml_node device = xmlIdentity.child(m_device[deviceId]);
	if (device)
	{
		GPSlong = device.child(L"����").text().as_float();
		GPSlat = device.child(L"γ��").text().as_float();
		GPSheight = device.child(L"�߶�").text().as_float();
	}
	return make_tuple(GPSlong, GPSlat, GPSheight);
}

void RadarParamConfigLoader::writeTimeDiff(tuple<int32_t, int32_t, int32_t> timeDiff)
{
	pugi::xml_document doc;
	if (!doc.load_file(filePath.c_str()))
		return;
	auto dev = doc.child(L"����").child(L"λ��").child(m_device[get<0>(timeDiff)]);
	if (!dev)
		return;
	if (dev.first_attribute().as_bool())
		return;
	pugi::char_t buf[128] = { 0 };
	swprintf_s(buf, L"%d", get<1>(timeDiff));
	dev.child(L"ʱ����").first_child().set_value(buf);
	swprintf_s(buf, L"%d", get<2>(timeDiff));
	dev.child(L"ʱ�����").first_child().set_value(buf);
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
	auto dev = doc.child(L"����").child(L"λ��").child(m_device[deviceId]);
	if (!dev.first_attribute().as_bool())
		return make_tuple(0, 0, 0);
	return make_tuple(
		1,
		dev.child(L"ʱ����").text().as_int(),
	    dev.child(L"ʱ�����").text().as_int());
}