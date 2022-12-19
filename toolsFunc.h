#pragma once
#include <time.h>
#include <iostream>
#include <windows.h>
#include <winsock.h>
#pragma comment(lib,"wsock32.lib")
#include "PositionSocketAPI.h"
#include "TargetMatch.h"
#include "node/radar/radarDF.pb.h"
#define Epslion 1e-8

struct UTCTime
{
	int year;
	int mon;
	int day;
	int hour;
	int min;
	int second;
	int millisecond;
};

static const int PULSE_SAMPLE_LENGTH = 10;

//时分秒转换为UTC函数
template<class T>
uint64_t timeConvertToMsec(const T& stru)
{
	UTCTime inputTime;
	inputTime.year = stru.year; 
	inputTime.mon = stru.month;    
	inputTime.day = stru.day;        
	inputTime.hour = stru.hour;       
	inputTime.min = stru.minute;       
	inputTime.second = stru.sec;
	inputTime.millisecond = stru.msec;
	return utcToMsec(inputTime);
}

template<class T1, class T2>
void JudgeValue(T1& newVal, const std::pair<T1, T2>& range)
{
	T1 tempVal = newVal;
	if (tempVal > range.second)
	{
		tempVal = range.second;
	}
	else if (tempVal < range.first)
	{
		tempVal = range.first;
	}
	newVal = tempVal;
}

bool JudgeEqualofDouble(double data1, double data2);
std::string GetNowTime();
std::list<ZBSYB_RADAR_SOCKET::ParamConfParam> convertToDeviceParam(const radarDF::UploadRadarParams& uploadradarparams);
double getMaxSearchBand(const radarDF::UploadRadarParams& uploadradarparams);
ZBSYB_RADAR_DIRECTION::TargetItem convertDataFrameToMatchTargetItem(ZBSYB_RADAR_SOCKET::StartStopFrame* startStopFrame);
ZBSYB_RADAR_DIRECTION::DetectDataInput convertDataFrameToDetectDataInput(ZBSYB_RADAR_SOCKET::StartStopFrame* startStopFrame);
radarDF::MatchResult convertTargetOutputToMatchResult(const ZBSYB_RADAR_DIRECTION::TargetOutput& targetOutput);
uint64_t utcToMsec(const UTCTime& utc);
UTCTime msecToUtc(const uint64_t& milliSeconds);
struct timeval getTimeVal();
uint64_t getCurrentTimeMsec();
UTCTime getCurrentUtc();
std::string getCurrentUtcStr();
std::pair<uint64_t, uint32_t> getCurrentTimeStamp();
uint32_t ipToUint(const std::string& address);
std::string uintToIp(uint32_t address);