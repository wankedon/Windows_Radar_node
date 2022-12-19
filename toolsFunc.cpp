#include "../pch.h"
#include "toolsFunc.h"
#include <fmt_/format.h>
#include "PositionSocketAPI.h"
#include "TargetMatch.h"

bool JudgeEqualofDouble(double data1, double data2)
{
	if (fabs(data1 - data2) < Epslion)
		return true;
	return false;
}

//得到当前时间函数
std::string GetNowTime()
{
	time_t setTime;
	time(&setTime);
	tm ptm;
	localtime_s(&ptm, &setTime);
	std::string time = fmt::format("{}-{}-{} {}:{}:{}",
		ptm.tm_year + 1900,
		ptm.tm_mon + 1,
		ptm.tm_mday,
		ptm.tm_hour,
		ptm.tm_min,
		ptm.tm_sec
	);
	return time;
}

//雷达库参数proto消息转设备数据帧
std::list<ZBSYB_RADAR_SOCKET::ParamConfParam> convertToDeviceParam(const radarDF::UploadRadarParams& uploadradarparams)
{
	static const uint32_t US_PER_MS = 1000;
	std::list<ZBSYB_RADAR_SOCKET::ParamConfParam> confParamList;
	uint32_t indexNum = 0;
	auto size = uploadradarparams.radar_search_params_size();
	for (auto& radarparam : uploadradarparams.radar_search_params())
	{
		ZBSYB_RADAR_SOCKET::ParamConfParam confParam;
		confParam.listCount = size;
		confParam.indexNum = indexNum++;
		confParam.startFrequency = (uint32_t)radarparam.freq_range().start() * US_PER_MS;
		confParam.endFrequency = (uint32_t)radarparam.freq_range().stop() * US_PER_MS;
		confParam.searchTime = (uint32_t)radarparam.search_time();
		confParam.searchBand = (uint32_t)radarparam.search_band() * US_PER_MS;
		confParam.pulseWidthStart = (uint32_t)radarparam.pw_range().start() * US_PER_MS;         //תus
		confParam.pulseWidthEnd = (uint32_t)radarparam.pw_range().stop() * US_PER_MS;            //תus
		confParam.periodRepeatStart = (uint32_t)radarparam.rpperiod_range().start() * US_PER_MS; //תus
		confParam.periodRepeatEnd = (uint32_t)radarparam.rpperiod_range().stop() * US_PER_MS;     //תus
		confParam.targetSignalType = (uint32_t)radarparam.target_signal_type();
		confParam.noiseThreshold = (uint32_t)radarparam.noise_threshold();
		confParamList.push_back(confParam);
	}
	return confParamList;
}

double getMaxSearchBand(const radarDF::UploadRadarParams& uploadradarparams)
{
	auto params = convertToDeviceParam(uploadradarparams);
	std::vector<double> bws;
	for (auto param : params)
	{
		bws.push_back(param.searchBand);
	}
	auto maxBW = max_element(bws.begin(), bws.end());
	return *maxBW;
}

//设备数据帧转匹配算法数据结构体
ZBSYB_RADAR_DIRECTION::TargetItem convertDataFrameToMatchTargetItem(ZBSYB_RADAR_SOCKET::StartStopFrame* startStopFrame)
{
	ZBSYB_RADAR_DIRECTION::TargetItem targetItem;
	targetItem.sweepIdx = startStopFrame->scan_index;
	targetItem.segmentIdx = startStopFrame->LO_RF;
	targetItem.timeStamp = timeConvertToMsec(*startStopFrame);
	targetItem.DOA_azi = startStopFrame->azi;
	targetItem.DOA_ele = startStopFrame->ele;
	for (int i = 0; i < PULSE_SAMPLE_LENGTH; i++)
	{
		targetItem.RF[i] = startStopFrame->fre[i];
		targetItem.PW[i] = startStopFrame->pw[i];
		targetItem.PRI[i] = startStopFrame->pri[i];
	}
	return targetItem;
}

//设备数据帧转匹配算法数据结构体
ZBSYB_RADAR_DIRECTION::DetectDataInput convertDataFrameToDetectDataInput(ZBSYB_RADAR_SOCKET::StartStopFrame* startStopFrame)
{
	ZBSYB_RADAR_DIRECTION::DetectDataInput targetItem;
	targetItem.ID = 0;
	targetItem.nScan = startStopFrame->scan_index;
	targetItem.Lo = startStopFrame->LO_RF;
	targetItem.modType = startStopFrame->type;
	targetItem.aziDOA = startStopFrame->azi_j0;
	targetItem.eleDOA = startStopFrame->ele_j0;
	targetItem.time = timeConvertToMsec(*startStopFrame);
	for (int i = 0; i < 10; i++)
	{
		targetItem.RF[i] = startStopFrame->fre[i];
		targetItem.PW[i] = startStopFrame->pw[i];
		targetItem.PRI[i] = startStopFrame->pri[i];
	}
	targetItem.gridPulseNums = startStopFrame->grid_pulse_num;
	targetItem.sortPulseNums = startStopFrame->cluster_pulse_num;
	targetItem.frq_max = startStopFrame->frq_max;
	targetItem.frq_min = startStopFrame->frq_min;
	return targetItem;
}

radarDF::MatchResult convertTargetOutputToMatchResult(const ZBSYB_RADAR_DIRECTION::TargetOutput& targetOutput)
{
	radarDF::MatchResult result;
	result.set_target_id(targetOutput.ID);
	result.set_module_type(targetOutput.modType);
	result.mutable_occour_time()->set_seconds(targetOutput.time/1000);
	result.mutable_occour_time()->set_nanos((uint64_t)targetOutput.time % 1000);	
	result.mutable_first_time()->set_seconds(targetOutput.firstTime / 1000);
	result.mutable_first_time()->set_nanos((uint64_t)targetOutput.time % 1000);
	result.set_azi_mean(targetOutput.aziDOA_mean);
	result.set_azi_std(targetOutput.aziDOA_STD);
	auto dir = result.mutable_direction();
	dir->set_azimuth(targetOutput.aziDOA);
	dir->set_pitch(targetOutput.eleDOA);
	auto pulse = result.mutable_pulse_sample();
	pulse->set_carrier_freq(targetOutput.RF);
	pulse->set_pulse_width(targetOutput.PW);
	pulse->set_repeat_period(targetOutput.PRI);
	auto stdpulse = result.mutable_std_pulse_sample();
	stdpulse->set_carrier_freq(targetOutput.RF_STD);
	stdpulse->set_pulse_width(targetOutput.PW_STD);
	stdpulse->set_repeat_period(targetOutput.PRI_STD);
	result.set_fuse_target_num(targetOutput.NUM);
	return result;
}

uint64_t utcToMsec(const UTCTime& utc)
{
	struct tm tm_time;
	memset(&tm_time, 0, sizeof(tm_time));
	tm_time.tm_year = utc.year - 1900;
	tm_time.tm_mon = utc.mon - 1;
	tm_time.tm_mday = utc.day;
	tm_time.tm_hour = utc.hour;
	tm_time.tm_min = utc.min;
	tm_time.tm_sec = utc.second;
	return (uint64_t)mktime(&tm_time) * 1000 + (uint64_t)utc.millisecond;
}

UTCTime msecToUtc(const uint64_t& milliSeconds)
{
	UTCTime utc;
	time_t seconds = milliSeconds / 1000;
	tm* local = localtime((time_t*)&seconds);
	utc.year = local->tm_year + 1900;
	utc.mon = local->tm_mon + 1;
	utc.day = local->tm_mday;
	utc.hour = local->tm_hour;
	utc.min = local->tm_min;
	utc.second = local->tm_sec;
	utc.millisecond = milliSeconds % 1000;
	return utc;
}

struct timeval getTimeVal()
{
	struct timeval tp;
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp.tv_sec = clock;
	tp.tv_usec = wtm.wMilliseconds * 1000;
	return tp;
}

uint64_t getCurrentTimeMsec()
{
	struct timeval timeval = getTimeVal();
	return (uint64_t)timeval.tv_sec * 1000 + (uint64_t)timeval.tv_usec / 1000;
}

UTCTime getCurrentUtc()
{
	return msecToUtc(getCurrentTimeMsec());
}

std::string getCurrentUtcStr()
{
	auto utc = getCurrentUtc();
	return fmt::format
	(
		"{}-{}-{}-{}-{}-{}",
		utc.year,
		utc.mon,
		utc.day,
		utc.hour,
		utc.min,
		utc.second
	);
}

std::pair<uint64_t, uint32_t> getCurrentTimeStamp()
{
	struct timeval timeval = getTimeVal();
	return std::make_pair(timeval.tv_sec, timeval.tv_usec / 1000);
}

uint32_t ipToUint(const std::string& address)
{
	return inet_addr(address.c_str());
}

std::string uintToIp(uint32_t address)
{
	struct in_addr addr;
	addr.s_addr = address;
	char* ntpAddressStr = inet_ntoa(addr);
	return std::string(ntpAddressStr);
}