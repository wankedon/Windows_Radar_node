/**
 * @file TargetSnapShot.cpp
 * @brief 测向目标快照
 * @author 装备事业部软件组 王克东
 * @version 0.1
 * @date 2021-11-19
 *
 * @copyright Copyright (c) 2021  中国电子科技集团公司第四十一研究所
 *
 */
#include "../pch.h"
#include "TargetSnapShot.h"
#include "toolsFunc.h"

using namespace std;
using namespace radarDF;
using namespace ZBSYB_RADAR_SOCKET;

TargetSnapShot::TargetSnapShot(std::function<void(DFResultItem& result)> aquairFunc)
	:m_aquairFunc(aquairFunc)
{
}

TargetSnapShot::~TargetSnapShot()
{
}

void TargetSnapShot::inputTarget(shared_ptr<Target> target)
{
	m_targetList.push_back(target);
}

void TargetSnapShot::aquireDFResultItem()
{
	if (m_targetList.size() == 0)
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	for (auto target : m_targetList)
	{
		m_aquairFunc(generateDFresultItem(target));
	}
	m_targetList.clear();
}

DFResultItem TargetSnapShot::generateDFresultItem(shared_ptr<Target> target)
{
	auto targetItem = target->getLastItem();
	auto resultItem = extractDFResultItem(targetItem.targetForRead);
	resultItem.set_target_id(target->id());
	*resultItem.mutable_match_result() = target->getMatchResult();
	resultItem.set_gi_azi(targetItem.giazi);
	return resultItem;
}

DFResultItem TargetSnapShot::extractDFResultItem(TargetForRead& targetFrame)
{
	DFResultItem item;
	//脉冲族
	*item.mutable_pulse_cluster() = extractPulseCluster(targetFrame);
	//方位俯仰
	auto direction = item.mutable_target_direction();
	direction->mutable_raw_dir()->set_azimuth(targetFrame.matchData.aziDOA);
	direction->mutable_raw_dir()->set_pitch(targetFrame.matchData.eleDOA);
	direction->mutable_decoupled_dir()->set_azimuth(targetFrame.matchData.aziDOA);
	direction->mutable_decoupled_dir()->set_pitch(targetFrame.matchData.eleDOA);
	//获取结果的时间
	item.mutable_acquire_time()->set_seconds(targetFrame.matchData.time / 1000);
	item.mutable_acquire_time()->set_nanos((uint64_t)targetFrame.matchData.time % 1000);
	//设备位置
	auto position = item.mutable_device_position();
	position->set_altitude(targetFrame.GPSheight);
	position->set_longitude(targetFrame.GPSlong);
	position->set_latitude(targetFrame.GPSlat);
	return item;
}

PulseCluster TargetSnapShot::extractPulseCluster(TargetForRead& targetFrame)
{
	auto matchresult = targetFrame.matchData;
	PulseCluster pulseCluster;
	pulseCluster.set_freq(matchresult.RF);
	pulseCluster.set_type(matchresult.modType);
	for (int i = 0; i < PULSE_SAMPLE_LENGTH; i++)
	{
		auto pulseSample = pulseCluster.add_pulse_samples();
		pulseSample->set_carrier_freq(targetFrame.fre[i]);
		pulseSample->set_pulse_width(matchresult.PW);
		pulseSample->set_repeat_period(matchresult.PRI);
	}
	//幅度
	auto ampt = pulseCluster.mutable_ampt();
	ampt->set_cur(targetFrame.amp);
	ampt->set_up(targetFrame.amp_max);
	ampt->set_down(targetFrame.amp_min);
	ampt->set_mean(targetFrame.amp_mean);
	return pulseCluster;
}