/**
 * @file TargetBuffer.h
 * @brief ����Ŀ�껺����
 * @author װ����ҵ������� ���˶�
 * @version 0.1
 * @date 2021-11-19
 *
 * @copyright Copyright (c) 2021  �й����ӿƼ����Ź�˾����ʮһ�о���
 *
 */
#include "../pch.h"
#include "TargetBuffer.h"
#include "TargetSnapShot.h"
#include "ParamsRange.h"
#include "toolsFunc.h"
#include "StringConv.h"
#include "RadarParamConfigLoader.h"
#include "Miscellaneous.h"

using namespace std;
using namespace radarDF;
using namespace ZBSYB_RADAR_SOCKET;

TargetBuffer::TargetBuffer(const uint32_t& deviceid, const double& searchBand, function<void(DFResultItem& result)> aquairFunc)
	:m_snapShot(make_unique<TargetSnapShot>(aquairFunc))
{
	m_algorithm = make_unique<AlgorithmCaller>(&gerenalConfigData(deviceid, searchBand), MatchPolicy::POLICY_FIRST);
}

TargetBuffer::~TargetBuffer()
{

}

ZBSYB_RADAR_DIRECTION::ConfigData TargetBuffer::gerenalConfigData(const uint32_t& deviceid,const double& searchBand)
{
	wstring path = LocateModulePath();
	RadarParamConfigLoader cl(path + L"\\�ڵ��������.xml");
	auto thresholdData = cl.loadThreshold(deviceid);
	ZBSYB_RADAR_DIRECTION::ConfigData configData;
	configData.taskParams.freqBand = searchBand;
	configData.thresholdData.angle = thresholdData.angle;
	configData.thresholdData.rf = thresholdData.rf;
	configData.thresholdData.multi = thresholdData.multi;
	std::string filename;
	if (thresholdData.cacheFileName == "0")
		filename = getCurrentUtcStr() + "_" + to_string(deviceid) + ".json";
	else
		filename = thresholdData.cacheFileName;
	configData.thresholdData.cacheFileName = StrConvert::wstringToUTF8(path) + "\\IDCacheFile\\" + filename;
	return configData;
}

void TargetBuffer::changeMatchPolicy(const MatchPolicy& policy)
{
	m_algorithm->changePolicy(policy);
}

void TargetBuffer::inputRawItem(const StartStopFrame* item)
{
	//if (m_algorithm == nullptr)
	//	m_algorithm = make_unique<AlgorithmCaller>(&gerenalConfigData(deviceid, m_searchBand), MatchPolicy::POLICY_FIRST);
	ZBSYB_RADAR_DIRECTION::TargetForRead targetForRead;
	if (item == nullptr)
		return;
	StartStopFrame rawItem = *item;
	if (rawItem.frq == 0)
		return;
	if ((rawItem.azi_j0 < 0) && (!JudgeEqualofDouble(rawItem.azi_j0, -1)))
		return;
	auto result = m_algorithm->targetMatching(&rawItem);
	if (result.ID < 0)
		return;
	targetForRead.matchData = result;
	judgeRawItem(targetForRead, rawItem);
	updateTargetCluster(targetForRead, rawItem.head);
}

//��������
void TargetBuffer::judgeRawItem(ZBSYB_RADAR_DIRECTION::TargetForRead& targetForRead,StartStopFrame& item)
{
	targetForRead.cw_fn = item.cw_yn;
	JudgeValue(item.GPSheight, ALTITUDE);
	targetForRead.GPSheight = item.GPSheight;
	JudgeValue(item.GPSlong, LONGITUE);
	targetForRead.GPSlong = item.GPSlong;
	JudgeValue(item.GPSlat, LATITUDE);
	targetForRead.GPSlat = item.GPSlat;
	JudgeValue(item.amp, AMP);
	targetForRead.amp = item.amp;
	JudgeValue(item.amp_max, AMPLITUDE);
	targetForRead.amp_max = item.amp_max;
	JudgeValue(item.amp_min, AMPLITUDE);
	targetForRead.amp_min = item.amp_min;
	JudgeValue(item.amp_mean, AMPLITUDE);
	targetForRead.amp_mean = item.amp_mean;
	for (int i = 0; i < PULSE_SAMPLE_LENGTH; i++)
	{
		auto freq = item.fre[i];
		JudgeValue(freq, FRE);
		targetForRead.fre[i] = freq;
	}
}

//����Ŀ��
void TargetBuffer::updateTargetCluster(const ZBSYB_RADAR_DIRECTION::TargetForRead& targetForRead, const float& giazi)
{
	time_t startTime = time(nullptr) - LAST_RECORD_SPAN;
	auto targetPtr = findTarget(targetForRead.matchData.ID);
	ItemOfTarget itemOfTarget;
	itemOfTarget.targetForRead = targetForRead;
	itemOfTarget.giazi = giazi;
	if (targetPtr)
	{
		targetPtr->addNewItem(itemOfTarget);
	}
	else
	{
		if (m_targetCluster.size() < TARGET_COUNT.second)
			 addNewTarget(itemOfTarget);
	}
	if (m_targetCluster.size() > TARGET_COUNT.first)
	{
		auto targetPtr = findInactiveTarget(startTime);
		if (targetPtr)
			eraseTarget(targetPtr);
	}
	if (m_snapShot)
		inputTargetToSnapshot();
}

//����Ŀ��
std::shared_ptr<Target> TargetBuffer::findTarget(const int& id)
{
	lock_guard<mutex> lg(m_mutex);
	for (auto& target : m_targetCluster)
	{
		if (target->isMySelf(id))
			return target;
	}
	return nullptr;
}

//�����޸���Ŀ��
std::shared_ptr<Target> TargetBuffer::findInactiveTarget(const time_t& startTime)
{
	for (auto& target : m_targetCluster)
	{
		if (!target->isActive(startTime))
			return target;
	}
	return nullptr;
}

//ɾ��Ŀ��
void TargetBuffer::eraseTarget(std::shared_ptr<Target> target)
{
	auto iter = std::find(m_targetCluster.begin(), m_targetCluster.end(), target);
	if (iter != m_targetCluster.end())
		m_targetCluster.erase(iter);
}

//�����Ŀ��
void TargetBuffer::addNewTarget(const ItemOfTarget& targetForRead)
{
	m_targetCluster.push_back(make_shared<Target>(targetForRead));
}

//���������һ��ʱ���ڵ�Ŀ��
void TargetBuffer::inputTargetToSnapshot()
{
	lock_guard<mutex> lg(m_mutex);
	time_t startTime = time(nullptr) - SNAPSHOT_TIME;
	for (auto target : m_targetCluster)
	{
		if (target->isActive(startTime))
			m_snapShot->inputTarget(target);
	}
	m_snapShot->aquireDFResultItem();
}