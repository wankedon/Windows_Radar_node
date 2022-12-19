/**
 * @file TargetBuffer.cpp
 * @brief 测向目标缓冲区
 * @author 装备事业部软件组 王克东
 * @version 0.1
 * @date 2021-11-19
 *
 * @copyright Copyright (c) 2021  中国电子科技集团公司第四十一研究所
 *
 */
#pragma once
#include "PositionSocketAPI.h"
#include "Target.h"
#include "node/radar/radarDF.pb.h"
#include "AlgorithmCaller.h"

class AlgorithmCaller;
class TargetSnapShot;
class Target;
class TargetBuffer
{
public:
	TargetBuffer(const uint32_t& deviceid, const double& searchBand,std::function<void(radarDF::DFResultItem& result)> aquairFunc);
	~TargetBuffer();

public:
	void inputRawItem(const ZBSYB_RADAR_SOCKET::StartStopFrame* item);
	void changeMatchPolicy(const radarDF::MatchPolicy& policy);

private:
	void judgeRawItem(ZBSYB_RADAR_DIRECTION::TargetForRead& targetForRead, ZBSYB_RADAR_SOCKET::StartStopFrame& item);
	void updateTargetCluster(const ZBSYB_RADAR_DIRECTION::TargetForRead& targetForRead,const float& giazi);
	std::shared_ptr<Target> findTarget(const int& id);
	std::shared_ptr<Target> findInactiveTarget(const time_t& startTime);
	void eraseTarget(std::shared_ptr<Target> target);
	void addNewTarget(const ItemOfTarget& targetForRead);
	void inputTargetToSnapshot();
	ZBSYB_RADAR_DIRECTION::ConfigData gerenalConfigData(const uint32_t& deviceid,const double& searchBand);

private:
	std::mutex m_mutex;
	std::vector<std::shared_ptr<Target>> m_targetCluster;
	std::unique_ptr<TargetSnapShot> m_snapShot;
	std::unique_ptr<AlgorithmCaller> m_algorithm;

private:
	static const int SNAPSHOT_TIME = 1;
	static const int SCALE = 10;
};