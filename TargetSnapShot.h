/**
 * @file TargetSnapShot.h
 * @brief 测向目标快照
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

class Target;
class TargetSnapShot
{
public:
	TargetSnapShot(std::function<void(radarDF::DFResultItem& result)> aquairFunc);
	~TargetSnapShot();

public:
	void inputTarget(std::shared_ptr<Target> target);
	void aquireDFResultItem();

private:
	radarDF::DFResultItem generateDFresultItem(std::shared_ptr<Target> target);
	radarDF::DFResultItem extractDFResultItem(TargetForRead& targetForRead);
	radarDF::PulseCluster extractPulseCluster(TargetForRead& targetForRead);

private:
	std::list<std::shared_ptr<Target>> m_targetList;
	std::function<void(radarDF::DFResultItem& result)> m_aquairFunc;
};