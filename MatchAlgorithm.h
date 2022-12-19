/**
 * @file MatchAlgorithm.h
 * @brief 策略模式调用目标分选算法
 * @author 装备事业部软件组 王克东
 * @version 0.1
 * @date 2021-11-19
 *
 * @copyright Copyright (c) 2021  中国电子科技集团公司第四十一研究所
 *
 */
#pragma once
#include "TargetMatch.h"
#include "PositionSocketAPI.h"
using namespace ZBSYB_RADAR_DIRECTION;

class MatchAlgorithm
{

public:
	MatchAlgorithm() = default;
	~MatchAlgorithm() = default;

public:
	virtual int matching(TargetItem* item) = 0;
	virtual TargetOutput targetMatching(DetectDataInput* item)=0;

protected:
	uint32_t m_algorithmHandle;
};

class FirstMatchAlgorithm :public MatchAlgorithm
{
public:
	FirstMatchAlgorithm(ZBSYB_RADAR_DIRECTION::ConfigData* configData)
	{
		m_algorithmHandle = ZBSYB_RADAR_DIRECTION::createInstance(configData);
	}

	~FirstMatchAlgorithm()
	{
		if (m_algorithmHandle != 0)
		{
			ZBSYB_RADAR_DIRECTION::destroyInstance(m_algorithmHandle);
			m_algorithmHandle = 0;
		}
	}

public:
	virtual int matching(TargetItem* item)
	{
		return ZBSYB_RADAR_DIRECTION::matching(m_algorithmHandle, item);
	}

	virtual TargetOutput targetMatching(DetectDataInput* item)
	{
		return ZBSYB_RADAR_DIRECTION::targetMatching(m_algorithmHandle, item);
	}
};