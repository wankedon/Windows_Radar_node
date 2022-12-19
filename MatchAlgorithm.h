/**
 * @file MatchAlgorithm.h
 * @brief ����ģʽ����Ŀ���ѡ�㷨
 * @author װ����ҵ������� ���˶�
 * @version 0.1
 * @date 2021-11-19
 *
 * @copyright Copyright (c) 2021  �й����ӿƼ����Ź�˾����ʮһ�о���
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