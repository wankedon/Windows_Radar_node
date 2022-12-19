/**
 * @file Target.cpp
 * @brief ����Ŀ��
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
#include "node/radar/radarDF.pb.h"
using namespace ZBSYB_RADAR_DIRECTION;

struct ItemOfTarget
{
	TargetForRead targetForRead;
	float giazi;
};

class Target
{
public:
	Target(const ItemOfTarget& itemOfTarget);
	~Target();

public:
	bool isActive(const time_t& startTime);
	bool isMySelf(const int& id) const;
	void addNewItem(const ItemOfTarget& itemOfTarget);
	uint32_t id() const { return m_id; }
	ItemOfTarget getLastItem();
	radarDF::MatchResult getMatchResult() { return m_matchResult; }

private: 
	std::mutex m_mutex;
	const int m_id;
	radarDF::MatchResult m_matchResult;
	std::list<std::pair<time_t, ItemOfTarget>> m_itemList;
	static const int MAX_ITEM_COUNT = 500;
};