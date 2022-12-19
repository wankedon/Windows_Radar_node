/**
 * @file Target.h
 * @brief ����Ŀ��
 * @author װ����ҵ������� ���˶�
 * @version 0.1
 * @date 2021-11-19
 *
 * @copyright Copyright (c) 2021  �й����ӿƼ����Ź�˾����ʮһ�о���
 *
 */
#include "../pch.h"
#include "Target.h"
#include <time.h>
#include "toolsFunc.h"

Target::Target(const ItemOfTarget& itemOfTarget)
	:m_id(itemOfTarget.targetForRead.matchData.ID)
{
	addNewItem(itemOfTarget);
}

Target::~Target()
{
}

bool Target::isMySelf(const int& id) const
{
	return m_id == id ? true : false;
}

void Target::addNewItem(const ItemOfTarget& itemOfTarget)
{
	std::lock_guard<std::mutex> lg(m_mutex);
	if (m_itemList.size() > MAX_ITEM_COUNT)
		m_itemList.pop_front();
	m_itemList.push_back({ time(nullptr), itemOfTarget });
	m_matchResult = convertTargetOutputToMatchResult(itemOfTarget.targetForRead.matchData);
}

bool Target::isActive(const time_t& startTime)
{
	std::lock_guard<std::mutex> lg(m_mutex);
	if (m_itemList.empty())
		return false;
	return m_itemList.back().first > startTime;
}

ItemOfTarget Target::getLastItem()
{
	if (!m_itemList.empty())
		return m_itemList.back().second;
	else
		return ItemOfTarget();
}