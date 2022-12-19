/**
 * @file RadarDirectProcess.cpp
 * @brief �״�������ݴ���
 * @author װ����ҵ������� ���˶�
 * @version 0.1
 * @date 2021-10-17
 *
 * @copyright Copyright (c) 2021  �й����ӿƼ����Ź�˾����ʮһ�о���
 *
 */
#include "../pch.h"
#include "RadarDFProcess.h"
#include "TargetBuffer.h"

using namespace std;
using namespace radarDF;
using namespace ZBSYB_RADAR_SOCKET;

RadarDFProcess::RadarDFProcess(const uint32_t& deviceid, const double& searchBand)
{
	m_targetBuffer = make_unique<TargetBuffer>(deviceid,searchBand,
		[this](DFResultItem& dfItem)
		{
			addDFResultItemToBuffer(dfItem);
		});
}

RadarDFProcess::~RadarDFProcess()
{

}

void RadarDFProcess::changeMatchPolicy(const MatchPolicy& policy)
{
	//�ı��ѡ����
	m_targetBuffer->changeMatchPolicy(policy);
}

void RadarDFProcess::inputRawFrame(const StartStopFrame* startStopFrame)
{
	m_targetBuffer->inputRawItem(startStopFrame);
}

DFResult RadarDFProcess::getDFResult()
{
	DFResult result;
	auto& items = *result.mutable_items();
	auto ptr = getDFBuffer();
	if (ptr)
	{
		for (auto& item : *ptr)
		{
			*items.Add() = item;
		}
	}
	return result;
}

unique_ptr<list<DFResultItem>> RadarDFProcess::getDFBuffer()
{
	lock_guard<mutex> lg(m_bufferLock);
	unique_ptr<list<DFResultItem>> result;
	result.swap(m_itemBuffer);
	return result;
}

void RadarDFProcess::addDFResultItemToBuffer(const DFResultItem& dfItem)
{
	lock_guard<mutex> lg(m_bufferLock);
	if (m_itemBuffer == nullptr)
		m_itemBuffer = make_unique<list<DFResultItem>>();
	m_itemBuffer->emplace_back(dfItem);
}