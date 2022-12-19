/**
 * @file RadarDirectProcess.h
 * @brief �״�������ݴ���
 * @author װ����ҵ������� ���˶�
 * @version 0.1
 * @date 2021-10-17
 *
 * @copyright Copyright (c) 2021  �й����ӿƼ����Ź�˾����ʮһ�о���
 *
 */
#pragma once
#include "PositionSocketAPI.h"
#include "node/radar/radarDF.pb.h"

class TargetBuffer;
class RadarDFProcess
{
public:
	RadarDFProcess(const uint32_t& deviceid, const double& searchBand);
	~RadarDFProcess();

public:
	void changeMatchPolicy(const radarDF::MatchPolicy& policy);
	void inputRawFrame(const ZBSYB_RADAR_SOCKET::StartStopFrame* startStopFrame);
	radarDF::DFResult getDFResult();

private:
	std::unique_ptr<std::list<radarDF::DFResultItem>> getDFBuffer();
	void addDFResultItemToBuffer(const radarDF::DFResultItem& dfItem);

private:
	std::mutex m_bufferLock;
	std::unique_ptr<TargetBuffer> m_targetBuffer;
	std::unique_ptr<std::list<radarDF::DFResultItem>> m_itemBuffer;
	std::function<void(radarDF::DFResultItem& dfItem)> m_aquaireDFItemFunc;
};