/**
 * @file RadarDirectTask.h
 * @brief �״��������
 * @author װ����ҵ������� ���˶�
 * @version 0.1
 * @date 2021-10-17
 *
 * @copyright Copyright (c) 2021  �й����ӿƼ����Ź�˾����ʮһ�о���
 *
 */
#pragma once
#include "../NodeTask.h"
#include <future>
#include "PositionSocketAPI.h"
#include "node/radar/radarDF.pb.h"

class RadarDFDevice;
class RadarDFProcess;
class RadarDFTask : public NodeTask
{
public:
	RadarDFTask(const TaskInitEntry& entry);
	~RadarDFTask();

public:
	bool config(MessageExtractor& extractor) override;
	ErrorType start() override;
	ErrorType onCmd(MessageExtractor& extractor) override;
	void stop() override;

private:
	bool onUploadRadarParam(MessageExtractor& extractor);
	bool onCompassCalibrate();
	bool onTimeCalibrate(MessageExtractor& extractor);
	bool onDFStart(MessageExtractor& extractor);
	bool onDFStop();
	bool onChangeMatchPolicy(MessageExtractor& extractor);
	void taskLoop();
	bool startDF();
	void reset();

private:
	double m_searchBand;
	uint32_t m_interval;
	uint32_t m_resultSize;
	std::atomic_bool m_stopFlag;     ///< ���߳�ֹͣ�ı�־
	std::future<void> m_asyncFuture; ///< �̵߳�future���
	std::unique_ptr<RadarDFProcess> m_process;
	std::shared_ptr<RadarDFDevice> m_device;
};