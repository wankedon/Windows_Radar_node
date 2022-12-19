/**
 * @file RadarDirectTask.h
 * @brief 雷达测向任务
 * @author 装备事业部软件组 王克东
 * @version 0.1
 * @date 2021-10-17
 *
 * @copyright Copyright (c) 2021  中国电子科技集团公司第四十一研究所
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
	std::atomic_bool m_stopFlag;     ///< 让线程停止的标志
	std::future<void> m_asyncFuture; ///< 线程的future结果
	std::unique_ptr<RadarDFProcess> m_process;
	std::shared_ptr<RadarDFDevice> m_device;
};