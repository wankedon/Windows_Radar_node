/**
 * @file RadarDirectTask.cpp
 * @brief 雷达测向任务
 * @author 装备事业部软件组 王克东
 * @version 0.1
 * @date 2021-10-17
 *
 * @copyright Copyright (c) 2021  中国电子科技集团公司第四十一研究所
 *
 */
#include "../pch.h"
#include "RadarDFDevice.h"
#include "PositionSocketAPI.h"
#include "RadarDFTask.h"
#include "RadarDFProcess.h"
#include "node/radar/radarDFInternal.pb.h"
#include "toolsFunc.h"
#include "Logger.h"

using namespace std;
using namespace radarDF;
using namespace ZBSYB_RADAR_SOCKET;

RadarDFTask::RadarDFTask(const TaskInitEntry& entry)
	:NodeTask(entry),
	m_device(downcast<RadarDFDevice>()),
	m_resultSize(0),
	m_interval(0),
	m_stopFlag(true),
	m_searchBand(0)
    //m_process(make_unique<RadarDFProcess>())
{
	
}

RadarDFTask::~RadarDFTask()
{
	stop();
}

bool RadarDFTask::config(MessageExtractor& extractor)
{
	return true;
}

ErrorType RadarDFTask::start()
{
	if (!m_asyncFuture.valid())
	{
		setTaskStatus(TaskStatus::T_RUN);
		LOG("Device{} Create RadarDirectTask", m_device->id().value());
		return ERR_NONE;
	}
	else
	{
		return ERR_BUSY;
	}
}

ErrorType RadarDFTask::onCmd(MessageExtractor& extractor)
{
	DFTaskCmd cmd;
	if (!extractor.extract(cmd))
		return ERR_PARAM;
	bool erro;
	switch (cmd)
	{
	case UPLOAD_RADAR_PARAM:
		erro = onUploadRadarParam(extractor);
		break;
	case CALIBERATE:
		erro = onCompassCalibrate();
		break;
	case START_DF:
		erro = onDFStart(extractor);
		break;
	case STOP_DF:
		erro = onDFStop();
		break;
	case CHANGE_MATCH_POLICY:
		erro = onChangeMatchPolicy(extractor);
		break;
	case TIME_CALIB:
		erro = onTimeCalibrate(extractor);
		break;
	default:
		break;
	}
	return erro == true ? ERR_NONE : ERR_COMMAND_FAILED;
}

bool RadarDFTask::onUploadRadarParam(MessageExtractor& extractor)
{
	UploadRadarParams param;
	if (!extractor.deserialize(param))
		return false;
	m_searchBand = getMaxSearchBand(param);
	LOG("Device{} UploadRadarParam", m_device->id().value());
	if (!m_stopFlag)
	{
		if (onDFStop())
		{
			if (m_device->sendParamsConfCmd(param) == PS_ERR_NONE)
				return startDF();
		}
	}
	else
	{
		return m_device->sendParamsConfCmd(param) == PS_ERR_NONE ? true : false;
	}
}

bool RadarDFTask::onCompassCalibrate()
{
	return m_device->sendCompassCalibrateCmd() == PS_ERR_NONE ? true : false;
}

bool RadarDFTask::onTimeCalibrate(MessageExtractor& extractor)
{
	uint32_t ntpIp;
	if (!extractor.extract(ntpIp))
		return false;
	LOG("Device{} TimeCalibrate", m_device->id().value());
	return m_device->sendTimeCalibCmd(ntpIp) == PS_ERR_NONE ? true : false;
}

bool RadarDFTask::onChangeMatchPolicy(MessageExtractor& extractor)
{
	MatchPolicy policy;
	if (!extractor.extract(policy))
		return false;
	if (m_process)
	{
		m_process->changeMatchPolicy(policy);
		return true;
	}
	else
	{
		return false;
	}
}

bool RadarDFTask::onDFStart(MessageExtractor& extractor)
{
	if (!extractor.extract(m_interval))
		return false;
	reset();
	return startDF();
}

bool RadarDFTask::startDF()
{
	m_process = make_unique<RadarDFProcess>(m_device->id().value(), m_searchBand);
	auto func = [this](const ZKResult* zkResult) 
	{
		m_process->inputRawFrame(zkResult->startstopframe);
	};
	if (m_device->sendStartDetectTaskCmd(func, nullptr) != PS_ERR_NONE)
		return false;
	m_asyncFuture = std::async([this]() {taskLoop(); });
	LOG("Device{} StartDF Task", m_device->id().value());
	return true;
}

bool RadarDFTask::onDFStop()
{
	reset();
	LOG("StopDF Task");
	return m_device->sendStopDetectTaskCmd()== PS_ERR_NONE ? true : false;
}

void RadarDFTask::stop()
{
	reset();
	setTaskStatus(TaskStatus::T_ABORT);
	LOG("Device{} Delete RadarDirectTask", m_device->id().value());
	m_device->abolishAcquireFunc();//回收接受原始数据函数
	m_device->sendSelfDetectCmd(); //设备回到自检状态
}

void RadarDFTask::reset()
{
	if (!m_stopFlag)
	{
		m_stopFlag.store(true);
		if (m_asyncFuture.valid())
		{
			m_asyncFuture.get();
		}
	}
}

void RadarDFTask::taskLoop()
{
	m_stopFlag = false;
	while (!m_stopFlag)
	{
		auto result = m_process->getDFResult();
		if (result.items_size())
		{
			MessageBuilder mb;
			mb.serializeToTop(result);
			sendTaskData(mb);
			//m_resultSize += result.items_size();
			LOG("Device{} Send {} DFResultItem To Server", m_device->id().value(), result.items_size());
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(m_interval));
	}
}