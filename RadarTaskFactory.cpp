/**
 * @file RadarTaskFactory.cpp
 * @brief 雷达测向任务工厂
 * @author 装备事业部软件组 王克东
 * @version 0.1
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022  中国电子科技集团公司第四十一研究所
 *
 */
#include "../pch.h"
#include "RadarTaskFactory.h"
#include "RadarDFTask.h"
#include "../Device.h"

using namespace std;

RadarTaskFactory::RadarTaskFactory()
{
}

std::shared_ptr<NodeTask> RadarTaskFactory::createTask(NodeTask::TaskInitEntry& entry, const DeviceRequestFunc& deviceRequest)
{
	auto primaryType = entry.type.pri_task_type();
	auto secondaryType = entry.type.sec_task_type();
	if (primaryType != RADAR_DF_TASK)
		return nullptr;
	std::shared_ptr<NodeTask> task = nullptr;
	entry.device = deviceRequest({ entry.header.task_runner().device_id(), RADAR_DF });
	if (!entry.device)
		return nullptr;
	if (entry.header.task_runner().device_id().value() == 0)	//若自动指派需在此指定具体设备id
	{
		*(entry.header.mutable_task_runner()->mutable_device_id()) = entry.device->id();
	}
	switch (secondaryType)
	{
	case DIRECTION_FINDING_SPATIAL_SPECTRUM:
		task = make_shared<RadarDFTask>(entry);
		break;
	default:
		break;
	}
	return task;
}