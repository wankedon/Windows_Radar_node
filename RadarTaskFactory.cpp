/**
 * @file RadarTaskFactory.cpp
 * @brief �״�������񹤳�
 * @author װ����ҵ������� ���˶�
 * @version 0.1
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022  �й����ӿƼ����Ź�˾����ʮһ�о���
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
	if (entry.header.task_runner().device_id().value() == 0)	//���Զ�ָ�����ڴ�ָ�������豸id
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