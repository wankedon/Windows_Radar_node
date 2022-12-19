#include "pch.h"
#include "TaskManager.h"
#include "PushPull.h"
#include "TaskCreator.h"
#include "NodeTask.h"
#include "Logger.h"
#include "node/nodeInternal.pb.h"
#include "AbnormalMsgInfo.h"
#include "RadarTaskFactory.h"

using namespace std;

TaskManager::TaskManager(const ZeromqLinkCfg& sourceCfg, const std::string& path, DeviceRequestFunc deviceRequest)
	:dataSender(make_unique<StreamSource>(sourceCfg))
{
	// ��ʼ���������ķ�����
	TaskDataSendFunc func = [this](MessageBuilder& builder)
	{
		return dataSender->send(builder);
	};
	string dbPath = path + "/sensor.db";
	taskCreator = make_unique<TaskCreator>(deviceRequest, func);
	taskCreator->addFactory(make_shared<RadarTaskFactory>());
}

TaskManager::~TaskManager()
{
}

unique_ptr<CmdHeader> TaskManager::onRemoteRequest(MessageExtractor& extractor)
{
	CmdHeader header;
	TaskCmd cmdType;
	if (!extractor.deserialize(header))
		return nullptr;
	if (!extractor.extract(cmdType))
		return nullptr;
	switch (cmdType)
	{
	case T_START:
		taskStart(extractor, header);
		break;
	case T_STOP:
		taskStop(header);
		break;
	case T_MODIFY:	//����������û�������������ָ��
		taskCmd(extractor, header);
		break;
	default:
		header.set_error_code(ERR_PARAM);
		break;
	}
	return make_unique<CmdHeader>(header);
}

void TaskManager::taskStart(MessageExtractor& extractor, CmdHeader& header)
{
	TaskType type;
	extractor.deserialize(type);
	auto newTask = taskCreator->createTask(header, type);
	if (newTask == nullptr)
	{
		header.set_error_code(ERR_NODE_TASK_ASSIGN);	//������Ӧ�Ĵ�����
		return;
	}
	if (!newTask->config(extractor))//��������
	{
		header.set_error_code(ERR_PARAM);
		return;
	}
	auto error = newTask->start();	//���óɹ�������
	if (error == ERR_NONE)
	{
		auto key = combineId(newTask->taskId(), newTask->deviceId());
		tasks[key] = newTask;	//���������б�
		*header.mutable_task_runner()->mutable_device_id() = newTask->deviceId();
	}
	header.set_error_code(error);
	
}

void TaskManager::taskCmd(MessageExtractor& extractor, CmdHeader& header)
{
	auto key = combineId(header.task_id(), header.task_runner().device_id());
	auto iter = tasks.find(key);	//����nodeDevice����task
	if (iter != tasks.end())
	{
		header.set_error_code(iter->second->onCmd(extractor));	//������ȥִ������
	}
	else
	{
		header.set_error_code(ERR_TASK_NOT_FOUND);
	}
}

void TaskManager::taskStop(CmdHeader& header)
{
	auto key = combineId(header.task_id(), header.task_runner().device_id());
	auto iter = tasks.find(key);
	if (iter != tasks.end())
	{
		iter->second->stop();	//����ֹͣ��һ���ֹͣ��������ɺ�̨�̣߳����ܻ�����ֱ��������������߳��˳�
		header.set_error_code(ERR_NONE);
		tasks.erase(iter);	//�Ӽ��������������
		LOG("task {:x} deleted, cur task count:{}", header.task_id().value(), tasks.size());
	}
	else
	{
		header.set_error_code(ERR_TASK_NOT_FOUND);
	}
}

void TaskManager::taskAbortAll()
{
	for (auto t : tasks)
	{
		t.second->stop();
	}
	tasks.clear();
}

void TaskManager::onTimer(AbnormalMsgInfoClient& abnormalMsgInfo)
{
	auto record = abnormalMsgInfo.receivedRecord();
	for (auto iter = tasks.begin(); iter != tasks.end();)
	{
		if(iter->second->canErase(record))
		{
			LOG("task {:x} abnormal deleted, cur task count:{}", iter->second->taskId().value(), tasks.size() - 1);
			iter->second->stop();
			iter = tasks.erase(iter);	//��������
		}
		else
		{
			iter++;
		}
	}
}

void TaskManager::updateInfo(NodeInfo& info) const
{
	auto* ts = info.mutable_tasks();
	for (auto& t : tasks)
	{
		*ts->Add() = t.second->getSummary();
	}
}
	 
