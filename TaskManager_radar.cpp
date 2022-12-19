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
	// 初始化任务结果的发送器
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
	case T_MODIFY:	//任务进行中用户输入的任务控制指令
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
		header.set_error_code(ERR_NODE_TASK_ASSIGN);	//填入响应的错误码
		return;
	}
	if (!newTask->config(extractor))//配置任务
	{
		header.set_error_code(ERR_PARAM);
		return;
	}
	auto error = newTask->start();	//配置成功后即启动
	if (error == ERR_NONE)
	{
		auto key = combineId(newTask->taskId(), newTask->deviceId());
		tasks[key] = newTask;	//加入任务列表
		*header.mutable_task_runner()->mutable_device_id() = newTask->deviceId();
	}
	header.set_error_code(error);
	
}

void TaskManager::taskCmd(MessageExtractor& extractor, CmdHeader& header)
{
	auto key = combineId(header.task_id(), header.task_runner().device_id());
	auto iter = tasks.find(key);	//根据nodeDevice查找task
	if (iter != tasks.end())
	{
		header.set_error_code(iter->second->onCmd(extractor));	//让任务去执行命令
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
		iter->second->stop();	//任务停止，一般会停止任务的若干后台线程，可能会阻塞直到所有任务相关线程退出
		header.set_error_code(ERR_NONE);
		tasks.erase(iter);	//从集合中清除该任务
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
			iter = tasks.erase(iter);	//清理任务
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
	 
