/**
 * @file RadarTaskFactory.h
 * @brief 雷达测向任务工厂，派生自NodeTaskFactory
 * @author 装备事业部软件组 王克东
 * @version 0.1
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022  中国电子科技集团公司第四十一研究所
 *
 */
#pragma once
#include "../NodeTaskFactory.h"

class RadarTaskFactory : public NodeTaskFactory
{
public:
    RadarTaskFactory();
    ~RadarTaskFactory() = default;
public:
    std::shared_ptr<NodeTask> createTask(NodeTask::TaskInitEntry&, const DeviceRequestFunc&) override;
};