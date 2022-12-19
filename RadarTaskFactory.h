/**
 * @file RadarTaskFactory.h
 * @brief �״�������񹤳���������NodeTaskFactory
 * @author װ����ҵ������� ���˶�
 * @version 0.1
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022  �й����ӿƼ����Ź�˾����ʮһ�о���
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