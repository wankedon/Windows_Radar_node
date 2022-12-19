/**
 * @file TargetSnapShot.h
 * @brief ����Ŀ�����
 * @author װ����ҵ������� ���˶�
 * @version 0.1
 * @date 2021-11-19
 *
 * @copyright Copyright (c) 2021  �й����ӿƼ����Ź�˾����ʮһ�о���
 *
 */
#pragma once
#include "PositionSocketAPI.h"
#include "Target.h"
#include "node/radar/radarDF.pb.h"

class Target;
class TargetSnapShot
{
public:
	TargetSnapShot(std::function<void(radarDF::DFResultItem& result)> aquairFunc);
	~TargetSnapShot();

public:
	void inputTarget(std::shared_ptr<Target> target);
	void aquireDFResultItem();

private:
	radarDF::DFResultItem generateDFresultItem(std::shared_ptr<Target> target);
	radarDF::DFResultItem extractDFResultItem(TargetForRead& targetForRead);
	radarDF::PulseCluster extractPulseCluster(TargetForRead& targetForRead);

private:
	std::list<std::shared_ptr<Target>> m_targetList;
	std::function<void(radarDF::DFResultItem& result)> m_aquairFunc;
};