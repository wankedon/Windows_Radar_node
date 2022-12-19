#include "../pch.h"
#include "AlgorithmCaller.h"
#include "MatchAlgorithm.h"
#include "toolsFunc.h"

AlgorithmCaller::AlgorithmCaller(ConfigData* configData,radarDF::MatchPolicy policy)
	:m_policy(policy)
{
	//changePolicy(policy);
	m_algorithm = std::make_shared<FirstMatchAlgorithm>(configData);
}

AlgorithmCaller::~AlgorithmCaller()
{
	
}

void AlgorithmCaller::changePolicy(const radarDF::MatchPolicy& policy)
{
	m_policy = policy;
	/*
	switch (policy)
	{
	case MatchMethod::PolicyFirst:
		m_algorithm = std::make_shared<FirstMatchAlgorithm>();
		break;
	case MatchMethod::PolicySecond:
		m_algorithm = std::make_shared<SecondMatchAlgorithm>();
		break;
	default:
		break;
	}
	*/
}

int AlgorithmCaller::matching(ZBSYB_RADAR_SOCKET::StartStopFrame* startStopFrame)
{
	auto item = convertDataFrameToMatchTargetItem(startStopFrame);
	return m_algorithm->matching(&item);
}

ZBSYB_RADAR_DIRECTION::TargetOutput AlgorithmCaller::targetMatching(ZBSYB_RADAR_SOCKET::StartStopFrame* startStopFrame)
{
	auto item = convertDataFrameToDetectDataInput(startStopFrame);
	switch (m_policy)
	{
	case radarDF::MatchPolicy::POLICY_FIRST:
		return m_algorithm->targetMatching(&item);;
		break;
	case radarDF::MatchPolicy::POLICY_SECOND:
		return ZBSYB_RADAR_DIRECTION::TargetOutput();
		break;
	default:
		break;
	}
}