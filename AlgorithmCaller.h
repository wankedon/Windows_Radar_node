#pragma once
#include "PositionSocketAPI.h"
#include "TargetMatch.h"
#include "node/radar/radarDF.pb.h"

class MatchAlgorithm;
class AlgorithmCaller
{

public:
	AlgorithmCaller(ZBSYB_RADAR_DIRECTION::ConfigData* configData,radarDF::MatchPolicy policy);
	~AlgorithmCaller();

public:
	void changePolicy(const radarDF::MatchPolicy& policy);
	int matching(ZBSYB_RADAR_SOCKET::StartStopFrame* startStopFrame);
	ZBSYB_RADAR_DIRECTION::TargetOutput targetMatching(ZBSYB_RADAR_SOCKET::StartStopFrame* startStopFrame);

private:
	radarDF::MatchPolicy m_policy;
	std::shared_ptr<MatchAlgorithm> m_algorithm;
	static const int COUNT = 10;
};