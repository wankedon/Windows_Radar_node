/**
 * @file ParamsRange.h
 * @brief 参数范围限制
 * @author 装备事业部软件组 王克东
 * @version 0.1
 * @date 2021-11-19
 *
 * @copyright Copyright (c) 2021  中国电子科技集团公司第四十一研究所
 *
 */
#pragma once
#include "../pch.h"
static const  uint16_t LAST_RECORD_SPAN = 300;
static const std::pair<uint16_t, uint16_t> TARGET_COUNT(10,20);
static const std::pair<float, float> LONGITUE(0, 180);
static const std::pair<float, float> LATITUDE(0, 180);
static const std::pair<float, float> ALTITUDE(-1000, 8848);
static const std::pair<uint16_t, uint16_t> FRE(500, 18000);
static const std::pair<float, float> FREQ(500, 18000);
static const std::pair<int32_t, int32_t> AMP(-167, 60);
static const std::pair<float, float> AMPLITUDE(-167, 60);
static const std::pair<float, float> AZI_DECOUPLE(0, 360);
static const std::pair<float, float> ELE_DECOUPLE(-90, 90);
static const std::pair<float, float> PULSE_WIDTH(0, 2048);
static const std::pair<float, float> REPEAT_TIMES(0, 10000);