#include "../pch.h"
#include "RadarDFDevice.h"
#include "toolsFunc.h"
#include "Logger.h"
#include "RadarParamConfigLoader.h"
#include "Miscellaneous.h"

using namespace std;
using namespace radarDF;

RadarDFDevice::RadarDFDevice(const DeviceConfig& devConf)
	:Device(RADAR_DF, D_IDLE),
	m_name(fmt::format("RadarDevice {}", devConf.address().ip())),
	m_handle(0),
	m_connected(false),
	m_diffTime(TimeDiff{ 0,0 }),
	m_timeCalibCmdError(PS_Error::PS_ERR_SEND),
	m_config(make_unique<RadarParamConfigLoader>(LocateModulePath()+ L"\\节点参数配置.xml"))
{
	if (devConf.has_id())
		*deviceInfo.mutable_id() = devConf.id();
	//填充回调函数
	SessionHandler callback{ 0 };
	callback.userPointer = this;
	callback.cbZK.cbCon = zkConnectionEventHandler;
	callback.cbZK.cbResult = zkResultHandler;
	callback.cbYC.cbCon = ycConnectionEventHandler;
	callback.cbYC.cbResult = ycResultHandler;
	//建立连接
	auto ipStr = fmt::format("{}:{}", devConf.address().ip(), devConf.address().port());
	PS_CreateConnect(&m_handle, ipStr.c_str(), &callback);
	loadDevicePosition(id().value());
}

RadarDFDevice::~RadarDFDevice()
{
	if (m_handle != 0)
	{
		PS_CloseConnect(m_handle);
		m_handle = 0;
	}
}

void RadarDFDevice::loadDevicePosition(uint32_t deviceId)
{
	auto pos = m_config->loadPosition(deviceId);
	m_pos.set_longitude(get<0>(pos));
	m_pos.set_latitude(get<1>(pos));
	m_pos.set_altitude(get<2>(pos));
}

//设置获取结果函数
void RadarDFDevice::setAcquireFunc(zkAcquireFunc zkFunc, ycAcquireFunc ycFunc)
{
	m_ZKResultAcquireFunc = zkFunc;
	m_YCResultAcquireFunc = ycFunc;
}

//废除获取结果函数
void RadarDFDevice::abolishAcquireFunc()
{
	m_ZKResultAcquireFunc = nullptr;
	m_ZKResultAcquireFunc = nullptr;
}

void RadarDFDevice::zkConnectionEventHandler(ChannelState state, void* userPointer)
{
	RadarDFDevice* self = (RadarDFDevice*)userPointer;
	self->onZKConnectionEvent(state);
}

void RadarDFDevice::onZKConnectionEvent(ChannelState state)
{
	m_connected = (state == CONNECTED); //用终控通路判断连接
	DeviceStatus status = m_connected ? D_IDLE : D_OFFLINE;
	deviceInfo.set_status(status);
}

void RadarDFDevice::ycConnectionEventHandler(ChannelState state, void* userPointer)
{
	RadarDFDevice* self = (RadarDFDevice*)userPointer;
	self->onYCConnectionEvent(state);
}

void RadarDFDevice::onYCConnectionEvent(ChannelState state)
{
	//m_connected = (state == CONNECTED); //用遥测通路判断连接
	//DeviceStatus status = m_connected ? D_IDLE : D_OFFLINE;
	//deviceInfo.set_status(status);
}

void RadarDFDevice::zkResultHandler(const ZKResult* zkResult, void* userPointer)
{
	RadarDFDevice* self = (RadarDFDevice*)userPointer;
	self->onZKResult(zkResult);
}

void RadarDFDevice::ycResultHandler(const ZBSYB_RADAR_SOCKET::YCResult* ycResult, void* userPointer)
{
	RadarDFDevice* self = (RadarDFDevice*)userPointer;
	self->onYCResult(ycResult);
}

void RadarDFDevice::onYCResult(const ZBSYB_RADAR_SOCKET::YCResult* ycResult)
{
	if (m_YCResultAcquireFunc)
	{
		m_YCResultAcquireFunc(ycResult);
	}
}

void RadarDFDevice::onZKResult(const ZKResult* zkResult)
{
	if (zkResult == nullptr)
		return;
	ZKResult tempZkResult = *zkResult;
	if ((tempZkResult.startstopframe != nullptr) || (tempZkResult.locktrackframe != nullptr) || (zkResult->selfdetectframe != nullptr))
	{
		//自检/启动停止/锁定跟踪指令任何其一回传数据都包含设备信息
		if (tempZkResult.startstopframe != nullptr)
		{
#ifdef DCLJ
			calibrateTime(tempZkResult.startstopframe);
#endif
			extractDeviceInfo(tempZkResult.startstopframe);
		}
		else if (tempZkResult.locktrackframe != nullptr)
		{
			extractDeviceInfo(tempZkResult.locktrackframe);
		}
		else if (tempZkResult.selfdetectframe != nullptr)
		{
			extractDeviceInfo(tempZkResult.selfdetectframe);
		}
	}
	if (m_ZKResultAcquireFunc)
	{
		m_ZKResultAcquireFunc(&tempZkResult);
	}
}

void RadarDFDevice::calibrateTime(StartStopFrame* startstopframe)
{
	UTCTime deviceUtc;
	if (m_timeCalibCmdError == PS_Error::PS_ERR_NONE)
	{
		auto deviceMsec = timeConvertToMsec(*startstopframe) + m_diffTime.sec * 1000 + m_diffTime.msec;
		deviceUtc = msecToUtc(deviceMsec);
	}
	else
	{
		deviceUtc = getCurrentUtc();
	}
	startstopframe->year = deviceUtc.year;
	startstopframe->month = deviceUtc.mon;
	startstopframe->day = deviceUtc.day;
	startstopframe->hour = deviceUtc.hour;
	startstopframe->minute = deviceUtc.min;
	startstopframe->sec = deviceUtc.second;
	startstopframe->msec = deviceUtc.millisecond;
}

//填充设备信息
void RadarDFDevice::updateDeviceInfo(Position pos, double temperature)
{
	*deviceInfo.mutable_position() = pos;
	deviceInfo.clear_physicals();
	auto physical = deviceInfo.add_physicals();
	physical->set_type(Physical::TEMPERATURE);
	physical->set_value(temperature);
	physical->set_unit("");
}

bool RadarDFDevice::checkConnect()
{
	if (!m_connected)
	{
		CLOG("{} Not Connect", m_name);
		return false;
	}
	return true;
}

//输出指令响应信息
void RadarDFDevice::responsePrint(PS_Error error, string cmdName)
{
	if (error == PS_Error::PS_ERR_NONE)
		CLOG("{} {}...", m_name, cmdName);
	else
		CLOG("{} {} Send Failed", m_name, cmdName);
}

//自检
PS_Error RadarDFDevice::sendSelfDetectCmd()
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	auto error = PS_SendSelfDetect(m_handle);
	responsePrint(error, "SelfDetect");
	return error;
}

PS_Error RadarDFDevice::sendStartDetectTaskCmd(zkAcquireFunc zkFunc, ycAcquireFunc ycFunc)
{
	setAcquireFunc(zkFunc, ycFunc);
	if (!checkConnect())
		return PS_ERR_CONNECT;
	auto error = PS_SendStartStop(m_handle, true);
	responsePrint(error, "Start Detect");
	return error;
}

PS_Error RadarDFDevice::sendStopDetectTaskCmd()
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	auto error = PS_SendStartStop(m_handle, false);
	responsePrint(error, "Stop Detect");
	abolishAcquireFunc();//回收接受原始数据函数
	return error;
}

//锁定跟踪
PS_Error RadarDFDevice::sendSendLockTrackCmd(const UploadRadarParams& uploadradarparams)
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	PS_Error error = PS_ERR_NONE;
	list<ParamConfParam> confParamList = convertToDeviceParam(uploadradarparams);
	for (auto& cp : confParamList)
	{
		LockTrackParam lp;
		memcpy(&lp, &cp, sizeof(ParamConfParam));
		error = PS_SendLockTrack(m_handle, &lp);
		responsePrint(error, "LockTrack");
	}
	return error;
}

//雷达库装库
PS_Error RadarDFDevice::sendParamsConfCmd(const UploadRadarParams& uploadradarparams)
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	PS_Error error = PS_ERR_NONE;
	list<ParamConfParam> confParamList = convertToDeviceParam(uploadradarparams);
	for (auto& cp : confParamList)
	{
		Sleep(15);
		ParamConfFrame paramConfigBackData;
		error = PS_SendParaConf(&paramConfigBackData, m_handle, &cp);
		responsePrint(error, "ParamsConf");
	}
	return error;
}

//复位
PS_Error RadarDFDevice::sendResetCmd()
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	ResetFrame resetBackData;
	auto error = PS_SendReset(&resetBackData, m_handle);
	responsePrint(error, "Reset");
	return error;
}

//GPS模块配置
PS_Error RadarDFDevice::sendGPSModConfCmd()
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	GPSConfFrame gpsconfframe;
	auto error = PS_SendGPSModConf(&gpsconfframe, m_handle);
	responsePrint(error, "GPSModConf");
	return error;
}

//电子罗盘校准
PS_Error RadarDFDevice::sendCompassCalibrateCmd()
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	CompassCalFrame compassData;
	auto error = PS_SendCompassCalib(&compassData, m_handle);
	responsePrint(error, "CompassCalibrate");
	return error;
}

//IP配置
PS_Error RadarDFDevice::sendIPConfCmd(uint32_t address)
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	IpConfFrame ipconfframe;
	auto error = PS_SendIPConf(&ipconfframe, m_handle, address);
	responsePrint(error, "IPConf");
	return error;
}

//低功耗
PS_Error RadarDFDevice::sendLowPowerCmd(uint32_t option)
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	LowPowerConFrame lowpowerconframe;
	auto error = PS_SendLowPower(&lowpowerconframe, m_handle, option);
	responsePrint(error, "LowPower");
	return error;
}

//波门控制
PS_Error RadarDFDevice::sendWaveGateCmd(uint32_t option)
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	WaveGateSwitchFrame wavegateswitchframe;
	auto error = PS_SendWaveGate(&wavegateswitchframe, m_handle, option);
	responsePrint(error, "WaveGate");
	return error;
}

//AGC控制
PS_Error RadarDFDevice::sendAGCCtrlCmd(uint32_t option, uint32_t agcvalue)
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	AgcControlFrame agccontrolframe;
	auto error = PS_SendAGCCtrl(&agccontrolframe, m_handle, option, agcvalue);
	responsePrint(error, "AGCCtrl");
	return error;
}

//检测门限控制
PS_Error RadarDFDevice::sendDetectThrdCmd(uint32_t value)
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	DetectThresholdFrame detectthresholdframe;
	auto error = PS_SendDetectThrd(&detectthresholdframe, m_handle, value);
	responsePrint(error, "DetectThrd");
	return error;
}

//软件更新
PS_Error RadarDFDevice::sendSoftwareUpdateCmd(uint32_t option, uint32_t type)
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	SoftwareUpdateFrame softwareupdateframe;
	auto error = PS_SendSoftwareUpdate(&softwareupdateframe, m_handle, option, type);
	responsePrint(error, "SoftwareUpdate");
	return error;
}

//设备状态查询
PS_Error RadarDFDevice::sendDeviceStatusSearchCmd()
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	DeviceStatusSearchFrame devicestatussearchframe;
	auto error = PS_SendDeviceStatusSearch(&devicestatussearchframe, m_handle);
	responsePrint(error, "DeviceStatusSearch");
	return error;
}

//标校
PS_Error RadarDFDevice::sendStandardCalibCmd(uint32_t frqvalue)
{
	if (!checkConnect())
		return PS_ERR_CONNECT;
	StandardCalibFrame standardcalibframe;
	auto error = PS_SendStandardCalib(&standardcalibframe, m_handle, frqvalue);
	responsePrint(error, "StandardCalib");
	return error;
}

PS_Error RadarDFDevice::sendTimeCalibCmd(uint32_t ntpIp)
{
	loadTimeZone();
	if (!checkConnect())
		return PS_ERR_CONNECT;
	TimeCalibFrame timecalibframe;
	m_timeCalibCmdError = PS_SendTimeCalib(&timecalibframe, m_handle, ntpIp);
	responsePrint(m_timeCalibCmdError, "TimeCalib_NTPServer:" + uintToIp(ntpIp));
	if (m_timeCalibCmdError != PS_Error::PS_ERR_NONE)
		return m_timeCalibCmdError;
	else
		return calculateDiffTime(timecalibframe);
}

void RadarDFDevice::loadTimeZone()
{
	m_diffTime = TimeDiff{ 0,0 };
	if (m_config->loadTimezone() == 8)
		m_diffTime.sec = 28800;
}

PS_Error RadarDFDevice::calculateDiffTime(const TimeCalibFrame& timecalibframe)
{
	PS_Error erro= PS_Error::PS_ERR_NONE;
	if (timecalibframe.T1_time.sec != 0)
	{
		m_diffTime.sec = m_diffTime.sec + timecalibframe.Diff_time.sec;
		m_diffTime.msec = timecalibframe.Diff_time.msec;
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_GREEN);
		LOG("设备{} 对时成功,回传的时差为:{}.{}", id().value(),timecalibframe.Diff_time.sec, timecalibframe.Diff_time.msec);
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
	}
	else
	{
		auto diffTime = getCurrentTimeMsec()- timeConvertToMsec(timecalibframe);
		m_diffTime.sec = diffTime / 1000;
		m_diffTime.msec = diffTime % 1000;
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_RED);
		LOG("设备{} 对时失败,本地计算的时差:{}.{}", id().value(), m_diffTime.sec, m_diffTime.msec);
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
		erro = PS_Error::PS_ERR_SEND;
	}
	loadWriteTimeDiff();
	return erro;
}

void RadarDFDevice::loadWriteTimeDiff()
{
	auto timediff = m_config->loadTimeDiff(id().value());
	if (get<0>(timediff) == 1)
	{
		m_diffTime.sec = get<1>(timediff);
		m_diffTime.msec = get<2>(timediff);
		LOG("读取配置时差:{}.{}", m_diffTime.sec, m_diffTime.msec);
	}
	m_config->writeTimeDiff(make_tuple(id().value(), m_diffTime.sec, m_diffTime.msec));
	LOG("保存时差:{}.{}", m_diffTime.sec, m_diffTime.msec);
}

void RadarDFDevice::onTimer()
{
}