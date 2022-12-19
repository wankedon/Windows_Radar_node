#pragma once
#include "../Device.h"
#include "PositionSocketAPI.h"
#include "node/radar/radarDF.pb.h"

using namespace ZBSYB_RADAR_SOCKET;

using zkAcquireFunc = std::function<void(const ZBSYB_RADAR_SOCKET::ZKResult* db)>;
using ycAcquireFunc = std::function<void(const ZBSYB_RADAR_SOCKET::YCResult* db)>;

class RadarParamConfigLoader;
class RadarDFDevice : public Device
{
public:
	RadarDFDevice(const DeviceConfig& devConf);
	~RadarDFDevice();

public:
	bool isConnected() const { return m_connected; }
	void abolishAcquireFunc();
	void setDeviceStatus(const DeviceStatus& status)
	{
		deviceInfo.set_status(status);
	}
	virtual void onTimer() override;

public:
	PS_Error sendSelfDetectCmd();
	PS_Error sendStartDetectTaskCmd(zkAcquireFunc zkFunc, ycAcquireFunc ycFunc);
	PS_Error sendStopDetectTaskCmd();
	PS_Error sendSendLockTrackCmd(const radarDF::UploadRadarParams& uploadradarparams);
	PS_Error sendParamsConfCmd(const radarDF::UploadRadarParams& uploadradarparams);
	PS_Error sendResetCmd();
	PS_Error sendGPSModConfCmd();
	PS_Error sendCompassCalibrateCmd();
	PS_Error sendIPConfCmd(uint32_t address);
	PS_Error sendLowPowerCmd(uint32_t option);
	PS_Error sendWaveGateCmd(uint32_t option);
	PS_Error sendAGCCtrlCmd(uint32_t option, uint32_t agcvalue);
	PS_Error sendDetectThrdCmd(uint32_t value);
	PS_Error sendSoftwareUpdateCmd(uint32_t option, uint32_t type);
	PS_Error sendDeviceStatusSearchCmd();
	PS_Error sendStandardCalibCmd(uint32_t frqvalue);
	PS_Error sendTimeCalibCmd(uint32_t ntpAddress);

private:
	static void zkConnectionEventHandler(ZBSYB_RADAR_SOCKET::ChannelState state, void* userPointer);
	static void ycConnectionEventHandler(ZBSYB_RADAR_SOCKET::ChannelState state, void* userPointer);
	static void zkResultHandler(const ZBSYB_RADAR_SOCKET::ZKResult* zkResult, void* userPointer);
	static void ycResultHandler(const ZBSYB_RADAR_SOCKET::YCResult* ycResult, void* userPointer);
	void onZKConnectionEvent(ZBSYB_RADAR_SOCKET::ChannelState state);
	void onYCConnectionEvent(ZBSYB_RADAR_SOCKET::ChannelState state);
	void onZKResult(const ZBSYB_RADAR_SOCKET::ZKResult* zkResul);
	void onYCResult(const ZBSYB_RADAR_SOCKET::YCResult* ycResult);

	//更新设备信息 自检无温度
	template<class T>
	void extractDeviceInfo(const T& frame)
	{
		Position pos;
		if (m_pos.longitude() == 0 || m_pos.latitude() == 0 || m_pos.altitude() == 0)
		{
			pos.set_altitude(frame->GPSheight);
			pos.set_latitude(frame->GPSlat);
			pos.set_longitude(frame->GPSlong);
		}
		else
		{
			pos = m_pos;
		}
		double temperature = frame->temperature;
		updateDeviceInfo(pos, temperature);
	}

	void updateDeviceInfo(Position pos, double temperature);
	void setAcquireFunc(zkAcquireFunc zkFunc, ycAcquireFunc ycFunc);
	void responsePrint(ZBSYB_RADAR_SOCKET::PS_Error error, std::string cmdName);
	bool checkConnect();
	void calibrateTime(ZBSYB_RADAR_SOCKET::StartStopFrame* startstopframe);
	PS_Error calculateDiffTime(const ZBSYB_RADAR_SOCKET::TimeCalibFrame& timecalibframe);
	void loadWriteTimeDiff();
	void loadDevicePosition(uint32_t deviceId);
	void loadTimeZone();

private:
	PS_Error m_timeCalibCmdError;
	int m_handle;
	bool m_connected;
	const std::string m_name;
	Position m_pos;
	ZBSYB_RADAR_SOCKET::TimeDiff m_diffTime;
	zkAcquireFunc m_ZKResultAcquireFunc;
	ycAcquireFunc m_YCResultAcquireFunc;
	std::unique_ptr<RadarParamConfigLoader> m_config;
};