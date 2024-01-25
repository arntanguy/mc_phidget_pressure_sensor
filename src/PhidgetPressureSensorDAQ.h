#pragma once

#include <mc_rtc/logging.h>
#include <mc_rtc/type_name.h>
#include <map>
#include <mutex>
#include <phidget22.h>
#include <string>
#include <thread>

namespace pps
{

constexpr unsigned int DEFAULT_HUB_SERIAL_NUMBER = 683870;
constexpr unsigned int DEFAULT_FREQUENCY = 100;
#define SAFECALL(x, ...) safecall(#x, x, __VA_ARGS__)

struct PhidgetPressureSensorData
{
  double current = 0;
  double pressure = 0;
};

struct SensorConfig
{
  SensorConfig() {}
  SensorConfig(unsigned port) : port(port) {}
  SensorConfig(unsigned port, double coeff1, double coeff2) : port(port), coeff1(coeff1), coeff2(coeff2) {}
  unsigned port = 0;
  // @lea set correct initial value such that they don't affect the computation
  double coeff1 = 0;
  double coeff2 = 0;
};

struct PhidgetPressureSensor
{
  PhidgetPressureSensor(const std::string name,
                        unsigned int hubSerialNumber,
                        const SensorConfig & sensorConfig,
                        bool required = true)
  : name_(name), hubSerialNumber_(hubSerialNumber), sensorConfig_(sensorConfig), required_(required)
  {
  }

  ~PhidgetPressureSensor()
  {
    close();
  }

  template<typename T, typename... Args>
  bool safecall(std::string && name, T fun, Args &&... args)
  {
    ;
    if(fun(args...) != EPHIDGET_OK)
    {
      if(required_)
      {
        mc_rtc::log::error_and_throw("[PhidgetPressureSensor] Error in Phidget API call to function {}", name);
      }
      else
      {
        mc_rtc::log::warning("[PhidgetPressureSensor] Error in Phidget API call to function {}", name);
      }
      return false;
    }
    return true;
  }

  bool init()
  {
    auto portNumber = sensorConfig_.port;
    mc_rtc::log::info("Init with hub serial {} and port {}", hubSerialNumber_, portNumber);
    return SAFECALL(PhidgetCurrentInput_create, &sensor)
           && SAFECALL(Phidget_setDeviceSerialNumber, (PhidgetHandle)sensor, hubSerialNumber_)
           && SAFECALL(Phidget_setHubPort, (PhidgetHandle)sensor, portNumber)
           && SAFECALL(Phidget_openWaitForAttachment, (PhidgetHandle)sensor, 1000);
  }

  bool close()
  {
    bool success = true;
    return SAFECALL(Phidget_close, (PhidgetHandle)sensor) && SAFECALL(PhidgetCurrentInput_delete, &sensor);
  }

  bool read()
  {
    bool success = SAFECALL(PhidgetCurrentInput_getCurrent, sensor, &data_.current);
    if(success)
    {
      // @lea do whatever computation you want with sensorConfig.coeff1, sensorConfig.coeff2
      data_.pressure = (data_.current - 0.004) * (10.0 / 0.016);
    }
    return success;
  }

  inline PhidgetPressureSensorData data() const noexcept
  {
    return data_;
  }

  inline const std::string & name() const noexcept
  {
    return name_;
  }

protected:
  bool required_ = true;
  std::string name_;
  unsigned int hubSerialNumber_ = 0;
  SensorConfig sensorConfig_;
  PhidgetCurrentInputHandle sensor;
  PhidgetPressureSensorData data_;
};

struct PhidgetPressureSensorDAQ
{
  PhidgetPressureSensorDAQ(const std::map<std::string, SensorConfig> & sensors,
                           unsigned int hubSerialNumber = DEFAULT_HUB_SERIAL_NUMBER,
                           double frequency = DEFAULT_FREQUENCY,
                           bool required = true);

  ~PhidgetPressureSensorDAQ()
  {
    reading_ = false;
  }

  void readSensorData()
  {
    std::lock_guard<std::mutex> lock(readMutex_);
    for(auto & [name, sensor] : sensors_)
    {
      latestData_[name] = sensor.data();
    }
  }

  const std::map<std::string, PhidgetPressureSensorData> & getLatestSensorData() const
  {
    return latestData_;
  }

protected:
  bool required_ = true;
  int hubSerialNumber_ = 0;
  double freq_ = DEFAULT_FREQUENCY;
  std::map<std::string, PhidgetPressureSensor> sensors_;
  std::thread th_;
  mutable std::mutex readMutex_;
  std::atomic<bool> reading_{true};
  std::map<std::string, PhidgetPressureSensorData> latestData_;
};

} // namespace pps
