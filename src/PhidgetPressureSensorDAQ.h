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

template<typename T, typename... Args>
void safecall(std::string && name, T fun, Args &&... args)
{
  auto res = fun(args...);
  if(res != EPHIDGET_OK)
  {
    mc_rtc::log::error_and_throw("[PhidgetPressureSensor] Error in Phidget API call to function {}", name);
  }
}

#define SAFECALL(x, ...) safecall(#x, x, __VA_ARGS__)

struct PhidgetPressureSensorData
{
  double current = 0;
  double pressure = 0;
};

struct PhidgetPressureSensor
{
  PhidgetPressureSensor(const std::string name, unsigned int hubSerialNumber, unsigned int portNumber)
  : name_(name), hubSerialNumber_(hubSerialNumber), portNumber_(portNumber)
  {
  }

  ~PhidgetPressureSensor()
  {
    close();
  }

  void init()
  {
    mc_rtc::log::info("Init with hub serial {} and port {}", hubSerialNumber_, portNumber_);
    SAFECALL(PhidgetCurrentInput_create, &sensor);
    SAFECALL(Phidget_setDeviceSerialNumber, (PhidgetHandle)sensor, hubSerialNumber_);
    SAFECALL(Phidget_setHubPort, (PhidgetHandle)sensor, portNumber_);
    SAFECALL(Phidget_openWaitForAttachment, (PhidgetHandle)sensor, 1000);
  }

  void close()
  {
    SAFECALL(Phidget_close, (PhidgetHandle)sensor);
    SAFECALL(PhidgetCurrentInput_delete, &sensor);
  }

  void read()
  {
    SAFECALL(PhidgetCurrentInput_getCurrent, sensor, &data_.current);
    data_.pressure = (data_.current - 0.004) * (10.0 / 0.016);
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
  std::string name_;
  unsigned int hubSerialNumber_ = 0;
  unsigned int portNumber_ = 0;
  PhidgetCurrentInputHandle sensor;
  PhidgetPressureSensorData data_;
};

struct PhidgetPressureSensorDAQ
{
  PhidgetPressureSensorDAQ(const std::map<std::string, unsigned int> & sensors,
                           unsigned int hubSerialNumber = DEFAULT_HUB_SERIAL_NUMBER,
                           double frequency = DEFAULT_FREQUENCY);

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
  int hubSerialNumber_ = 0;
  double freq_ = DEFAULT_FREQUENCY;
  std::map<std::string, PhidgetPressureSensor> sensors_;
  std::thread th_;
  mutable std::mutex readMutex_;
  std::atomic<bool> reading_{true};
  std::map<std::string, PhidgetPressureSensorData> latestData_;
};

} // namespace pps
