#include "PhidgetPressureSensorDAQ.h"
#include <mc_rtc/logging.h>
#include <chrono>
#include <phidget22.h>

namespace pps
{

template<typename ContainerT, class FwdIt, class Pr>
void erase_if(ContainerT & items, FwdIt it, FwdIt Last, Pr Pred)
{
  for(; it != Last;)
  {
    if(Pred(*it))
      it = items.erase(it);
    else
      ++it;
  }
}

PhidgetPressureSensorDAQ::PhidgetPressureSensorDAQ(const std::map<std::string, SensorConfig> & sensors,
                                                   unsigned int hubSerialNumber,
                                                   double frequency,
                                                   bool required)
: hubSerialNumber_(hubSerialNumber), freq_(frequency)
{

  for(const auto & [name, sensorConfig] : sensors)
  {
    sensors_.try_emplace(name, name, hubSerialNumber_, sensorConfig, required);
  }

  th_ = std::thread(
      [this]
      {
        double dt = 1 / freq_;
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(dt));
        // Try to initialize sensor.
        // If it fails to initialize, remove it such that no subsequent reads will be attempted
        erase_if(sensors_, sensors_.begin(), sensors_.end(), [](auto & s) { return !s.second.init(); });
        while(reading_)
        {
          std::lock_guard<std::mutex> lock(readMutex_);
          for(auto & [name, sensor] : sensors_)
          {
            /* mc_rtc::log::info("Reading {}", name); */
            sensor.read();
          }
          std::this_thread::sleep_for(duration);
        }
        for(auto & [name, sensor] : sensors_)
        {
          sensor.close();
        }
      });
}
} // namespace pps
