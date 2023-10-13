#include "PhidgetPressureSensorDAQ.h"
#include <mc_rtc/logging.h>
#include <chrono>
#include <phidget22.h>
#include <mc_rtc/clock.h>

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

PhidgetPressureSensorDAQ::PhidgetPressureSensorDAQ(const std::map<std::string, unsigned int> & sensors,
                                                   unsigned int hubSerialNumber,
                                                   double frequency,
                                                   bool required)
: hubSerialNumber_(hubSerialNumber), freq_(frequency)
{

  for(const auto & [name, port] : sensors)
  {
    sensors_.try_emplace(name, name, hubSerialNumber_, port, required);
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
          auto start_t = mc_rtc::clock::now();
          {
            std::lock_guard<std::mutex> lock(readMutex_);
            for(auto & [name, sensor] : sensors_)
            {
              /* mc_rtc::log::info("Reading {}", name); */
              sensor.read();
            }
            newData_ = true;
          }
          auto end_t = mc_rtc::clock::now();
          using duration_ms = std::chrono::duration<double, std::milli>;
          duration_ms ellapsed = end_t - start_t;
          if(ellapsed.count() < dt * 1000)
          {
            mc_rtc::log::info("Sleeping for: {}", dt * 1000 - ellapsed.count());
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(dt * 1000 - ellapsed.count()));
          }
          else
          {
                        mc_rtc::log::info("Too slow, not sleeping");

          }
        }
        for(auto & [name, sensor] : sensors_)
        {
          sensor.close();
        }
      });
#ifndef WIN32
  // Lower thread priority so that it has a lesser priority than the real time
  // thread
  auto th_handle = th_.native_handle();
  int policy = 0;
  sched_param param{};
  pthread_getschedparam(th_handle, &policy, &param);
  param.sched_priority = 10;
  if(pthread_setschedparam(th_handle, SCHED_RR, &param) != 0)
  {
    mc_rtc::log::warning("[PhidgetPressureSensorDAQ] Failed to lower thread priority. If you are running on a real-time system, "
                         "this might cause latency to the real-time loop.");
  }
#endif
}
} // namespace pps
