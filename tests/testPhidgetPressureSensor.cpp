#include <PhidgetPressureSensorDAQ.h>

using namespace pps;

int main(int argc, char * argv[])
{

  PhidgetPressureSensorDAQ daq{{
                                   {"Sensor0", 0},
                                   /* {"Sensor1", 5} */
                               },
                               pps::DEFAULT_HUB_SERIAL_NUMBER,
                               pps::DEFAULT_FREQUENCY};

  while(true)
  {
    daq.readSensorData();
    for(auto & [name, data] : daq.getLatestSensorData())
    {
      mc_rtc::log::info("{} = {}", name, data.pressure);
    }
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(1));
    std::this_thread::sleep_for(duration);
  }
  return 0;
}
