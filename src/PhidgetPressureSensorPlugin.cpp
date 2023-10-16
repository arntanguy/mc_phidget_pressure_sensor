#include "PhidgetPressureSensorPlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>
#include "PhidgetPressureSensorDAQ.h"

namespace mc_plugin
{

PhidgetPressureSensorPlugin::~PhidgetPressureSensorPlugin() = default;

void PhidgetPressureSensorPlugin::init(mc_control::MCGlobalController & controller,
                                       const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("PhidgetPressureSensorPlugin::init called with configuration:\n{}", config.dump(true, true));

  config("required", required_);

  std::map<std::string, mc_rtc::Configuration> hubs = config("hubs", mc_rtc::Configuration{});
  if(hubs.empty())
  {
    warn_or_throw("[PhidgetPressureSensorPlugin] No sensor configuration provided");
  }
  for(const auto & [hubName, hubConfig] : hubs)
  {
    std::map<std::string, mc_rtc::Configuration> sensorsConfig = hubConfig("sensors", mc_rtc::Configuration{});
    std::map<std::string, unsigned int> sensors;
    if(!hubConfig.has("serial_number"))
    {
      warn_or_throw("[PhidgetPressureSensorPlugin] No serial_number provided for hub {}", hubName);
      continue;
    }
    auto hubSerialNumber = hubConfig("serial_number");
    auto hubFrequency = hubConfig("frequency", pps::DEFAULT_FREQUENCY);
    iterRate_ = static_cast<size_t>(1. / hubFrequency * 1000);
    if(sensorsConfig.empty())
    {
      warn_or_throw("[PhidgetPressureSensorPlugin] No sensors in configuration for hub {}", hubName);
    }
    else
    {
      for(const auto & [sensorName, sensorConfig] : sensorsConfig)
      {
        if(!sensorConfig.has("port"))
        {
          warn_or_throw("[PhidgetPressureSensorPlugin] Sensor {} on hub {} has no port specified in its configuration");
        }
        else
        {
          auto port = sensorConfig("port");
          sensors.try_emplace(sensorName, port);
        }
      }
    }
    if(sensors.empty())
    {
      warn_or_throw("[PhidgetPressureSensorPlugin] No sensor provided for hub {}", hubName);
    }
    else
    {
      hubs_.try_emplace(hubName, sensors, hubSerialNumber, hubFrequency, required_);
    }
  }
}

void PhidgetPressureSensorPlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("PhidgetPressureSensorPlugin::reset called");
}

void PhidgetPressureSensorPlugin::before(mc_control::MCGlobalController & controller)
{
  for(auto & [hubName, hubSensors] : hubs_)
  {
    if(iter_++ != 0 && iter_ % iterRate_ != 0) continue;

    hubSensors.readSensorData();

    if(!init_)
    {
      auto & ctl = controller.controller();
      for(auto & [hubName, hubSensors] : hubs_)
      {
        for(const auto & sensorData : hubSensors.getLatestSensorData())
        {
          auto & sensorName = sensorData.first;
          auto & data = sensorData.second;
          std::string pressureName = fmt::format("PhidgetPressureSensor_{}_{}_pressure", hubName, sensorName).c_str();
          ctl.logger().addLogEntry(pressureName, this, [&data]() { return data.pressure; });
          ctl.logger().addLogEntry(fmt::format("PhidgetPressureSensor_{}_{}_current", hubName, sensorName).c_str(),
                                   this, [&data]() { return data.current; });
          ctl.gui()->addElement(
              this, {"PhidgetPressureSensorPlugin", sensorName}, mc_rtc::gui::ElementsStacking::Horizontal,
              mc_rtc::gui::Button("Plot Pressure",
                                  [this, pressureName, &ctl, &data]()
                                  {
                                    ctl.gui()->addPlot(pressureName,
                                                       mc_rtc::gui::plot::X("Time (s)", [this]() { return t_; }),
                                                       mc_rtc::gui::plot::Y(
                                                           "Pressure", [&data]() { return data.pressure; },
                                                           mc_rtc::gui::Color::Blue, mc_rtc::gui::plot::Style::Solid));
                                  }),
              mc_rtc::gui::Button("Stop Plot Pressure",
                                  [&ctl, pressureName]() { ctl.gui()->removePlot(pressureName); }));
        }
      }
      init_ = true;
    }
  }
}

void PhidgetPressureSensorPlugin::after(mc_control::MCGlobalController & controller)
{
  t_ += controller.controller().timeStep;
}

mc_control::GlobalPlugin::GlobalPluginConfiguration PhidgetPressureSensorPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("PhidgetPressureSensorPlugin", mc_plugin::PhidgetPressureSensorPlugin)
