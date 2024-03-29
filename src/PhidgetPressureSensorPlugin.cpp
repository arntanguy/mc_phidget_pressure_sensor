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
    std::map<std::string, pps::SensorConfig> sensors;
    if(!hubConfig.has("serial_number"))
    {
      warn_or_throw("[PhidgetPressureSensorPlugin] No serial_number provided for hub {}", hubName);
      continue;
    }
    auto hubSerialNumber = hubConfig("serial_number");
    auto hubFrequency = hubConfig("frequency", pps::DEFAULT_FREQUENCY);
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
          pps::SensorConfig config;
          sensorConfig("port", config.port);
          sensorConfig("coeff1", config.coeff1);
          sensorConfig("coeff2", config.coeff2);
          sensors.try_emplace(sensorName, config);
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
  static unsigned int iter = 0;
  for(auto & [hubName, hubSensors] : hubs_)
  {
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
          ctl.datastore().make_call("PhidgetPressureSensor::" + sensorName + "::pressure",
                                    [&data]() { return data.pressure; });
          ctl.gui()->addElement(this, {"PhidgetPressureSensorPlugin", sensorName},
                                mc_rtc::gui::Label("Pressure", [&data]() { return data.pressure; }),
                                mc_rtc::gui::Label("Current", [&data]() { return data.current; }));
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
