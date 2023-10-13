/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rtc/logging.h>
#include <PhidgetPressureSensorDAQ.h>
#include <utility>

namespace mc_plugin
{

struct PhidgetPressureSensorPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~PhidgetPressureSensorPlugin() override;

  template<typename... Args>
  bool warn_or_throw(Args &&... args)
  {
    if(required_)
    {
      mc_rtc::log::error_and_throw(std::forward<Args>(args)...);
      return true;
    }
    else
    {
      mc_rtc::log::warning(std::forward<Args>(args)...);
      return false;
    }
  }

private:
  std::map<std::string, pps::PhidgetPressureSensorDAQ> hubs_;
  double t_ = 0;
  bool init_ = false;
  bool required_ = true;
  size_t iter_ = 0;
  size_t iterRate_ = 0;
};

} // namespace mc_plugin
