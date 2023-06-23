/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <PhidgetPressureSensorDAQ.h>

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

private:
  std::map<std::string, pps::PhidgetPressureSensorDAQ> hubs_;
  double t_ = 0;
  bool init_ = false;
};

} // namespace mc_plugin
