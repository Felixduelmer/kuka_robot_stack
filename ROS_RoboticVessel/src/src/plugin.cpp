#include "robotic_vessel/plugin.h"
#include "robotic_vessel/factory.h"

#ifdef WIN32
extern "C" __declspec(dllexport) ImFusion::ImFusionPlugin* createPlugin()
#else
extern "C" ImFusion::ImFusionPlugin* createPlugin()
#endif
{
  return new ImFusion::ROS_RoboticVessel::Plugin;
}

namespace ImFusion {
namespace ROS_RoboticVessel {

Plugin::Plugin() {
  algorithm_factory_ = std::make_unique<PluginAlgorithmFactory>();
  algorithm_controller_factory_ = std::make_unique<PluginControllerFactory>();
}

const ImFusion::AlgorithmFactory* Plugin::getAlgorithmFactory() { return algorithm_factory_.get(); }

const ImFusion::AlgorithmControllerFactory* Plugin::getAlgorithmControllerFactory() {
  return algorithm_controller_factory_.get();
}
}  // namespace ROS_RoboticVessel
}  // namespace ImFusion
