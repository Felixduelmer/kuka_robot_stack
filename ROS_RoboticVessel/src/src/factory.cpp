#include "robotic_vessel/factory.h"
#include "robotic_vessel/algorithm.h"
#include "robotic_vessel/controller.h"

namespace ImFusion {
namespace Live3DCompunding {

PluginAlgorithmFactory::PluginAlgorithmFactory() { registerAlgorithm<PluginAlgorithm>("ROS;Live3DCompunding"); }

AlgorithmController* PluginControllerFactory::create(ImFusion::Algorithm* a) const {
  if (PluginAlgorithm* algorithm = dynamic_cast<PluginAlgorithm*>(a)) { return new PluginController(algorithm); }
  return nullptr;
}

}  // namespace Live3DCompunding
}  // namespace ImFusion
