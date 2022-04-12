#include "robotic_vessel/factory.h"
#include "robotic_vessel/algorithm.h"
#include "robotic_vessel/controller.h"

namespace ImFusion {
    namespace ROS_RoboticVessel {

        PluginAlgorithmFactory::PluginAlgorithmFactory() {
            registerAlgorithm<PluginAlgorithm>("ROS;ROS_RoboticVessel");
        }

        AlgorithmController *PluginControllerFactory::create(ImFusion::Algorithm *a) const {
            if (PluginAlgorithm * algorithm = dynamic_cast<PluginAlgorithm *>(a)) {
                return new PluginController(algorithm);
            }
            return nullptr;
        }

    }  // namespace ROS_RoboticVessel
}  // namespace ImFusion
