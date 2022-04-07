#include "robotic_vessel/algorithm.h"

namespace ImFusion {
namespace Live3DCompunding {

bool PluginAlgorithm::createCompatible(const DataList& data, Algorithm** a) {
  if (data.size() != 0) { return false; }
  if (a) { *a = new PluginAlgorithm(); }
  return true;
}

void PluginAlgorithm::compute() {}

void PluginAlgorithm::configure(const Properties* p) {
  if (p == nullptr) { return; }
  //p->param("something", something_);
}

void PluginAlgorithm::configuration(Properties* p) const {
  if (p == nullptr) { return; }
  //p->setParam("something", something_);
}

void PluginAlgorithm::run(){
}

void PluginAlgorithm::startStream(ImageStream* stream) {
}

void PluginAlgorithm::stopStream() {
}

}  // namespace Live3DCompunding
}  // namespace ImFusion
