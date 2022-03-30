#include "live_3d_compounding/algorithm.h"

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
  while(run_);
}

void PluginAlgorithm::startStream(ImageStream* stream) {
  //auto rosOutStr = std::make_unique<ROSTopicImageOutStream>("Cepha Image Stream", "/imfusion/cephasonics");
  //auto rtos = rosOutStr.get();
  
  ROSTopicImageOutStream* rosOutStr = new ROSTopicImageOutStream("Cepha Image Stream", "/imfusion/epiphan_grabber");
  usStream = stream;
  stream->addListener
  //usStream->addListener(rtos);
  stream.
  usStream->addListener(rosOutStr);
  //rtos->start();
  rosOutStr->start();
  usStream->start();

  start();
}

void PluginAlgorithm::stopStream() {
  run_ = false;
  usStream->removeListener(rosOutStr);
  delete rosOutStr;
}

}  // namespace Live3DCompunding
}  // namespace ImFusion
