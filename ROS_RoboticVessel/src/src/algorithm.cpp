#include <ImFusion/Base/ImFusionFile.h>
#include "robotic_vessel/algorithm.h"
#include <ImFusion/US/UltrasoundSweep.h>           // for UltrasoundSweep
#include <ImFusion/Stream/TrackingStreamListener.h>
#include <ImFusion/Stream/FakeTrackingStream.h>
#include <ImFusion/Stream/PlaybackTrackingStream.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/US/FreehandUSWorkflowAlgorithm.h>
#include <iostream>


namespace ImFusion {
namespace ROS_RoboticVessel {

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

void PluginAlgorithm::imageCallBack() {

}

void PluginAlgorithm::startDummyStream(MainWindowBase* main) {
    ImFusionFile file("/data1/volume1/data/felix_data/sweeps_imfusion/patient_2/Felix-02.imf");
    file.open(0);

    DataList dataList;
    file.load(dataList);
    auto usSweep = dynamic_cast<UltrasoundSweep*>(dataList.getItem(0));
    auto components = usSweep->components();




}

}  // namespace ROS_RoboticVessel
}  // namespace ImFusion
