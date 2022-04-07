#include "imf_publisher_plugin/algorithm.h"
#include <ImFusion/Base/Data.h>                    // for Data
#include <ImFusion/Base/DataList.h>                // for DataList
#include <ImFusion/Base/Log.h>                     // for LOG_INFO
#include <ImFusion/Base/Mat.h>                     // for vec3
#include <ImFusion/Base/Selection.h>               // for Selection
#include <ImFusion/GL/SharedImageSet.h>            // for SharedImageSet
#include <ImFusion/Stream/ImageStream.h>           // for ImageStream
#include <ImFusion/Stream/LiveTrackingStream.h>    // for LiveTrackingStream
#include <ImFusion/US/FanMask.h>                   // for FanMask
#include <ImFusion/US/GlSweepCompounding.h>        // for GlSweepCompounding
#include <ImFusion/US/USSweepRecorderAlgorithm.h>  // for USSweepRecorderAlg...
#include <ImFusion/US/UltrasoundGeometry.h>        // for UltrasoundGeometry
#include <ImFusion/US/UltrasoundSweep.h>           // for UltrasoundSweep
#include <memory>                                  // for shared_ptr, __shar...
#include <string>                                  // for string
#include <vector>                                  // for vector

namespace ImFusion {
namespace ImfPublisher {

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

void PluginAlgorithm::startStream() {
  fake_image_stream->start();
  fake_tracking_stream->start();
  sweeper_ = std::make_shared<Sweeper>(fake_tracking_stream, fake_image_stream);


  sweeper_->recordSweep();
  

  // //auto rosOutStr = std::make_unique<ROSTopicImageOutStream>("Cepha Image Stream", "/imfusion/cephasonics");
  // //auto rtos = rosOutStr.get();
  // ROSTopicImageOutStream* rosOutStr = new ROSTopicImageOutStream("Cepha Image Stream", "/imfusion/cephasonics");
  // usStream = stream;
  // //usStream->addListener(rtos);
  // usStream->addListener(rosOutStr);
  // //rtos->start();
  // rosOutStr->start();
  // usStream->start();

  // start();
}

void PluginAlgorithm::stopStream() {

sweeper_->saveSweep();
sweeper_->compound();

  // run_ = false;
  // usStream->removeListener(rosOutStr);
  // delete rosOutStr;
}

Sweeper::Sweeper(LiveTrackingStream* tracking_stream, ImageStream* ultrasound_stream, const std::string& sweep_name)
  : sweep_name_(sweep_name) {
  // Place both streams into vectors (the recorder algorithm takes vectors as input).
  std::vector<Stream*> streamVector{tracking_stream, ultrasound_stream};

  // Add stream vectors to recorder algorithm.
  algorithm_ = std::make_shared<USSweepRecorderAlgorithm>(streamVector);

  // Set geometry of the ultrasound probe. These values are tunned for the IFL convex probe.
  // probeGeometry_ = UltrasoundGeometry(UltrasoundGeometry::CONVEX, 320, -180, 256, 600, 30);
  // TODO load param from config file or similar.
  probe_geometry = UltrasoundGeometry(UltrasoundGeometry::CONVEX, 410, 853, 324, 823, 30);
}

void Sweeper::recordSweep() {
  // Start recording.
  algorithm_->start();
  LOG_INFO("Started recording sweep");
}

void Sweeper::saveSweep() {
  // Stop recording.
  algorithm_->stop();
  LOG_INFO("Stopped recording sweep");
  // Output the recorded streams.
  OwningDataList data_list;
  data_list = algorithm_->takeOutput();

  // Extract the ultrasound sweep from the data.
  UltrasoundSweep* us_sweep = static_cast<UltrasoundSweep*>(data_list.dataListView().getItem(0));

  std::cout << "size of the recorded us sweep: " << std::to_string(us_sweep->size()) << std::endl;
  // TODO check if passing no selection at all results in selecting all the sweep frames.
  // Select frames on which to compute the compounding.
  Selection frame_selection;
  frame_selection.setAll(us_sweep->size());
  us_sweep->setSelection(frame_selection);

  // Apply the mask to the ultrasound sweep.
  applyMaskToSweep(us_sweep);

  // Add sweep to the data model.
  // mainWindow_->dataModel()->add(usSweep,"US sweep");
  partial_sweeps_.push_back(us_sweep);
}

void Sweeper::addSweep(UltrasoundSweep* us_sweep){


    std::cout << "size of the recorded us sweep: " << std::to_string(us_sweep->size()) << std::endl;
    // TODO check if passing no selection at all results in selecting all the sweep frames.
    // Select frames on which to compute the compounding.
    Selection frame_selection;
    frame_selection.setAll(us_sweep->size());
    us_sweep->setSelection(frame_selection);

    // Apply the mask to the ultrasound sweep.
    applyMaskToSweep(us_sweep);

    // Add sweep to the data model.
    // mainWindow_->dataModel()->add(usSweep,"US sweep");
    partial_sweeps_.push_back(us_sweep);
}

void Sweeper::compound() {
  // Check at least one sweep was recorded.
  if (hasData()) {
    // Pass all the sweeps at once to the compounding algorithm.
    auto compounder = std::make_shared<GlSweepCompounding>(*partial_sweeps_.back());
    // Set the options of the compounder.
    compounder->setSpacing(0);
    compounder->setHideSweep(false);
    compounder->setMode(0);
    compounder->setVerbose(true);
    compounder->compute();
    DataList result;
    compounder->output(result);

    // Save the result in the algorithm data member.
    us_volume_.emplace_back(dynamic_cast<SharedImageSet*>(result.getItem(0)));
  } else {
    LOG_ERROR("UltrasoundCompounding - Cannot coumpound, no data was recorded yet.");
  }
}

void Sweeper::applyMaskToSweep(UltrasoundSweep* sweep) {
  // Set the probe geometry to the sweep.
  sweep->setGeometry(probe_geometry);

  // Create a fan mask from the probe geometry. TRY IF DEFAULT WHICH=-1 WORKS ON ALL SWEEP FRAMES.
  auto mask = std::make_shared<FanMask>(sweep->geometry(), sweep->width(), sweep->height(), vec3(1, 1, 1));
  // Apply that mask to all frames.
  for (int frame = 0; frame < sweep->size(); frame++) { sweep->setMask(mask, frame); }
}

bool Sweeper::hasData() {
  // Check the size of the vector where the sweeps are stored.
  return (!partial_sweeps_.empty());
}


}  // namespace ImfPublisher
}  // namespace ImFusion
