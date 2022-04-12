#include "robotic_vessel/sweeper.h"

namespace ImFusion {
namespace ROS_RoboticVessel {

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
  DataList data_list;
  algorithm_->output(data_list);

  // Extract the ultrasound sweep from the data.
  UltrasoundSweep* us_sweep = static_cast<UltrasoundSweep*>(data_list.getItem(0));

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

}  // namespace ROS_RoboticVessel
}  // namespace ImFusion
