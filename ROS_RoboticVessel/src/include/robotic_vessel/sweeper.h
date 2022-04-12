#pragma once

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
namespace ROS_RoboticVessel {

class Sweeper {
public:
  /**
   * @brief Class constructor.
   * @param robotStream pointer to the robot tracking stream.
   * @param USstream pointer to the ultrasound image stream.
   */
  Sweeper(LiveTrackingStream* tracking_stream, ImageStream* us_stream, const std::string& sweep_name = "");

  virtual ~Sweeper() = default;

  /**
   * @brief Returns the last compounded volume.
   */
  inline SharedImageSet* getVolume() { return us_volume_.back(); }
  inline UltrasoundSweep* getSweep() { return partial_sweeps_.back(); }
  inline int getNumberFrames() { return partial_sweeps_.back()->size(); }

  /**
   * @brief Adds the ultrasound stream and the robot tracking stream to the recorder algorithm.
   */
  void recordSweep();

  /**
   * @brief Takes the recorded US stream and robot tracking stream and saves it as a sweep. Adds the the sweep to the
   * Main Window data model.
   */
  void saveSweep();

  /**
   * @brief Takes the last recorded sweep and compounds them into one volume. Adds the volume to the data model.
   */
  void compound();

private:
  /**
   * @brief Checks that the partial sweeps vector has at least one element.
   */
  bool hasData();

  /**
   * @brief Adds a mask to the ultrasound image stream so that the borders of the image are not taken into account by
   * the compounding.
   */
  void applyMaskToSweep(UltrasoundSweep* sweep);

  std::shared_ptr<USSweepRecorderAlgorithm> algorithm_{nullptr};
  UltrasoundGeometry probe_geometry{};
  const std::string sweep_name_{""};
  std::vector<UltrasoundSweep*> partial_sweeps_{};
  std::vector<SharedImageSet*> us_volume_{};
};


}  // namespace ROS_RoboticVessel
}  // namespace ImFusion
