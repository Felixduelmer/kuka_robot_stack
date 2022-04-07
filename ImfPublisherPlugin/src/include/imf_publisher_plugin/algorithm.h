#pragma once

#include <ImFusion/Base/Algorithm.h>

#include <QApplication>
#include <QtCore/QThread>

#include <string>

#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/Cephasonics/CephasonicsStream.h>
#include <ImFusion/ROS/Stream/ROSTopicImageOutStream.h>

#include <ImFusion/Base/SharedImageSet.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/US/USSweepRecorderAlgorithm.h>
#include <ImFusion/US/UltrasoundSweep.h>
#include <ImFusion/Stream/FakeTrackingStream.h>
#include <ImFusion/Stream/FakeImageStream.h>
#include <ImFusion/GUI/DisplayWidgetMulti.h>

namespace ImFusion {
namespace ImfPublisher {

class Sweeper {
public:
  /**
   * @brief Class constructor.
   * @param robotStream pointer to the robot tracking stream.
   * @param USstream pointer to the ultrasound image stream.
   */
  Sweeper(LiveTrackingStream* tracking_stream, ImageStream* us_stream, const std::string& sweep_name = "");
  Sweeper();
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

  void addSweep(UltrasoundSweep* us_sweep);

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

//class PluginAlgorithm : public QObject, public ImFusion::Algorithm {
class PluginAlgorithm : public QThread, public ImFusion::Algorithm {
  Q_OBJECT
public:
  PluginAlgorithm() = default;
  ~PluginAlgorithm() = default;

  void compute();

  void run() override;

  void startStream();
  void stopStream();

  //! Methods implementing the Configurable interface.
  void configure(const Properties* p);
  void configuration(Properties* p) const;

  static bool createCompatible(const DataList& data, Algorithm** a = nullptr);

private:
  ImageStream* usStream;
  ROSTopicImageOutStream* rosOutStr;
  FakeTrackingStream* fake_tracking_stream{};
  FakeImageStream* fake_image_stream{};
  std::shared_ptr<Sweeper> sweeper_{};

protected:
  bool run_{true};

};

}  // namespace DemoROS
}  // namespace ImFusion
