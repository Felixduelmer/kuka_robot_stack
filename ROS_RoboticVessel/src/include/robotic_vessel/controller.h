#pragma once

#include <ImFusion/GUI/AlgorithmController.h>
#include <QtWidgets/QWidget>

#include <memory>

#include <ImFusion/ROS/Stream/ROSTopicImageOutStream.h>
#include<ImFusion/Cephasonics/CephasonicsStream.h>
/************************************************************************
 *                           ImFusion
 * **********************************************************************/
#include <ImFusion/Base/Algorithm.h>  // for IoAlgorithm
#include <ImFusion/Base/DataList.h>   // for DataList
#include <ImFusion/Base/AlgorithmListener.h>  // for AlgorithmListener
#include <ImFusion/Base/Log.h>                // for LOG_INFO
#include <ImFusion/Base/Properties.h>         // for Properties
#include <ImFusion/Base/MemImage.h>
#include <ImFusion/Base/SharedImage.h>
#include <ImFusion/Base/SharedImageSet.h>
#include <ImFusion/Base/Progress.h>
#include <ImFusion/Base/OpenCV.h>
#include <ImFusion/Base/ImFusionFile.h>
#include <ImFusion/Base/TypedImage.h>
#include <ImFusion/Base/TrackingStream.h>
#include <ImFusion/GL/GlVolumeCompounding.h>
#include <ImFusion/GL/GlSliceView.h>
#include <ImFusion/GUI/AnnotationModel.h>
#include <ImFusion/GUI/DataWidget.h>
#include <ImFusion/GUI/DisplayWidgetMulti.h>
#include <ImFusion/GUI/ImageView2D.h>
#include <ImFusion/GUI/ImageView3D.h>
#include <ImFusion/GUI/Interactive.h>
#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/GUI/AlgorithmController.h>
#include <ImFusion/IO/BackgroundExporter.h>
#include <ImFusion/Stream/OpenIGTLinkStreamData.h>
#include <ImFusion/Stream/OpenIGTLinkImageStream.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/Stream/TrackingStreamData.h>
#include <ImFusion/US/UltrasoundSweep.h>
#include <ImFusion/US/USConfidenceMapAlgorithm.h>
#include <ImFusion/US/FanMask.h>
#include <ImFusion/US/UltrasoundGeometry.h>
#include <ImFusion/US/GlSweepCompounding.h>
#include <ImFusion/US/USSweepRecorderAlgorithm.h>
#include <ImFusion/US/USSweepCalibrator.h>

/************************************************************************
 *                           Qt
 * **********************************************************************/
#include <QtWidgets/QWidget>
#include <QtDebug>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <memory>
#include <QtCore/QObject>
#include <qt5/QtNetwork/QTcpServer>  //for TCP
#include <qt5/QtNetwork/QTcpSocket>  //for TCP
#include <QTimer>
#include <QTime>
#include <qmutex.h>
#include <unistd.h>
#include <functional>
class Ui_Controller;

namespace ImFusion {
namespace Live3DCompunding {

class PluginAlgorithm;

class PluginController : public QWidget, public AlgorithmController{
  Q_OBJECT
public:
  /// Constructor with the algorithm instance
  PluginController(PluginAlgorithm* algorithm);
  
  virtual ~PluginController() = default;

  void init();

public slots:
  void onStartClicked();
  void onStopClicked();
  void onTestClicked();

private:
  std::shared_ptr<Ui_Controller> ui_{nullptr};  ///< The actual GUI
  PluginAlgorithm* algorithm_{nullptr};         ///< The algorithm instance

};

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


}  // namespace Live3DCompunding
}  // namespace ImFusion