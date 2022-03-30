#include "live_3d_compounding/controller.h"
#include "live_3d_compounding/algorithm.h"
#include <QDebug>

#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/Base/Log.h>

#include <ImFusion/Base/DataModel.h>




//jzl
//#include <ImFusion/US/USSweepRecorderAlgorithm.h>
#include <ImFusion/GUI/DisplayWidgetMulti.h>
//#include <ImFusion/Base/ImageResamplingAlgorithm.h>

//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/core/mat.hpp>

//#include <vector>


namespace ImFusion {
namespace Live3DCompunding {

PluginController::PluginController(PluginAlgorithm* algorithm) : AlgorithmController(algorithm), algorithm_{algorithm} {
  ui_ = std::make_shared<Ui_Controller>();
  ui_->setupUi(this);
}

void PluginController::init() {
  addToAlgorithmDock();

  connect(ui_->pbtStart, &QPushButton::clicked, this, &PluginController::onStartClicked);
  connect(ui_->pbtStop, &QPushButton::clicked, this, &PluginController::onStopClicked);
  connect(ui_->PBT_TEST, &QPushButton::clicked, this, &PluginController::onTestClicked);


}

void PluginController::onStartClicked() {
  ImageStream* usStream;
  StreamListener streamListener;
  auto cepha_stream = m_main->dataModel()->get("Ultrasound Stream");
  usStream = static_cast<ImageStream*>(cepha_stream);

  algorithm_->startStream(usStream);
}

void PluginController::onStopClicked() {
  algorithm_->stopStream();
}

void PluginController::onTestClicked() {

  int a  = ui_->buttonGroup->checkedId();
  std::cout<<a<<std::endl;


}



}  // namespace Live3DCompunding
}  // namespace ImFusion
