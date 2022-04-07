#include "robotic_vessel/controller.h"
#include "robotic_vessel/algorithm.h"
#include <QDebug>

#include <ImFusion/Base/Log.h>
#include <ImFusion/Base/DataModel.h>

#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/GUI/DisplayWidgetMulti.h>

#include "ui_controller.h"



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

}

void PluginController::onStartClicked() {
  ImageStream* usStream;
  auto cepha_stream = m_main->dataModel()->get("Ultrasound Stream");
  usStream = static_cast<ImageStream*>(cepha_stream);

  algorithm_->startStream(usStream);
}

void PluginController::onStopClicked() {
  algorithm_->stopStream();
}

void PluginController::onTestClicked() {



}



}  // namespace Live3DCompunding
}  // namespace ImFusion
