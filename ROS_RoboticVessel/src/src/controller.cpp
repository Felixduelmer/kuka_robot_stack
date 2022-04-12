#include "robotic_vessel/controller.h"
#include "robotic_vessel/algorithm.h"
#include <QDebug>

#include <ImFusion/Base/Log.h>
#include <ImFusion/Base/DataModel.h>

#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/GUI/DisplayWidgetMulti.h>


#include <ImFusion/US/GlSweepCompounding.h>
#include <ImFusion/Base/ImFusionFile.h>
#include <ImFusion/US/UltrasoundSweep.h>
#include <ImFusion/Stream/FakeImageStream.h>
#include <iostream>
#undef slots
#include <torch/script.h> // One-stop header
#define slots Q_SLOTS

#include "ui_controller.h"
#include "robotic_vessel/vesselNet.h"
#include <opencv2/opencv.hpp>


namespace ImFusion {
    namespace ROS_RoboticVessel {

        PluginController::PluginController(PluginAlgorithm *algorithm)
                : AlgorithmController(algorithm), algorithm_{algorithm} {
            ui_ = std::make_shared<Ui_Controller>();
            ui_->setupUi(this);
        }


        void PluginController::init() {
            addToAlgorithmDock();

            connect(ui_->pbtStart, &QPushButton::clicked, this, &PluginController::onStartClicked);
            connect(ui_->pbtStop, &QPushButton::clicked, this, &PluginController::onStopClicked);

            sweepRecAndComp = new SweepRecAndComp(m_main);
            cv::namedWindow("usImage");
            cv::namedWindow("dopplerImage");
            cv::namedWindow("segImage");
        }

        void PluginController::onStartClicked() {


        }

        void PluginController::onStopClicked() {
            ImFusionFile file("/data1/volume1/data/felix_data/sweeps_imfusion/patient_2/Felix-02.imf");
            file.open(0);

            DataList dataList;
            file.load(dataList);

            SharedImageSet* sis = dataList.getImage(Data::UNKNOWN);
            auto imageStream = new PlaybackStream(*sis);
            auto processor = new MyImageStreamLiveProcessingAlgorithm(imageStream);
            imageStream->open();
            imageStream->start();

        }

        void PluginController::addToMainDataModel(Data *data, const std::string &name) {
            m_main->dataModel()->add(data, name);
        }


    }  // namespace ROS_RoboticVessel
}  // namespace ImFusion
