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
#include "robotic_vessel/vessel_segmentation_listener.h"
#include <opencv2/opencv.hpp>

const bool useDumyData = false;

namespace ImFusion {
    namespace ROS_RoboticVessel {

        PluginController::PluginController(PluginAlgorithm *algorithm)
                : AlgorithmController(algorithm), algorithm_{algorithm} {
            ui_ = std::make_shared<Ui_Controller>();
            ui_->setupUi(this);
        }


        void PluginController::init() {
            addToAlgorithmDock();

            connect(ui_->pbtConnectRobot, &QPushButton::clicked, this, &PluginController::onRobotConnectedClicked);
            connect(ui_->pbtStart, &QPushButton::clicked, this, &PluginController::onStartClicked);
            connect(ui_->pbtStop, &QPushButton::clicked, this, &PluginController::onStopClicked);
            connect(ui_->pbtSegmentation, &QPushButton::clicked, this,
                    &PluginController::onStartSegmentationClicked);

            sweepRecAndComp = new SweepRecAndComp(m_main);
            robControl = new RobotControl(m_main);
//            cv::namedWindow("usImage");
//            cv::namedWindow("dopplerImage");
//            cv::namedWindow("segImage");
        }

        void PluginController::onStartClicked() {
            sweepRecAndComp->startSweepRecording();
        }

        void PluginController::onStopClicked() {
            imageStream->stop();
            imageStream->close();
            sweepRecAndComp->stop();

        }

        void PluginController::addToMainDataModel(Data *data, const std::string &name) {
            m_main->dataModel()->add(data, name);
        }

        void PluginController::onRobotConnectedClicked() {
            robControl->connect("IFLConvex");
            robControl->addStreamToDataModel(m_main->dataModel());

        }

        void PluginController::onStartSegmentationClicked() {
            if (useDumyData) {
                ImFusionFile file("/data1/volume1/data/felix_data/sweeps_imfusion/patient_2/Felix-02.imf");
                file.open(0);

                DataList dataList;
                file.load(dataList);

                SharedImageSet *sis = dataList.getImage(Data::UNKNOWN);
                imageStream = new PlaybackStream(*sis);
            } else {
                imageStream = static_cast<ImageStream *>(m_main->dataModel()->get("Ultrasound Stream"));
            }
            auto pLiveSegmentationStream = new LiveSegmentationStream(imageStream);
            m_main->dataModel()->add(pLiveSegmentationStream, "pLiveSegmentationStream");
            imageStream->open();
            imageStream->start();
        }


    }  // namespace ROS_RoboticVessel
}  // namespace ImFusion
