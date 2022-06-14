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


const bool useDummyData = false;
const bool performSegmentationAndCompound = true;

namespace ImFusion {
    namespace ROS_RoboticVessel {

        PluginController::PluginController(PluginAlgorithm *algorithm)
                : AlgorithmController(algorithm), algorithm_{algorithm} {
            ui_ = std::make_shared<Ui_Controller>();
            ui_->setupUi(this);
            qRegisterMetaType<cv::Mat>("cvMat");
        }


        void PluginController::init() {
            addToAlgorithmDock();

            connect(ui_->pbtConnectRobot, &QPushButton::clicked, this, &PluginController::onConnectRobotClicked);
            connect(ui_->pbtStart, &QPushButton::clicked, this, &PluginController::onStartClicked);
            connect(ui_->pbtStop, &QPushButton::clicked, this, &PluginController::onStopClicked);
            connect(ui_->pbtSegmentation, &QPushButton::clicked, this,
                    &PluginController::onStartSegmentationClicked);
            connect(ui_->pbtnAddPoint, &QPushButton::clicked, this, &PluginController::onClickedpbtnAddPoint);
            connect(ui_->pbtnDeletePoints, &QPushButton::clicked, this, &PluginController::onClickedpbtnDeletePoints);
            connect(ui_->pbtnImpedanceCtrl, &QPushButton::clicked, this, &PluginController::onStartImpedanceControl);


            robControl = new RobotControl();
            sweepRecAndComp = new SweepRecAndComp(m_main);
            tracker = new Tracker();
            liveSegmentationStream = new LiveSegmentationStream();

            connect(robControl, &RobotControl::reachedStartingPoint, this, &PluginController::startRecording);
            connect(robControl, &RobotControl::reachedEndPoint, this, &PluginController::onStopClicked);
            connect(tracker, &Tracker::lostDoppler, robControl, &RobotControl::lostDopplerSignal);
            connect(tracker, &Tracker::foundDoppler, robControl, &RobotControl::foundDopplerSignal);


            connect(liveSegmentationStream, &LiveSegmentationStream::newDopplerImage, tracker,
                    &Tracker::doppler_tracker);
            cv::namedWindow("dopplerImage");
            cv::namedWindow("annotationImage");
            cv::namedWindow("contImage");

        }

        void PluginController::startRecording() {
            sweepRecAndComp->startSweepRecording();
        }


        void PluginController::onStartClicked() {
            robControl->executeTrajectory();
        }

        void PluginController::onStopClicked() {
//            imageStream->stop();
//            imageStream->close();
            sweepRecAndComp->stop();
//            robControl->applyPositionControlMode();

        }

        void PluginController::onConnectRobotClicked() {
            robControl->connectRobot("IFLConvex");
            robControl->addStreamToDataModel(m_main->dataModel());
        }

        void PluginController::onClickedpbtnAddPoint() {

            if (robControl->isRobotConnected()) {
                robControl->addPointConfiguration(robControl->getCurrentRobotTransformMatrix());
            }
        }

        void PluginController::onStartImpedanceControl() {
//            if (robControl->isRobotConnected()) {
//                robControl->reExecuteTrajectory();
//            }
            robControl->performFanMotion();
        }


        void PluginController::onClickedpbtnDeletePoints() {
            robControl->deletePointConfigurations();
        }


        void PluginController::onStartSegmentationClicked() {
            if (useDummyData) {
                ImFusionFile file(
                        "/data1/volume1/data/felix_data/results_sweeps/original_sweep_31_05_12_33_20.imf");
                file.open(0);

                DataList dataList;
                file.load(dataList);

                SharedImageSet *sis = dataList.getImage(Data::UNKNOWN);
                imageStream = new PlaybackStream(*sis);
            } else {
                imageStream = static_cast<ImageStream *>(m_main->dataModel()->get("Ultrasound Stream"));
            }
            liveSegmentationStream->addInputStream(imageStream);
            m_main->dataModel()->add(liveSegmentationStream, "liveSegmentationStream");
            imageStream->open();
            imageStream->start();
        }

    }  // namespace ROS_RoboticVessel
}  // namespace ImFusion
