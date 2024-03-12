#include <ImFusion/Base/ImFusionFile.h>
#include "robotic_vessel/algorithm.h"
#include "robotic_vessel/vessel_segmentation_listener.h"
#include <ImFusion/US/UltrasoundSweep.h>           // for UltrasoundSweep
#include <ImFusion/Stream/TrackingStreamListener.h>
#include <ImFusion/Stream/FakeTrackingStream.h>
#include <ImFusion/Stream/FakeImageStream.h>
#include <ImFusion/Stream/PlaybackTrackingStream.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/LiveUS/FreehandUSWorkflowAlgorithm.h>
#include <iostream>
#include <ImFusion/Base/DataModel.h>
#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/US/UltrasoundSweepRingBuffer.h>
#include <ImFusion/LiveUS/USSweepRecorderAlgorithm.h>
#include <ImFusion/LiveUS/USSweepRecorderController.h>


namespace ImFusion {
    namespace ROS_RoboticVessel {

        bool PluginAlgorithm::createCompatible(const DataList &data, Algorithm **a) {
            if (data.size() != 0) { return false; }
            if (a) { *a = new PluginAlgorithm(); }
            return true;
        }

        void PluginAlgorithm::compute() {}

        void PluginAlgorithm::configure(const Properties *p) {
            if (p == nullptr) { return; }
            //p->param("something", something_);
        }

        void PluginAlgorithm::configuration(Properties *p) const {
            if (p == nullptr) { return; }
            //p->setParam("something", something_);
        }

        void PluginAlgorithm::run() {
        }

        void PluginAlgorithm::imageCallBack() {


        }

        void PluginAlgorithm::startDummyStream(MainWindowBase *main) {
//    ImFusionFile file("/data1/volume1/data/felix_data/sweeps_imfusion/patient_2/Felix-02.imf");
//    file.open(0);
//
//    DataList dataList;
//    file.load(dataList);
//    auto usSweep = dynamic_cast<UltrasoundSweep*>(dataList.getItem(0));
//    auto components = usSweep->components();

//
//            LiveTrackingStream *robStream;
//            ImageStream *imageStream = new FakeImageStream();
//            imageStream->open();
//            imageStream->start();
//            robStream = new FakeTrackingStream();
////            usStream = static_cast<ImageStream *>(main->dataModel()->get("liveSegmentationStream"));
//            robStream->open();
//            robStream->start();
//            std::vector<Stream *> vec;
//            vec.push_back((imageStream));
//            vec.push_back((robStream));
//
//            USSweepRecorderAlgorithm *sweepRecorderAlgorithm = new USSweepRecorderAlgorithm(vec);
//            USSweepRecorderController *mySweepController = new USSweepRecorderController(
//                    sweepRecorderAlgorithm);
////            mySweepController->init();
//            sweepRecorderAlgorithm->start();
//            sweepRecorderAlgorithm->recorder();
//
//            auto sweepBuffer = mySweepController->liveSweep();
//            sweepBuffer->setBufferSize(100);
//            main->dataModel()->add(*sweepRecorderAlgorithm->trackingStream(), "live strream");


        }

    }  // namespace ROS_RoboticVessel
}  // namespace ImFusion
