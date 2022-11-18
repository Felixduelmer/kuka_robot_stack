#include "robotic_vessel/sweep_rec_and_comp.h"
#include <ImFusion/Base/Log.h>
#include <ImFusion/GL/GlVolumeCompounding.h>
#include <ImFusion/US/GlSweepCompounding.h>
#include <ImFusion/GL/SharedImageSet.h> // for SharedImageSet
#include <ImFusion/IO/BackgroundExporter.h>
#include <ImFusion/Base/DataModel.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/US/UltrasoundSweep.h>
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <iostream>
#include <chrono>
#include <ImFusion/US/USSweepRecorderAlgorithm.h>
#include <ImFusion/US/USSweepRecorderController.h>
#include <ImFusion/US/LiveSweepRecordingVisualizationController.h>
#include <ImFusion/US/UltrasoundSweepRingBuffer.h>
#include <ImFusion/Base/BasicImageProcessing.h>
#include <ImFusion/GL/IntensityMask.h>


#define ON_NOTREADY -1
#define ON_READYTOSTART 0
#define ON_FINAL 1

namespace ImFusion {
    namespace ROS_RoboticVessel {

        SweepRecAndComp::SweepRecAndComp(MainWindowBase *mainWindowBase)
                : m_main(mainWindowBase) {
        }

        void SweepRecAndComp::startSweepRecording() {
            LiveTrackingStream *robStream;
            ImageStream *usStream;

            if (m_useLiveData) {
                robStream = static_cast<LiveTrackingStream *>(m_main->dataModel()->get(
                        "Robot Tracking"));
                usStream = static_cast<ImageStream *>(m_main->dataModel()->get("liveSegmentationStream"));
            } else {
                robStream = new FakeTrackingStream();
                usStream = static_cast<ImageStream *>(m_main->dataModel()->get("liveSegmentationStream"));
                robStream->open();
                robStream->start();
            }
            usStream->setModality(Data::Modality::ULTRASOUND);
            std::vector<Stream *> vec;
            vec.push_back((usStream));
            vec.push_back((robStream));
            ImageStream *USStream = static_cast<ImageStream *>(m_main->dataModel()->get("Ultrasound Stream"));
            vec.push_back((USStream));

            sweepRecorderAlgorithm = new USSweepRecorderAlgorithm(vec);
            sweepRecorderAlgorithm->setTemporalOffset(0.63);
            sweepRecorderAlgorithm->setTrackingQualityThreshold(0.1);
            sweepRecorderAlgorithm->start();
            Properties ctrlRecorder;
            ctrlRecorder.addSubProperties("Controller");
            m_main->addAlgorithm(sweepRecorderAlgorithm, &ctrlRecorder);
            USSweepRecorderController *controller = dynamic_cast<USSweepRecorderController *>(m_main->getAlgorithmController(
                    sweepRecorderAlgorithm));

            ringBuffer = controller->liveSweep();
            ringBuffer->setBufferSize(999999999);
//            IntensityMask intensityMask = new IntensityMask();
//            ringBuffer->setMask()
//            fanMotionTimer = new QTimer(this);
//            connect(fanMotionTimer, SIGNAL(timeout()), this, SLOT(onUpdateVolume()));
//            fanMotionTimer->start(15000);
        }

        void SweepRecAndComp::onUpdateVolume() {
            numberOfPartialSweeps += 1;
            DataList datalist;
            sweepRecorderAlgorithm->stop();
            sweepRecorderAlgorithm->output(datalist);
//            sweepRecorderAlgorithm->start();
            UltrasoundSweep *usSweep = static_cast<UltrasoundSweep *>(datalist.getItem(0));
            UltrasoundSweep *usSweepOriginal = static_cast<UltrasoundSweep *>(datalist.getItem(1));

            // necessary???
            Selection sel;
            Selection sel2;

            sel.setAll(usSweep->size());
            sel.setAll(usSweepOriginal->size());
            usSweep->setSelection(sel);
            usSweepOriginal->setSelection(sel2);
//            sel.setAll(ringBuffer->size());
//            ringBuffer->setSelection(sel);
//             usSweep->tracking()->setTemporalOffset(m_robotCtrlUS->getTemporalCalibration());
            // setConvexGeometry(usSweep);
            auto *sc2 = new GlSweepCompounding(*usSweep);
            sc2->setMode(0); // GPU    4 is CPU Maximum
            std::chrono::steady_clock::time_point compoundingStarted = std::chrono::steady_clock::now();
            sc2->compute();  // We do the volume compounding
            std::chrono::steady_clock::time_point compoundingEnded = std::chrono::steady_clock::now();
            DataList datalist3;
            sc2->output(datalist3);

            std::cout << "Compounding took = "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(compoundingEnded - compoundingStarted).count()
                      << "[µs]" << std::endl;

            m_main->dataModel()->add(datalist3.getItem(0), "Partial Volume ");
//            m_main->dataModel()->add(usSweep, "Partial Sweep ");

            if (m_exportSweeps) {
                auto sis = static_cast<SharedImageSet *>(datalist3.getItem(0));
                sis->get()->sync();
                auto str = getDayAndTime();
                BackgroundExporter *sweepExporter = new BackgroundExporter();
                sweepExporter->save(usSweep, "/data1/volume1/data/felix_data/results_sweeps/sweep_" + str + ".imf");
                BackgroundExporter *originalSweepExporter = new BackgroundExporter();
                originalSweepExporter->save(usSweepOriginal,
                                            "/data1/volume1/data/felix_data/results_sweeps/original_sweep_" + str +
                                            ".imf");
                BackgroundExporter *volExporter = new BackgroundExporter();
                volExporter->save(sis, "/data1/volume1/data/felix_data/results_sweeps/vol_" + str + ".imf");
            }

            delete sc2;
//            if (numberOfPartialSweeps > 2) {
////                fanMotionTimer->stop();
//                compoundAllSweeps();
//            }

            // if (!partial)
            // {
            //     if (m_SweepRecorder->numberOfPartialSweeps != 1)
            //         compoundAllSweeps(m_SweepRecorder->numberOfPartialSweeps);
            // }
        }

        void SweepRecAndComp::stop() {
            onUpdateVolume();
//            /?fanMotionTimer->stop();
//            delete sweepRecorderAlgorithm;
        }

        void SweepRecAndComp::compoundAllSweeps() {
            DataList outputVolume;
            GlVolumeCompounding *volumeCompounding;

            std::vector<SharedImageSet *> container = m_main->dataModel()->getImages(Data::VOLUME);
            auto *container2 = new SharedImageSet(); // don't own raw data

            for (int jj = container.size() - numberOfPartialSweeps; jj < container.size(); jj++) {
                container2->add(container[jj]->get(0));
            }

            volumeCompounding = new GlVolumeCompounding(container2); // own the container
            volumeCompounding->setUseGPU(true);
            volumeCompounding->setMode(GlVolumeCompounding::MEDIAN);
            volumeCompounding->compute();
            volumeCompounding->output(outputVolume);

            m_main->dataModel()->add(outputVolume.getItem(0), "Compounded Volume");

            if (m_exportSweeps) {
                auto sis = static_cast<SharedImageSet *>(outputVolume.getItem(0));
                sis->get()->sync();
                auto str = getDayAndTime();
                BackgroundExporter *sweepExporter = new BackgroundExporter();
                sweepExporter->save(sis, "/data1/volume1/data/felix_data/robotic_sweeps/complete_vol_" + str + ".imf");
            }
        }

        std::string SweepRecAndComp::getDayAndTime() {
            time_t rawtime;
            struct tm *timeinfo;
            char buffer[80];

            time(&rawtime);
            timeinfo = localtime(&rawtime);

            strftime(buffer, sizeof(buffer), "%d_%m_%H_%M_%S", timeinfo);
            std::string str(buffer);

            return str;
        }
    } // namespace vision_control
} // namespace ImFusion
