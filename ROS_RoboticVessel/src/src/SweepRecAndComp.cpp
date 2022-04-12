#include "robotic_vessel/SweepRecAndComp.h"
#include <ImFusion/Base/Log.h>
#include <ImFusion/GL/GlVolumeCompounding.h>
#include <ImFusion/US/GlSweepCompounding.h>
#include <ImFusion/GL/SharedImageSet.h> // for SharedImageSet
#include <ImFusion/IO/BackgroundExporter.h>
#include <ImFusion/Base/DataModel.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/US/UltrasoundSweep.h>
#include <ImFusion/Base/ImFusionFile.h>
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <iostream>
#include <chrono>

#define ON_NOTREADY -1
#define ON_READYTOSTART 0
#define ON_FINAL 1

namespace ImFusion {
    namespace ROS_RoboticVessel {

        SweepRecAndComp::SweepRecAndComp(MainWindowBase *mainWindowBase)
                : m_main(mainWindowBase) {
        }

        void SweepRecAndComp::startSweepRecording() {

            if (m_useDummyData) {
                ImFusionFile file("/home/robotics-verse/Desktop/testSweep.imf");
                file.open(0);

                DataList dataList;
                file.load(dataList);
                m_dataList = dataList;
            } else {
                LiveTrackingStream *robStream = static_cast<LiveTrackingStream *>(m_main->dataModel()->get(
                        "Robot Tracking"));
                ImageStream *usStream = static_cast<ImageStream *>(m_main->dataModel()->get("Ultrasound Stream"));
                std::vector<Stream *> vec;
                vec.push_back((usStream));
                vec.push_back((robStream));

                myMultiUSSweepRecorderAlgorithm = new USSweepRecorderAlgorithm(vec);
                myMultiUSSweepRecorderAlgorithm->start();
            }

            onUpdateVolume();
            timer = new QTimer(this);
            connect(timer, SIGNAL(timeout()), this, SLOT(onUpdateVolume()));
            timer->start(300);
        }

        void SweepRecAndComp::onUpdateVolume() {
            numberOfPartialSweeps += 1;
            if (numberOfPartialSweeps == 30) {
                timer->stop();
            }
            DataList datalist;
            if (m_useDummyData) {
                datalist = m_dataList;
            } else {
                myMultiUSSweepRecorderAlgorithm->stop();
                myMultiUSSweepRecorderAlgorithm->output(datalist);
            }
            UltrasoundSweep *usSweep = static_cast<UltrasoundSweep *>(datalist.getItem(0));

            // necessary???
            //   usSweep->tracking()->setTemporalOffset(146);
            Selection sel;
            sel.setFirst(fmax(((usSweep->size() / 30) * numberOfPartialSweeps)-200, 0));
            sel.setLast((usSweep->size() / 30) * numberOfPartialSweeps);

            usSweep->setSelection(sel);
            // usSweep->tracking()->setTemporalOffset(m_robotCtrlUS->getTemporalCalibration());
            // setConvexGeometry(usSweep);
            auto *sc2 = new GlSweepCompounding(*usSweep);
            sc2->setMode(0); // GPU    4 is CPU Maximum
            sc2->compute();  // We do the volume compounding
            DataList datalist3;
            sc2->output(datalist3);

            m_main->dataModel()->add(datalist3.getItem(0), "Partial Volume ");
//            m_main->dataModel()->add(usSweep, "Partial Sweep ");

            if (m_exportSweeps) {
                auto sis = static_cast<SharedImageSet *>(datalist3.getItem(0));
                sis->get()->sync();
                auto str = getDayAndTime();
                BackgroundExporter *sweepExporter = new BackgroundExporter();
                sweepExporter->save(usSweep, "/home/javi/Data/VienaRoboticSpine/sweep_" + str + ".imf");
                BackgroundExporter *volExporter = new BackgroundExporter();
                volExporter->save(sis, "/home/javi/Data/VienaRoboticSpine/vol_" + str + ".imf");
            }

            delete sc2;
//            if (numberOfPartialSweeps > 1)
//                compoundAllSweeps();

            // if (!partial)
            // {
            //     if (m_SweepRecorder->numberOfPartialSweeps != 1)
            //         compoundAllSweeps(m_SweepRecorder->numberOfPartialSweeps);
            // }
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
