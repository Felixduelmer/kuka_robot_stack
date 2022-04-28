#pragma once

#include <Eigen/Dense>
#include <QtCore/QObject>
#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/US/USSweepRecorderAlgorithm.h>
#include <ImFusion/US/USSweepRecorderController.h>
#include <ImFusion/Base/DataList.h>
#include <QtCore/QThread>
#include <ImFusion/Stream/FakeTrackingStream.h>
#include <ImFusion/US/UltrasoundSweepRingBuffer.h>

namespace ImFusion {
    namespace ROS_RoboticVessel {

        class SweepRecAndComp : public QObject {
        Q_OBJECT
        public:
            explicit SweepRecAndComp(MainWindowBase *mainWindowBase);

            void startSweepRecording();

            void compoundAllSweeps();
            void stop();

            std::string getDayAndTime();
        public slots:
            void onUpdateVolume();


        private:
            MainWindowBase *m_main{nullptr};
            QTimer *timer;
            bool m_useLiveData = true;
            bool m_exportSweeps = false;
            int numberOfPartialSweeps{0};
            DataList m_dataList;
            UltrasoundSweepRingBuffer* ringBuffer;
            USSweepRecorderAlgorithm *myMultiUSSweepRecorderAlgorithm{};


        };


    }  // namespace vision_control
}  // namespace ImFusion
