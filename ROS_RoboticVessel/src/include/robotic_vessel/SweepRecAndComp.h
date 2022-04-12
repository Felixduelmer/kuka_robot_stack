#pragma once

#include <Eigen/Dense>
#include <QtCore/QObject>
#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/US/USSweepRecorderAlgorithm.h>
#include <ImFusion/Base/DataList.h>
#include <QtCore/QThread>

namespace ImFusion {
    namespace ROS_RoboticVessel {

        class SweepRecAndComp : public QObject {
        Q_OBJECT
        public:
            explicit SweepRecAndComp(MainWindowBase *mainWindowBase);

            void startSweepRecording();

            void compoundAllSweeps();

            std::string getDayAndTime();
        public slots:
            void onUpdateVolume();


        private:
            MainWindowBase *m_main{nullptr};
            QTimer *timer;
            bool m_useDummyData = true;
            bool m_exportSweeps = false;
            int numberOfPartialSweeps{0};
            DataList m_dataList;
            USSweepRecorderAlgorithm *myMultiUSSweepRecorderAlgorithm{};


        };


    }  // namespace vision_control
}  // namespace ImFusion
