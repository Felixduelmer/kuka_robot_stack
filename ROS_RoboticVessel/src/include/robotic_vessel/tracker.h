#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <math.h>
#include <QThread>
#include <QObject>
#include <boost/circular_buffer.hpp>
#include "ContourProperties.h"

namespace ImFusion {
    namespace ROS_RoboticVessel {

        class Tracker : public QObject {
            Q_OBJECT
        public:
            Tracker();

            ~Tracker();

        signals:
            void lostDoppler();
            void foundDoppler();

        public slots:

            void doppler_tracker(cv::Mat image);

        private slots:

            void cleanup();

        private:
            int numPast = 15;
            int occ = 3;
            int dista = 50;
            int* numPastFrames = &numPast;
            int* minOccurence = &occ;
            int* maxDistance = &dista;
            int trackId = 0;
//            std::unique_ptr<QThread> m_thread;
            std::map<int, boost::circular_buffer<ContourProperties>> trackingObjects;
            boost::circular_buffer<bool> objectDetected = boost::circular_buffer<bool>(numPast);
        };
    }
}
