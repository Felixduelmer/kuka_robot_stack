#pragma once

#include <ImFusion/Base/Algorithm.h>

#include <QObject>
#include <QtCore/QThread>

#include <string>

#include <ImFusion/Stream/ImageStream.h>
#include <QTimer>
#include <ImFusion/Stream/PlaybackStream.h>
#include <ImFusion/GUI/MainWindowBase.h>
//#include <ImFusion/Cephasonics/CephasonicsStream.h>
//#include <ImFusion/ROS/Stream/ROSTopicImageOutStream.h>

namespace ImFusion {
    namespace ROS_RoboticVessel {

//class PluginAlgorithm : public QObject, public ImFusion::Algorithm {
        class PluginAlgorithm : public QThread, public ImFusion::Algorithm {
        Q_OBJECT
        public:
            PluginAlgorithm() = default;

            ~PluginAlgorithm() = default;

            void compute();

            void run() override;

            void preprocess_Image(ImageStream *usStream);

            void startDummyStream(MainWindowBase* main);

            //! Methods implementing the Configurable interface.
            void configure(const Properties *p);

            void configuration(Properties *p) const;

            static bool createCompatible(const DataList &data, Algorithm **a = nullptr);

        public
            slots:

            void imageCallBack();

        private:
            ImageStream *usStream;
            QTimer *timer;
            PlaybackStream *playbackStream;

//  ROSTopicImageOutStream* rosOutStr;

        protected:
            bool run_{true};

        };
    }  // namespace ROS_RoboticVessel
}  // namespace ImFusion
