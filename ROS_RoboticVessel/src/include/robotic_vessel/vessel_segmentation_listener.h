#pragma once

#include <ImFusion/Base/Algorithm.h>
#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/Stream/StreamListener.h>
#include <ImFusion/Base/MemImage.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include "robotic_vessel/sweep_rec_and_comp.h"
#include "robotic_vessel/tracker.h"
#include <QtCore/QObject>

#undef slots

#include <torch/script.h> // One-stop header

#define slots Q_SLOTS

namespace ImFusion {
    namespace ROS_RoboticVessel {

        class LiveSegmentationStream : public QObject, public ImageStream, public StreamListener {
        Q_OBJECT
        public:
            LiveSegmentationStream();

            ~LiveSegmentationStream() override { m_inStream->removeListener(this); }

            void addInputStream(ImageStream *imgStream);

            bool open() override;

            bool close() override;

            bool start() override;

            bool stop() override;

            bool isRunning() const override;

            std::string uuid() override;

            void onStreamData(const StreamData &streamData) override;

            at::Tensor preProcessData(std::unique_ptr<MemImage> memImage);

            void initState();

            static at::Tensor toTensor(const cv::Mat &img_us, const cv::Mat &img_doppler);

        signals:

            void newDopplerImage(cv::Mat image);


        private:
            ImageStream *m_inStream;
            torch::jit::script::Module model;
            std::vector<torch::jit::IValue> inputs;
            c10::List<torch::Tensor> state;
        };


    }
}
