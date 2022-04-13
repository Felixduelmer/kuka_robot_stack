
#include <ImFusion/Base/Algorithm.h>
#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/Stream/StreamListener.h>
#include <ImFusion/Base/MemImage.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include "robotic_vessel/sweep_rec_and_comp.h"
#undef slots
#include <torch/script.h> // One-stop header
#define slots Q_SLOTS

namespace ImFusion {
    namespace ROS_RoboticVessel {

        class LiveSegmentationStream : public ImageStream, public StreamListener {
        public:
            LiveSegmentationStream(ImageStream *imgStream);
            LiveSegmentationStream();
            ~LiveSegmentationStream() override { m_inStream->removeListener(this); }

            bool open() override;

            bool close() override;

            bool start() override;

            bool stop() override;

            bool isRunning() const override;

            std::string uuid() override;

            void onStreamData(const StreamData &streamData) override;

            at::Tensor preProcessData(std::unique_ptr<MemImage> memImage);
            void initState();
            static at::Tensor toTensor(const cv::Mat& img_us, const cv::Mat& img_doppler);

        private:
            ImageStream *m_inStream;
            torch::jit::script::Module model;
            std::vector<torch::jit::IValue> inputs;
            c10::List<torch::Tensor> state;
        };


    }
}
