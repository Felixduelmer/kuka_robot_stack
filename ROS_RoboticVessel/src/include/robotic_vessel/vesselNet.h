
#include <ImFusion/Base/Algorithm.h>
#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/Stream/StreamListener.h>
#include <ImFusion/Base/MemImage.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#undef slots
#include <torch/script.h> // One-stop header
#define slots Q_SLOTS

namespace ImFusion {
    namespace ROS_RoboticVessel {

        class VesselSegmentationProcessingStream
                : public ImageStream, public StreamListener {
        public:
            explicit VesselSegmentationProcessingStream(ImageStream *imgStream);

            ~VesselSegmentationProcessingStream() override { m_inStream->removeListener(this); }

            void onStreamData(const StreamData &streamData) override;

            at::Tensor preProcessData(std::unique_ptr<MemImage> memImage);
            void initState();
            static at::Tensor toTensor(const cv::Mat& img_us, const cv::Mat& img_doppler);
            bool open() override {return true;}
            bool close() override {return true;}
            bool start() override {return true;}
            bool stop() override {return true;}
            std::string uuid() override {return "test";}
            bool isRunning() const override {return true;} ;

        private:
            ImageStream *m_inStream;
            torch::jit::script::Module model;
            std::vector<torch::jit::IValue> inputs;
            c10::List<torch::Tensor> state;

        };


        class MyImageStreamLiveProcessingAlgorithm : public Algorithm {
        public:
            MyImageStreamLiveProcessingAlgorithm(ImageStream *imgStream)
                    : m_inStream(imgStream), m_outStream(new VesselSegmentationProcessingStream(m_inStream)) {}

            void compute() override { m_status = (int) Status::Success; }

            void output(DataList &dataOut) override { dataOut.add(m_outStream); }

        private:
            ImageStream *m_inStream;
            VesselSegmentationProcessingStream *m_outStream;
        };


    }
}
