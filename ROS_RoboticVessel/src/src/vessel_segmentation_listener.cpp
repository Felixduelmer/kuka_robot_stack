#include "robotic_vessel/vessel_segmentation_listener.h"

#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/Base/MemImage.h>
#include <chrono>
#include <ImFusion/Stream/TrackingStreamData.h>


namespace ImFusion {
    namespace ROS_RoboticVessel {
        LiveSegmentationStream::LiveSegmentationStream() {
            try {
                model = torch::jit::load("/home/robotics-verse/projects/felix/traced_butterfly_model.pt");
                model.to(at::kCUDA);
                initState();
            } catch (...) {
                std::cout << "encountered an error loading the model!" << std::endl;
            }
        }

        void LiveSegmentationStream::onStreamData(const StreamData &streamData) {
            std::chrono::steady_clock::time_point receivedImage = std::chrono::steady_clock::now();
            auto *imgData = dynamic_cast<const ImageStreamData *>(&streamData);
            std::unique_ptr<MemImage> image = imgData->images().front()->clone2();
            at::Tensor tensor = preProcessData(std::move(image));
//            std::cout << "max image value: " << torch::max(tensor) << std::endl;
            inputs.clear();
            tensor = tensor.toType(torch::kFloat);
            inputs.push_back(tensor.to(at::kCUDA));
            inputs.push_back(state);
            c10::intrusive_ptr<c10::ivalue::Tuple> output;
            std::chrono::steady_clock::time_point begin_model = std::chrono::steady_clock::now();
            try {
                output = model.forward(inputs).toTuple();
            }
            catch (std::exception &e) {
                std::cout << e.what() << std::endl;
                return;
            }
            std::chrono::steady_clock::time_point end_model = std::chrono::steady_clock::now();
            auto output_image_tensor = output->elements()[0].toTensor();
//            std::cout << torch::max(output_image_tensor) << std::endl;
            output_image_tensor = output_image_tensor.squeeze().detach();
            output_image_tensor = torch::round(output_image_tensor).mul(255).to(torch::kU8);
            output_image_tensor = output_image_tensor.to(torch::kCPU);

            auto newState = output->elements()[1].toList();
            state.clear();
            for (int i = 0; i < newState.size(); i++) {
                state.push_back(newState.get(i).toTensor().detach());
            }

            cv::Mat cvOutputImage = cv::Mat(cv::Size(320, 320), CV_8U, output_image_tensor.data_ptr<uchar>());
//            cv::imshow("segImage", cvOutputImage);
            std::chrono::steady_clock::time_point receivedImageEnd = std::chrono::steady_clock::now();
//            std::cout << "Model took = "
//                      << std::chrono::duration_cast<std::chrono::milliseconds>(end_model - begin_model).count()
//                      << "[µs]" << std::endl;
//            std::cout << "Whole step took = "
//                      << std::chrono::duration_cast<std::chrono::milliseconds>(receivedImageEnd - receivedImage).count()
//                      << "[µs]" << std::endl;

            int nWidth = 501;
            int nHeight = 699;
            cv::Mat cvOutputResized(nHeight, nWidth, CV_8UC1, cv::Scalar(255));
            cv::resize(cvOutputImage, cvOutputResized, cvOutputResized.size(), 0, 0);

//                 prepare stream outpuy
            MemImage *outImg = MemImage::create(Image::UBYTE, nWidth, nHeight, 1, 1);
            memcpy(outImg->data(), cvOutputResized.data, cvOutputResized.rows * cvOutputResized.cols * sizeof(uchar));

            outImg->setSpacing(31.8 / nWidth, 45.0 / nHeight, 1);
            ImageStreamData oisd(this);
            oisd.setTimestampArrival(imgData->timestampArrival());
            oisd.setTimestampDevice(imgData->timestampDevice());
            std::vector<MemImage *> streamImages = {outImg};
            oisd.setImages(streamImages);
            updateListenersData(oisd);
        }

        at::Tensor LiveSegmentationStream::preProcessData(std::unique_ptr<MemImage> memImage) {
            //copy to cv mat image
            cv::Mat cv_imgIn(memImage->height(), memImage->width(), CV_8UC3);  //uchar
            memcpy(cv_imgIn.data, memImage->data(), memImage->byteSize());

            //cut image
            auto us_image = cv_imgIn(cv::Rect(472, 136, 501, 699));
            auto doppler_image = cv_imgIn(cv::Rect(1004, 136, 501, 699));
            //resize image
            cv::Mat doppler_image_resized(320, 320, CV_8UC1, cv::Scalar(255));
            cv::Mat us_image_resized(320, 320, CV_8UC1, cv::Scalar(255));
            cv::resize(doppler_image, doppler_image_resized, doppler_image_resized.size(), 0, 0);
            cv::resize(us_image, us_image_resized, us_image_resized.size(), 0, 0);
            cv::cvtColor(doppler_image_resized, doppler_image_resized, cv::COLOR_BGR2HSV);

            //limit doppler
            cv::Mat threshold, doppler_thresholded, doppler_thres_col, doppler_final, us_final(320, 320, CV_8U);
            cv::inRange(doppler_image_resized, cv::Scalar(0, 100, 20), cv::Scalar(180, 255, 255), threshold);
            cv::bitwise_and(doppler_image_resized, doppler_image_resized, doppler_thresholded, threshold);

            //convert to grayscale images
            cv::cvtColor(doppler_thresholded, doppler_thres_col, cv::COLOR_HSV2BGR);
            cv::cvtColor(doppler_thres_col, doppler_final, cv::COLOR_BGR2GRAY);
            cv::cvtColor(us_image_resized, us_final, cv::COLOR_BGR2GRAY);

//            cv::imshow("usImage", us_final);
//            cv::imshow("dopplerImage", doppler_final);
            emit newDopplerImage(doppler_final.clone());

            return toTensor(us_final, doppler_final);

        }

        at::Tensor LiveSegmentationStream::toTensor(const cv::Mat &img_us, const cv::Mat &img_doppler) {
            at::Tensor tensor_us = torch::from_blob(img_us.data, {1, 1, img_us.rows, img_us.cols}, torch::kU8);
            at::Tensor tensor_doppler = torch::from_blob(img_doppler.data, {1, 1, img_doppler.rows, img_doppler.cols},
                                                         torch::kU8);

            std::vector<at::Tensor> vectorList;
            vectorList.push_back(tensor_us);
            vectorList.push_back(tensor_doppler);
            at::TensorList list_tensor = at::TensorList(vectorList);
            at::Tensor tensor = torch::concat(list_tensor, 1);
            return tensor;
        }

        void LiveSegmentationStream::initState() {
            state.push_back(torch::zeros({1, 64, 20, 20}).to(at::kCUDA));
            state.push_back(torch::zeros({1, 64, 20, 20}).to(at::kCUDA));
            state.push_back(torch::zeros({1, 128, 20, 20}).to(at::kCUDA));
        }

        bool LiveSegmentationStream::open() {
            return true;
        }

        bool LiveSegmentationStream::close() {
            return true;
        }

        bool LiveSegmentationStream::start() {
            return true;
        }

        bool LiveSegmentationStream::stop() {
            return true;
        }

        bool LiveSegmentationStream::isRunning() const {
            return true;
        }

        std::string LiveSegmentationStream::uuid() {
            return std::__cxx11::string("dummy id");
        }

        void LiveSegmentationStream::addInputStream(ImageStream* imgStream) {
            m_inStream = imgStream;
            m_inStream->addListener(this);
        }


    }
}

