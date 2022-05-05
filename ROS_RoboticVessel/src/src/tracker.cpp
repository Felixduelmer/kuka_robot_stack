//
// Created by robotics-verse on 02.05.22.
//
#include "include/robotic_vessel/tracker.h"

namespace ImFusion {
    namespace ROS_RoboticVessel {
        Tracker::Tracker() {
//            qRegisterMetaType<cv::Mat>("cvMat");
            m_thread.reset(new QThread);
            moveToThread(m_thread.get());
            m_thread->start();
        }

        Tracker::~Tracker() {
            QMetaObject::invokeMethod(this, "cleanup");
            m_thread->wait();
        }

        static void dummyCallback(int img, void*) {}


        void Tracker::doppler_tracker(cv::Mat image) {

            long _numPastFrames = *numPastFrames;
            float _minOccurence = (*minOccurence)/10.0;
            int _maxDistance = *maxDistance;
            cv::createTrackbar("MaxDistance",
                               "dopplerImage", maxDistance,
                               150, &dummyCallback); // Create a Trackbar to choose type of Threshold
            cv::createTrackbar("MinOccurence",
                               "dopplerImage", minOccurence,
                               10, &dummyCallback); // Create a Trackbar to choose type of Threshold
            cv::createTrackbar("NumPastFrame",
                               "dopplerImage", numPastFrames,
                               50, &dummyCallback); // Create a Trackbar to choose type of Threshold
//            cv::Mat image(320, 320, CV_8UC1, cv::Scalar(255));


            std::vector<cv::Point> contours_poly;
            std::vector<std::vector<cv::Point>> contours;
            float radius = 0;
            cv::Point_<float> center;

            std::vector<ContourProperties> contPropCurrFrame;

            //finding the contours in the doppler image
            image = image.setTo(255, image > 10);
            cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            cv::Mat contImage = cv::Mat::zeros(image.size(), CV_8UC1);

            //adding the center point and radius to a list;
            for (auto &contour: contours) {
                cv::approxPolyDP(contour, contours_poly, 10, true);
                cv::minEnclosingCircle(contours_poly, center, radius);
                if (radius < 5) {
                    continue;
                }
                contPropCurrFrame.push_back(ContourProperties(center, radius));
                cv::circle(contImage, center, radius, 255, -1);
            }
            std::cout << " Size of detected contourproperties: " << contours.size() << std::endl;
            std::cout << " Size of detected contourproperties with large enough radius: " << contPropCurrFrame.size() << std::endl;



            for (auto &trackingObj: trackingObjects) {
                // Vector to store column elements
                std::vector<double> distance;
                std::vector<double> area;

                for (auto &contProp: contPropCurrFrame) {
                    distance.push_back(cv::norm(
                            trackingObj.second.front().getCenter() - contProp.getCenter()));
                }
                //if there is an object at a similar location add it to the buffer of this object otherwise add a dummy object
                for(int i= 0; i < distance.size(); i++){
                    area.push_back(distance[i]<_maxDistance ? contPropCurrFrame[i].getArea() : 0);
                }
                long idx = std::max_element(area.begin(), area.end()) - area.begin();
                if (!contPropCurrFrame.empty() && area[idx] != 0) {
                    trackingObj.second.push_front(contPropCurrFrame[idx]);
                    contPropCurrFrame.erase(contPropCurrFrame.begin() + idx);
                } else {
                    auto ref = trackingObj.second.front();
                    trackingObj.second.push_front(ContourProperties(ref.getCenter(), ref.getRadius(), true));
                }
            }

            //add new Ids found
            std::cout << "Found new contour properties: " << contPropCurrFrame.size() << std::endl;
            std::cout << "Number of tracking Objects: " << trackingObjects.size() << std::endl;
            for (int t = 0; t < contPropCurrFrame.size(); t++) {
                trackingObjects[trackId] = boost::circular_buffer<ContourProperties>(_numPastFrames);
                trackingObjects[trackId].push_front(contPropCurrFrame[0]);
                trackId++;
            }

            cv::Mat annotations = cv::Mat::zeros(image.size(), CV_8UC1);

            if(trackingObjects.size() != 0) {
                for (auto it = trackingObjects.begin(); it != trackingObjects.end();) {
                    int numOccurences = count_if(it->second.begin(), it->second.end(),
                                                 [](const ContourProperties &cp) { return !cp.isDummyValue(); });
                    if (numOccurences > _minOccurence * _numPastFrames) {
                        auto ref = it->second.front();
                        //probably mean of radius is good
                        cv::circle(annotations, ref.getCenter(), ref.getRadius(), 255, -1);
                        ++it;
                    } else {
                        //clean up tracking objects if this object has not been seen recently
                        if (it->second.size() > _minOccurence * _numPastFrames) {
                            it = trackingObjects.erase(it);
                        } else {
                            ++it;
                        }
                    }
                }
            }
            cv::imshow("contImage", contImage);
            cv::imshow("dopplerImage", image);
            cv::imshow("annotationImage", annotations);


//
//
////            std::cout << contPropCurrFrame.size() << std::endl;
//            for (std::pair<int, boost::circular_buffer<ContourProperties>> trackingObject: trackingObjects) {
//                bool object_exists = false;
//                for (auto &contProperty: contPropCurrFrameCopy) {
//                    double distance = cv::norm(
//                            contProperty.getCenter() - trackingObject.second[0].getCenter());
//                    // Update IDs position
//                    //If distance is smaller then predefined distance object exists
//                    //then check if in the current vector of contours has a contour which is bigger -> prefer this
//                    //If
//
//                    if (distance < _maxDistance) {
//                        if (object_exists) {
//                            if (trackingObject.second[0].getArea() > contProperty.getArea()) {
//                                std::cout << "Object is skipped" << std::endl;
//                                continue;
//                            } else {
//                                contPropCurrFrame.push_back(trackingObject.second[0]);
//                                trackingObject.second.pop_front();
//                            }
//                        }
//                        trackingObject.second.push_front(contProperty);
//                        object_exists = true;
//                        std::cout << "Object exist" << std::endl;
//                        contPropCurrFrame.erase(std::remove(contPropCurrFrame.begin(), contPropCurrFrame.end(),
//                                                            contProperty), contPropCurrFrame.end());
//                    }
//                }
//                // Pop element if it has not been visible recently or add a dummy value
//                if (!object_exists) {
//                    std::cout << "Object didn't exist" << std::endl;
//                    long numOccurences = count_if(trackingObject.second.begin(),
//                                                  trackingObject.second.end(),
//                                                  [](const ContourProperties &cp) { return !cp.isDummyValue(); });
//                    if (numOccurences < _minOccurence * _numPastFrames &&
//                        trackingObject.second.size() >= _minOccurence * _numPastFrames) {
//                        trackingObjects.erase(trackingObject.first);
//                        std::cout << "erased element form tracking elements: " << std::endl;
//                    } else {
//                        trackingObject.second.push_front(ContourProperties());
//                    }
//                }
////                std::cout << "Size in for loop: " << contPropCurrFrame.size() << std::endl;
//
//            }
//            //add new Ids found
//            std::cout << "Found new contour properties: " << contPropCurrFrame.size() << std::endl;
//            for (int t = 0; t < contPropCurrFrame.size(); t++) {
//                trackingObjects[trackId] = boost::circular_buffer<ContourProperties>(_numPastFrames);
//                trackingObjects[trackId].push_front(contPropCurrFrame[0]);
//                trackId++;
//            }
////            std::cout << "Test 3" << std::endl;
//
//
//            cv::Mat annotations = cv::Mat::zeros(image.size(), CV_8UC3);
//            for (const auto &trackingObj: trackingObjects) {
//                int numOccurences = count_if(trackingObj.second.begin(), trackingObj.second.end(),
//                                             [](const ContourProperties &cp) { return !cp.isDummyValue(); });
////                std::cout << "number of occurences of track " << trackingObj.first << ": " << numOccurences
////                          << std::endl;
//                if (numOccurences > _minOccurence * _numPastFrames) {
//                    auto firstOccurence = std::find_if(trackingObj.second.begin(), trackingObj.second.end(),
//                                                       [](const ContourProperties &cp) { return !cp.isDummyValue(); });
//                    cv::circle(annotations, firstOccurence->getCenter(), firstOccurence->getRadius(), 255, -1);
//                }
//            }

        }

        void Tracker::cleanup() {
            //do cleanup if necessary

            m_thread->quit();
        }

    }

}