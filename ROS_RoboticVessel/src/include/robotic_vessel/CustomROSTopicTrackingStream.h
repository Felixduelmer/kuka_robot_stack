/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved. */
#pragma once

#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ros/subscriber.h>
#include <iiwa_msgs/CartesianPose.h>
#include <ImFusion/ROS/ROSConversions.h>
#include <ImFusion/Stream/CreateStreamIoAlgorithm.h>
#include <ImFusion/Core/Parameter.h>

namespace ImFusion
{
	class TrackingStreamData;
	class CustomROSTopicTrackingStream : public LiveTrackingStream
	{
	public:
		explicit CustomROSTopicTrackingStream(const std::string& name = "ROS Topic Tracking Stream");

		~CustomROSTopicTrackingStream() { };

        //void configure(const Properties* p) override;
        //void configuration(Properties* properties) const override;

        bool open() override;
        bool close() override;
        bool start() override;
        bool stop() override;
        bool isRunning() const override;

        std::string uuid() override;


        std::vector<TrackingInstrument> devices() const override;

        ImFusion::Parameter<std::string> p_topicName = {"Ros Topic", "/iiwa/state/CartesianPose", *this};

    private:

        std::string m_topicType = "iiwa_msgs/CartesianPose";                 ///< Type of the ROS topic on which the transform will be published
        std::string m_trackingInstrumentName = "robotFlange";    ///< Name of the tracking instrument relative to this stream

        int m_queueSize = 10;
        mutable std::mutex m_lastDataMtx;    ///< Synchronizes access to m_trackingInstruments
        std::vector<TrackingInstrument> m_trackingInstruments;
        std::unique_ptr<ros::Subscriber> m_listener;
        std::function<void(const iiwa_msgs::CartesianPose&)> m_callback;


        void transformCallback(unsigned long long timestamp, const ImFusion::mat4& newTransform);
	};

}
