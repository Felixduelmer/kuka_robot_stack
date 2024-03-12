#include <ImFusion/Core/Log.h>
#include <ImFusion/Stream/TrackingStreamData.h>
#include "CustomROSTopicTrackingStream.h"
#include <ros/node_handle.h>
#include <ImFusion/ROS/ROSManager.h>
#include <ImFusion/ROS/ROSConversions.h>


namespace ImFusion
{

    CustomROSTopicTrackingStream::CustomROSTopicTrackingStream(const std::string& name)
            : LiveTrackingStream(name)
    {
        m_listener = std::make_unique<ros::Subscriber>();

        m_callback = [this](const iiwa_msgs::CartesianPose&msg){
            auto mat = rosPoseToMat4(msg.poseStamped.pose);
            auto timestamp = rosTimeToTimestampMs(msg.poseStamped.header.stamp);
            transformCallback(timestamp, mat);
        };
    }

    bool CustomROSTopicTrackingStream::open()
    {
        return true;
    }

    bool CustomROSTopicTrackingStream::close()
    {
        return stop();
    }

    bool CustomROSTopicTrackingStream::isRunning() const
    {
        return true;
    }

    std::vector<TrackingInstrument> CustomROSTopicTrackingStream::devices() const
    {
        std::lock_guard<std::mutex> handle(m_lastDataMtx);
        return m_trackingInstruments;
    }

    std::string CustomROSTopicTrackingStream::uuid()
    {
        return std::to_string(reinterpret_cast<ptrdiff_t>(this));
    }

    bool CustomROSTopicTrackingStream::start()
    {
        if (!ROS::ImFusionROSManager::instance().isConnectedToMaster())
        {
            LOG_ERROR("Cannot start ROS Topic TrackingStream: cannot connect to ROS master");
            return false;
        }

        if (p_topicName.value().empty() || m_trackingInstrumentName.empty())
        {
            LOG_ERROR("cannot start ROS Topic TrackingStream for topic " << p_topicName.value() << " and TrackingInstrument " << m_trackingInstrumentName);
            return false;
        }

        {
            std::string topicType = ROS::ImFusionROSManager::instance().typeOfROSTopic(p_topicName);
            if (!topicType.empty() && m_topicType != topicType)
            {
                LOG_INFO("Updating the type of the ROS TrackingStream topic: " << p_topicName.value() << " to: " << topicType);
                m_topicType = topicType;
            }
        }

        if (m_topicType == "iiwa_msgs/CartesianPose")
        {

            *m_listener = ROS::ImFusionROSManager::instance().nodeHandle().subscribe<iiwa_msgs::CartesianPose>(
                    p_topicName, m_queueSize, [this](const ros::MessageEvent<iiwa_msgs::CartesianPose const>& msg) {
                        auto const_ref_msg = *(msg.getConstMessage().get());
                        m_callback(const_ref_msg);
                    });
        }
        else
        {
            LOG_ERROR("Unsupported ROS topic type " << m_topicType << " to create a TrackingStream");
            return false;    // unsupported topic type
        }
        return isRunning();
    }

    bool CustomROSTopicTrackingStream::stop()
    {
        m_listener = nullptr;
        return true;
    }


    void CustomROSTopicTrackingStream::transformCallback(const unsigned long long timestamp, const ImFusion::mat4& newTransform)
    {
        TrackingInstrument ti;
        ti.name = m_trackingInstrumentName;
        ti.active = true;
        ti.matrix = newTransform;
        ti.timeStamp = timestamp;
        ti.quality = 1;
        std::vector<TrackingInstrument> instruments = {ti};

        TrackingStreamData trackingData(this, instruments);
        trackingData.setTimestampDevice(timestamp);
        trackingData.setTimestampArrival(std::chrono::system_clock::now());

        {
            std::lock_guard<std::mutex> handle(m_lastDataMtx);
            m_trackingInstruments = instruments;
        }
        updateListenersData(trackingData);

        // update own matrix so that it shows in GUI
        Data::setMatrix(newTransform);
    }
}