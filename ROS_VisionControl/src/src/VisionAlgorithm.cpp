#include "vision_control/VisionAlgorithm.h"

#include <ImFusion/Core/Log.h>
#include <ImFusion/Stream/IgtlConnection.h>
#include <ImFusion/Stream/TrackingStreamData.h>
#include <QDebug>
#include <QVector3D>
using namespace std;
using namespace cv;
namespace ImFusion {
namespace vision_control {


PluginAlgorithm::PluginAlgorithm()
 {
  onInitROS();
    probe_rotation_.block<4, 4>(0, 0) << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;
}


//execute a movement along defined points
void PluginAlgorithm::executeTrajectory() {

    LOG_INFO("Start to exexute Trajectory");
    n_poses = manual_traj_points_.size();
    LOG_INFO("Number of points to go to: " + std::to_string(n_poses));
    FinishedMoveToNewPointCallback();

}



//Callback to continue trajectory
void PluginAlgorithm::FinishedMoveToNewPointCallback() {

    LOG_INFO("FinishedMoveToNewPointCallback");
    if (m_nTrajPoints < n_poses && m_nTrajPoints >= 0) {

      onMoveToNewPoint();
      m_nTrajPoints++;
    }
    else {
    if(m_nTrajPoints == n_poses) {
    LOG_INFO("reached final point");
    m_nTrajPoints = 0;
    }

}
}

//execute a movement along defined points
void PluginAlgorithm::onMoveToNewPoint() {

    LOG_INFO("onMoveToNewPoint");
    LOG_INFO("Current destination point : " + std::to_string(m_nIteration));
    executeCartesianCommand(manual_traj_points_[m_nTrajPoints].pose, true, std::bind(&PluginAlgorithm::FinishedMoveToNewPointCallback, this));

}



//initialize the ros related node.
void PluginAlgorithm::onInitROS() {
    std::map<std::string, std::string> emptyArgs;
    if (!ros::isInitialized()) { ros::init(emptyArgs, "iiwaRos"); }
    ros_spinner_ = std::make_unique<ros::AsyncSpinner>(1);
    ros_spinner_->start();

    // create a ROS handle to connect the neural network and the movement tracking parts
    ros::NodeHandle nh;
    this->m_ros_initialized = true;
    LOG_INFO(this->m_ros_initialized);
}


void PluginAlgorithm::connect(const std::string &probe_name) {
    //innitialize Ros and iiwaRos object
    std::map<std::string, std::string> emptyArgs;
    if (!ros::isInitialized()) { ros::init(emptyArgs, "iiwaRos"); }
    ros_spinner_ = std::make_unique<ros::AsyncSpinner>(1);
    ros_spinner_->start();

    ros::NodeHandle node_handle;

    pose_state_.init("iiwa", std::bind(&PluginAlgorithm::poseCallback, this, std::placeholders::_1));
    wrench_state_.init("iiwa", std::bind(&PluginAlgorithm::wrenchCallback, this, std::placeholders::_1));
    torque_state_.init("iiwa");

    pose_command_.init("iiwa");
    linear_pose_command_.init("iiwa");

    control_mode_.init("iiwa");

    IgtlConnection dummy_connection("Service robot connection");
    tracking_stream_ = new IgtlTrackingStream(dummy_connection, "Robot");
    tracking_stream_->open();
    is_robot_connected_ = true;
    loadCalibrationFromFile("IFLUSCalibration.config", probe_name);

    emit robotConnected();
}

void PluginAlgorithm::disconnect() {
  if (ros::ok()) {
    if (ros_spinner_ != nullptr) {
      ros_spinner_->stop();
      ros_spinner_ = nullptr;
    }
  }

  //! If the stream was never passed to a DataModel, we have to dispose it.
  if (owning_stream_ && tracking_stream_ != nullptr) { delete tracking_stream_; }
  is_robot_connected_ = false;
  emit robotDisconnected();
}

PluginAlgorithm::~PluginAlgorithm() { disconnect(); }

bool PluginAlgorithm::createCompatible(const DataList& data, Algorithm** a) {
  if (data.size() != 0) { return false; }
  if (a) { *a = new PluginAlgorithm(); }
  return true;
}

void PluginAlgorithm::compute() {
}

void PluginAlgorithm::configure(const Properties* p) {
  if (p == nullptr) { return; }
  p->param("something", something_);
}

void PluginAlgorithm::configuration(Properties* p) const {
  if (p == nullptr) { return; }
  p->setParam("something", something_);
}

void PluginAlgorithm::doSomething() { emit somethingHappened(); }

void PluginAlgorithm::poseCallback(const iiwa_msgs::CartesianPose& pose){
    if (is_robot_connected_){
        std::lock_guard<std::mutex> lock{pose_mutex_};
        current_tip_pose_ = pose.poseStamped;   //this would be updated after connect automatically

        //! This object is disposed by the destructor of TrackingStreamData, or at least so it says its documentation.
        //! If you try to handle its lifetime -> CRASH, so leave it as it is.
        TrackingInstrument tracking_instrument;
        tracking_instrument.active = true;
        tracking_instrument.name = "US";
        tracking_instrument.quality = 1;
//        auto image_center_pose = poseToEigenMat4(pose.poseStamped.pose, 1000) * probe_rotation_ * ultrasound_calibration_;

        auto image_center_pose = poseToEigenMat4(pose.poseStamped.pose, 1000) * probe_rotation_;

        tracking_instrument.matrix = image_center_pose;

        current_image_center_pose_.pose = eigenMat4ToPose(image_center_pose);
        current_image_center_pose_.header = pose.poseStamped.header;

        std::vector<TrackingInstrument> instrument_vector = {tracking_instrument};
        TrackingStreamData datas(tracking_stream_, instrument_vector);

        std::chrono::system_clock::time_point arrivalTime = std::chrono::high_resolution_clock::now();
        datas.setTimestampArrival(arrivalTime);

        tracking_stream_->signalNewData.emitSignal(datas); 
        // sendStreamData(datas);

        emit poseChanged();
      }
    }

void PluginAlgorithm::wrenchCallback(const iiwa_msgs::CartesianWrench &wench) {
    if (is_robot_connected_){
        std::lock_guard<std::mutex> lock{wrench_mutex_};
        current_tip_wrench_ = wench.wrench;   //this would be updated after connect automatically

        emit wrenchChanged();
      }
    }

void PluginAlgorithm::executeCartesianCommand(const geometry_msgs::Pose& pose, bool linear,
                                              const std::function<void()>& callback) {
    LOG_INFO("enter executeCartesianCommand");
    if (is_robot_connected_) {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "iiwa_link_0";
    ps.pose = pose;

    if (linear == true) {
      if (callback == nullptr) {
        linear_pose_command_.setPose(ps);
      } else {
        linear_pose_command_.setPose(ps, callback);
      }
    } else {
      if (callback == nullptr) {
        pose_command_.setPose(ps);
      } else {
        pose_command_.setPose(ps, callback);
      }
    }
  } else {
    LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
  }
}

void PluginAlgorithm::executeCartesianCommand(const Eigen::Matrix4d& matrix, bool linear,
                                              const std::function<void()>& callback) {
  executeCartesianCommand(eigenMat4ToPose(matrix), linear, callback);
}

void PluginAlgorithm::executeCartesianCommand(const Eigen::Quaterniond& q, const Eigen::Vector3d& t, bool linear,
                                              std::function<void()> callback) {
  Eigen::Matrix4d matrix{Eigen::Matrix4d::Identity()};
  matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
  matrix.block<3, 1>(0, 3) = t;
  executeCartesianCommand(eigenMat4ToPose(matrix), linear, callback);
}

// THIS IS IN METERS
geometry_msgs::PoseStamped PluginAlgorithm::getCurrentRobotPose() {
  assert(is_robot_connected_ == true &&
         "The robot has to be connected before receiving any state. Call the 'connect()' method.");
  std::lock_guard<std::mutex> lock{pose_mutex_};
  return current_tip_pose_;
}

// THIS IS IN METERS
Eigen::Matrix4d PluginAlgorithm::getCurrentRobotTransformMatrix(bool in_millimeters) {                 //自变量为：是否是“厘米”单位
  assert(is_robot_connected_ == true &&
         "The robot has to be connected before receiving any state. Call the 'connect()' method.");
  std::lock_guard<std::mutex> lock{pose_mutex_};
  double scaling_factor{1};
  if (in_millimeters) { scaling_factor = 1000; }
  return poseToEigenMat4(current_tip_pose_.pose, scaling_factor);
}

geometry_msgs::PoseStamped PluginAlgorithm::getCurrentImageCenterPose() {
  assert(is_robot_connected_ == true &&
         "The robot has to be connected before receiving any state. Call the 'connect()' method.");
  std::lock_guard<std::mutex> lock{pose_mutex_};
  return current_image_center_pose_;
}

//wrench
geometry_msgs::Wrench PluginAlgorithm::getCurrentRobotWrench() {
  assert(is_robot_connected_ == true &&
         "The robot has to be connected before receiving any state. Call the 'connect()' method.");
  std::lock_guard<std::mutex> lock{wrench_mutex_};  //std::lock_guard用来提供自动为互斥量上锁和解锁的功能
  return current_tip_wrench_;
}


geometry_msgs::Pose PluginAlgorithm::eigenMat4ToPose(Eigen::Matrix4d matrix, double scaling_factor){
    Eigen::Quaterniond q{matrix.block<3, 3>(0, 0)};
    Eigen::Vector3d t{matrix.block<3, 1>(0,3)};  // block matrix:from (0,3) select a (3,1) matrix

    geometry_msgs::Pose pose;
    pose.position.x = t[0] * scaling_factor;
    pose.position.y = t[1] * scaling_factor;
    pose.position.z = t[2] * scaling_factor;

    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();

    return pose;
}

Eigen::Matrix4d PluginAlgorithm::poseToEigenMat4(const geometry_msgs::Pose &pose, double scaling_factor){         //由机器人pos计算
    Eigen::Matrix4d matrix{Eigen::Matrix4d::Identity()};                                                          //初始化4x4矩阵
    matrix.block<3, 3>(0, 0) =       //Block of size (p,q), starting at (i,j) //左上角3*3
            Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)    //四元数 (w,x,y,z) constant scalar
            .toRotationMatrix();
    matrix.block<4, 1>(0, 3) = Eigen::Vector4d(pose.position.x * scaling_factor, pose.position.y * scaling_factor,//
                                               pose.position.z * scaling_factor, 1);
    return matrix;
}

void PluginAlgorithm::loadCalibrationFromFile(const std::string& file_name, const std::string& desired_probe) {
  std::string config_file_path = configuration_dir + file_name;
  std::ifstream config_file(config_file_path);
  if (config_file.is_open()) {
    std::string probe_name;
    while (std::getline(config_file, probe_name)) {
      for (int x = 0; x < 4; x++) {
        std::string line;
        std::getline(config_file, line);
        std::istringstream iss(line);
        for (int y = 0; y < 4; y++) {
          std::string s;
          std::getline(iss, s, ';');
          ultrasound_calibration_(x, y) = std::stod(s);
        }
      }
      std::string line;
      std::getline(config_file, line);
      temporal_calibration_ = std::stof(line);
      if (probe_name.compare(desired_probe) == 0) {
        config_file.close();
//        ultrasound_calibration_ = Eigen::MatrixXd::Identity(4, 4);  //jzl
        ultrasound_calibration_ = Eigen::MatrixXd::Identity(4, 4);
        ultrasound_calibration_.block<3, 1>(0, 3) << 0, 0, 27.5;
        LOG_INFO("" << std::endl
                    << "Calibration found at " << config_file_path << " for probe " << desired_probe << ": "
                    << std::endl
                    << ultrasound_calibration_ << std::endl
                    << "Temporal Calibration: " << temporal_calibration_ << std::endl
                    << std::endl);

        return;
      }
    }
  } else {
    LOG_ERROR("Unable to open file");
  }
  LOG_ERROR("Couldn't find a calibration file at " << config_file_path << " for " << desired_probe
                                                   << ", I will load identity. " << std::endl);
  ultrasound_calibration_ = Eigen::MatrixXd::Identity(4, 4);
  ultrasound_calibration_.block<3, 1>(0, 3) << 0, 27.5, 0;
  LOG_INFO(ultrasound_calibration_ << std::endl);
}

////////////////////////////JZL OrienAdj task
void PluginAlgorithm::fanShapMotion(double doffsetAngle, double dStepAngle, int nRoationAxis, int nFanMotionType) {
///  just need to adjust the angle of Ra!
    LOG_INFO("Enter into FanMotion");
    if (is_robot_connected_) {
        LOG_INFO(nFanMotionType);
        if(FAN_MOTION_STYPE_CONTINOUS == nFanMotionType) {
            LOG_INFO("Enter into FanMotionC");
//            calculateEndPoints(doffsetAngle, nRoationAxis);   //rotate around base
            calculateEndPointsTCP(doffsetAngle, nRoationAxis);   //rotate around TCP
            movementFinishCallback();
        }
        else if(FAN_MOTION_STYPE_STEP == nFanMotionType) {
            LOG_INFO("Enter into FanMotionStep");
//            calculateEndPointsStep(doffsetAngle, dStepAngle, nRoationAxis);   //rotate around base
            calculateEndPointsStepTCP(doffsetAngle, dStepAngle, nRoationAxis);   //rotate around TCP
            m_nIteration = 0;
            stepMovementFinishCallback();
        }

    } else {
        LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
    }
}

//rotate around base frame
void PluginAlgorithm::calculateEndPoints(double fOffsetAngle, int nRotationAxis) {
     if (is_robot_connected_) {
         auto robot_pose = getCurrentRobotTransformMatrix(true);
         Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
         Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //

         Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
         if (ROTATION_X == nRotationAxis) {
             offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI;   //unit rad
             LOG_INFO(fOffsetAngle);
         }
         else if (ROTATION_Y == nRotationAxis) {
             offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI;   //unit rad
         }
         else {
             LOG_INFO("the rotation is wrong");
         }
         Eigen::Vector3d initEulerAngle = eulerAngles - offsetAngleRad;
         Eigen::Vector3d finalEulerAngle = eulerAngles + offsetAngleRad;

         LOG_INFO(initEulerAngle);
         LOG_INFO(finalEulerAngle);

         //back to pose(4*4 matrix)
         Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(initEulerAngle[0], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(initEulerAngle[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(initEulerAngle[2], Eigen::Vector3d::UnitZ());
         m_initialPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
         m_initialPose.block<3, 1>(0, 3) = translation/1000.0;

         tempQuaternion = Eigen::AngleAxisd(finalEulerAngle[0], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(finalEulerAngle[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(finalEulerAngle[2], Eigen::Vector3d::UnitZ());
         m_finalPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
         m_finalPose.block<3, 1>(0, 3) = translation/1000.0;
     } else {
         LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
     }
}


double angle(double x1, double y1, double x2, double y2)
{
  double angle_temp;
  double xx, yy;
  xx = x2 - x1;
  yy = y2 - y1;
  if (xx == 0.0)
    angle_temp = PI / 2.0;
  else
    angle_temp = atan(fabs(yy / xx));

  if ((xx < 0.0) && (yy >= 0.0))
    angle_temp = PI - angle_temp;
  else if ((xx < 0.0) && (yy < 0.0))
    angle_temp = PI + angle_temp;
  else if ((xx >= 0.0) && (yy < 0.0))
    angle_temp = PI * 2.0 - angle_temp;
  return (angle_temp);
};



}  // namespace OrienAdj
}  // namespace ImFusion
