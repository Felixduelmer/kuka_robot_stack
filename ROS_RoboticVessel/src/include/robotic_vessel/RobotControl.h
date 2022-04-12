#pragma once

#include <ImFusion/Base/Algorithm.h>
#include <ImFusion/Base/DataModel.h>
#include <ImFusion/Stream/OpenIGTLinkTrackingStream.h>
#include <ImFusion/Stream/OpenIGTLinkImageStream.h>

#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/cartesian_wrench.hpp>
#include <iiwa_ros/state/joint_torque.hpp>

#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose_linear.hpp>
#include <iiwa_ros/command/generic_command.hpp>

#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>

#include <iiwa_msgs/DOF.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <Eigen/Dense>
#include <QObject>
#include <string>
#include <vector>
#include <QVector>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>


/************************************************************************
 *                           ROS
 * **********************************************************************/
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>

#define PI 3.14159265358979323846
#define FP 0.051264
#define FP1 0.0375

const int ON_FINAL_POSE = 1;
const int ON_BACK_INITIAL_POSE = 2;


const int FAN_MOTION_STYPE_CONTINOUS = 0;
const int FAN_MOTION_STYPE_STEP = 1;

const int ROTATION_X = 0;
const int ROTATION_Y = 1;

//vision control
const int ON_CURRENT_POSE = -1;
const int ON_INITIAL_POSE = 0;
const double INITHEIGHT = 0.0;   //mm

namespace ImFusion {
namespace ROS_RoboticVessel {

class RobotControl : public QObject{
    Q_OBJECT
public:
  RobotControl();
  ~RobotControl();

  /**
   * @brief Connect to the robot.
   * @param [in] probe_name - the name of the ultrasound probe being used. The respective calibration will be loaded
   * from from IFLUSCalibration file.
   */
  void connect(const std::string& probe_name);
  void disconnect();
  inline bool isRobotConnected() { return is_robot_connected_; }

  /**
   * @brief Make the robot execute a Cartesian motion to the given target pose.
   * @param [in] pose - Target pose for the motion.
   * @param [in] linear - If true, a linear motion will be executed.
   * @param [in] callback - Optional callback function to be called when the robot reaches the given target pose.
   */
  void executeCartesianCommand(const geometry_msgs::Pose& pose, bool linear,
                               const std::function<void()>& callback = nullptr);

  /**
   * @brief Make the robot execute a Cartesian motion to the given target pose.
   * @param [in] matrix - Target pose for the motion.
   * @param [in] linear - If true, a linear motion will be executed.
   * @param [in] callback - Optional callback function to be called when the robot reaches the given target pose.
   */
  void executeCartesianCommand(const Eigen::Matrix4d& matrix, bool linear,
                               const std::function<void()>& callback = nullptr);


  /**
   * @brief Make the robot execute a Cartesian motion to the given target pose.
   * @param [in] q - Rotation component of the target pose for the robot motion.
   * @param [in] t - Translational component of the target pose for the robot motion.
   * @param [in] linear - If true, a linear motion will be executed.
   * @param [in] callback - Optional callback function to be called when the robot reaches the given target pose.
   */
  void executeCartesianCommand(const Eigen::Quaterniond& q, const Eigen::Vector3d& t, bool linear,
                               std::function<void()> callback = nullptr);

  /**
   * @brief Activate position mode on the robot.
   */
  void applyPositionControlMode() {
    control_mode_.setPositionControlMode();
  }

  /**
   * @brief Activate DesiredForce mode on the robot with the given values.
   * @param [in] dof - The Degee of Freedong the force should be applied to.
   * @param [in] force - The force to be applied [N]
   * @param [in] stiffness - the Cartesian stiffness to apply on the given direction.
   */
  void applyDesiredForce(int dof, double force, double stiffness) {
    control_mode_.setDesiredForceMode(dof, force, stiffness);
  }


  // no need
  /**
   * @brief Save the given robot pose as the 'Home' Pose.
   * @param [in] pose - the pose to become the new 'Home' pose.
   */
  void setRobotHomeConfiguration(const geometry_msgs::PoseStamped& pose) { home_pose_ = pose; }

  // no need
  /**
   * @brief Returns the saved "Home" pose for the robot.
   */
  geometry_msgs::PoseStamped getRobotHomeConfiguration() { return home_pose_; }

  // no need
  /**
   * @brief Save the given robot pose as the 'OutOfWorkspace' Pose.
   * @param [in] pose - the pose to become the new 'OutOfWorkspace' pose.
   */
  void setRobotOutConfiguration(const geometry_msgs::PoseStamped& pose) { out_pose_ = pose; }

  // no need
  /**
   * @brief Returns the saved "OutOfWorkspace" pose for the robot.
   */
  geometry_msgs::PoseStamped getRobotOutConfiguration() { return out_pose_; }

  // no need
  /**
   * @brief Returns the ultrasound temporal calibration.
   * It is loaded from the configuration file.
   */
  inline float getTemporalCalibration() { return temporal_calibration_; }


  // needed to get the current pose and restart the motion from
  /**
   * @brief Returns the latest Cartesian pose of the robot tool tip.
   *
   * The tool tip is selected according to the tool loaded using the ROS parameter.
   */
  geometry_msgs::PoseStamped getCurrentRobotPose();


  // no need
  /**
   * @brief Returns the latest Cartesian pose f the robot tool tip as an homogeneous matrix.
   *
   * The tool tip is selected according to the tool loaded using the ROS parameter.
   *
   * @param [in] in_millimeters - If true, the translation elements of the returned matrix will be in millimters.
   */
  Eigen::Matrix4d getCurrentRobotTransformMatrix(bool in_millimeters = false);

  // no need
  /**
   * @brief Returns the latest Cartesian pose f the robot tool tip as an homogeneous matrix.
   *
   * The tool tip is selected according to the tool loaded using the ROS parameter.
   *
   * @param [in] in_millimeters - If true, the translation elements of the returned matrix will be in millimters.
   */
  geometry_msgs::Wrench getCurrentRobotWrench();

  // no need
  /**
   * @brief Returns the latest Cartesian pose of the image center.
   *
   * This is computed according to the calibration file that has been loaded upon connection.
   * See loadCalibrationFromFile.
   */
  geometry_msgs::PoseStamped getCurrentImageCenterPose();

  // no need
  /**
   * @brief Load the Ultrasound calibration from the given file_name looking for the calibration of the given probe
   * name.
   * @param [in] file_name - The config file to load the calibration from. The folder containing this file is defined by
   * the CMake variable CONFIG_DIR, in the main CMake file of this plugin.
   * @param [in] probe - Name of the probe to load the calibration of. If this probe name is not found in the provided
   * calibration file, the ultrasound calibration will be omitted.
   */
  void loadCalibrationFromFile(const std::string& file_name, const std::string& probe);


  void onGotoPose(const  Eigen::Matrix4d& pose, bool callback = true);
  void onStepGotoPose(const  Eigen::Matrix4d& pose, bool callback = true);

  void onInitROS();



  //used for saving the two end points
  Eigen::Matrix4d m_initialPose{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d m_finalPose{Eigen::Matrix4d::Identity()};
  QVector <Eigen::Matrix4d> m_vecPose;
  Eigen::Matrix4d m_rotationMatrix{Eigen::Matrix4d::Identity()};

  Eigen::Matrix4d m_initialFrameFan{Eigen::Matrix4d::Identity()};   //pose when we start fan searching


signals:
  void poseChanged();
  void wrenchChanged();
  void robotConnected();
  void robotDisconnected();

private:
  void poseCallback(const iiwa_msgs::CartesianPose &pose);
  void wrenchCallback(const iiwa_msgs::CartesianWrench& wench);


  /**
   * @brief calculate_pose calculates the new pose, do the transformation given the angle and rotation axis.
   * @param pos index of a pose in m_scan_points
   * @param fOffsetAngle angle in degrees
   * @param nRotationAxis
   * @return
   */
  Eigen::Matrix4d calculate_pose(size_t pos, double fOffsetAngle, int nRotationAxis);

  /**
   * @brief update_pose a function which updates the given pose index
   * in the scan points ventor, using the given rotation angle and and rotation direction
   * @param pos index of a pose in m_scan_points
   * @param fOffsetAngle angle in degrees
   * @param nRotationAxis
   */
  void update_pose(size_t pos, double fOffsetAngle, int nRotationAxis);
  /**
   * @brief calculate_weight After a pose changed due to the confidence map result, then the coming step_size many points in
   * trajectory also need to be updated. However the effect of the change needs to decrease each point, this function
   * calculates the effect weight for each point
   * @param curr_pos
   * @param pos
   * @param step_size
   * @return weight, float
   */
  float calculate_weight(size_t curr_pos, size_t pos, size_t step_size=3);

  /**
   * @brief eigenMat4ToPose Creates geometry msg pose from eigen matrix 4x4
   * @param matrix
   * @param scaling_factor
   * @return
   */
  geometry_msgs::Pose eigenMat4ToPose(Eigen::Matrix4d matrix, double scaling_factor = 1);

  /**
   * @brief poseToEigenMat4 Creates eigen matrix 4x4 from geometry msg pose
   * @param pose
   * @param scaling_factor
   * @return
   */
  Eigen::Matrix4d poseToEigenMat4(const geometry_msgs::Pose& pose, double scaling_factor = 1);


  float prev_theta = 0.0f;
  geometry_msgs::Pose m_movement_occured_pose;
  Data* m_cepha_stream;
  Data* m_robot_stream;
  bool isStopped = false;

  // the pixel height and width, this is calculated by 55mm depth and 37.5 mm width image
  float m_pixel_height = 0.076f; // in mm
  float m_pixel_width = 0.26f; // in mm

  bool m_rotation_direction = true; // the direction where the robot need to move after the confidence map is calculated
  // if true then it mean to the positive dir around the rotation axis

  // the poses that will be used in the ultrasound swep
  std::vector<Eigen::VectorXd> m_scanPointPoses;

  iiwa_ros::state::CartesianPose pose_state_{};
  iiwa_ros::state::CartesianWrench wrench_state_{};
  iiwa_ros::state::JointTorque torque_state_{};

  iiwa_ros::command::CartesianPose pose_command_{};
  iiwa_ros::command::CartesianPoseLinear linear_pose_command_{};

  iiwa_ros::service::ControlModeService control_mode_{};
  iiwa_ros::service::TimeToDestinationService time_to_destination_{};

  std::unique_ptr<ros::AsyncSpinner> ros_spinner_{nullptr};
  OpenIGTLinkTrackingStream* tracking_stream_{nullptr};
  bool owning_stream_{true};
  bool is_robot_connected_{false};

  std::string configuration_dir{CONFIG_DIR};
  Eigen::Matrix4d probe_rotation_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d ultrasound_calibration_{Eigen::Matrix4d::Identity()};
  float temporal_calibration_{0};
  //"IFLConvex"

    //! The current pose of the center of the image, according to the loaded tool from the calibration file.
  geometry_msgs::PoseStamped current_image_center_pose_{};

  //! The current pose of the robot tip, according to the used tool defined in ROS.
  geometry_msgs::PoseStamped current_tip_pose_{};
  //! The current pose of the robot tip, according to the used tool defined in ROS.
  geometry_msgs::Wrench current_tip_wrench_{};
  geometry_msgs::PoseStamped home_pose_{};
  geometry_msgs::PoseStamped out_pose_{};
  std::mutex pose_mutex_{};
  std::mutex wrench_mutex_{};

  int m_fanStatus{-1};
  bool m_ros_initialized{false};

};
}  // namespace robotic_vessel
}  // namespace ImFusion
