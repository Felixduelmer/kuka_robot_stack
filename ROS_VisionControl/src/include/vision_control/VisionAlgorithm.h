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

/*******************************************************************
* OPENCV INCLUDES
*******************************************************************/
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


/************************************************************************
 *                           ROS
 * **********************************************************************/
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>

#include <vision_control/VisionStreamOptimizer.h>

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
namespace vision_control {

class PluginAlgorithm : public QObject, public ImFusion::Algorithm {
  Q_OBJECT
public:
  PluginAlgorithm();
  ~PluginAlgorithm();



  void compute();

  void doSomething();

  //! Methods implementing the Configurable interface.
  void configure(const Properties* p);
  void configuration(Properties* p) const;

  //JZL
  /**
   * @brief Connect to the robot.
   * @param [in] probe_name - the name of the ultrasound probe being used. The respective calibration will be loaded
   * from from IFLUSCalibration file.
   */
  void connect(const std::string& probe_name);
  void disconnect();
  inline bool isRobotConnected() { return is_robot_connected_; }

  /**
   * @brief Add the tracking stream generated by thi algorithm to the provided DataModel.
   * @param [in] data_model
   */
  void addStreamToDataModel(DataModel* data_model) {
    data_model->add(tracking_stream_, "Robot Tracking");
    owning_stream_ = false;
  }

  static bool createCompatible(const DataList& data, Algorithm** a = nullptr);

  //JZL
  /// Motions.
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


  // no need
  ////////////////////OrienAdj Tasks
  /**
   * @brief robot would keep in current  position and change the orientation like fan
   * @param [in] vecAxis - rotate around axis
   * @param [in] currentPose - current pose of robot
   * @param [in] dUpLimitedAngle (unit is degree) - the up limited angle relative to current angle (about special axis)
   * @param [in] dDownLimitedAngle (unit is degree) - the up limited angle relative to current angle (about special axis)
   */
  void fanShapMotion(double dUpLimitedAngle, double dStepAngle, int nRoationAxis, int nFanMotionType = FAN_MOTION_STYPE_CONTINOUS);

  /**
   * @brief robot would keep in current  position and change the orientation like fan
   * @param [in] vecAxis - rotate around this axis
   * @param [in] dRotationRadius (unit is mm) - the up limited angle relative to current angle (about special axis)
   * @param [in] dHeight (unit is mm) - the radius is mearsured in sepcialed height
   */
  void coneShapMotion(Eigen::Vector3d vecAxis, double dRotationRadius, double dHeight);

  //continuous mode and step by step mode
  void calculateEndPoints(double fOffsetAngle, int nRotationAxis = ROTATION_X);   //unit is degree
  void calculateEndPointsTCP(double fOffsetAngle, int nRotationAxis = ROTATION_X);   //around TCP
  void calculateEndPointsStep(double fOffsetAngle, double dStep = 2.0, int nRotationAxis = ROTATION_X);   //unit is degree
  void calculateEndPointsStepTCP(double fOffsetAngle, double dStep = 2.0, int nRotationAxis = ROTATION_X);   //around TCP

  void RotateAroundTCP(double fOffsetAngle, int nRotationAxis);

  Eigen::Vector3d calculateReleativeRatationAngle(Eigen::Matrix3d& initialFrame, Eigen::Matrix3d& currentFrame);  //used to calculate eular angle relate to pose base

  void onGotoPose(const  Eigen::Matrix4d& pose, bool callback = true);
  void onStepGotoPose(const  Eigen::Matrix4d& pose, bool callback = true);

  int onGetStepFanMotionIteration();
  void stopStepFanMotion();

  void onInitROS();



  //used for saving the two end points
  Eigen::Matrix4d m_initialPose{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d m_finalPose{Eigen::Matrix4d::Identity()};
  QVector <Eigen::Matrix4d> m_vecPose;
  Eigen::Matrix4d m_rotationMatrix{Eigen::Matrix4d::Identity()};

  Eigen::Matrix4d m_initialFrameFan{Eigen::Matrix4d::Identity()};   //pose when we start fan searching

  //! The current pose of the center of the image, according to the loaded tool from the calibration file.
  geometry_msgs::PoseStamped current_image_center_pose_{};

  //vision control
  void setMoveStatus(int poseStatus);
  void setCurrentScanPointsIteration(int Iteration);

  void stopStepMotion();
  void VisionMovementFinishCallback();

  void onVisionStepGotoPose(const Eigen::Quaterniond& qPose, const Eigen::Vector3d& translation, bool callback = true);
  int getCurrentScanPointIndex();
  Eigen::Matrix4f getHomoTransMatrix(const std::vector<Eigen::Vector3f> &srcPoints, const std::vector<Eigen::Vector3f> &dstPoints);


  // the poses that will be used in the ultrasound swep
  std::vector<Eigen::VectorXd> m_scanPointPoses;
  std::vector<Eigen::VectorXd> m_updatedScanPointPoses;
  std::vector<Eigen::VectorXd> updateScanPointPoses(const std::vector<Eigen::VectorXd> &prePoses, const Eigen::Matrix4f &tranformation);
  int m_scanPointsNumber{0};
  int m_MoveStatus{-1};  //used to entere callback function for vision control

  // movement tracking
  bool restart_robot_pressed = false;
  bool m_resetting_position = false; // this flag is true when the robot is in the break point, and here the confidence map
  // does not need to computed.
  int m_break_point{0};
  bool m_motion_back_to_normal = false; // this flag is true, when the robot is recovered from the break point and
  // go back to the normal, if this is true, then the recording of US sweep starts again
  StreamOptimizer * m_usListener;
  void transform_trajectory();
  void write_txt_file(std::string file_name, unsigned int break_point = 0, bool f_half = false);
  void write_transformation(Eigen::Matrix4f transformation);
  inline void set_us_stream(ImageStream * stream) { usStream = stream; };
  inline ImageStream* get_us_stream(){ return usStream; };
  void start_streaming();
  void stop_streaming();
  float optimize_curr_probe_pose(MemImage* memImage);
  inline cv::Mat getConfimapResult() { return m_cvMatConfiMapUChar; }
  inline bool getRotationDirection() { return m_rotation_direction; }

  /**
   * @brief start_recording_us_stream
   */
  void start_recording_us_stream();

  /**
   * @brief set_new_stream
   * @param us_stream
   * @param robot_stream
   */
  void set_new_stream(Data* us_stream, Data* robot_stream);

  /**
   * @brief stop_recording_us_stream
   */
  void stop_recording_us_stream();

  /**
   * @brief unet_segmentation Sends the US image received from cephasonics to neural network part.
   * @param us_image
   */
  void unet_segmentation(cv::Mat us_image);

  /**
   * @brief find_closest_centroid given all the contours in the image along with the ground truth centroid
   * the algorithm finds the closest contour centroid to the ground truth
   * @param contours vector of contours, each contour consists of vector of points
   * @param gt_centroid a point which shows the gt centroid
   * @return
   */
  cv::Point find_closest_centroid(const std::vector<std::vector<cv::Point>> &contours, const cv::Point & gt_centroid);

  // callbacks
  /**
   * @brief receive_us_results a callback function, subscribes to segmentation results topic and gets the
   * segmented US images
   * @param img_msg the segmented image
   */
  void receive_us_results(sensor_msgs::ImageConstPtr img_msg);
  /**
   * @brief check_movement a callback which listens to the messages come from the movement detector,
   * if movement occured, then the break point is set
   * @param movement_occured true or false
   */
  void check_movement(const std_msgs::BoolPtr movement_occured);
  std::vector<cv::Mat> m_segmented_images; // a vector to collect all the segmented US images.
  unsigned int m_transformation_count{0}; // used to give identity number for different transformations and sweeps, occured during
  // US examination
  unsigned int img_idx{1}; // used for saving the images with different names, for testing purposes




  //zcy
  void onRoutePlanning();
  cv::Mat onBoneExtraction(cv::Mat img);
  void onSurfaceExtraction(cv::Mat img_bone, int i);
  void onCoordinateTransformation();


  //Felix
  void addPointConfiguration(const geometry_msgs::PoseStamped& pose) {   manual_traj_points_.push_back  (pose); }
  void deletePointConfigurations() {manual_traj_points_.clear();}
  void executeTrajectory();
  void onMoveToNewPoint();
  void FinishedMoveToNewPointCallback();




signals:
  void somethingHappened();

  //JZL
  void poseChanged();
  void wrenchChanged();
  void robotConnected();
  void robotDisconnected();

private:
  //JZL
  void poseCallback(const iiwa_msgs::CartesianPose &pose);
  void wrenchCallback(const iiwa_msgs::CartesianWrench& wench);
  //JZL_from roboticSpineAlgorithmController
  void movementFinishCallback();
  void stepMovementFinishCallback();

  /**
   * @brief calculate_confidance_map calculates the confidence map using the image from US stream
   */
  void calculate_confidance_map();
  /**
   * @brief calculate_rotation_angle using the calculated confidence map, the rotation angle and direction is calculated
   * @return the rotation angle in degrees
   */
  float calculate_rotation_angle();

  /**
   * @brief reset_trajectory_using_confimap_result given the rotation angle, the trajectory is getting updated.
   * @param rotation_angle in degrees
   */
  void reset_trajectory_using_confimap_result(float rotation_angle);

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
   * @brief getDayAndTime used to create a string including day and time, to be used in
   * naming of sweeps
   * @return string
   */
  std::string getDayAndTime();

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

  // movement detection

  ConfidentMapConvex* m_confiMapConvex;
  SharedImageSet* m_sharedImageSetCar;
  DataList* m_dataListCar;
  DataList* m_dataListConfiMap;
  cv::Mat m_cvMatConfiMapUChar; // the confidence map is saved here after calculation

  // the pixel height and width, this is calculated by 55mm depth and 37.5 mm width image
  float m_pixel_height = 0.076f; // in mm
  float m_pixel_width = 0.26f; // in mm

  bool m_rotation_direction = true; // the direction where the robot need to move after the confidence map is calculated
  // if true then it mean to the positive dir around the rotation axis
  USSweepRecorderAlgorithm* m_multiUSSweepRecorder;


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

  //! The current pose of the robot tip, according to the used tool defined in ROS.
  geometry_msgs::PoseStamped current_tip_pose_{};
  //! The current pose of the robot tip, according to the used tool defined in ROS.
  geometry_msgs::Wrench current_tip_wrench_{};
  geometry_msgs::PoseStamped home_pose_{};
  geometry_msgs::PoseStamped out_pose_{};
  std::mutex pose_mutex_{};
  std::mutex wrench_mutex_{};

  //Felix
  std::vector<geometry_msgs::PoseStamped> manual_traj_points_{};
  int m_nTrajPoints{0};
  int n_poses{0};


  std::string something_{"something"};

  int m_fanStatus{-1};
  int m_nIteration{0};   //used in visionStepCallback, shows the destination point from the current position


  // subscribers and publishers
  ros::Subscriber m_check_movement_sub;
  ros::Publisher m_calc_transformation_pub;
  ros::Publisher m_unet_segmentation_pub;
  ros::Subscriber m_unet_segmentation_sub;
  ImageStream* usStream;
  bool m_ros_initialized{false};

};
}  // namespace OrienAdj
}  // namespace ImFusion
