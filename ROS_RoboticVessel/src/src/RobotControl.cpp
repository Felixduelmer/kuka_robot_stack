#include "robotic_vessel/RobotControl.h"

#include <ImFusion/Base/Log.h>
#include <ImFusion/Stream/OpenIGTLinkConnection.h>
#include <ImFusion/Stream/TrackingStreamData.h>
#include <QDebug>
#include <QVector3D>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <geometry_msgs/Twist.h>
#include <iiwa_msgs/SetSmartServoLinSpeedLimits.h>

using namespace std;
using namespace cv;
namespace ImFusion {
    namespace ROS_RoboticVessel {


        RobotControl::RobotControl() {
            onInitROS();

            probe_rotation_.block<4, 4>(0, 0) << 0, 0, -1, 0,
                    -1, 0, 0, 15.6,
                    0, 1, 0, 0,
                    0, 0, 0, 1;
            connect(this, &RobotControl::poseChanged, this, &RobotControl::customPoseCallback);
            timer = new QTimer(this);
            timer->setSingleShot(true);
            timer->setInterval(2000);
            connect(timer, &QTimer::timeout, this, &RobotControl::performFanMotion);
        }

        RobotControl::~RobotControl() { disconnect(); }

        //execute a movement along defined points
        void RobotControl::executeTrajectory() {
            LOG_INFO("Start to execute Trajectory");
            motionState = TRAJECTORY_MOTION;
            std::transform(manualTrajPoints.begin(), manualTrajPoints.end(), std::back_inserter(qManualTrajPoints),
                           [](Eigen::Matrix4d mat) { return Eigen::Quaterniond{mat.block<3, 3>(0, 0)}; });
            start_pose = getCurrentRobotPose();
            applyDesiredForce(iiwa_msgs::DOF::Z, 1, 200);
            LOG_INFO("Number of points to go to: " + std::to_string(manualTrajPoints.size()));
            onMoveToNewPoint();
        }

        //execute a movement along defined points
        void RobotControl::reExecuteTrajectory() {
            LOG_INFO("Start to reExecute Trajectory without doppler movement");
            blockFanMotion = true;
            motionState = TRAJECTORY_MOTION;
            //return the block
            std::for_each(manualTrajPoints.begin(), manualTrajPoints.end(),
                          [](Eigen::Matrix4d mat) {
                              return mat.block<3, 3>(0, 0) = Eigen::Quaterniond{
                                      mat.block<3, 3>(0, 0)}.toRotationMatrix();
                          });
            start_pose = getCurrentRobotPose();
            LOG_INFO("Number of points to go to: " + std::to_string(manualTrajPoints.size()));
            onMoveToNewPoint();
        }


//Callback to continue trajectory
        void RobotControl::FinishedMoveToNewPointCallback() {

            LOG_INFO("FinishedMoveToNewPointCallback");
            if (currentTargetPoint < manualTrajPoints.size()) {
                if (currentTargetPoint == 1) {
                    QTimer::singleShot(500, this, &RobotControl::reachedStartingPoint);
                    LOG_INFO("Applying Force!");
                }
//                applyDesiredForce(iiwa_msgs::DOF::Z, 3, 200);
                onMoveToNewPoint();
            } else {
                if (currentTargetPoint == manualTrajPoints.size()) {
                    LOG_INFO("reached final point");
                    motionState = NO_TRACKED_MOTION;
                    currentTargetPoint = 0;
                    applyPositionControlMode();
                    executeCartesianCommand(start_pose.pose, true);
                    LOG_INFO("Going to home position");
                    emit reachedEndPoint();
                }
            }
        }

//execute a movement along defined points
        void RobotControl::onMoveToNewPoint() {
            LOG_INFO("onMoveToNewPoint");
            LOG_INFO("Current destination point : " + std::to_string(currentTargetPoint));
            executeCartesianCommand(manualTrajPoints[currentTargetPoint], true);
        }


//initialize the ros related node.
        void RobotControl::onInitROS() {
            std::map<std::string, std::string> emptyArgs;
            if (!ros::isInitialized()) { ros::init(emptyArgs, "iiwaRos"); }
            ros_spinner_ = std::make_unique<ros::AsyncSpinner>(1);
            ros_spinner_->start();

            // create a ROS handle to connect the neural network and the movement tracking parts
            ros::NodeHandle nh;

            this->m_ros_initialized = true;
            LOG_INFO(this->m_ros_initialized);
        }

        void RobotControl::connectRobot(const std::string &probe_name) {
            //innitialize Ros and iiwaRos object
            std::map<std::string, std::string> emptyArgs;
            if (!ros::isInitialized()) { ros::init(emptyArgs, "iiwaRos"); }
            ros_spinner_ = std::make_unique<ros::AsyncSpinner>(1);
            ros_spinner_->start();

            ros::NodeHandle node_handle;

            pose_state_.init("iiwa", std::bind(&RobotControl::poseCallback, this, std::placeholders::_1));
            wrench_state_.init("iiwa", std::bind(&RobotControl::wrenchCallback, this, std::placeholders::_1));
            torque_state_.init("iiwa");

            pose_command_.init("iiwa");
            linear_pose_command_.init("iiwa");

            control_mode_.init("iiwa");

            ros::ServiceClient client = node_handle.serviceClient<iiwa_msgs::SetSmartServoLinSpeedLimits>(
                    "/iiwa/configuration/setSmartServoLinLimits");
            iiwa_msgs::SetSmartServoLinSpeedLimits srv;
            srv.request.max_cartesian_velocity.angular.x = 0.05;
            srv.request.max_cartesian_velocity.angular.y = 0.05;
            srv.request.max_cartesian_velocity.angular.z = 0.05;
            srv.request.max_cartesian_velocity.linear.x = 0.005;
            srv.request.max_cartesian_velocity.linear.y = 0.005;
            srv.request.max_cartesian_velocity.linear.z = 0.005;
            if (client.call(srv)) {
                ROS_INFO("Setting Smart Servo Speed success");
            } else {
                ROS_ERROR("Failed to call service SetSmartServoLinSpeedLimits");
            }

            node_handle.setParam("/iiwa/toolName", "linear");

            OpenIGTLinkConnection dummy_connection("Service robot connection");
            tracking_stream_ = new OpenIGTLinkTrackingStream(dummy_connection, "Robot");
            tracking_stream_->open();
            is_robot_connected_ = true;
            loadCalibrationFromFile("IFLUSCalibration.config", probe_name);

            emit robotConnected();
        }

        void RobotControl::disconnect() {
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

        void RobotControl::poseCallback(const iiwa_msgs::CartesianPose &pose) {
            if (is_robot_connected_) {
                std::lock_guard<std::mutex> lock{pose_mutex_};
                current_tip_pose_ = pose.poseStamped;   //this would be updated after connect automatically

                //! This object is disposed by the destructor of TrackingStreamData, or at least so it says its documentation.
                //! If you try to handle its lifetime -> CRASH, so leave it as it is.
                auto *tracking_instrument = new TrackingInstrument();
                tracking_instrument->active = true;
                tracking_instrument->name = "US";
                tracking_instrument->quality = 1;
//        auto image_center_pose = poseToEigenMat4(pose.poseStamped.pose, 1000) * probe_rotation_ * ultrasound_calibration_;

                auto image_center_pose = poseToEigenMat4(pose.poseStamped.pose, 1000) * probe_rotation_;

                tracking_instrument->matrix = image_center_pose;

                current_image_center_pose_.pose = eigenMat4ToPose(image_center_pose);
                current_image_center_pose_.header = pose.poseStamped.header;

                std::vector<TrackingInstrument *> instrument_vector{tracking_instrument};
                TrackingStreamData datas(tracking_stream_, instrument_vector);

                std::chrono::system_clock::time_point arrivalTime = std::chrono::high_resolution_clock::now();
                datas.setTimestampArrival(arrivalTime);

                tracking_stream_->sendStreamData(datas);

                emit poseChanged();
            }
        }

        void RobotControl::wrenchCallback(const iiwa_msgs::CartesianWrench &wench) {
            if (is_robot_connected_) {
                std::lock_guard<std::mutex> lock{wrench_mutex_};
                current_tip_wrench_ = wench.wrench;   //this would be updated after connect automatically

                emit wrenchChanged();
            }
        }

        float RobotControl::calculate_weight(size_t curr_pos, size_t pos, size_t step_size) {
            auto last_point_pos = (curr_pos + step_size) - (pos - curr_pos) + 1;

            Eigen::Vector3d curr_position(m_scanPointPoses[curr_pos](0), m_scanPointPoses[curr_pos](1),
                                          m_scanPointPoses[curr_pos](2));
            Eigen::Vector3d curr_dest(m_scanPointPoses[last_point_pos](0), m_scanPointPoses[last_point_pos](1),
                                      m_scanPointPoses[last_point_pos](2));
            auto over_all_distance = 0.0f;
            auto partial_distance = std::sqrt(
                    std::pow(curr_dest.x() - curr_position.x(), 2) + std::pow(curr_dest.y() - curr_position.y(), 2) +
                    std::pow(curr_dest.z() - curr_position.z(), 2));

            for (size_t i = curr_pos + 1; i <= (curr_pos + step_size); i++) {

                Eigen::Vector3d c_temp(m_scanPointPoses[i](0), m_scanPointPoses[i](1), m_scanPointPoses[i](2));

                over_all_distance +=
                        std::pow(curr_position.x() - c_temp.x(), 2) + std::pow(curr_position.y() - c_temp.y(), 2) +
                        std::pow(curr_position.z() - c_temp.z(), 2);
            }

            std::cout << std::pow(partial_distance, 2) / over_all_distance << std::endl;
            return std::pow(partial_distance, 2) / over_all_distance;
        }

        void RobotControl::executeCartesianCommand(const geometry_msgs::Pose &pose, bool linear,
                                                   const std::function<void()> &callback) {
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

        void RobotControl::executeCartesianCommand(const Eigen::Matrix4d &matrix, bool linear,
                                                   const std::function<void()> &callback) {
            executeCartesianCommand(eigenMat4ToPose(matrix), linear, callback);
        }

        void RobotControl::executeCartesianCommand(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, bool linear,
                                                   std::function<void()> callback) {
            Eigen::Matrix4d matrix{Eigen::Matrix4d::Identity()};
            matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
            matrix.block<3, 1>(0, 3) = t;
            executeCartesianCommand(eigenMat4ToPose(matrix), linear, callback);
        }

// THIS IS IN METERS
        geometry_msgs::PoseStamped RobotControl::getCurrentRobotPose() {
            assert(is_robot_connected_ == true &&
                   "The robot has to be connected before receiving any state. Call the 'connect()' method.");
            std::lock_guard<std::mutex> lock{pose_mutex_};
            return current_tip_pose_;
        }

// THIS IS IN METERS
        Eigen::Matrix4d
        RobotControl::getCurrentRobotTransformMatrix(bool in_millimeters) {                 //??????????????????????????????????????????
            assert(is_robot_connected_ == true &&
                   "The robot has to be connected before receiving any state. Call the 'connect()' method.");
            std::lock_guard<std::mutex> lock{pose_mutex_};
            double scaling_factor{1};
            if (in_millimeters) { scaling_factor = 1000; }
            return poseToEigenMat4(current_tip_pose_.pose, scaling_factor);
        }

        geometry_msgs::PoseStamped RobotControl::getCurrentImageCenterPose() {
            assert(is_robot_connected_ == true &&
                   "The robot has to be connected before receiving any state. Call the 'connect()' method.");
            std::lock_guard<std::mutex> lock{pose_mutex_};
            return current_image_center_pose_;
        }

//wrench
        geometry_msgs::Wrench RobotControl::getCurrentRobotWrench() {
            assert(is_robot_connected_ == true &&
                   "The robot has to be connected before receiving any state. Call the 'connect()' method.");
            std::lock_guard<std::mutex> lock{wrench_mutex_};  //std::lock_guard??????????????????????????????????????????????????????
            return current_tip_wrench_;
        }


        geometry_msgs::Pose RobotControl::eigenMat4ToPose(Eigen::Matrix4d matrix, double scaling_factor) {
            Eigen::Quaterniond q{matrix.block<3, 3>(0, 0)};
            Eigen::Vector3d t{matrix.block<3, 1>(0, 3)};  // block matrix:from (0,3) select a (3,1) matrix

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

        Eigen::Matrix4d
        RobotControl::poseToEigenMat4(const geometry_msgs::Pose &pose, double scaling_factor) {         //????????????pos??????
            Eigen::Matrix4d matrix{
                    Eigen::Matrix4d::Identity()};                                                          //?????????4x4??????
            matrix.block<3, 3>(0, 0) =       //Block of size (p,q), starting at (i,j) //?????????3*3
                    Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z)    //????????? (w,x,y,z) constant scalar
                            .toRotationMatrix();
            matrix.block<4, 1>(0, 3) = Eigen::Vector4d(pose.position.x * scaling_factor,
                                                       pose.position.y * scaling_factor,//
                                                       pose.position.z * scaling_factor, 1);
            return matrix;
        }

        void RobotControl::loadCalibrationFromFile(const std::string &file_name, const std::string &desired_probe) {
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
                                    << "Calibration found at " << config_file_path << " for probe " << desired_probe
                                    << ": "
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


        void RobotControl::update_pose(size_t pos, double fOffsetAngle, int nRotationAxis) {
            auto new_pose = calculate_pose(pos, fOffsetAngle, nRotationAxis);
            Eigen::Quaterniond new_quaternion(new_pose.block<3, 3>(0, 0));
            m_scanPointPoses[pos](6) = new_quaternion.w();
            m_scanPointPoses[pos](3) = new_quaternion.x();
            m_scanPointPoses[pos](4) = new_quaternion.y();
            m_scanPointPoses[pos](5) = new_quaternion.z();
        }

        Eigen::Matrix4d RobotControl::calculate_pose(size_t pos, double fOffsetAngle, int nRotationAxis) {
            Eigen::Quaterniond quaPose(m_scanPointPoses[pos](6), m_scanPointPoses[pos](3), m_scanPointPoses[pos](4),
                                       m_scanPointPoses[pos](5));
            Eigen::Vector3d vecPosition(m_scanPointPoses[pos](0), m_scanPointPoses[pos](1), m_scanPointPoses[pos](2));

            double x = m_scanPointPoses[pos](0);
            double y = m_scanPointPoses[pos](1);
            double z = m_scanPointPoses[pos](2);

            double qw = m_scanPointPoses[pos](6);
            double qx = m_scanPointPoses[pos](3);
            double qy = m_scanPointPoses[pos](4);
            double qz = m_scanPointPoses[pos](5);

            Eigen::Matrix4d curr_pos;
            curr_pos << 2 * (std::pow(qw, 2) + std::pow(qx, 2)) - 1, 2 * (qx * qy - qw * qz), 2 *
                                                                                              (qx * qz + qw * qy), x,
                    2 * (qx * qy + qw * qz), 2 * (std::pow(qw, 2) + std::pow(qy, 2)) - 1, 2 * (qy * qz - qw * qx), y,
                    2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 2 * (std::pow(qw, 2) + std::pow(qz, 2)) - 1, z,
                    0, 0, 0, 1;
            Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
            if (ROTATION_X == nRotationAxis) {
                offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI;   //unit rad
                //LOG_INFO(fOffsetAngle);
            } else if (ROTATION_Y == nRotationAxis) {
                offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI;   //unit rad
            } else {
                LOG_INFO("the rotation is wrong");
            }

            Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
                                                Eigen::AngleAxisd(offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
                                                Eigen::AngleAxisd(offsetAngleRad[2], Eigen::Vector3d::UnitZ());

            Eigen::Matrix4d rotation{Eigen::Matrix4d::Identity()};
            rotation.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
            rotation.block<3, 1>(0, 3) << 0, 0, 0;

            curr_pos = curr_pos * rotation;
            curr_pos.block<3, 1>(0, 3) = vecPosition;

            return curr_pos;
        }

        void RobotControl::performFanMotion() {
            blockFanMotion = true;
            motionState = FAN_MOTION;
            applyPositionControlMode();
            Eigen::Quaterniond qCurrent{getCurrentRobotTransformMatrix().block<3, 3>(0, 0)};
            auto distance = qCurrent.angularDistance(qManualTrajPoints[currentTargetPoint]);
            std::cout << "Distance: " << distance << std::endl;
            assert(distance < 40.0 * (M_PI / 180.0));
            if (fanIter == 0) {
//                applyPositionControlMode();
                fanTrajPoints.clear();
                auto distance1 = calculateDistance(5, ROTATION_Y);
                auto distance2 = calculateDistance(-5, ROTATION_Y);
                std::cout << "Distance 1 (rotating +5): " << distance1 << ", Distance 2 (rotating -5): " << distance2
                          << std::endl;
                if (distance1 < distance2) {
                    offsetArr = {5, 5, -10, -5, -5};
                } else {
                    offsetArr = {-5, -5, 10, 5, 5};
                }
            } else {
                if (distance > 20.0 * (M_PI / 180.0)) {
                    std::cout << "Crossed threshold, going back!" << std::endl;
                    auto lastSign = offsetArr[fanIter - 1] > 0 ? 1 : -1;
                    for (int i = int(fanIter); i < offsetArr.size(); i++) {
                        offsetArr[i] = -lastSign * 5;
                    }
                }

            }
            printf("Offset Array size: %i, fanIter: %i, nextValue: %f\n", offsetArr.size(), fanIter,
                   offsetArr[fanIter]);
            //check if doppler has been found
            if (!doppler_found && fanIter < offsetArr.size()) {
                RotateAroundTCP(offsetArr[fanIter], ROTATION_Y, true);
            } else {
                Eigen::Matrix4d currentPose = getCurrentRobotTransformMatrix();
                if (doppler_found) {
                    LOG_INFO("We found the doppler --> continuing");
                    //override the current orientation with the newly found
                    manualTrajPoints[currentTargetPoint].block<3, 3>(0, 0) = currentPose.block<3, 3>(0, 0);
                    motionState = TRAJECTORY_MOTION;
                    applyDesiredForce(iiwa_msgs::DOF::Z, 1, 200);
                    onMoveToNewPoint();
                    fanIter = 0;
                    std::cout << "START removed Flag" << std::endl;
                    QTimer::singleShot(3000, this, &RobotControl::removeFlag);
                } else {
                    LOG_INFO("Rotation didn't conclude in a desired result --> continuing");
                    RotateAroundTCP(offsetArr[2] > 0 ? -10 : 10, ROTATION_Y, true);
                }
            }
        }

        double RobotControl::calculateDistance(double fOffsetAngle, int nRotationAxis) {
            auto robot_pose = getCurrentRobotTransformMatrix(true);
            Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
            Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
            if (ROTATION_X == nRotationAxis) {
                offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI;   //unit rad
                LOG_INFO(fOffsetAngle);
            } else if (ROTATION_Y == nRotationAxis) {
                offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI;   //unit rad
                LOG_INFO(fOffsetAngle);
            } else {
                LOG_INFO("the rotation is wrong");
            }
            //obtain the transformation between the target about TCP frame
            Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
                                                Eigen::AngleAxisd(offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
                                                Eigen::AngleAxisd(offsetAngleRad[2], Eigen::Vector3d::UnitZ());
            Eigen::Matrix4d PoseRotateTCP{Eigen::Matrix4d::Identity()};

            PoseRotateTCP.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
            PoseRotateTCP.block<3, 1>(0, 3) << 0, 0, 0;

            PoseRotateTCP = robot_pose * PoseRotateTCP;
            PoseRotateTCP.block<3, 1>(0, 3) = translation / 1000.0;
            Eigen::Quaterniond qCurrent{getCurrentRobotTransformMatrix().block<3, 3>(0, 0)};
            return qCurrent.angularDistance(Eigen::Quaterniond{PoseRotateTCP.block<3, 3>(0, 0)});
        }

        void RobotControl::RotateAroundTCP(double fOffsetAngle, int nRotationAxis, bool callBack) {
            LOG_INFO("Rotating around axis to perform a fan motion");
            auto robot_pose = getCurrentRobotTransformMatrix(true);
            Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
            Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
            if (ROTATION_X == nRotationAxis) {
                offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI;   //unit rad
                LOG_INFO(fOffsetAngle);
            } else if (ROTATION_Y == nRotationAxis) {
                offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI;   //unit rad
                LOG_INFO(fOffsetAngle);
            } else {
                LOG_INFO("the rotation is wrong");
            }
            //obtain the transformation between the target about TCP frame
            Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
                                                Eigen::AngleAxisd(offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
                                                Eigen::AngleAxisd(offsetAngleRad[2], Eigen::Vector3d::UnitZ());
            Eigen::Matrix4d PoseRotateTCP{Eigen::Matrix4d::Identity()};

            PoseRotateTCP.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
            PoseRotateTCP.block<3, 1>(0, 3) << 0, 0, 0;

            PoseRotateTCP = robot_pose * PoseRotateTCP;
            PoseRotateTCP.block<3, 1>(0, 3) = translation / 1000.0;

            if (callBack) {
                fanTrajPoints.push_back(PoseRotateTCP);
                executeCartesianCommand(PoseRotateTCP, true);
            } else {
                executeCartesianCommand(eigenMat4ToPose(PoseRotateTCP), true);
                sleep(2);
            }

        }

        void RobotControl::lostDopplerSignal() {
            if (motionState == TRAJECTORY_MOTION && currentTargetPoint != 0 && !blockFanMotion) {
                LOG_INFO("Lost Doppler Signal");
                doppler_found = false;
                performFanMotion();
            }
        }

        void RobotControl::foundDopplerSignal() {
            if (motionState == FAN_MOTION && !doppler_found) {
                LOG_INFO("Found Doppler Signal");
                doppler_found = true;
            }
        }

        void RobotControl::customPoseCallback() {
            if (motionState > NO_TRACKED_MOTION) {
                //500 hz
                customPoseCallbackIterator++;
                if (customPoseCallbackIterator % 10 == 0) {
                    if (motionState == FAN_MOTION) {
                        if (fanTrajPoints[fanIter].isApprox(
                                getCurrentRobotTransformMatrix(), 1e-2)) {
                            if (fanIter >= offsetArr.size()) {
                                motionState = TRAJECTORY_MOTION;
                                applyDesiredForce(iiwa_msgs::DOF::Z, 1, 200);
                                onMoveToNewPoint();
                                fanIter = 0;
                                std::cout << "START removed Flag" << std::endl;
                                QTimer::singleShot(3000, this, &RobotControl::removeFlag);
                            } else {
                                std::cout << "reached fan point continue to do next point" << std::endl;
                                ++fanIter;
                                if (!timer->isActive()) {
                                    timer->start();
                                }
                            }
                        }
                    } else if (motionState == TRAJECTORY_MOTION) {
                        if (manualTrajPoints[currentTargetPoint].isApprox(
                                getCurrentRobotTransformMatrix(), 1e-2)) {
                            LOG_INFO("We have reached the next point");
                            currentTargetPoint++;
                            FinishedMoveToNewPointCallback();
                        }
                    }
                }
                    //If we get stuck somewhere try to reach that point again
                else if (customPoseCallbackIterator >= 200) {
                    auto currPose = getCurrentRobotTransformMatrix();
                    if (currPose.isApprox(lastPose, 1e-5)) {
                        std::cout << "we are stuck trying to get to current end position" << std::endl;
                        applyPositionControlMode();
                        if (motionState == FAN_MOTION) {
                            executeCartesianCommand(fanTrajPoints[fanIter], true);
                        } else if (motionState == TRAJECTORY_MOTION) {
                            executeCartesianCommand(manualTrajPoints[currentTargetPoint], true);
                        }
                    }
                    lastPose = currPose;
                    customPoseCallbackIterator = 0;
                }
            }
        }

        void RobotControl::removeFlag() {
            blockFanMotion = false;
            std::cout << "removed Flag" << std::endl;
        }


        double angle(double x1, double y1, double x2, double y2) {
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
