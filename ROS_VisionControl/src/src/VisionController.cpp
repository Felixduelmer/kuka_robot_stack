#include "vision_control/VisionController.h"
#include "ui_controller.h"
#include <iiwa_msgs/DOF.h>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <cstring>
#include<sstream>
#include<fstream>
#include <unistd.h>


using namespace std;
using namespace cv;
namespace ImFusion {
namespace vision_control {

PluginController::PluginController(PluginAlgorithm* algorithm)
    : AlgorithmController(algorithm)
    , algorithm_{algorithm} {
  ui_ = std::make_shared<Ui_Controller>();
  ui_->setupUi(this);
  ui_->cmbRotationStep->setEnabled(false);   //disable the step value when start the plugin

  //TCP
  tcpServer = new QTcpServer(this);
  serverSocket = new QTcpSocket(this);
  m_forceData.resize(6);

  //synchronous acquisition
  m_recordTimer  = new QTimer(this);
  m_recordTimer->setTimerType(Qt::PreciseTimer);

  m_updateTimer = new QTimer(this);
  m_updateTimer->setTimerType(Qt::PreciseTimer);

  //vision control
  // m_breakPointIndex = 0;
  // m_confiMapConvex = new ConfidentMapConvex();
  //m_check_movement_sub =
}

void PluginController::init() {
  // addToAlgorithmDock();

  //update the robot state after checking checking box

  // connect(ui_->pbtnStartUSStream, &QPushButton::clicked, this, &PluginController::onStartUSStreamClicked);
  // connect(ui_->pbtnStopUSStream, &QPushButton::clicked, this, &PluginController::onStopUSStreamClicked);
  // connect(ui_->pbtnLoadSweep_imf, &QPushButton::clicked, this, &PluginController::onLoadImfSweepClicked);
  // connect(ui_->pbtnComputerConfiMap, &QPushButton::clicked, this, &PluginController::onComputeConfiMapClicked);
  // connect(ui_->pbtnVisualizeConfimap, &QPushButton::clicked, this, &PluginController::onVisualizeConfiMapClicked);
  // connect(ui_->pbtn_test_rotation, &QPushButton::clicked, this, &PluginController::onTestRotationButtonClicked);

  // to connect and disconnect the robot with the tool used.
  connect(ui_->pbtnConnect, &QPushButton::clicked, this, &PluginController::onConnectToRobotClick);
  connect(ui_->pbtnDisconnect, &QPushButton::clicked, this, &PluginController::onDisconnectFromRobotClick);

  // to use it in force mode set the stiffness and force and click on the force mode, else use the
  // position mode
  connect(ui_->pbtnForceMode, &QPushButton::clicked, this, &PluginController::onForceModeClick);
  connect(ui_->pbtnPositionMode, &QPushButton::clicked, this, &PluginController::onPositionModeClick);

  connect(ui_->pbtnSaveHome, &QPushButton::clicked, this, &PluginController::onSaveHome);
  connect(ui_->pbtnSaveOut, &QPushButton::clicked, this, &PluginController::onSaveOut);
  connect(ui_->pbtnGoHome, &QPushButton::clicked, this, &PluginController::onGoHome);
  connect(ui_->pbtnGoOut, &QPushButton::clicked, this, &PluginController::onGoOut);
  connect(ui_->pbtnExcuteCmd, &QPushButton::clicked, this, &PluginController::onExecuteMovementClick);

  //update the state when the state changed! changed to use timer
  connect(algorithm_, &PluginAlgorithm::poseChanged, this, &PluginController::updateUIToLastPose);
  connect(algorithm_, &PluginAlgorithm::wrenchChanged, this, &PluginController::updateUIToLastWrench);

  //Synchronous acquisition
  QObject::connect(m_recordTimer, &QTimer::timeout, this, &PluginController::onSynRecord);

  //update the robot state in UI
  QObject::connect(ui_->chbRecordWrench, &QCheckBox::stateChanged, this, &PluginController::onchbRecordStateChanged);
  QObject::connect(ui_->chbUpdateState, &QCheckBox::stateChanged, this, &PluginController::onChbUpdateStateChanged);
  QObject::connect(m_updateTimer, &QTimer::timeout, this, &PluginController::onUpdateState);

  //VISION CONTROL PLUGIN

  // // read the ultrasound poses from a file
  // connect(ui_->pbtnReadPosesFromFile, &QPushButton::clicked, this, &PluginController::onReadPosesClick);

  // // execute the ultrasound scan, and stop it anytime with a button
  // connect(ui_->pbtnExecScan, &QPushButton::clicked, this, &PluginController::onExecScanClick);

  // // go to the initial pose in the list of scan poses.
  // connect(ui_->pbtnGoIntiPos, &QPushButton::clicked, this, &PluginController::onGoInitPosClick);

  // // once the continue scan is clicked then the trasnformation will be calculated
  // // the robot will move to the position where it left off and then will start scaning again.
  // connect(ui_->pbtnContinueScan, &QPushButton::clicked, this, &PluginController::onContinueScanClicked);

  // start and stop the ultrasound sweep recording

  // load and do 3d compound of the sweep
  // connect(ui_->pbtnDownsampleSweep, &QPushButton::clicked, this, &PluginController::onDownsampleUSSweepClicked);
  // connect(ui_->pbtnCreateUSForLabelsWithoutStich, &QPushButton::clicked, this, &PluginController::onClickedCreateSweepFromLabel);
  // connect(ui_->pbtn_stitching, &QPushButton::clicked, this, &PluginController::onClickedStitching);
  // connect(ui_->pbtnWriteCurrRobotPose, &QPushButton::clicked, this, &PluginController::onClickedWriteRobotPose);

  connect(ui_->pbtnInitROS, &QPushButton::clicked, this, &PluginController::onClickedpbtnInitROS);

  // Test cases ->

  connect(ui_->pbtnAddPoint, &QPushButton::clicked, this, &PluginController::onClickedpbtnAddPoint);
  connect(ui_->pbtnExecuteTrajectory, &QPushButton::clicked, this, &PluginController::onClickedpbtnExecuteTrajectory);
  connect(ui_->pbtnDeletePoints, &QPushButton::clicked, this, &PluginController::onClickedpbtnDeletePoints);


  //zcy
  // connect(ui_->pbtnRoutePlanning, &QPushButton::clicked, this, &PluginController::onClickedpbtnRoutePlanning);
  // connect(ui_->pbtnPrepareForScan, &QPushButton::clicked, this, &PluginController::onClickedpbtnPrepareForScan);
  // connect(ui_->pbtnRunUSScan, &QPushButton::clicked, this, &PluginController::onClickedpbtnRunUSScan);
  // connect(ui_->pbtnLoadSweepPosition, &QPushButton::clicked, this, &PluginController::onClickedpbtnLoadSweepPosition);
  // connect(ui_->pbtnLoadSweepImage, &QPushButton::clicked, this, &PluginController::onClickedpbtnLoadSweepImage);
  // connect(ui_->pbtnExtractBoneSurface, &QPushButton::clicked, this, &PluginController::onClickedpbtnExtractBoneSurface);
  // connect(ui_->pbtbGetUSPointCloud, &QPushButton::clicked, this, &PluginController::onClickedpbtbGetUSPointCloud);
  // connect(ui_->pbtnShowSurface, &QPushButton::clicked, this, &PluginController::onClickedpbtnShowSurface);
  // connect(ui_->pbtnReplaceWithLabel, &QPushButton::clicked, this, &PluginController::onClickedpbtnReplaceWithLabel);
}



void PluginController::onClickedpbtnInitROS() {
    algorithm_->onInitROS();

}

void PluginController::onClickedpbtnAddPoint(){
     if (algorithm_->isRobotConnected()) {algorithm_->addPointConfiguration(algorithm_->getCurrentRobotPose());}
}

void PluginController::onClickedpbtnExecuteTrajectory(){
         if (algorithm_->isRobotConnected()) {algorithm_->executeTrajectory();};
}


void PluginController::onClickedpbtnDeletePoints(){
        algorithm_->deletePointConfigurations();
}



void PluginController::onClickedWriteRobotPose() {
  std::ofstream outfile;
  std::string file_path = ui_->ledt_trajectory_file_to_write->text().toStdString();
  outfile.open(file_path, std::ios_base::app); // append instead of overwrite
  auto pose = algorithm_->getCurrentRobotPose();
  outfile << std::to_string(pose.pose.position.x) << " " << std::to_string(pose.pose.position.y)
          << " " << std::to_string(pose.pose.position.z) << " " << std::to_string(pose.pose.orientation.x)
          << " " << std::to_string(pose.pose.orientation.y) << " " << std::to_string(pose.pose.orientation.z)
          << " " << std::to_string(pose.pose.orientation.w) << "\n";
  outfile.close();

}

/*****************************************
 * This function execute scan task
 ***************************************/


// void PluginController::onGoInitPosClick() {   //到路径中第一个点上方50mm
//     if(algorithm_->m_scanPointsNumber==0){
//         std::cout<<"There are no points in the scan trajectory."<<std::endl;
//         exit(EXIT_FAILURE);
//     }
//     else{
//       algorithm_->m_transformation_count = 0;
//       Eigen::Quaterniond quaPose(algorithm_->m_scanPointPoses[0](6), algorithm_->m_scanPointPoses[0](3), algorithm_->m_scanPointPoses[0](4), algorithm_->m_scanPointPoses[0](5));
//       Eigen::Vector3d vecPosition(algorithm_->m_scanPointPoses[0](0), algorithm_->m_scanPointPoses[0](1), algorithm_->m_scanPointPoses[0](2) + 50.0f / 1000.0f);
//       LOG_INFO("position:");
//       LOG_INFO(vecPosition);
//       algorithm_->executeCartesianCommand(quaPose, vecPosition, false);
//       // at the moment we are in the position -1
//       // next position needs to be the first point of the sweep
//       algorithm_->m_MoveStatus = ON_INITIAL_POSE;
//     }
// }

void PluginController::onExecuteMovementClick() {  //点击execute cmd
  Eigen::Vector3d rotAngles(ui_->ledtAngRa->text().toDouble(), ui_->ledtAngRb->text().toDouble(), //读入Ra Rb Rc到rotAngles向量
                            ui_->ledtAngRc->text().toDouble());
  rotAngles *= M_PI / 180.;   //转弧度
  algorithm_->executeCartesianCommand(
          Eigen::AngleAxisd(rotAngles[0], Eigen::Vector3d::UnitX()) *           //AngleAxisd(旋转角度，旋转轴)
          Eigen::AngleAxisd(rotAngles[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rotAngles[2], Eigen::Vector3d::UnitZ()),
          Eigen::Vector3d(ui_->ledtPosX->text().toDouble() / 1000, ui_->ledtPosY->text().toDouble() / 1000, //位置坐标
                          ui_->ledtPosZ->text().toDouble() / 1000), true);
}

void PluginController::onPositionModeClick() {
  LOG_INFO("Applying position model!");
  algorithm_->applyPositionControlMode();
}

void PluginController::onForceModeClick() {
  LOG_INFO("Applying Force!");
  algorithm_->applyDesiredForce(iiwa_msgs::DOF::Z, ui_->ledtDesiredForce->text().toDouble(),
                                ui_->ledtStiffness->text().toDouble());
}

void PluginController::onSaveOut() {
    if (algorithm_->isRobotConnected()) { algorithm_->setRobotOutConfiguration(algorithm_->getCurrentRobotPose()); }
}

void PluginController::onSaveHome() {
    if (algorithm_->isRobotConnected()) {algorithm_->setRobotHomeConfiguration(algorithm_->getCurrentRobotPose());}
}

void PluginController::onGoHome() {
    LOG_INFO("enter onGoHome");
  algorithm_->executeCartesianCommand(algorithm_->getRobotHomeConfiguration().pose, true);
}

void PluginController::onGoOut() {
  algorithm_->executeCartesianCommand(algorithm_->getRobotOutConfiguration().pose, true);
}

void PluginController::onConnectToRobotClick() {
    algorithm_->connect("IFLConvex");
    algorithm_->addStreamToDataModel(m_main->dataModel());

    emit robotConnected();  //! Emit connection signal.
}

void PluginController::onDisconnectFromRobotClick() {
  algorithm_->disconnect();
  emit robotDisconnected();  //! Emit disconnection signal.
}

void PluginController::updateUIToLastPose(){
    if (algorithm_->isRobotConnected()) {
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);
        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);   //mm
        Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //rad


        if (ui_->chbUpdateState->isChecked()) {
            ui_->ledtPosX->setText(QString::number(translation.x()));
            ui_->ledtPosY->setText(QString::number(translation.y()));
            ui_->ledtPosZ->setText(QString::number(translation.z()));
            ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
            ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI));
            ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));
        }
    }
}

void PluginController::updateUIToLastWrench() {
    if (algorithm_->isRobotConnected()) {

        auto robot_wrench = algorithm_->getCurrentRobotWrench();                  //获取robot_wrench   //auto可以在声明变量时根据变量初始值的类型自动为此变量选择匹配的类型

        if (ui_->chbUpdateState->isChecked()) {                                   //勾选update按钮
            ui_->ledtForceX->setText(QString::number(robot_wrench.force.x));
            ui_->ledtForceY->setText(QString::number(robot_wrench.force.y));      //力
            ui_->ledtForceZ->setText(QString::number(robot_wrench.force.z));
            ui_->ledtToqueX->setText(QString::number(robot_wrench.torque.x));
            ui_->ledtToqueY->setText(QString::number(robot_wrench.torque.y));     //力矩
            ui_->ledtToqueZ->setText(QString::number(robot_wrench.torque.z));
        }

    }
}



// //TCPTrackingStreamData
// void PluginController::onRecForce()
// {
//      tcpServer->listen(QHostAddress::Any,9999);
//      connect(tcpServer, SIGNAL(newConnection()), this, SLOT(acceptConnection()));
// }

// void PluginController::acceptConnection()
// {
//     serverSocket = tcpServer->nextPendingConnection();
//     connect(serverSocket,SIGNAL(readyRead()),this,SLOT(replyToClient()));
// }

// void PluginController::replyToClient()
// {
//     qDebug() <<"here C1 bytes = " << serverSocket->bytesAvailable();
//     if(serverSocket->bytesAvailable()>100)
//     {
//         qDebug() <<"MSG:" <<serverSocket->readAll();   //avoid crash
//         serverSocket->flush();
//         return;
//     }
//     std::string msg= std::string(serverSocket->readAll());
//     std::string delimiter = ";";

//     size_t pos = 0;
//     int nindex(0); // represent the index in force

//     while ((pos = msg.find(delimiter)) != std::string::npos) {
//         m_forceData[nindex] = std::stof(msg.substr(0, pos));
//         msg.erase(0, pos + delimiter.length());
//         nindex++;
//     }
//     LOG_INFO(m_forceData.at(1));
// }


//Synchronous acquisition
void PluginController::onSynRecord()
{
    if (algorithm_->isRobotConnected()) {

        auto robot_wrench = algorithm_->getCurrentRobotWrench();
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);

        ui_->ledtForceX->setText(QString::number(robot_wrench.force.x));
        ui_->ledtForceY->setText(QString::number(robot_wrench.force.y));
        ui_->ledtForceZ->setText(QString::number(robot_wrench.force.z));
        ui_->ledtToqueX->setText(QString::number(robot_wrench.torque.x));
        ui_->ledtToqueY->setText(QString::number(robot_wrench.torque.y));
        ui_->ledtToqueZ->setText(QString::number(robot_wrench.torque.z));

        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);   //mm       //robot_pose最后一列放translation
        Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //rad
        //(0,1,2)represents X-Y-Z，pitch yaw roll


        ui_->ledtPosX->setText(QString::number(translation.x()));
        ui_->ledtPosY->setText(QString::number(translation.y())); //平移
        ui_->ledtPosZ->setText(QString::number(translation.z()));
        ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
        ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI)); //欧拉角
        ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));



        if (ui_->chbRecordWrench->isChecked()) {

            auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);

            //calculate the reference angle position between real time frame and inital frame
            Eigen::Matrix3d  initialFrameFan = algorithm_->m_initialFrameFan.block<3, 3>(0, 0);
            Eigen::Matrix3d  currentFrameFan = robot_pose.block<3, 3>(0, 0);
            Eigen::Vector3d reletiveAngle = algorithm_->calculateReleativeRatationAngle(initialFrameFan, currentFrameFan);

            QString path = QDir::currentPath();
            QDir dir;
            path=path +QString("/ros/zhongliang/");
            if (!dir.exists(path))
                dir.mkpath(path); // create the directory if needed
            QFile file(path + "ForceData.txt");
            QString szData;
            std::vector<float> forceData = m_forceData;
            if (file.open(QIODevice::ReadWrite | QIODevice::Append)) {
                QTextStream streamOut(&file);
                streamOut /*<< szData.setNum(robot_wrench.wrench.force.x)
                                                              << "       "  //space of two TAB*/
                        << QString::number(robot_wrench.force.x)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.force.y)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.force.z)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.torque.x)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.torque.y)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.torque.z)
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(0,0))   //x
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(1,0))   //x
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(2,0))   //x
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(0,1))   //y
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(1,1))   //y
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(2,1))   //y
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(0,2))   //z
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(1,2))   //z
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(2,2))   //z
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(0,3))
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(1,3))
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(2,3))
                        << "       "  //space of two TAB
                        << QString::number(reletiveAngle(0))
                        << "       "  //space of two TAB
                        << QString::number(reletiveAngle(1))
                        << "       "  //space of two TAB
                        << QString::number(reletiveAngle(2))
                        << "       "  //space of two TAB
                        << QString::number(forceData[0])
                        << "       "  //space of two TAB
                        << QString::number(forceData[1])
                        << "       "  //space of two TAB
                        << QString::number(forceData[2])
                        << "       "  //space of two TAB
                        << QString::number(forceData[3])
                        << "       "  //space of two TAB
                        << QString::number(forceData[4])
                        << "       "  //space of two TAB
                        << QString::number(forceData[5])
                        << "       "  //space of two TAB
                        << endl;
            }
            file.close();
        }
    }
}

void PluginController::onchbRecordStateChanged()
{
    if(true == ui_->chbRecordWrench->isChecked())
    {
        m_recordTimer->start(TIM_INTERVAL_REC);
    }
    else {
        m_recordTimer->stop();
    }

}

void PluginController::onUpdateState()
{
    if (algorithm_->isRobotConnected()) {

        auto robot_wrench = algorithm_->getCurrentRobotWrench();

        ui_->ledtForceX->setText(QString::number(robot_wrench.force.x));
        ui_->ledtForceY->setText(QString::number(robot_wrench.force.y));
        ui_->ledtForceZ->setText(QString::number(robot_wrench.force.z));
        ui_->ledtToqueX->setText(QString::number(robot_wrench.torque.x));
        ui_->ledtToqueY->setText(QString::number(robot_wrench.torque.y));
        ui_->ledtToqueZ->setText(QString::number(robot_wrench.torque.z));

    }

    if (algorithm_->isRobotConnected()) {
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);
        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);   //mm
        Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //rad

        ui_->ledtPosX->setText(QString::number(translation.x()));
        ui_->ledtPosY->setText(QString::number(translation.y()));
        ui_->ledtPosZ->setText(QString::number(translation.z()));
        ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
        ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI));
        ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));

    }
}

void PluginController::onChbUpdateStateChanged()
{
    if(true == ui_->chbUpdateState->isChecked())
    {
        m_updateTimer->start(TIM_INTERVAL_UPDATE);
    }
    else {
        m_updateTimer->stop();
    }
}

// void PluginController::start_recording(){
//       LOG_ERROR("Do Record!");

//       auto cepha_stream = m_main->dataModel()->get("Cephasonics Main Stream");
//       auto robot_stream = m_main->dataModel()->get("Robot Tracking");

//       algorithm_->set_new_stream(cepha_stream, robot_stream);
//       algorithm_->start_recording_us_stream();
// }

// std::string PluginController::getDayAndTime() {
//   time_t rawtime;
//   struct tm* timeinfo;
//   char buffer[80];

//   time(&rawtime);
//   timeinfo = localtime(&rawtime);

//   strftime(buffer, sizeof(buffer), "%d_%m_%H_%M_%S", timeinfo);
//   std::string str(buffer);

//   return str;
// }



}  // namespace OrienAdj
}  // namespace ImFusion



