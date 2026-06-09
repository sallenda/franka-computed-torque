/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages constraining
/// a lcmt_robot_plan message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <memory>

//Manually adding the libraries
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <ctime> 
#include <qpmad/solver.h>


#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class RobotPlanRunner {
 public:
  /// plant is aliased
  explicit RobotPlanRunner(const multibody::MultibodyPlant<double>& plant)
      : mplant(plant){
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);

     //Define the initial parameters for kp and kd matrices to be used in pd controller
    double kp = 110, kv = 11, kpOr = 60, kvOr = 8;
    
    //Define the scaling factor that you want your torque to be initialized. Defining here for easy access
    mScale = 0.85;

    //Initializing the parameters by calling the function
    InitDynamicParam(kp, kv, kpOr, kvOr);
  }

  // *************************** Read Matrix to read from text file. Use to read F & G Matrices **************************

  Eigen::MatrixXd readMatrix(const char *filename)
  {
    int cols = 0, rows = 0;
    double buff[10000];

      // Read numbers from file into buffer.
    std::ifstream infile;
    infile.open(filename);
    while (! infile.eof())
    {
      std::string line;
      getline(infile, line);

      int temp_cols = 0;
      std::stringstream stream(line);
      while(! stream.eof())
        stream >> buff[cols*rows+temp_cols++];

      if (temp_cols == 0)
        continue;

      if (cols == 0)
        cols = temp_cols;

      rows++;
    }

    infile.close();

    rows--;

      // Populate matrix with numbers.
    Eigen::MatrixXd result(rows,cols);
      for (int i = 0; i < rows; i++)
       for (int j = 0; j < cols; j++)
          result(i,j) = buff[ cols*i+j ];

    return result;
  }

  void Run() {
    mCur_time_us = -1;
    int64_t start_time_us = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.utime = mCur_time_us;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = kNumJoints;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(15) || iiwa_status_.utime == -1) {
        bias_ref = 0;//iiwa_status_.utime;
       }

      mCur_time_us = iiwa_status_.utime;// - bias_ref;

      UpdateDynamicParam();

      //Declare the joint torque vector
      Eigen::VectorXd joint_torque_cmd = Eigen::VectorXd::Zero(7);

      //Compute the pseudo inverse of the jacobian
      mPinv_Jv_robot1 = mJv_robot1.transpose() * (mJv_robot1 * mJv_robot1.transpose() + 0.00025 * Eigen::Matrix3d::Identity()).llt().solve(Eigen::Matrix3d::Identity());
      // std::cout << "mJv_robot1 = \n" << mJv_robot1 << std::endl;

      //Compute the torque by doing feedback linearization
      //Adding just the Coriolis term since gravity term is already added in torque controller.
      // joint_torque_cmd = mM_robot1 * mPinv_Jv_robot1 * (mDesired_ddx_position - mdJv_dq) + mCg_robot1;

      // Pass the optimization and the scaling factor that you want your torque limit to be constraint.
      RESCLF(mScale);
      // std::cout << " Main Check 1" << std::endl;

      //Declare the RESLCF body torque
      // Eigen::VectorXd bodyTorques = Eigen::VectorXd::Zero(7);
      // bodyTorques = mM_robot1 * pinv_Jv * (mDesired_ddx_position + mddq_RESCLF_robot - mdJv_dq) + mCg_robot1;
      
      // std::cout << "mE_robot = \n" << mE_robot << std::endl;
      // std::cout << "mdE_robot = \n" << mdE_robot << std::endl;
      // std::cout << "mDesired_ddx_position = \n" << mDesired_ddx_position << std::endl;
      // std::cout << "mEE_position_ = \n" << mEE_position_ << std::endl;
      // std::cout << "Feeback Linearization Torque = \n" << joint_torque_cmd << std::endl;
      // std::cout << "mBodyTorques_QPMAd_robot1 = \n" << mBodyTorques_QPMAd_robot1 << std::endl;
      //Passing the computed torque to iiwa command to apply on robot
      for (int joint = 0; joint < kNumJoints; joint++)
      {
        iiwa_command.joint_position[joint] = iiwa_status_.joint_position_measured[joint];
        iiwa_command.joint_torque[joint] = mBodyTorques_QPMAd_robot1[joint]; //mFBL_Control_robot1(joint);////joint_torque_cmd[joint];
      }
      
    //lcm_.publish(kLcmCommandChannel, &iiwa_command);
      
      // const double cur_traj_time_s = static_cast<double>(cur_time_us - start_time_us) / 1e6;

      iiwa_command.utime = iiwa_status_.utime;

      for (int joint = 0; joint < kNumJoints; joint++) {
        // iiwa_command.joint_position[joint] = desired_next(joint);
      }

      lcm_.publish(kLcmCommandChannel, &iiwa_command);
      counter++;
    }
  }

 private:
   //Function to initialize the parameters
  void InitDynamicParam(double kp, double kd, double kpOr, double kdOr)
  {
    //Creating default context for the plant
    mContext = mplant.CreateDefaultContext();

    //Copying end-effector name from urdf
    mEE_link = "iiwa_link_ee";

    //Defining the frame for end-effector
    mFrame_EE = &mplant.GetBodyByName(mEE_link).body_frame();

    //Initializing the joint position and velocity
    int nv = mplant.num_velocities();
    mq_robot = Eigen::VectorXd::Zero(nv); // Joint positions. We can also use kNumJoints
    mdq_robot = Eigen::VectorXd::Zero(nv); // Joint velocities.
    // mq_robot(1) = 30*3.14/180;
    // mq_robot(3) = -60*3.14/180;

//   mq_robot(0) = 0*3.14/180;
    mq_robot(1) = 52*3.14/180;
  // mq_robot(2) = 0*3.14/180;
    mq_robot(3) = -86*3.14/180;
  // mq_robot(4) = 0*3.14/180;
    mq_robot(5) = 39*3.14/180;
  // mq_robot(6) = 0*3.14/180;

    // std::cout << "mIiwa_q_ = " << mIiwa_q_.rows() << "*" << mIiwa_q_.cols() << std::endl;
    // std::cout << "iiwa_status_.num_joints = " << iiwa_status_.num_joints << std::endl;
    
    // This didnot work becuase iiwa_status is not initialized at the moment
    // for (int joint = 0; joint < iiwa_status_.num_joints; joint++) {
    //   mIiwa_q_[joint] = iiwa_status_.joint_position_measured[joint];
    //   mIiwa_qdot_[joint] = iiwa_status_.joint_velocity_estimated[joint];
    // }
   
    //Initialize mass and coriolis term
    mM_robot1 = Eigen::MatrixXd::Zero(nv, nv);;
    mCg_robot1 = Eigen::VectorXd::Zero(nv, nv);;
    
    int task_dim = 3; // Dimenstion of task space. 3 for position only
    int task_dim_ori = 3;  // Dimenstion of task space. 3 for orientation only

    //Initialize dimension for desired and actual position and velocity
    mE_robot = Eigen::VectorXd::Zero(task_dim);
    mDe_robot = Eigen::VectorXd::Zero(task_dim);

    mDesired_ee_ori = Eigen::VectorXd::Zero(task_dim_ori);
    mDesired_ee_ori_vel = Eigen::VectorXd::Zero(task_dim_ori);
    mEE_position = Eigen::VectorXd::Zero(task_dim);
    mEE_velocity = Eigen::VectorXd::Zero(task_dim);
    mEE_RPY = Eigen::VectorXd::Zero(task_dim);
    mDesired_ddx_position = Eigen::VectorXd::Zero(task_dim);

    // //Orientation of the robot
    mTargetRPY = Eigen::VectorXd::Zero(task_dim);
    // mTargetRot.set(Eigen::Matrix3d::Zero());
    mTargetRot.Identity();

    // std::cout << "mTargetRot = \n" << mTargetRot << std::endl;
    
    // mEE_w_robot = Eigen::VectorXd::Zero(task_dim);
    // mEE_dw_robot = Eigen::VectorXd::Zero(task_dim);

    //Initialize the Linear and Angular Jacobian matrix
    mJv_robot1 = Eigen::MatrixXd::Zero(task_dim, nv);
    mJw_robot1 = Eigen::MatrixXd::Zero(task_dim, nv);
    
    //Initialize the product dJ*dq for linear and angular jacobian
    mdJv_dq = Eigen::Vector3d::Zero();
    mdJw_dq = Eigen::Vector3d::Zero();

    //Initialize gains for position and velocity
    mKp.setZero();
    mKv.setZero();
    for (std::size_t i = 0; i < 3; ++i)
    {
      mKp(i,i) = kp;
      mKv(i,i) = kd;

      mKpOr(i,i) = kpOr;
      mKvOr(i,i) = kdOr;
    }
   
    // Define the plant to position given by iiwa_status
    // Since mContext is a pointer, we first get its value and then pass its address
    // since arguement expect the pointer so we have to pass its address.
    // We can also use mContext_.get() 
    mplant.SetPositions(&(*mContext), mq_robot); 
    mplant.SetVelocities(mContext.get(), mdq_robot);
    
    //Get the end-effector position. mContext is passed as a pointer becuase it is itself a pointer and we want to pass its value.
    mEE_link_pose = mplant.EvalBodyPoseInWorld(*mContext, mplant.GetBodyByName(mEE_link));

    //Get the end-effector velocity in spatial representation
    mEE_link_velocity = mplant.EvalBodySpatialVelocityInWorld(*mContext, mplant.GetBodyByName(mEE_link));
    
    //Define the position by taking the translation part.
    mEE_position = mEE_link_pose.translation();

    //Define the velocity by taking the translational part
    mEE_velocity = mEE_link_velocity.translational();
    
    //Define the target position by taking the end effector position
    mTargetPos = mEE_link_pose.translation();
    
    //Define the orientation by first converting math::rotationmatrix<double> to isometry
    mTargetRot = mEE_link_pose.GetAsIsometry3();
    mEE_Rot = mEE_link_pose.GetAsIsometry3();

    //Get the orientation part of the isometry and then represent it in XYZ euler angles. 
    // mEE_RPY = mEE_link_pose.GetAsIsometry3();

    // mTargetRPY = (mTargetRot.linear()).eulerAngles(0, 1, 2);
    mTargetRPY << 0, 1.57, 0;

    mEE_RPY = (mEE_Rot.linear()).eulerAngles(0, 1, 2);

    // dx = Eigen::VectorXd::Zero(3);
    // dx << 0.1, 0.02, -0.05;
    // mTargetPos = mTargetPos + dx;

    mTargetVel = Eigen::VectorXd::Zero(3);

    mFreq = 0.0002; //Set frequency for the sine wave of moving target

    mAmplitude = 0.2; //Set Amplitude for the moving target

    //Reading parameters for the 
    // mP = readMatrix("/home/murtaza/drake/examples/kuka_iiwa_arm/P.txt");
    // mF = readMatrix("/home/murtaza/drake/examples/kuka_iiwa_arm/F.txt");
    // mG = readMatrix("/home/murtaza/drake/examples/kuka_iiwa_arm/G.txt"); 
    
    //Not using files anymore as mP, mF, and mG are fine and do not change.
    mP = Eigen::MatrixXd::Zero(6, 6);
    mF = Eigen::MatrixXd::Zero(6, 6);
    mG = Eigen::MatrixXd::Zero(6, 3);

    mP << 1.73205, 0, 0, 1, 0, 0,
          0, 1.73205, 0, 0, 1, 0,
          0, 0, 1.73205, 0, 0, 1,
          1, 0, 0, 1.73205, 0, 0,
          0, 1, 0, 0, 1.73205, 0,
          0, 0, 1, 0, 0, 1.73205;

    mF << 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 1,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0;

    mG << 0, 0, 0,
          0, 0, 0,
          0, 0, 0,
          1, 0, 0,
          0, 1, 0,
          0, 0, 1;

    mTauLim_robot1 << 320, 320, 176, 176, 110, 40, 40; // Need to be adjusted when adding limits

    Eigen::Vector3d ObstacleOffset(0, 0, 0);
    // mObstaclePos = mEE_position + ObstacleOffset;
    mObstaclePos << 0.6, 0, 0.4;
    mObstacleVel << 0, 0, 0;
  }

  void UpdateDynamicParam()
  {
    // std::cout << "iiwa_status_.num_joints = " << iiwa_status_.num_joints << std::endl;
    for (int joint = 0; joint < iiwa_status_.num_joints; joint++)
    {
      mq_robot[joint] = iiwa_status_.joint_position_measured[joint];
      mdq_robot[joint] = iiwa_status_.joint_velocity_estimated[joint];
    }

     // Update context
    mplant.SetPositions(&(*mContext), mq_robot);
    mplant.SetVelocities(mContext.get(), mdq_robot);
    
    // Calculate mass matrix.
    mplant.CalcMassMatrix(*mContext, &mM_robot1);
    
    // Calculate Coriolis forces.
    mplant.CalcBiasTerm(*mContext, &mCg_robot1);

    // Get the end-effector position. mContext is passed as a pointer becuase it is itself a pointer and we want to pass its value.
    mEE_link_pose = mplant.EvalBodyPoseInWorld(*mContext, mplant.GetBodyByName(mEE_link));
    
    //Define the position by taking the translation part.
    mEE_position = mEE_link_pose.translation();
    
    //Get the end-effector velocity
    mEE_link_velocity = mplant.EvalBodySpatialVelocityInWorld(*mContext, mplant.GetBodyByName(mEE_link));

    mEE_velocity = mEE_link_velocity.translational();

    //Define the orientation by first converting math::rotationmatrix<double> to isometry
    mEE_Rot = mEE_link_pose.GetAsIsometry3();
    
    //Defining the end effector orientation in Euler XYZ to analyze it
    mEE_RPY = (mEE_Rot.linear()).eulerAngles(0, 1, 2);


    // Calculate Linear Jacobian
    mplant.CalcJacobianTranslationalVelocity(*mContext,
                                      multibody::JacobianWrtVariable::kQDot,
                                      *mFrame_EE,
                                      Eigen::Vector3d::Zero(),
                                      mplant.world_frame(),
                                      mplant.world_frame(),
                                      &mJv_robot1 
                                      );

    // Calculate spatial Jacobian. first three are angular jacobian and last 3 are linear jacobian
    mplant.CalcJacobianSpatialVelocity(*mContext,
                                      multibody::JacobianWrtVariable::kQDot,
                                      *mFrame_EE,
                                      Eigen::Vector3d::Zero(),
                                      mplant.world_frame(),
                                      mplant.world_frame(),
                                      &mJacobian 
                                      );
    

    mJw_robot1 = mJacobian.block(0,0,3,7);

    //Calculates dJ*dq
    mdJv_dq = mplant.CalcBiasTranslationalAcceleration(*mContext,
                                      multibody::JacobianWrtVariable::kV,
                                      *mFrame_EE,
                                      Eigen::Vector3d::Zero(),
                                      mplant.world_frame(),
                                      mplant.world_frame()
                                      ); 
    

    // CalcBiasSpatialAcceleration
    mdJ_Aug_dq_spatial1 = mplant.CalcBiasSpatialAcceleration(*mContext,
                                      multibody::JacobianWrtVariable::kV,
                                      *mFrame_EE,
                                      Eigen::Vector3d::Zero(),
                                      mplant.world_frame(),
                                      mplant.world_frame()
                                      ); 

    mdJ_Aug_dq_robot1 = mdJ_Aug_dq_spatial1.get_coeffs();

    if(counter > 500)
    {  
      //Update the target position if necassary. For now it is not being done
      // dx = mAmplitude * std::sin(2 * M_PI * mFreq * counter2 * 3e-3);//mCur_time_us * 3e-3); 
      // dx << 0.1,0.01,0.1;
      if(counter2 % 20 == 0)
      {
        if(check == 0)
        {
          dx = -0.005;
        }
        else if (check == 1)
        {
          dx  = 0.005;
        } 
      }
      if(mTargetPos(2) > 0.67)
      {
        check = 0;
      }
      else if(mTargetPos(2) < 0.20)
      {
        check = 1;
      }
      mTargetPos(2) = mTargetPos(2) + dx;
      dx = 0;
      // mTargetVel(1) = mAmplitude * 2 * M_PI * mFreq * std::cos(2 * M_PI * mFreq * counter2 * 3e-3);//mCur_time_us * 3e-3);
      counter2++;
    }

    //Error value between target position and end effector position
    mE_robot = mTargetPos - mEE_position;
    mDe_robot = mTargetVel - mEE_velocity; //Error value between target velocity and end effector velocity
    // std::cout << "mE_robot = \n" << mE_robot << std::endl;
    spdlog::info("mE_robot = \n{}",mE_robot );
    // spdlog::info("mDe_robot = \n{}",mDe_robot );
    spdlog::info("mTargetPos = \n{}",mTargetPos );
    spdlog::info("mEE_position = \n{}",mEE_position );
    // spdlog::info("dx = \n{}",dx );

    spdlog::info("mTargetRPY = \n{}",mTargetRPY);
    spdlog::info("mEE_RPY = \n{}",mEE_RPY);
    spdlog::info("mObstaclePos = \n{}",mObstaclePos);


    // spdlog::info("mTargetVel = \n{}",mTargetVel );
    // spdlog::info("mEE_velocity = \n{}",mEE_velocity );

    // std::cout << "mDe_robot = \n" << mDe_robot << std::endl;


    mDesired_ddx_position = mKp*mE_robot + mKv*mDe_robot; //Desired acceleration as PD control

    // Orientation control part
    // End-effector Orientation for robot
    Eigen::Quaterniond quat_robot1(mEE_Rot.linear());
    double quat_w_robot1 = quat_robot1.w();
    Eigen::Vector3d quat_xyz_robot1(
        quat_robot1.x(), quat_robot1.y(), quat_robot1.z());
    
    if (quat_w_robot1 < 0) {
        quat_w_robot1 *= -1.0;
        quat_xyz_robot1 *= -1.0;
    }

    Eigen::Quaterniond quatRef_robot1(
        Eigen::AngleAxisd(mTargetRPY(0), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(mTargetRPY(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(mTargetRPY(2), Eigen::Vector3d::UnitZ()));

    double quatRef_w_robot1 = quatRef_robot1.w();
    Eigen::Vector3d quatRef_xyz_robot1(
        quatRef_robot1.x(), quatRef_robot1.y(), quatRef_robot1.z());

    if (quatRef_w_robot1 < 0) {
        quatRef_w_robot1 *= -1.0;
        quatRef_xyz_robot1 *= -1.0;
    }

    if (std::pow(-quat_w_robot1 - quatRef_w_robot1, 2)
            + std::pow((-quat_xyz_robot1 - quatRef_xyz_robot1).norm(), 2)
        < std::pow(quat_w_robot1 - quatRef_w_robot1, 2)
              + std::pow((-quat_xyz_robot1 - quatRef_xyz_robot1).norm(), 2)) {
        quat_w_robot1 *= -1.0;
        quat_xyz_robot1 *= -1.0;
    }
    mQuatError_xyz_robot1 = quatRef_w_robot1 * quat_xyz_robot1
                           - quat_w_robot1 * quatRef_xyz_robot1
                           + quatRef_xyz_robot1.cross(quat_xyz_robot1);


    mEE_w_robot = mJw_robot1 * mdq_robot; // mEndEffector->getAngularVelocity() = Jw * dq
    mDwref_robot1 = -mKpOr * mQuatError_xyz_robot1 - mKvOr * mEE_w_robot;
    // std::cout << " Update DynamicPara Check 1" << std::endl;
    spdlog::info(" Update Dynamic Parameter Done");

  }

  void RESCLF(double scaling)
  {
      // ***************** Writing our own controller  - Feedback Linearization *******************************
    Eigen::Matrix J_Aug_robot1 = mJacobian;
    
    //  Pseudo inverse augmented jacobian and product of mass and pseudo inverse jacobian for robot 1
    Eigen::MatrixXd pinv_J_Aug_robot1 = J_Aug_robot1.transpose() * (J_Aug_robot1 * J_Aug_robot1.transpose() + 0.0025 * Eigen::MatrixXd::Identity(6, 6)).inverse();
    Eigen::MatrixXd M2_aug_robot1 = mM_robot1 * pinv_J_Aug_robot1;
    Eigen::MatrixXd M2_position_robot1 = mM_robot1 * mPinv_Jv_robot1;


    // // Inverse of Mass Matrix for robot1
    Eigen::MatrixXd invM_robot1 = mM_robot1.inverse();

    Eigen::Matrix<double, 6, 1> desired_ddx_Aug_robot1; 
    desired_ddx_Aug_robot1 << mDwref_robot1, mDesired_ddx_position ;
    // std::cout << "mDesired_ddx_position = \n" << mDesired_ddx_position << std::endl;
    // std::cout << "mDwref_robot1 = \n" << mDwref_robot1 << std::endl;
    

    //  Feedback linearization controller for robot1
    mFBL_Control_robot1 = M2_aug_robot1 * (desired_ddx_Aug_robot1 - mdJ_Aug_dq_robot1) + mCg_robot1;
    // mFBL_Control_robot1 = M2_position_robot1 * (mDesired_ddx_position - mdJv_dq) + mCg_robot1;


    size_t n = mF.rows();
    // std::cout << "n = mF.rows() = " << n <<  std::endl;

    //Define states for each operational space for robot1
    Eigen::Matrix<double, 6,1> EE_Eta_robot1, Or_Eta_robot1;
    EE_Eta_robot1 << mE_robot, mDe_robot; //x - targetPosition,dx;
    Or_Eta_robot1 << mQuatError_xyz_robot1, mEE_w_robot;

    // Defining eta for error in robot1
    Eigen::VectorXd mEta1_robot1(n), mEta2_robot1(n);
    mEta1_robot1 << EE_Eta_robot1;
    mEta2_robot1 << Or_Eta_robot1;

    // Defining the LfV_x, LgV_x and V_x for position in robot1
    Eigen::MatrixXd LfV_x_pos_robot1 = mEta1_robot1.transpose()*(mF.transpose()*mP+mP*mF)*mEta1_robot1;
    Eigen::MatrixXd LgV_x_pos_robot1 = 2*mEta1_robot1.transpose()*mP*mG;
    Eigen::MatrixXd V_x_pos_robot1 = mEta1_robot1.transpose()*mP*mEta1_robot1;

    // Defining the LfV_x, LgV_x and V_x for orientation in robot1
    Eigen::MatrixXd LfV_x_ori_robot1 = mEta2_robot1.transpose()*(mF.transpose()*mP+mP*mF)*mEta2_robot1;
    Eigen::MatrixXd LgV_x_ori_robot1 = 2*mEta2_robot1.transpose()*mP*mG;
    Eigen::MatrixXd V_x_ori_robot1 = mEta2_robot1.transpose()*mP*mEta2_robot1;
    // std::cout << " RESCLF Check 1" << std::endl;

    // //*****************************************************Barrier Part*************************************
    // //Adding Barrier Stuff
    Eigen::Vector3d x_robot1  = mEE_position;
    Eigen::Vector3d dx_robot1  = mEE_velocity;

    Eigen::Vector3d x_b = mObstaclePos; //mObstacle->getWorldTransform().translation();

    Eigen::Vector3d ddx_b(0,0,0); //= mObstacleAcceleration;

    Eigen::Vector3d e_b = x_robot1 - mObstaclePos;

    Eigen::Vector3d de_b = dx_robot1 - mObstacleVel;

    Eigen::Matrix3d A_Obstacle;
    A_Obstacle.setZero();
    double mObstacleRadius = 0.1;

    A_Obstacle(0, 0) = 1 / (mObstacleRadius * mObstacleRadius);
    A_Obstacle(1, 1) = 1 / (mObstacleRadius * mObstacleRadius);
    A_Obstacle(2, 2) = 1 / (mObstacleRadius * mObstacleRadius);

    // Degine Eta for Exponential Barrier Function
    Eigen::Vector2d eta;
    eta << e_b.transpose() * A_Obstacle * e_b - 1, 2 * e_b.transpose() * A_Obstacle * de_b;

    // Define K matrix containing poles. Not using this formulation. Leaving it for reference.
    double p0, p1, h_0, h_1, temp1;
    h_0 = e_b.transpose() * A_Obstacle * e_b - 1;
    h_1 = 2 * e_b.transpose() * A_Obstacle * de_b;
    temp1 = h_1 / h_0;
    p1 = 50 + temp1;
    p0 = 500;

    //Dealing with K_b for Barrier Formualation
    // K_b << 500,80;
    Eigen::Matrix<double, 1, 2> K_b;
    K_b << 160, 40; //2000,800;


    // std::cout << " RESCLF Check 2" << std::endl;

    //L_G_CBF deals with formulation for input \Tau, whereas L_G_CBF_OS deals with \mu from operational space by expanding \Tau.
    //Similarly for L_F_CBF and L_F_CBF_OS.
    Eigen::Matrix<double, 1, 7> L_G_CBF = 2 * e_b.transpose() * A_Obstacle * mJv_robot1 * invM_robot1;
    Eigen::Matrix<double, 1, 3> L_G_CBF_OS = 2 * (e_b).transpose() * A_Obstacle * mJv_robot1 * invM_robot1 * mM_robot1 * mPinv_Jv_robot1;

    Eigen::Matrix<double, 1, 1> L_F_CBF = 2 * de_b.transpose() * A_Obstacle * de_b + 2 * e_b.transpose() * A_Obstacle * (mdJv_dq - mJv_robot1 * invM_robot1 * mCg_robot1 - ddx_b);
    Eigen::Matrix<double, 1, 1> L_F_CBF_OS = 2 * de_b.transpose() * A_Obstacle * de_b + 2 * e_b.transpose() * A_Obstacle * (mdJv_dq - mJv_robot1 * invM_robot1 * mCg_robot1 - ddx_b) + 2 * e_b.transpose() * A_Obstacle * mJv_robot1 * invM_robot1 * (mM_robot1 * mPinv_Jv_robot1 * (mDesired_ddx_position - mdJv_dq) + mCg_robot1);
    Eigen::Matrix<double, 1, 2> eta_CBF = eta;
    Eigen::Matrix<double, 2, 1> K_CBF = K_b;

    //     // Set Optimization parameters for robot1 and robot2
    qpmad::MatrixIndex size = 7;
    qpmad::MatrixIndex num_ctr = 10;
    // static constexpr size_t size_dv = 7;
    // static constexpr size_t num_ctr1 = 7;

    // Initialize the variables for QPMad
    Eigen::VectorXd x; // Decision variables
    Eigen::MatrixXd H; // Quadratic cost used in x'Hx
    Eigen::VectorXd h; // Linear cost used in h'*x
    Eigen::MatrixXd A_x; // Constraint A_x*x
    Eigen::VectorXd Alb; // Lower bound for the constraint -b < A_x * x
    Eigen::VectorXd Aub; // Upper bound for the constraint  A_x * x < b
    Eigen::VectorXd lb; // Lower Constraint on decision variables
    Eigen::VectorXd ub; // Lower Constraint on decision variable
    // std::cout << " RESCLF Check 3" << std::endl;

      //Declaring Quadratic cost for robot1. Total of seven variables with 3 \mu for position and 3 \mu for orientation and 1 input for relaxation.
    H.setIdentity(size, size);
    // H.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    // H.setIdentity();
    double relaxation = 10;
    double gamma = 1e-3;

    H(6, 6) = relaxation;

    //Declaring linear cost for roboti
    h.setZero(size);
    // h.setZero();

    //Declaring constraint for robot1
    //Initializing A_x matrices.
    A_x.setZero(num_ctr, size);
    // A_x.setZero();

    //Constraint for Position
    A_x.block<1, 3>(1, 3) = LgV_x_pos_robot1;
    A_x(0, 6) = -1;

    //Constraint for orientation
    A_x.block<1, 3>(0, 1) = LgV_x_ori_robot1;

    // //Constraint for Obstacle avoidance barrier
    A_x.block<1, 3>(2, 3) = L_G_CBF_OS;

    // //Constraint for the the joints barrier
    // A_x.block<1, 3>(3, 0) = L_G_CBF_Joint;

    // // //Constraint for the Outer Box barrier
    // // A_x.block<1, 3>(4, 0) = L_G_CBF_OS_Outer;

    //Constraint for the Torque limits
    A_x.block<7, 6>(3, 0) = mM_robot1 * pinv_J_Aug_robot1;

    //Defining a vector to define the range of decision variables \mu and \delta(relaxation). Not used and will delete later.
    Eigen::Matrix<double, 7, 1> Huge_Limit_max;
    Huge_Limit_max << HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL;
    // std::cout << " RESCLF Check 4" << std::endl;

    //Defining the size of constraints.
    Alb.resize(num_ctr);
    Aub.resize(num_ctr);

    //Lower range for position constraint.
    Alb(0) = -HUGE_VAL;

    //Lower range for orientation constraint.
    Alb(1) = -HUGE_VAL;

    // // // Lower Constraint for Obstacle Barriers
    Alb(2) = -(L_F_CBF_OS(0, 0) + K_b * eta);
    //     std::cout << "RESCLF: check 6" << std::endl;

    // //Lower Constraints for Joints Barrier
    // Alb(3) = -(L_F_CBF_Joint(0, 0) + K_eta_CBF_Joint(0, 0));

    // //Lower Constraints for Outer Box Barrier
    // Alb(4) = -(L_F_CBF_OS_Outer(0, 0) + K_eta_CBF_Outer(0, 0)); //-(L_F_CBF_OS_Outer(0,0)) + K_b_Outer * eta_CBF_Outer);

    // //Lower Constraints values for torque
    Alb.segment<7>(3) = -mTauLim_robot1 - (mM_robot1 * pinv_J_Aug_robot1 * (desired_ddx_Aug_robot1 - mdJ_Aug_dq_robot1) + mCg_robot1);

    //Upper range for position constraint.
    Aub(0) = -LfV_x_pos_robot1(0, 0) - gamma * V_x_pos_robot1(0, 0);

    // //Upper range for orientation constraint.
    Aub(1) = -LfV_x_ori_robot1(0, 0) - gamma * V_x_ori_robot1(0, 0);

    // // //Upper range for obstacle avoidance barrier constraints
    Aub(2) = HUGE_VAL;

    // //Upper range for joints barrier constraint.
    // Aub(3) = HUGE_VAL;

    // // // //Upper range for outer box barrier constraint.
    // // // Aub(4) = HUGE_VAL;

    // //Upper range for torque constraints
    Aub.segment<7>(3) = mTauLim_robot1 - (mM_robot1 * pinv_J_Aug_robot1 * (desired_ddx_Aug_robot1 - mdJ_Aug_dq_robot1) + mCg_robot1);

    // //  If lb and ub is set to three variables with one constraint
    // lb << -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL;
    // ub << HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL;
    // std::cout << " RESCLF Check 5" << std::endl;

    Eigen::VectorXcd eivals = H.eigenvalues();
    // std::cout << "Eigenvalues of H =\n" << eivals << std::endl;

    qpmad::Solver solver;
    try
    {
      qpmad::Solver::ReturnStatus status = solver.solve(x, H, h, lb, ub, A_x, Alb, Aub);
    }
    catch(const std::exception& e)
    {
      std::cerr << "qpmad failed: " << e.what() << std::endl;
    }
       
    // if (status != qpmad::Solver::OK)
    // {
    //   std::cerr << "Error" << std::endl;
    // }
    // std::cout << " RESCLF Check 6" << std::endl;

    mBodyTorques_QPMAd_robot1 = mM_robot1 * pinv_J_Aug_robot1 * (desired_ddx_Aug_robot1 + x.head<6>() - mdJ_Aug_dq_robot1) + mCg_robot1;
    // std::cout << " RESCLF Check 7" << std::endl;

}


  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& mplant;
  lcmt_iiwa_status iiwa_status_;
  int64_t mCur_time_us, bias_ref;
  double mFreq, mAmplitude;


  //Manually defined variables for the controller
  std::unique_ptr<systems::Context<double>> mContext;
  std::string mEE_link; //end-effector link name
  const multibody::Frame<double>* mFrame_EE; // End effector frame
  Eigen::Vector3d mdJv_dq; //Product of Linear Jacobian Derivative and joint velocity = Jdot*q_dot
  Eigen::Vector3d mdJw_dq; //Product of Angular Jacobian Derivative and joint velocity
  Eigen::VectorXd mq_robot; // Joint positions.
  Eigen::VectorXd mdq_robot; // Joint velocities.
  Eigen::MatrixXd mM_robot1; // Mass matrix.
  Eigen::VectorXd mCg_robot1; // Coriolis vector.
  Eigen::VectorXd mE_robot; //Desired ee position
  Eigen::VectorXd mDe_robot; //Desired ee velocity
  Eigen::VectorXd mDesired_ee_ori; //Desired orientation position
  Eigen::VectorXd mDesired_ee_ori_vel; //Desired orientation velocity
  Eigen::VectorXd mDesired_ddx_position; //Desired ee acceleration
  Eigen::VectorXd mEE_position; //Current ee position
  Eigen::VectorXd mEE_velocity; //Current ee velocity
  Eigen::VectorXd mEE_RPY; //Current ee orientation
  Eigen::Isometry3d mEE_Rot; //Initial rotation matrix


  Eigen::Vector3d mEE_w_robot; //Current ee angular velocity (orientation)
  Eigen::Vector3d mQuatError_xyz_robot1; //Current ee angular position (orientation)
  Eigen::Vector3d mDwref_robot1; //Current ee angular velocity (orientation)
  Eigen::Matrix<double, 7,1> mTauLim_robot1; //Torque Limit of the robot
  Eigen::Vector3d mObstaclePos; //mObstacle->getWorldTransform().translation();
  Eigen::Vector3d mObstacleVel;
  Eigen::Matrix<double, 7,1> mBodyTorques_QPMAd_robot1;
  double mScale = 1;

  Eigen::MatrixXd mPinv_Jv_robot1; //Pseduo Inverse of Jacobian
  // Eigen::Vector3d mEE_dw_robot; //Current ee angular velocity (orientation velocity)
  multibody::SpatialAcceleration<double> mdJ_Aug_dq_spatial1; //Augmented Jacobian Derivative * dq
  Eigen::Matrix<double, 6, 1> mdJ_Aug_dq_robot1;
  Eigen::MatrixXd mFBL_Control_robot1;

  // Eigen::Vector3d dx; //Incremental distance for target
  double dx;
  Eigen::Vector3d mTargetPos; //Target Position
  Eigen::Vector3d mTargetVel; //Target Position
  Eigen::Vector3d mTargetRPY; // Target Orientation
  // math::RotationMatrix<double> mTargetRot; //Initial rotation matrix
  Eigen::Isometry3d mTargetRot; //Initial rotation matrix

  Eigen::MatrixXd mJv_robot1; // Linear Jacobian matrix.
  Eigen::MatrixXd mJw_robot1; // Angular Jacobian matrix.
  Eigen::Matrix<double, 6, 7> mJacobian; // Spatial Jacobian matrix.
  Eigen::Matrix3d mKp; //Defines gains for position in pd controller
  Eigen::Matrix3d mKv; //Define gains for velocity in pd controller

  Eigen::Matrix3d mKpOr; //Defines gains for angular position in pd controller
  Eigen::Matrix3d mKvOr; //Define gains for angular velocity in pd controller
  
  //Special matrices to load fator grpah
  Eigen::MatrixXd mF;
  Eigen::MatrixXd mG;
  Eigen::MatrixXd mP;
  math::RigidTransform<double> mEE_link_pose; // End effector pose
  multibody::SpatialVelocity<double> mEE_link_velocity; //end-effector velocity

  double counter = 1;
  int64_t counter2 = 0, check = 0;
};

int do_main() {
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_no_collision.urdf"));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base").body_frame());
  plant.Finalize();

  RobotPlanRunner runner(plant);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main() {
  return drake::examples::kuka_iiwa_arm::do_main();
}
