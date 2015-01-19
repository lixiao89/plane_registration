#include <iostream>
#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl_conversions/kdl_msg.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <eigen3/Eigen/Dense>

#include <math.h>   // PI
#include <vector>
#define PI 3.1415926


#define DEBUG 0

// base frame  ^ x                           cutter frame     / y
//             |                                             /
//             |                                            /
//             |-------> z                    x <----------|
//            /                                            |
//           /                                             |
//          /                                              |
//         V y                                             v z




using namespace KDL;

class WamMove
{
public:
  enum CONTROL_TYPE
    {
      CTRL_IDLE,
      CTRL_READY,
      CTRL_ERROR_POSE,
      CTRL_START_HYBRID,
      CTRL_START_CORRECT_X,
      CTRL_START_CORRECT_Y,
      CTRL_HYBRID,
      CTRL_CORRECT,
      CTRL_SINUSOID,
      CTRL_LOCAL_PROBING
    };
  
  WamMove(ros::NodeHandle* node):
    node_(node),
    ctrl_type_(CTRL_IDLE)
  {
    // pubs 
    pub_state_ = node->advertise<std_msgs::String>("/dvrk_psm1/set_robot_state", 10);
    pub_jnt_cmd_ = node->advertise<trajectory_msgs::JointTrajectoryPoint>("/gazebo/traj_rml/joint_traj_point_cmd", 1);
    pub_plane_reg_pts_ = node->advertise<std_msgs::Int32>("/plreg/ptlocation", 10);
    pub_err_gt_ = node->advertise<geometry_msgs::Vector3>("/plreg/errGroundTruth", 10);
    
    // subs
    //sub_joy_ = node->subscribe("/spacenav/joy", 1, &Control::cb_joy, this);
    sub_jr3_ = node->subscribe("/jr3/wrench", 1, &WamMove::cb_jr3, this);
    sub_key_ = node->subscribe("/hybrid/key", 1, &WamMove::cb_key, this);
    sub_states_ = node->subscribe("/gazebo/barrett_manager/wam/joint_states", 1, &WamMove::cb_jnt_states, this);
    sub_roterr_ = node->subscribe("/plreg/oriErr", 1, &WamMove::cb_oriErr, this);
    sub_est_normal = node->subscribe("/plreg/local_probing_est", 1, &WamMove::cb_probing_est, this);

    // class memeber
    init_kdl_robot();
    
    last_hpf_cmd = 0;
    err_gt.x = 0;
    err_gt.y = 0;
    err_gt.z = 0;
    
    
    // plane registration
    path = "/home/xli4217/wamdata.txt";
    ofs.open( path, std::ofstream::out );

    probing_est_normal << 0, 0, 0;
  }
  
  virtual ~WamMove(){}
  
  void update()
  {
    // do some update here
    
    // publish orientation error ground truth
    pub_err_gt_.publish( err_gt );
    
    // calculate forward kinematics
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
    

    // MODE
    if (ctrl_type_ == CTRL_HYBRID || ctrl_type_ == CTRL_START_HYBRID || ctrl_type_ == CTRL_CORRECT || ctrl_type_ == CTRL_SINUSOID || ctrl_type_ == CTRL_START_CORRECT_X || ctrl_type_ == CTRL_START_CORRECT_Y || ctrl_type_ == CTRL_LOCAL_PROBING )
      {
	
        
	// both initialized to zero
	Vector eevel;
	Vector eerot;
	
	if( ctrl_type_ == CTRL_HYBRID )
	  {
	    // eevel.z( -0.01 );
	    
	    // move in the x direction in cutter frame
	    eevel = cmd_frame_.M.UnitX();
	  }	
	
	
	
	// force control
	
	double cmd_force;
	if ( ctrl_type_ == CTRL_START_HYBRID || ctrl_type_ == CTRL_HYBRID ){
	  cmd_force = -4.0;
	}
	if ( ctrl_type_ == CTRL_SINUSOID ){
	  double currTime = ros::Time::now().toSec();
	  double freq = 10;
	  cmd_force = -2.0 * sin( 2*PI*freq*currTime ) - 2.0;
	  //std::cout<<cmd_force<<std::endl;
	  ofs << jr3_wrench_.torque.x() << std::endl; 
	}
	double err_force = cmd_force - jr3_wrench_.force.z();
	double cmd_v = 0.0;
	
	cmd_v = -0.0001 * err_force;
	if (cmd_v > 0.0001) cmd_v = 0.0001;
	if (cmd_v < -0.0001) cmd_v = -0.0001;
	if (fabs(cmd_v) < 0.00005) cmd_v = 0.0;
	
        

	//----------------for plane registration --------------------

	Frame correction_frame( Frame::Identity() );

	// Local Probing step
	if ( ctrl_type_ == CTRL_LOCAL_PROBING ){
	  
	  
	  bool first_vector_ready = false;
	  bool second_vector_ready = false;

	  // collect information for first vector
	  if ( first_vector_ready == false ){
	    double current_cutter_z = tip_frame_.p[2];
	    eevel = cmd_frame_.M.UnitX();
	    if ( last_hpf_cmd > 0 || cmd_v < 0 ){
	      plreg_msg.data = 1;
	    }
	    else{
	      plreg_msg.data = 0;
	    }
	    
	    if ( current_cutter_z > start_point[2] - 0.027 && current_cutter_z < start_point[2] - 0.03 ){
	      first_vector_ready == true;
	      plreg_msg.data = -1;
	    }
	  }
	  
	  // collect information for second vector
	  if ( first_vector_ready == true && second_vector_ready == false ){
	    double current_cutter_y = tip_frame_.p[1];
	    eevel = cmd_frame_.M.UnitY();
	    if ( last_hpf_cmd > 0 || cmd_v < 0 ){
	      plreg_msg.data = 2;
	    }
	    else{
	      plreg_msg.data = 0;
	    }
	    
	    if ( current_cutter_y > start_point[1] - 0.027 && current_cutter_y < start_point[1] - 0.03 ){
	      second_vector_ready == true;
	      
	    }
	  }

	  // both vectors ready, start calculating error and perform correction
	  if ( first_vector_ready == true && second_vector_ready == true ){
	    
	    std::cout << "start correcting ..." << std::endl;

	    plreg_msg.data = 3;
	    
	    Vector unitZ = tip_frame_.M.UnitZ();
	    Eigen::Vector3d cutter_z( unitZ[0], unitZ[1], unitZ[2] );
	    
	    Eigen::Vector3d rot_axis = cutter_z.cross( probing_est_normal );

	    // Covert rot_axis from base frame to cutter frame
	    Eigen::Vector3d rot_axis_cutter_frame = 

	    double rel_angle = cutter_z.dot( probing_est_normal ) / ( cutter_z.norm() * probing_est_normal.norm() );

	    if ( rel_angle > 0.2 ){

	      Eigen::Matrix3d skew;
	      skew << 0, -rot_axis(3), rot_axis(2),
		rot_axis(3), 0, -rot_axis(1),
		-rot_axis(2), rot_axis(1), 0;
	      
	      double rel_angle_scaled = 0.001 * rel_angle;

	      // Rodrigues Formula
	      Eigen::Matrix3d correction_rotation_temp = Eigen::Matrix3d::Identity() + skew*sin( rel_angle_scaled ) + skew*skew*( 1 - cos( rel_angle_scaled )*cos( rel_angle_scaledlam ) );

	      Rotation correction_rotation;
	      for ( int i = 0; i < 3; ++i )
		for ( int j = 0; j < 3; ++j)
		  correction_rotation( i, j ) = correction_rotation_temp( i, j );
	    
	      correction_frame.M = correction_rotation;
	    }
	    else{
	      correction_frame = Frame::Identity();
	      std::cout << "correction complete!" << std::endl;
	    }
	    
	  }
	  
	  pub_plane_reg_pts_.publish( plreg_msg );
	  
	} 

	// // for estimating rotation error around y
	// std_msgs::Int32 plreg_msg;
	// if( last_hpf_cmd > 0 || cmd_v < 0){	  
	//   // tells plane_registration node to estimate for rotation error around X
	//   if ( ctrl_type_ == CTRL_START_CORRECT_X )     {  plreg_msg.data = 1; }
	//   // tells plane_registration node to estimate for rotation error around Y
	//   if ( ctrl_type_ == CTRL_START_CORRECT_Y )     {  plreg_msg.data = 2; }
	// }
	// else{
	//     plreg_msg.data = 0;
	// }
	// pub_plane_reg_pts_.publish( plreg_msg );
	

	// Frame correction_frame( Frame::Identity() );
	// if ( ctrl_type_ == CTRL_START_CORRECT_X ){
	    
	//     // Move along the cutter Y direction to obtain a vector on cutting plane that is in-plane with cutter X vector
	//     eevel = cmd_frame_.M.UnitY();
	//     double current_cutter_y = tip_frame_.p[1];
	    
	//     // when cutter moved 0.2m, stop and start correcting
	//     if ( current_cutter_y < start_point[1] - 0.03 ){ 
	    
	//       eevel = Vector::Zero();
	      
	//       Rotation correction_rotation;
	      
	//       correction_rotation.DoRotX( correction_vel( oriErr(0) ) );
	      
	//       correction_frame.M = correction_rotation;

	//       std::cout << "Data Collected, Start Correcting ...." << std::endl;

	//     }
	// }
	// else if ( ctrl_type_ == CTRL_START_CORRECT_Y ){
	    
	//     // Move along the cutter X direction to obtain a vector on cutting plane that is in-plane with cutter X vector
	//   eevel = cmd_frame_.M.UnitX();
	//     double current_cutter_z = tip_frame_.p[2]; // center of cutter in base frame

	//     //std::cout<<current_cutter_z<<","<<start_point[2]<<std::endl;
	    
	//     // when cutter moved 0.2m, stop and start correcting
	//     if ( current_cutter_z < start_point[2] - 0.03 ){ 
	//       eevel = Vector::Zero();
	      
	//       Rotation correction_rotation;
	      
	//       correction_rotation.DoRotY( correction_vel( oriErr(1) ) );
	      
	//       correction_frame.M = correction_rotation;

	//       std::cout << "Data Collected, Start Correcting ...." << std::endl;
	//     }
	    
	//   }
	// else{
	//   correction_frame = Frame::Identity();
	// }
	
	//-------------------------------------------------------------


	double vel_gain = 0.001;
	double rot_gain = 0.4;
	
	Twist cmd_twist;
	cmd_twist.vel = vel_gain * eevel;
	cmd_twist.rot = rot_gain * eerot;
	// lock all rotations in base frame
	cmd_twist.rot(0) = 0.0; 
	cmd_twist.rot(1) = 0.0;  
          //cmd_twist.rot(2) = 0.0;  
          //cmd_twist.rot(0) = 0.0; //ignore x
        
	cmd_twist = cmd_frame_.M.Inverse(cmd_twist);
	cmd_frame_.Integrate(cmd_twist, 50);
	

	Frame force_frame;
	force_frame.p(2) = cmd_v;
	cmd_frame_ = cmd_frame_ * force_frame * correction_frame;

	// do ik
	ik_solver_->CartToJnt(jnt_pos_, cmd_frame_, jnt_cmd_);
	// std::cout << "cmd: " << jnt_cmd_.data.transpose() << std::endl;

	// send to robot
	trajectory_msgs::JointTrajectoryPoint msg_jnt_cmd;
	msg_jnt_cmd.positions.resize(num_jnts_);
	for (size_t i = 0; i < num_jnts_; i++)
          {
            msg_jnt_cmd.positions[i] = jnt_cmd_(i);
	  }
	pub_jnt_cmd_.publish(msg_jnt_cmd);
      }
  }

private:

  double correction_vel(const double& err_angle )
  {
    double cmd_v = -0.0001 * err_angle;
    if (cmd_v > 0.0001) cmd_v = 0.0001;
    if (cmd_v < -0.0001) cmd_v = -0.0001;
    if (fabs(cmd_v) < 0.00005) cmd_v = 0.0;

    return cmd_v;
  }

 
  void cb_jr3(const geometry_msgs::WrenchStamped &msg)
  {
    tf::wrenchMsgToKDL(msg.wrench, jr3_wrench_);
  }

  void cb_key(const std_msgs::String &msg)
  {
    ROS_INFO_STREAM("Key = " << msg.data);
    if (msg.data == "r")
      ctrl_ready();
    else if (msg.data == "ep")// move to error position
      ctrl_errorpos();
   
    else if (msg.data == "sh")// moving to position with only hybrid control
      ctrl_start_hybrid();
    
    else if ( msg.data == "cx" )
      ctrl_start_correct_x();

    else if ( msg.data == "cy" )
      ctrl_start_correct_y();
    
    else if (msg.data == "ss")
      ctrl_start_sinusoid();
      
    else if (msg.data == "h")// start moving with control
      ctrl_hybrid();
   
    else if (msg.data == "c")// start correcting
      ctrl_start_correcting();

    else if (msg.data == "lp") // start local probing
      ctrl_local_probing();
    
    else
      ROS_ERROR("Unsupported Commands");
  }

  void cb_jnt_states(const sensor_msgs::JointState &msg)
  {
    if (msg.position.size() != num_jnts_) {
      ROS_ERROR_STREAM("Joint states size mismatch  " << msg.position.size());
    }

    for (size_t i = 0; i < num_jnts_; i++) {
      jnt_pos_(i) = msg.position.at(i);
      jnt_vel_(i) = msg.velocity.at(i);
    }

#if DEBUG
    std::cout << "pos: " << jnt_pos_.data.transpose() << std::endl;
    std::cout << "vel: " << jnt_vel_.data.transpose() << std::endl;
    std::cout << std::endl;
#endif
  }

   
  // from plane_registration
  void cb_oriErr(const geometry_msgs::Vector3 &msg)
  {
    oriErr(0) = msg.x;
    oriErr(1) = msg.y;
    oriErr(2) = msg.z;
  }
      
  // callback for subscriber to estimated plane normal in the initial probing stage 
  void cb_probing_est(const geometry_msgs::Vector3 &msg){
    if( plreg_msg.data == 3 ){
    probing_est_normal(0) = msg.x;
    probing_est_normal(1) = msg.y;
    probing_est_normal(2) = msg.z;
    }
  }

  void init_kdl_robot()
  {
    std::string urdf;
    node_->param("/robot_description", urdf, std::string());

    urdf::Model urdf_model;
    urdf_model.initString(urdf);

    // get tree from urdf string
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromString(urdf, my_tree)) {
      ROS_ERROR("Failed to construct kdl tree");
      return;
    }
    std::string rootLink = "wam/base_link";
    std::string tipLink = "wam/cutter_tip_link";
    if (!my_tree.getChain(rootLink, tipLink, robot_))
      {
	ROS_ERROR("Failed to get chain from kdl tree, check root/rip link");
	return;
      }
    num_jnts_ = robot_.getNrOfJoints();

    // resize joint states
    jnt_pos_.resize(num_jnts_);
    jnt_vel_.resize(num_jnts_);
    jnt_cmd_.resize(num_jnts_);
    jnt_limit_min_.resize(num_jnts_);
    jnt_limit_max_.resize(num_jnts_);

    // get jnt limits from URDF model
    size_t i_jnt = 0;
    for (size_t i = 0; i < robot_.getNrOfSegments(); i++) {
      Joint jnt = robot_.getSegment(i).getJoint();
      if (jnt.getType() != Joint::None) {
        jnt_limit_min_(i_jnt) = urdf_model.joints_[jnt.getName()]->limits->lower;
        jnt_limit_max_(i_jnt) = urdf_model.joints_[jnt.getName()]->limits->upper;
        i_jnt++;
      }
    }

    // print limit for debugging
    std::cout << "min: " << jnt_limit_min_.data.transpose() << std::endl;
    std::cout << "max: " << jnt_limit_max_.data.transpose() << std::endl;

    // KDL Solvers
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_));
    ik_solver_.reset(
		     new KDL::ChainIkSolverPos_LMA(
						   robot_,
						   1E-5,
						   500,
						   1E-15));
  }

  void ctrl_ready()
  {
    ctrl_type_ = CTRL_IDLE;

    // send out ready
    trajectory_msgs::JointTrajectoryPoint msg_jnt_cmd;
    msg_jnt_cmd.positions.resize(num_jnts_);
    msg_jnt_cmd.positions[0] = 0.0;
    msg_jnt_cmd.positions[1] = -M_PI_2;
    msg_jnt_cmd.positions[2] = 0.0;
    msg_jnt_cmd.positions[3] = M_PI_2;
    msg_jnt_cmd.positions[4] = 0.0;
    msg_jnt_cmd.positions[5] = -1.2;
    msg_jnt_cmd.positions[6] = 0.0;
    pub_jnt_cmd_.publish(msg_jnt_cmd);
  }

  void ctrl_errorpos()
  {
    ctrl_type_ = CTRL_ERROR_POSE;

    fk_solver_->JntToCart( jnt_pos_, tip_frame_ );

    // sync cmd frame
    cmd_frame_ = tip_frame_;

    double xerr = 0*PI/180;
    double yerr = 5*PI/180;
    double zerr = 0*PI/180;

    Rotation roterr;
    roterr.DoRotX( xerr );
    roterr.DoRotY( yerr );
    roterr.DoRotZ( zerr );

    Frame err_frame_( roterr );
    
    cmd_frame_ = cmd_frame_*err_frame_;
    
    // do ik
    ik_solver_->CartToJnt(jnt_pos_, cmd_frame_, jnt_cmd_);
    //std::cout << "cmd: " << jnt_cmd_.data.transpose() << std::endl;

    // send to robot
    trajectory_msgs::JointTrajectoryPoint msg_jnt_cmd;
    msg_jnt_cmd.positions.resize(num_jnts_);
    for (size_t i = 0; i < num_jnts_; i++)
      {   
        msg_jnt_cmd.positions[i] = jnt_cmd_(i);
      }   
    pub_jnt_cmd_.publish(msg_jnt_cmd);

    // unit vector of x, y, z directions of tip frame
    Vector cutterX_ori = cmd_frame_.M.UnitX();
    Vector cutterY_ori = cmd_frame_.M.UnitY();
    Vector cutterZ_ori = cmd_frame_.M.UnitZ();


    //std::cout<<"tip frame: "<<tip_frame_<<std::endl;
    // std::cout<<"x vector: "<<cutterX_ori<<std::endl;
    // std::cout<<"y vector: "<<cutterY_ori<<std::endl;
    // std::cout<<"z vector: "<<cutterZ_ori<<std::endl;

    err_gt.x = 0;
    err_gt.y = atan2( cutterX_ori[0], cutterX_ori[2] )*180/PI;
    err_gt.z = 0;
  }     


  void ctrl_start_hybrid()
  {
    ctrl_type_ = CTRL_START_HYBRID;
    
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
   
    // sync cmd frame
    cmd_frame_ = tip_frame_;
  }

  void ctrl_start_correct_x()
  {
    ctrl_type_ = CTRL_START_CORRECT_X;
    
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
   
    // sync cmd frame
    cmd_frame_ = tip_frame_;
    
    start_point[0] = tip_frame_.p[0];
    start_point[1] = tip_frame_.p[1];
    start_point[2] = tip_frame_.p[2];
  }


  void ctrl_start_correct_y()
  {
    ctrl_type_ = CTRL_START_CORRECT_Y;
    
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
   
    // sync cmd frame
    cmd_frame_ = tip_frame_;
    start_point[0] = tip_frame_.p[0];
    start_point[1] = tip_frame_.p[1];
    start_point[2] = tip_frame_.p[2];
  }


void ctrl_local_probing()
  {
    ctrl_type_ = CTRL_LOCAL_PROBING;
    
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
   
    // sync cmd frame
    cmd_frame_ = tip_frame_;
    start_point[0] = tip_frame_.p[0];
    start_point[1] = tip_frame_.p[1];
    start_point[2] = tip_frame_.p[2];
  }


  void ctrl_start_sinusoid()
  {
    // enter hybrid
    ctrl_type_ = CTRL_SINUSOID;
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
    std::cout << "tip pose = \n" << tip_frame_ << std::endl;

    // sync cmd frame
    cmd_frame_ = tip_frame_;
  }


  void ctrl_hybrid()
  {
    // enter hybrid
    ctrl_type_ = CTRL_HYBRID;
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
    std::cout << "tip pose = \n" << tip_frame_ << std::endl;

    // sync cmd frame
    cmd_frame_ = tip_frame_;
  }


  void ctrl_start_correcting()
  {
    // enter hybrid
    ctrl_type_ = CTRL_CORRECT;
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
    std::cout << "tip pose = \n" << tip_frame_ << std::endl;

    // sync cmd frame
    cmd_frame_ = tip_frame_;
  } 



  double average_error( const double err_angle, std::vector<double>& err_vector, const int window_size )
  {
    err_vector.push_back( err_angle );
    if ( err_vector.size() > window_size ){
      err_vector.erase( err_vector.begin() );
    }
	
    double sum = 0;
    for ( int i = 0; i < err_vector.size(); ++i ){
      sum += err_vector.at(i);
    }

    double avg_error = sum / window_size;
    return avg_error;
  }

  // ros stuff
  ros::NodeHandle* node_;
  ros::Publisher pub_state_;
  ros::Publisher pub_jnt_cmd_;
  ros::Subscriber sub_joy_;
  ros::Subscriber sub_jr3_;
  ros::Subscriber sub_key_;
  ros::Subscriber sub_states_;
 

  // joint states
  JntArray jnt_pos_;   // jnt pos current
  JntArray jnt_vel_;   // jnt vel current
  JntArray jnt_cmd_;   // jnt pos command
  unsigned int num_jnts_;

  // joint limits
  JntArray jnt_limit_min_;
  JntArray jnt_limit_max_;

  // frame
  Frame tip_frame_;    // cutter tip frame
  Frame cmd_frame_;    // cmd cutter tip frame

  // sensor
  Wrench jr3_wrench_;
  Wrench cmd_wrench_;
  Twist twist_;

  // chain
  Chain robot_;

  // KDL solver
  boost::shared_ptr<ChainFkSolverPos> fk_solver_;
  boost::shared_ptr<ChainIkSolverPos> ik_solver_;

  // ctrl mode
  CONTROL_TYPE ctrl_type_;

  // --------- for plane registration ----------------
  std_msgs::Int32 plreg_pts;
  double last_hpf_cmd;
  geometry_msgs::Vector3 err_gt;
  // store orientation error information published from plane_registration node
  Vector oriErr;
  // store error and calculate average
  std::vector<double> avg_err_y;

  // temporary printout for matlab
  const char* path;
  std::ofstream ofs;

  Vector start_point;
 
  ros::Publisher pub_plane_reg_pts_;
  ros::Publisher pub_err_gt_;
 
  ros::Subscriber sub_roterr_;
  ros::Subscriber sub_est_normal; // for initial probing step

  Eigen::Vector3d probing_est_normal;

  std_msgs::Int32 plreg_msg;
 // ----------------------------------------------------

};



int main(int argc, char *argv[])
{
  using namespace KDL;

  std::cout << "kdl tester" << std::endl;
  ros::init(argc, argv, "kdl_parser_node");
  ros::NodeHandle node;
  ros::Rate rate(100);   // 50 hz

  WamMove ctrl(&node);

  while (ros::ok())
    {
      ros::spinOnce();
      ctrl.update();
      rate.sleep();
    }

  return 0;
}
