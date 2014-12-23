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
#include <math.h>   // PI
#include <eigen3/Eigen/Dense>
#define PI 3.1415926

#define DEBUG 0

using namespace KDL;

class PlaneRegistration
{
public:
  
  enum EST_STATE
  {
    IDLE,
    ESTIMATE
  };

  PlaneRegistration(ros::NodeHandle* node):
    node_(node),
    est_state(IDLE),
    process_data_num(50)
  {
    // pubs
    pub_state_ = node->advertise<std_msgs::String>("/dvrk_psm1/set_robot_state", 10);
    pub_roterr_est_ = node->advertise<geometry_msgs::Vector3>("/plreg/oriErr", 10);
    //   pub_err_gt_ = node->advertise<geometry_msgs::Vector3>("/plreg/errGroundTruth", 10);

    // subs
    sub_key_ = node->subscribe("/hybrid/key", 1, &PlaneRegistration::cb_key, this);
    sub_jr3_ = node->subscribe("/jr3/wrench", 1, &PlaneRegistration::cb_jr3, this);
    sub_states_ = node->subscribe("/gazebo/barrett_manager/wam/joint_states", 1, &PlaneRegistration::cb_jnt_states, this);
    sub_plreg_pts_ = node->subscribe("/plreg/ptslocation", 1, &PlaneRegistration::cb_ptsloc, this);
    
    init_kdl_robot();


    ptsloc = 0;
  }

  virtual ~PlaneRegistration(){}

  void update()
  {
    if ( est_state == ESTIMATE ){
      
      Eigen::Vector2d data_line( 0, 0 );
      double plane_ori;
      double cutter_ori;
      double yerr;
      fk_solver_->JntToCart( jnt_pos_, tip_frame_ );

      if ( ptsloc != 0 ){
	data_line = EvaluateMovingWindowLS( tip_frame_.p[2],
					    tip_frame_.p[0],
					    cutterZ,
					    cutterX,
					    process_data_num);	
      } 
      
      Vector cutterX_ori = tip_frame_.M.UnitX();
      Vector cutterZ_ori = tip_frame_.M.UnitZ();
      plane_ori = atan2( data_line(0)*10 + data_line(1), 10 );
      cutter_ori = atan2( cutterX_ori[0], cutterX_ori[2] );
      yerr = cutter_ori - plane_ori;

      std::cout<<"line: "<<plane_ori<<std::endl;

      // publish orientation error
      geometry_msgs::Vector3 ori_err;
      ori_err.x = 0;
      ori_err.y = yerr*180/PI;
      ori_err.z = 0;
      pub_roterr_est_.publish( ori_err );
    } 
  }

private:

  void cb_key(const std_msgs::String &msg){
    if (msg.data == "p")
      est_state = ESTIMATE;
  }

  void cb_jr3(const geometry_msgs::WrenchStamped &msg)
  {
    tf::wrenchMsgToKDL(msg.wrench, jr3_wrench_);
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


 void cb_ptsloc( const std_msgs::Int32 &msg )
    {
      ptsloc = msg.data;
    }

  Eigen::Vector2d EvaluateMovingWindowLS(const double& x,
					 const double& y,
					 std::vector<double>& xvec,
					 std::vector<double>& yvec,
					 int& windowSize){

    xvec.push_back(x);
    yvec.push_back(y);

    Eigen::Vector2d params;
    if(xvec.size() < 5){
      params<<0 ,0;
      return params;
    }

    if(xvec.size() > windowSize){
      xvec.erase(xvec.begin());
      yvec.erase(yvec.begin());
    }


    Eigen::MatrixXd X(xvec.size(),2);
    Eigen::VectorXd Y(yvec.size());
    for(int i = 0; i < xvec.size(); ++i)
      for(int j = 0; j < 2; ++j){
        if(j = 0){X(i,j) = xvec.at(i);}
        if(j = 1){X(i,j) = 1;}

        Y(i) = yvec.at(i);
      }

    params = (X.transpose()*X).inverse()*X.transpose()*Y;

    return params;

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
    //    jnt_cmd_.resize(num_jnts_);
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


  // ros stuff
  ros::NodeHandle* node_;
  ros::Publisher pub_state_;
  ros::Subscriber sub_jr3_;
  ros::Subscriber sub_states_;

  // joint states
  JntArray jnt_pos_;   // jnt pos current
  JntArray jnt_vel_;   // jnt vel current
  unsigned int num_jnts_;


  // frame
  Frame tip_frame_;    // cutter tip frame

  // sensor
  Wrench jr3_wrench_;
  
  //joint limits
  JntArray jnt_limit_min_;
  JntArray jnt_limit_max_;

  // chain
  Chain robot_;

  // KDL solver
  boost::shared_ptr<ChainFkSolverPos> fk_solver_;
  boost::shared_ptr<ChainIkSolverPos> ik_solver_;

  // -------- plane registration -----------
  ros::Publisher pub_roterr_est_;
  ros::Subscriber sub_plreg_pts_;
  ros::Subscriber sub_key_;
  int ptsloc;
  int process_data_num;
  EST_STATE est_state;
  
  // record cutter position data for registration error correction
  std::vector<double> cutterX;
  std::vector<double> cutterY;
  std::vector<double> cutterZ;
  //----------------------------------------
};



int main(int argc, char *argv[])
{
  using namespace KDL;

  std::cout << "kdl tester" << std::endl;
  ros::init(argc, argv, "plane_registration");
  ros::NodeHandle node;
  ros::Rate rate(30);   // 50 hz

  PlaneRegistration pr(&node);

  while (ros::ok())
  {
    ros::spinOnce();
    pr.update();
    rate.sleep();
  }

  return 0;
}
