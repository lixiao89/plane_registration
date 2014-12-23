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

#define PI 3.1415926

    // TODO
    //  - add joint pos callback
    //  - test different types of ik solvers


    // Steps :
    //  - r: set WAM to ready state
    //  - h: start hybrid controller


#define DEBUG 0

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
        CTRL_HYBRID
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

        // class memeber
        init_kdl_robot();

        last_hpf_cmd = 0;
	err_gt.x = 0;
	err_gt.y = 0;
	err_gt.z = 0;
      }

      virtual ~WamMove(){}

      void update()
      {
        // do some update here
	
	// publish orientation error ground truth
	pub_err_gt_.publish( err_gt );

        // MODE
        if (ctrl_type_ == CTRL_HYBRID || ctrl_type_ == CTRL_START_HYBRID)
        {
    //      ROS_INFO("in hybrid control");
          // compute cmd frame
    //      static KDL::Vector inc(0, 0, 0.001);
    //      cmd_frame_.p += inc;

    //      if (cmd_frame_.p.z() < 0.6) {
    //        inc = KDL::Vector(0, 0, 0.001);
    //      }
    //      else if (cmd_frame_.p.z() > 0.8) {
    //        inc = KDL::Vector(0, 0, -0.001);
    //      }

          
#if 1
          Vector eevel;
          Vector eerot;
          // if( ctrl_type_ == CTRL_START_HYBRID )
          // 	{
          // 	  Vector eevel( 0, 0, 0 );
          // 	  Vector eerot( 0, 0, 0 );
          // 	}
          if( ctrl_type_ == CTRL_HYBRID )
        {
          eevel.z( -0.01 );
        }

          double vel_gain = 0.1;
          double rot_gain = 0.4;
          //Twist cmd_twist = twist_;
          Twist cmd_twist;
          cmd_twist.vel = vel_gain * eevel;
          cmd_twist.rot = rot_gain * eerot;

          // ignore some axes
          cmd_twist.vel(0) = 0.0;  // ignore x
          cmd_twist.rot(1) = 0.0;  // ignore y
          cmd_twist.rot(2) = 0.0;  // ignore z
          //cmd_twist.rot(0) = 0.0; //ignore x
        
          cmd_twist = cmd_frame_.M.Inverse(cmd_twist);
          cmd_frame_.Integrate(cmd_twist, 50);
#endif


          // force control
          double cmd_force = -4.0;
          double err_force = cmd_force - jr3_wrench_.force.z();
          double cmd_v = 0.0;

          cmd_v = -0.0001 * err_force;
          if (cmd_v > 0.0001) cmd_v = 0.0001;
          if (cmd_v < -0.0001) cmd_v = -0.0001;
          if (fabs(cmd_v) < 0.00005) cmd_v = 0.0;

          //------ for plane registration -------------
          std_msgs::Int32 plreg_msg;
          if( last_hpf_cmd > 0 || cmd_v < 0){
              plreg_msg.data = 1;
          }
          else
          {
              plreg_msg.data = 0;
          }
            
          pub_plane_reg_pts_.publish( plreg_msg );
          //----------------------------
          Frame force_frame;
          force_frame.p(2) = cmd_v;
          cmd_frame_ = cmd_frame_ * force_frame;

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

  // void cb_joy(const sensor_msgs::Joy &msg)
  // {
  //   using namespace KDL;
  //   twist_.vel(0) = msg.axes[2];
  //   twist_.vel(1) = msg.axes[1];
  //   twist_.vel(2) = -msg.axes[0];
  //   twist_.rot(0) = msg.axes[5];
  //   twist_.rot(1) = msg.axes[4];
  //   twist_.rot(2) = -msg.axes[3];
  // }

  void cb_jr3(const geometry_msgs::WrenchStamped &msg)
  {
    tf::wrenchMsgToKDL(msg.wrench, jr3_wrench_);
  }

  void cb_key(const std_msgs::String &msg)
  {
    ROS_INFO_STREAM("Key = " << msg.data);
    if (msg.data == "r")
      ctrl_ready();
    else if (msg.data == "ep")
      ctrl_errorpos();
    else if (msg.data == "sh")
      ctrl_start_hybrid();
    else if (msg.data == "h")
      ctrl_hybrid();
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
    double yerr = 20*PI/180;
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
    std::cout<<"x vector: "<<cutterX_ori<<std::endl;
    std::cout<<"y vector: "<<cutterY_ori<<std::endl;
    std::cout<<"z vector: "<<cutterZ_ori<<std::endl;

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

  void ctrl_hybrid()
  {
    // enter hybrid
    ctrl_type_ = CTRL_HYBRID;
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
    std::cout << "tip pose = \n" << tip_frame_ << std::endl;

    // sync cmd frame
    cmd_frame_ = tip_frame_;
  }

  // ros stuff
  ros::NodeHandle* node_;
  ros::Publisher pub_state_;
  ros::Publisher pub_jnt_cmd_;
  ros::Publisher pub_plane_reg_pts_;
  ros::Publisher pub_err_gt_;
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
