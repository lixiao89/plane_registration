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
#include <eigen3/Eigen/LU>

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
      CTRL_HYBRID,
      CTRL_LOCAL_PROBING
    };

  enum NOISE_INJECTION
  {
    NO_NOISE,
    X_NOISE,
    Y_NOISE
  };
  
  WamMove(ros::NodeHandle* node);
  
  virtual ~WamMove();

  // main callback function
  void update();


private:

  // **********************  Plane Regristration *********************

  // turn angular error to reasonable velocity command
  double correction_vel(const double& err_angle );
  
  // performs a probing motion and returns a correction_frame in the cutter frame
  // returns true if error between cutter z and plane z < 0.2 degrees, return false otherwise
  // Inputs:
  // tip_frame: homogeneous matrix for current cutter pose
  // cmd_frame: command frame
  // last_hpf_command: velocity in the cutter z direction given by the hybrid controller at last time step
  // curr_hpf_command: ... at current time step (used to determine if current cutter position can be used for plane vector estimation, more useful for compliant surfaces )
  // starting_point: coordinate of the cutter when probing starts
  
  // Output:
  // cutter_linear_vel: gives the desired cutter velocity vector
  // correction_frame: correction frame in local coordinates
  bool local_probing( const Frame& tip_frame,
		      const Frame& cmd_frame,
		      const double& last_hpf_cmd,
		      double& curr_hpf_cmd,
		      const Vector& starting_point,
		      Vector& cutter_linear_vel,
		      Frame& correction_frame );
  
  // performs correction during execution of the cutting task (align cutter x axis)
  bool correct_x_during_cutting( const Frame& tip_frame,
				 const double& last_hpf_cmd,
				 double& curr_hpf_cmd,
				 Frame& correction_frame );

  // perform correction during cutting ( align cutter y axis )
  void correct_y_during_cutting( const Frame& tip_frame,
				 Frame& correction_frame );

  // Returns a frame (homogeneous matrix) which multiplied by local_frame aligns the local frame's 'axis' (can be 'x', 'y', 'z') axis with goal_vector (specified in the base_frame)  
  void find_correction_frame( const Frame& local_frame,
			      const char& axis,
			      const Eigen::Vector3d& goal_vector,
			      Frame& correction_frame,
			      double& relative_angle);
  
// ********************** Subscriber Callbacks ************************
 
  void cb_jr3(const geometry_msgs::WrenchStamped &msg);
  
  void cb_key(const std_msgs::String &msg);
 
  void cb_jnt_states(const sensor_msgs::JointState &msg);
  
  
  // from plane_registration
  void cb_oriErr(const geometry_msgs::Vector3 &msg);
  
  // callback for subscriber to estimated plane normal in the initial probing stage 
  void cb_probing_est(const geometry_msgs::Vector3 &msg);
  
  void cb_traj_unit_vector(const geometry_msgs::Vector3 &msg);

 // **************** Robot Initialization And Control States *************
    
  void init_kdl_robot();
  
  void ctrl_ready();
  
  void ctrl_errorpos();
  
  void ctrl_start_hybrid();
  
  void ctrl_local_probing();
  
  void ctrl_hybrid(); 


  // *************  FIELD VARIABLES *************************************
 

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

  // indicate if initialization is ready
  bool initialize_ready;

  // for show purpose only !
  NOISE_INJECTION inject_noise;
  NOISE_INJECTION inject_noise_last;

  // --------- for plane registration ----------------
  std_msgs::Int32 plreg_pts;
  double last_hpf_cmd;
  geometry_msgs::Vector3 err_gt;
 
  // store orientation error information published from plane_registration node
  Vector oriErr;
 
  // temporary printout for matlab
  const char* path;
  std::ofstream ofs;

  Vector start_point;
 
  ros::Publisher pub_plane_reg_pts_;
  ros::Publisher pub_err_gt_;
  // cues the plane_registration node if the current process is local probing or cutting
  ros::Publisher pub_plreg_trigger_;
 
  ros::Subscriber sub_roterr_;



  // For Correction During Local Probing

  ros::Subscriber sub_est_normal; // for initial probing step

  Eigen::Vector3d probing_est_normal;
 
  bool first_vector_ready;
  bool second_vector_ready;
  bool ascend_ready;
 
  // For Correction During Cutting

  ros::Subscriber sub_traj_unit_vector_;
  Eigen::Vector3d traj_unit_vector;
  bool start_correcting_x;

  double start_hybrid_time;

  // for telling plane_registration node if current cutter position can be used to estimate plane vector and what use of current motion.
  // corresponeds to 'ptsloc'
  std_msgs::Int32 plreg_msg;

  
  // if err_pose = 1, give rotation error around x increment of +3 degrees
  // if err_pose = -1, ..........................x............. -3 degrees
  // if err_pose = 2, ......................... y ............. +3 degrees,
  // if err_pose = -2, ....................... y .............. -3 dgrees
  int err_pose; 

  // plreg_process_cue = 1: local probing
  // plreg_process_cue = 2: cutting
  std_msgs::Int32 plreg_process_cue;
 // ----------------------------------------------------

};

