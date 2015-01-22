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

  PlaneRegistration( ros::NodeHandle* node );

  ~PlaneRegistration();

  // main loop callback
  void update();
 
 private:


  // Process keyboard events
  void cb_key(const std_msgs::String &msg);

  //****************** Subscriber Callbacks ***********************
  void cb_jr3(const geometry_msgs::WrenchStamped &msg);
 
  void cb_jnt_states(const sensor_msgs::JointState &msg);


  void cb_ptsloc( const std_msgs::Int32 &msg );
  
  void cb_plreg_trigger( const std_msgs::Int32 &msg );

  //******************** Plane Registration ************************



  // In LOCAL PROBING stage, if assume base frame and plane doesn't move during this step, cutter is commanded to move in two directions each of 0.2m, this gives two vectors which cross product is the normal of the plane. The cross product of this normal with the cutter normal provides a roataion axis for the two normals to align
  void local_probing(); 
		      

  // during cutting phase, estimate and publish a unit vector along the trajectory of the cutter (used for correction around cutter y axis)  
  void estimate_during_cutting();

  // Estimate a average unit vector according to the cutter's trajectory
  Eigen::Vector3d estimate_trj_unit_vector( Eigen::Vector3d& start_point,
					    Eigen::Vector3d& curr_point,
					    Eigen::MatrixXd& unit_vector_history);


  void remove_matrix_column(Eigen::MatrixXd& matrix, unsigned int colToRemove);


  // Calculate average of 'data' collected over time using a window size of 'window_size'
  double average( const double data,
		  std::vector<double>& data_vector,
		  const int window_size );
 
  // Evaluate a moving window least square and return the slope and intercept of the 2d line
  Eigen::Vector2d EvaluateMovingWindowLS(const double& x,
					 const double& y,
					 std::vector<double>& xvec,
					 std::vector<double>& yvec,
					 int& windowSize);

  // initialize robot for kinematics calculation
  void init_kdl_robot();
 


  //****************** PRIVATE FIELD VARIABLES *********************

  // ros stuff
  ros::NodeHandle* node_;
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
  int process_data_num;
  
  // record cutter position data for registration error correction
  std::vector<double> cutterX;
  std::vector<double> cutterY;
  std::vector<double> cutterZ;

  std::vector<double> err_vectorX;
  std::vector<double> err_vectorY;

  std::vector<double> plane_ori_vectorX;
  std::vector<double> plane_ori_vectorY;
  EST_STATE est_state;
  
  int reset_start_point;
  Eigen::Vector3d startPoint;


  // Indicates the meaning of the current cutter motion
  // during initial probing step: 
  // ptsloc = 0; data not suitable for estimation
  // ptsloc = 1: obtaining first vector on plane
  // ptsloc = 2: obtaining second vector on plane
  // ptsloc = 3: finished with motion, start calculating plane normal
  // ptsloc = -1: transition from first vector motion to second, reinitial startPoint
  
  // during cutting:
  // ptsloc = 1: data can be used for estimation
  int ptsloc;
  int ptsloclast;
  double error_last;

  // subscribed from wam_control
  // plreg_process_cue = 1: local probing
  // plreg_process_cue = 2: cutting
  int plreg_process_cue;
  int plreg_process_cue_last;
  // for local probing step
  ros::Publisher pub_local_probing_normal_est_;
  
  Eigen::MatrixXd unit_vector_history_1;
  Eigen::MatrixXd unit_vector_history_2;
  
  Eigen::Vector3d plane_vec_1;
  Eigen::Vector3d plane_vec_2;

  geometry_msgs::Vector3 local_probing_est;

  // for during cutting step
  ros::Subscriber sub_plreg_trigger_;

  ros::Publisher pub_traj_unit_vector_;

  // estimated unit plane vector
  Eigen::Vector3d trajectory_unit_vector;
  // used when current cutter position can't be used for estimation
  Eigen::Vector3d trajectory_unit_vector_last;
  // 3xn matrix containing historical estimated unit plane vectors for averaging
  Eigen::MatrixXd trajectory_unit_vector_history;
  // message to publish
  geometry_msgs::Vector3 trajectory_unit_vector_msg;

  
  double last_reset_start_point_time;
  // used to reset start points
  Eigen::MatrixXd curr_point_history;
  //----------------------------------------
};

