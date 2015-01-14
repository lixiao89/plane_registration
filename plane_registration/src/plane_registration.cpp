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

  PlaneRegistration( ros::NodeHandle* node ): 
    node_(node),
    est_state(IDLE),
    process_data_num(50),
    reset_start_point( 0 ),
    error_last( 0 ),
    ptsloclast( 0 )
  {
    // pubs 
    pub_roterr_est_ = node->advertise<geometry_msgs::Vector3>("/plreg/oriErr", 10);

    // subs
    sub_key_ = node->subscribe("/hybrid/key", 1, &PlaneRegistration::cb_key, this);
    sub_jr3_ = node->subscribe("/jr3/wrench", 1, &PlaneRegistration::cb_jr3, this);
    sub_states_ = node->subscribe("/gazebo/barrett_manager/wam/joint_states", 1, &PlaneRegistration::cb_jnt_states, this);
    sub_plreg_pts_ = node->subscribe("/plreg/ptlocation", 1, &PlaneRegistration::cb_ptsloc, this);
    
    init_kdl_robot();

    // for plane registration
    ptsloc = 0;
    plane_ori_vectorX.push_back( 0 );
    plane_ori_vectorY.push_back( 0 );
  }

  virtual ~PlaneRegistration(){}

  void update()
  {
    if ( est_state == ESTIMATE ){
      orientation_error_estimate();
    } 
  }

private:


  // Process keyboard events
  void cb_key(const std_msgs::String &msg){
    if (msg.data == "p"){
      est_state = ESTIMATE;
      reset_start_point = 1;
    }
    
  }


  // ---------- Subscriber Callbacks ----------------
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

    if ( reset_start_point == 1 ){
      fk_solver_->JntToCart( jnt_pos_, tip_frame_ );
      startPoint << tip_frame_.p[0], tip_frame_.p[1], tip_frame_.p[2];
      reset_start_point = 0;
    }

  }


 void cb_ptsloc( const std_msgs::Int32 &msg )
    {
      ptsloc = msg.data;
    }


  // FUNCTION RESPONSIBLE FOR ESTIMATION
  void orientation_error_estimate(){
    
    // stores coefficients for the estimated line on the cutting surface
    Eigen::Vector2d data_line( 0, 0 );
    geometry_msgs::Vector3 ori_err;

    fk_solver_->JntToCart( jnt_pos_, tip_frame_ );

    // estimate x rotation error if ptsloc = 1, ( comparing local y axis )
    if ( ptsloc == 1 ){
 
      Eigen::Vector2d start_point( startPoint(1), startPoint(0) );
      Eigen::Vector2d curr_point( tip_frame_.p[1], tip_frame_.p[0] );

      double plane_ori = average_plane_orientation1D( start_point,
						      curr_point,
						      plane_ori_vectorX,
						      process_data_num );
        
      Vector cutterY_ori = tip_frame_.M.UnitY();
      double cutter_y_angle = atan2( cutterY_ori[0], cutterY_ori[2] );
  
      if ( cutter_y_angle < 0 ) { cutter_y_angle += 2*PI; }

      double error_before_avg = plane_ori - cutter_y_angle;

      double error_after_avg = average( error_before_avg, err_vectorX, 20 );
	
      ori_err.x = error_after_avg*180 / PI;
      ori_err.y = 0;
      ori_err.z = 0;
      pub_roterr_est_.publish( ori_err );
      
      std::cout<<"x error: "<< ori_err.x << std::endl;
 
      error_last = ori_err.x;
      ptsloclast = 1;
   
    }     
    // estimate y rotation error if ptsloc = 2 ( comparing local x axis )
    if ( ptsloc == 2 ){
     
      Eigen::Vector2d start_point( startPoint(2), startPoint(0) );
      Eigen::Vector2d curr_point( tip_frame_.p[2], tip_frame_.p[0] );

      double plane_ori = average_plane_orientation1D( start_point,
						      curr_point,
						      plane_ori_vectorY,
						      process_data_num );
        
      Vector cutterX_ori = tip_frame_.M.UnitX();
      double cutter_x_angle = atan2( cutterX_ori[0], cutterX_ori[2] );
  
      if ( cutter_x_angle < 0 ) { cutter_x_angle += 2*PI; }
      
      double error_before_avg = plane_ori - cutter_x_angle;

      double error_after_avg = average( error_before_avg, err_vectorY, 20 );


      ori_err.x = 0;
      ori_err.y = error_after_avg*180 / PI;
      ori_err.z = 0;
      pub_roterr_est_.publish( ori_err );
      
      std::cout<<"y error: "<< ori_err.y << std::endl;
      
      error_last = ori_err.y;
      ptsloclast = 2;
      
      // data_line = EvaluateMovingWindowLS( tip_frame_.p[2],
      // 					  tip_frame_.p[0],
      // 					  cutterZ,
      // 					  cutterX,
      // 					  process_data_num);	
     
      // Vector cutterX_ori = tip_frame_.M.UnitX();
      // plane_ori = atan2( data_line(0)*10 + data_line(1), 10 );
      // cutter_ori = atan2( cutterX_ori[0], cutterX_ori[2] );
      // yerr = cutter_ori - plane_ori;
      
      // // publish orientation error
      // geometry_msgs::Vector3 ori_err;
      // ori_err.x = 0;
      // if ( yerr*180/PI > 0 )     { ori_err.y = 180 - yerr*180/PI; }
      // if ( yerr*180/PI < 0 )     { ori_err.y = -( 180 + yerr*180/PI); }
      // ori_err.z = 0;
      // pub_roterr_est_.publish( ori_err );
      
      // std::cout<<"y error: "<< ori_err.y << std::endl;

      
    
    }
    if ( ptsloc == 0 ){
      if ( ptsloclast == 1 ){
	ori_err.x = error_last;
	ori_err.y = 0;
	ori_err.z = 0;
	pub_roterr_est_.publish( ori_err );
     	
      }
      if ( ptsloclast == 2 ){
	ori_err.x = 0;
	ori_err.y = error_last;
	ori_err.z = 0;
	pub_roterr_est_.publish( ori_err );
      }
    }     

    
  }


  double average_plane_orientation1D( const Eigen::Vector2d& startPoint,
				      const Eigen::Vector2d& currPoint,
				      std::vector<double>& plane_ori_vector,
				      const int plane_ori_est_size ){

    if ( plane_ori_vector.size() > plane_ori_est_size ){
      plane_ori_vector.erase( plane_ori_vector.begin() );
    }

    Eigen::Vector2d currVector = currPoint - startPoint;
    double currAngle = atan2( currVector(1), currVector(0) );
    if ( currAngle < 0 )  { currAngle += 2*PI; };

    double result = average( currAngle,
			     plane_ori_vector,
			     plane_ori_est_size );

    return result;

  }


  double average( const double err_angleX,
		  std::vector<double>& err_vectorX,
		  const int window_size )
  {
    err_vectorX.push_back( err_angleX );
    if ( err_vectorX.size() > window_size ){
      err_vectorX.erase( err_vectorX.begin() );
    }
    
    double sumX = 0;
    for ( int i = 0; i < err_vectorX.size(); ++i ){
      sumX += err_vectorX.at(i);
    }

    double avg_errorX = sumX / window_size;
    return avg_errorX;
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

    if ( isnan(params(0)) ){
      params(0) = 0;
    }
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

  std::vector<double> err_vectorX;
  std::vector<double> err_vectorY;

  std::vector<double> plane_ori_vectorX;
  std::vector<double> plane_ori_vectorY;
  
  int reset_start_point;
  Eigen::Vector3d startPoint;

  int ptsloclast;
  double error_last;
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
