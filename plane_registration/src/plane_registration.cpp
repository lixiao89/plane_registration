#include "plane_registration.h"


PlaneRegistration::PlaneRegistration( ros::NodeHandle* node ): node_(node),
							       est_state(IDLE),
							       process_data_num(50),
							       reset_start_point( 0 ),
                                                               error_last( 0 ),
                                                               ptsloclast( 0 )
{
    // pubs 
    pub_roterr_est_ = node->advertise<geometry_msgs::Vector3>("/plreg/oriErr", 10);
    pub_local_probing_normal_est_ = node->advertise<geometry_msgs::Vector3>("/plreg/local_probing_est", 10);

    // subs
    sub_key_ = node->subscribe("/hybrid/key", 1, &PlaneRegistration::cb_key, this);
    sub_jr3_ = node->subscribe("/jr3/wrench", 1, &PlaneRegistration::cb_jr3, this);
    sub_states_ = node->subscribe("/gazebo/barrett_manager/wam/joint_states", 1, &PlaneRegistration::cb_jnt_states, this);
    sub_plreg_pts_ = node->subscribe("/plreg/ptlocation", 1, &PlaneRegistration::cb_ptsloc, this);
    
    init_kdl_robot();

    // --- for plane registration ---
    ptsloc = 0;
    plane_ori_vectorX.push_back( 0 );
    plane_ori_vectorY.push_back( 0 );

    // for local probing step
    unit_vector_history_1.resize( 3, 1 );
    unit_vector_history_1 << 0, 0, 0;
    unit_vector_history_2.resize( 3, 1 );
    unit_vector_history_2 << 0, 0, 0;
}

PlaneRegistration::~PlaneRegistration(){}

void PlaneRegistration::update()
{
  if ( est_state == ESTIMATE ){
    //orientation_error_estimate();
    local_probing();
  } 
}


// Process keyboard events
void PlaneRegistration::cb_key(const std_msgs::String &msg){
  if (msg.data == "p"){
    est_state = ESTIMATE;
    reset_start_point = 1;
  }
  
}


  // ---------- Subscriber Callbacks ----------------
void PlaneRegistration::cb_jr3(const geometry_msgs::WrenchStamped &msg)
{
  tf::wrenchMsgToKDL(msg.wrench, jr3_wrench_);
}


void PlaneRegistration::cb_jnt_states(const sensor_msgs::JointState &msg)
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


void PlaneRegistration::cb_ptsloc( const std_msgs::Int32 &msg )
{
  ptsloc = msg.data;
  if ( ptsloc == -1 ){
    reset_start_point = 1;
  }
 
}


  // FUNCTION RESPONSIBLE FOR ESTIMATION
void PlaneRegistration::orientation_error_estimate(){
  
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


double PlaneRegistration::average_plane_orientation1D( const Eigen::Vector2d& startPoint,
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


double PlaneRegistration::average( const double data,
				   std::vector<double>& data_vector,
				   const int window_size )
{
  data_vector.push_back( data );
  if ( data_vector.size() > window_size ){
    data_vector.erase( data_vector.begin() );
  }
  
  double sumX = 0;
  for ( int i = 0; i < data_vector.size(); ++i ){
    sumX += data_vector.at(i);
  }
  
  double data_avg = sumX / window_size;
  return data_avg;
}



Eigen::Vector2d PlaneRegistration::EvaluateMovingWindowLS(const double& x,
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


void PlaneRegistration::init_kdl_robot()
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


void PlaneRegistration::remove_matrix_column(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
  unsigned int numRows = matrix.rows();
  unsigned int numCols = matrix.cols()-1;
  
  if( colToRemove < numCols )
    matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);
  
  matrix.conservativeResize(numRows,numCols);
}


Eigen::Vector3d PlaneRegistration::estimate_trj_unit_vector( Eigen::Vector3d& start_point,
							     Eigen::Vector3d& curr_end_point,
							     Eigen::MatrixXd& unit_vector_history ){							     					    

  Eigen::Vector3d curr_vector = ( curr_end_point - start_point );
  Eigen::Vector3d curr_unit_vector = curr_vector / curr_vector.norm();
  
  int num_rows = unit_vector_history.rows();
  int num_cols = unit_vector_history.cols();

  unit_vector_history.conservativeResize( num_rows, num_cols + 1 );
  unit_vector_history.col( num_cols ) = curr_unit_vector;

  if ( unit_vector_history.cols() > 25 ){
    remove_matrix_column( unit_vector_history, 0 );
  }

  Eigen::Vector3d avg_vector = unit_vector_history.rowwise().mean();

  return avg_vector;

}


void PlaneRegistration::local_probing(){
  
  geometry_msgs::Vector3 ori_err;
  
  fk_solver_->JntToCart( jnt_pos_, tip_frame_ );
  
  Eigen::Vector3d start_point( startPoint(0), startPoint(1), startPoint(2) );
  Eigen::Vector3d curr_point( tip_frame_.p[0], tip_frame_.p[1], tip_frame_.p[2] );

  // cutter moving in the first straight line, estimate the first vector
  if ( ptsloc == 1 ){
          
   plane_vec_1 =  estimate_trj_unit_vector( start_point,
					    curr_point,
					    unit_vector_history_1 ); 				
  
   // std::cout<<"vector 1: "<< plane_vec_1 << std::endl;
    
    ptsloclast = 1;
    
  }

  // cutter moving in another direction, estimate the second vector
  if ( ptsloc == 2 ){

  plane_vec_2 =  estimate_trj_unit_vector( start_point,
					   curr_point,
					   unit_vector_history_2 ); 				
  
    
  //std::cout<<"vector 2: "<< plane_vec_2 << std::endl;
    
    ptsloclast = 2;
    
  }

  // finished collecting two vectors on plane, calculate the normal
  if ( ptsloc == 3 ){
    
    Eigen::Vector3d normal;

    normal = plane_vec_1.cross( plane_vec_2 );

    // make sure the reported normal makes an angle < 90 degrees with the cutter z axis
    Eigen::Vector3d tipZ( tip_frame_.M.UnitZ()[0],
			  tip_frame_.M.UnitZ()[1],
			  tip_frame_.M.UnitZ()[2] );
    if ( normal.dot( tipZ ) < 0 ){
      normal = -normal;
    }

    Eigen::Vector3d unit_normal = normal / normal.norm();

    std::cout << "est. Normal: "<< std::endl;
    std::cout << normal <<std::endl;
 
    local_probing_est.x = unit_normal(0);
    local_probing_est.y = unit_normal(1);
    local_probing_est.z = unit_normal(2);
    pub_local_probing_normal_est_.publish( local_probing_est );
    
  }
  
    

}
