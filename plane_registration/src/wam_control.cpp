#include "wam_control.h"

  
WamMove:: WamMove(ros::NodeHandle* node):
    node_(node),
    ctrl_type_(CTRL_IDLE),
    inject_noise(NO_NOISE),
    inject_noise_last(NO_NOISE),
    first_vector_ready(false),
    second_vector_ready(false),
    ascend_ready(false),
    initialize_ready(false),
    err_pose(0),
    start_correcting_x(false)
  {
    // pubs 
    pub_state_ = node->advertise<std_msgs::String>("/dvrk_psm1/set_robot_state", 10);
    pub_jnt_cmd_ = node->advertise<trajectory_msgs::JointTrajectoryPoint>("/gazebo/traj_rml/joint_traj_point_cmd", 1);
    pub_plane_reg_pts_ = node->advertise<std_msgs::Int32>("/plreg/ptlocation", 10);
    pub_err_gt_ = node->advertise<geometry_msgs::Vector3>("/plreg/errGroundTruth", 10);
    pub_plreg_trigger_= node->advertise<std_msgs::Int32>("/plreg/trigger", 10);

    // subs
    //sub_joy_ = node->subscribe("/spacenav/joy", 1, &Control::cb_joy, this);
    sub_jr3_ = node->subscribe("/jr3/wrench", 1, &WamMove::cb_jr3, this);
    sub_key_ = node->subscribe("/hybrid/key", 1, &WamMove::cb_key, this);
    sub_states_ = node->subscribe("/gazebo/barrett_manager/wam/joint_states", 1, &WamMove::cb_jnt_states, this);
    sub_roterr_ = node->subscribe("/plreg/oriErr", 1, &WamMove::cb_oriErr, this);
    sub_est_normal = node->subscribe("/plreg/local_probing_est", 1, &WamMove::cb_probing_est, this);
    sub_traj_unit_vector_ = node->subscribe("/plreg/traj_unit_vector", 1, &WamMove::cb_traj_unit_vector, this);
      
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
    plreg_process_cue.data = 0;
    start_hybrid_time = ros::Time::now().toSec();
  }
  
WamMove::~WamMove(){}
  
void WamMove::update()
  {
    // do some update here
    if ( initialize_ready == true ){
      // initialized end effector(cutter) linear and rotational velocities to zero
      Vector eevel;
      Vector eerot;
      
      // calculate forward kinematics
      fk_solver_->JntToCart(jnt_pos_, tip_frame_);
    
    	
      // force control
      double cmd_force;
      if ( ctrl_type_ == CTRL_START_HYBRID || ctrl_type_ == CTRL_HYBRID ){
	cmd_force = -4.0;
      }
      double err_force = cmd_force - jr3_wrench_.force.z();
      double cmd_v = correction_vel( err_force );	
        

      //----------------for plane registration --------------------

      Frame correction_frame( Frame::Identity() );

      // Local Probing Step
     
      // performs two straight line motions in different directions to obtain an estimate of plane normal
      if ( ctrl_type_ == CTRL_LOCAL_PROBING ){
	if ( local_probing( tip_frame_,
			    cmd_frame_,
			    last_hpf_cmd,
			    cmd_v,
			    start_point,
			    eevel,
			    correction_frame ) ){
	    ctrl_type_ = CTRL_START_HYBRID;
	  }
	
	// tells plane_registration to perform local probing step 
	plreg_process_cue.data = 1;
      } 
	  

      // Cutting Step
      
      if ( ctrl_type_ == CTRL_HYBRID ){
	
	if ( !correct_x_during_cutting( tip_frame_, 
					last_hpf_cmd, 
					cmd_v,
					correction_frame ) ){
	// move along cutter x axis if not correcting
	eevel = cmd_frame_.M.UnitX();
	  
	}
	     
	// give 6 secs of data collection and estimation time before activating correction
	if ( ros::Time::now().toSec() - start_hybrid_time < 6){
	  eevel = cmd_frame_.M.UnitX();
	  correction_frame = Frame::Identity();
	}
	plreg_process_cue.data = 2;
      }
      
      pub_plreg_trigger_.publish( plreg_process_cue );
     

      //----------------------  Inject Noise ------------------------------
      
      Frame noise_frame( Frame::Identity() );

      if ( inject_noise == X_NOISE ){
	double xerr = 0.0001; 
	Rotation roterr;
	roterr.DoRotY( xerr );
	noise_frame.M = roterr;
      }

      if (inject_noise == Y_NOISE){
      }

      // -----------------------------------------------------------------
	double vel_gain = 0.001;
	double rot_gain = 0.4;
	
	Twist cmd_twist;
	cmd_twist.vel = vel_gain * eevel;
	cmd_twist.rot = rot_gain * eerot;
	// lock all rotations in base frame
	cmd_twist.rot(0) = 0.0; 
	cmd_twist.rot(1) = 0.0;  
          
	cmd_twist = cmd_frame_.M.Inverse(cmd_twist);
	cmd_frame_.Integrate(cmd_twist, 50);	

	Frame force_frame;
	force_frame.p(2) = cmd_v;
	cmd_frame_ = cmd_frame_ * force_frame * correction_frame * noise_frame;

	// do ik
	ik_solver_->CartToJnt(jnt_pos_, cmd_frame_, jnt_cmd_);

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


double WamMove::correction_vel(const double& err_angle )
  {
    double cmd_v = -0.0001 * err_angle;
    if (cmd_v > 0.0001) cmd_v = 0.0001;
    if (cmd_v < -0.0001) cmd_v = -0.0001;
    if (fabs(cmd_v) < 0.00005) cmd_v = 0.0;

    return cmd_v;
  }

 
void WamMove::cb_jr3(const geometry_msgs::WrenchStamped &msg)
  {
    tf::wrenchMsgToKDL(msg.wrench, jr3_wrench_);
  }

void WamMove::cb_key(const std_msgs::String &msg)
  {
    ROS_INFO_STREAM("Key = " << msg.data);
    if (msg.data == "r"){
      ctrl_ready();
      initialize_ready = false;
    }
    else if (msg.data == "px"){// move to error position
      ctrl_errorpos();
      err_pose = 1;
      initialize_ready = false;
    }
    else if (msg.data == "nx"){// move to error position
      ctrl_errorpos();
      err_pose = -1;
      initialize_ready = false;
    }
    else if (msg.data == "py"){// move to error position
      ctrl_errorpos();
      err_pose = 2;
      initialize_ready = false;
    }
    else if (msg.data == "ny"){// move to error position
      ctrl_errorpos();
      err_pose = -2;
      initialize_ready = false;
    }
    else if (msg.data == "sh"){// moving to position with only hybrid control
      ctrl_start_hybrid();
      initialize_ready = true;
    }
    else if (msg.data == "h"){// start moving with control
      ctrl_hybrid();
      initialize_ready = true;
      start_hybrid_time = ros::Time::now().toSec();
    }
    else if (msg.data == "xn"){
      inject_noise = X_NOISE;
      inject_noise_last = X_NOISE;
    }
    else if (msg.data == "yn"){
      inject_noise = Y_NOISE; 
    }
    else if (msg.data == "lp"){ // start local probing
      ctrl_local_probing();
      initialize_ready = true;
      first_vector_ready = false;
      second_vector_ready = false;
      ascend_ready = false;
    }
    else
      ROS_ERROR("Unsupported Commands");
  }

void WamMove::cb_jnt_states(const sensor_msgs::JointState &msg)
  {
    if (msg.position.size() != num_jnts_) {
      ROS_ERROR_STREAM("Joint states size mismatch  " << msg.position.size());
    }

    for (size_t i = 0; i < num_jnts_; i++) {
      jnt_pos_(i) = msg.position.at(i);
      jnt_vel_(i) = msg.velocity.at(i);
    }

  }

   
  // from plane_registration
void WamMove::cb_oriErr(const geometry_msgs::Vector3 &msg)
  {
    oriErr(0) = msg.x;
    oriErr(1) = msg.y;
    oriErr(2) = msg.z;
  }
      
  // callback for subscriber to estimated plane normal in the initial probing stage 
void WamMove::cb_probing_est(const geometry_msgs::Vector3 &msg){
    if( plreg_msg.data == 3 ){
    probing_est_normal(0) = msg.x;
    probing_est_normal(1) = msg.y;
    probing_est_normal(2) = msg.z;
    }
  }


void WamMove::cb_traj_unit_vector(const geometry_msgs::Vector3 &msg){

    traj_unit_vector(0) = msg.x;
    traj_unit_vector(1) = msg.y;
    traj_unit_vector(2) = msg.z;

}

void WamMove::init_kdl_robot()
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

void WamMove::ctrl_ready()
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

void WamMove::ctrl_errorpos()
  {
    ctrl_type_ = CTRL_ERROR_POSE;
 
    fk_solver_->JntToCart( jnt_pos_, tip_frame_ );

    // sync cmd frame
    cmd_frame_ = tip_frame_;

    double xerr, yerr;

    if ( err_pose == 1 ){
      xerr = 3*PI/180;
      yerr = 0*PI/180;
    }
    if ( err_pose == -1 ){
      xerr = -3*PI/180;
      yerr = 0*PI/180;
    }
    if ( err_pose == 2 ){
      xerr = 0*PI/180;
      yerr = 3*PI/180;
    }
    if ( err_pose == -2 ){
      xerr = 0*PI/180;
      yerr = -3*PI/180;
    }
    err_pose = 0;

    Rotation roterr;
    roterr.DoRotX( xerr );
    roterr.DoRotY( yerr );

    Frame err_frame_( roterr );
    
    cmd_frame_ = cmd_frame_*err_frame_;
    
    // do ik
    ik_solver_->CartToJnt(jnt_pos_, cmd_frame_, jnt_cmd_);

    // send to robot
    trajectory_msgs::JointTrajectoryPoint msg_jnt_cmd;
    msg_jnt_cmd.positions.resize(num_jnts_);
    for (size_t i = 0; i < num_jnts_; i++)
      {   
        msg_jnt_cmd.positions[i] = jnt_cmd_(i);
      }   
    pub_jnt_cmd_.publish(msg_jnt_cmd);

  }     


void WamMove::ctrl_start_hybrid()
  {
    ctrl_type_ = CTRL_START_HYBRID;
    
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
   
    // sync cmd frame
    cmd_frame_ = tip_frame_;
  }


void WamMove::ctrl_local_probing()
  {
    ctrl_type_ = CTRL_LOCAL_PROBING;
    
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
   
    // sync cmd frame
    cmd_frame_ = tip_frame_;
    start_point[0] = tip_frame_.p[0];
    start_point[1] = tip_frame_.p[1];
    start_point[2] = tip_frame_.p[2];
  }



void WamMove::ctrl_hybrid()
  {
    // enter hybrid
    ctrl_type_ = CTRL_HYBRID;
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
    std::cout << "tip pose = \n" << tip_frame_ << std::endl;

    // sync cmd frame
    cmd_frame_ = tip_frame_;
  }


 
bool WamMove::local_probing( const Frame& tip_frame,
			     const Frame& cmd_frame,
			     const double& last_hpf_cmd,
			     double& curr_hpf_cmd,
			     const Vector& starting_point,
			     Vector& cutter_linear_vel,
			     Frame& correction_frame ){
  
    
	  // --------- collect information for first vector -------------
	  if ( first_vector_ready == false ){
	    if ( last_hpf_cmd > 0 || curr_hpf_cmd < 0 ){
	      plreg_msg.data = 1;
	    }
	    else{
	      plreg_msg.data = 0;
	    }	    
	    pub_plane_reg_pts_.publish( plreg_msg );
	    std::cout << "collecting first vector ..." <<std::endl;
	    
	    double current_cutter_z = tip_frame.p[2];
	    cutter_linear_vel = cmd_frame.M.UnitX();

	    // for informing plane_registration to reset start point during transition from vector 1 motion to vector 2
	    if ( current_cutter_z < starting_point[2] - 0.017 && current_cutter_z > starting_point[2] - 0.02 ){
	      plreg_msg.data = -1;
	      std::cout << "first vector ready" <<std::endl;
	    }
	    if ( current_cutter_z < starting_point[2] - 0.02 ){
	      first_vector_ready = true;
	    }
	  }
	  
	  // ---------- collect information for second vector ---------------
	  if ( first_vector_ready == true && second_vector_ready == false ){
	    if ( last_hpf_cmd > 0 || curr_hpf_cmd < 0 ){
	      plreg_msg.data = 2;
	    }
	    else{
	      plreg_msg.data = 0;
	    }
	    pub_plane_reg_pts_.publish( plreg_msg );
	    std::cout << "collecting second vector" << std::endl;
	    
	    double current_cutter_y = tip_frame_.p[1];
	    cutter_linear_vel = cmd_frame.M.UnitY();
	    if ( current_cutter_y < starting_point[1] - 0.02 ){
	      second_vector_ready = true;
	      std::cout << "second vector ready" << std::endl;
	    }
	  }

	  // ------ both vectors ready, start calculating error and perform CORRECTION -----
	  if ( first_vector_ready == true && second_vector_ready == true ){
	    
	    // stop hybrid control when correcting
	    curr_hpf_cmd = 0;

	      std::cout << "start correcting ..." << std::endl;
	      std::cout << "estimated normal is " << std::endl;
	      std::cout << probing_est_normal <<std::endl;
	      plreg_msg.data = 3;
	      pub_plane_reg_pts_.publish( plreg_msg );

	      // ascend the cutter in the local -z direction to avoid collision while correcting
	      double current_cutter_x = tip_frame_.p[0];
	      if ( ascend_ready == false ){
		cutter_linear_vel = -cmd_frame.M.UnitZ();
		std::cout << "ascending to avoid collision while correcting" << std::endl;
		if ( current_cutter_x > starting_point[0] + 0.01 ){
		  ascend_ready = true;
		}
	      }
	      
	    // start correcting process... 
	      if ( probing_est_normal.norm() != 0 ){ 
		
		double rel_angle;
		
		find_correction_frame( tip_frame_,
				       'z',
				       probing_est_normal,
				       correction_frame,
				       rel_angle );
		std::cout << "Error angle is: " << rel_angle << std::endl;

		if ( rel_angle > 0.4 ){
		  return false;
		}
	    
		else{
		  correction_frame = Frame::Identity();
		  std::cout << "correction complete!" << std::endl;
		  return true;
		}
	    }  
	  }

}




bool WamMove::correct_x_during_cutting( const Frame& tip_frame,
					const double& last_hpf_cmd,
				        double &curr_hpf_cmd,
					Frame& correction_frame){
  
  if ( last_hpf_cmd > 0 || curr_hpf_cmd < 0 ){
    plreg_msg.data = 1;
  }
  else{
    plreg_msg.data = 0;
  }	    
  pub_plane_reg_pts_.publish( plreg_msg );
  
  

  if ( traj_unit_vector.norm() != 0 ){
    
    std::cout << "estimated plane vector along cutter x is: " << std::endl;
    std::cout << traj_unit_vector << std::endl;

    double rel_angle;
    find_correction_frame( tip_frame,
			   'x',
			   traj_unit_vector,
			   correction_frame,
			   rel_angle );

    std::cout << "x error is: " << rel_angle << std::endl;

    if ( rel_angle > 3 ){
      start_correcting_x = true;
      std::cout << "cutter x axis mis-alignment exceeds 3 degrees, start correcting ..." << std::endl;
    }
    if ( rel_angle < 0.4 ){
      start_correcting_x = false;
      std::cout << "correction complete." << std::endl;
    }
   
    if ( start_correcting_x == true ){
      curr_hpf_cmd = 0;
      // HACK! For show only, remove this when in use
      inject_noise = NO_NOISE;
      return true;
    }
    if ( start_correcting_x == false ){
      correction_frame = Frame::Identity();
      // HACK!
      inject_noise = inject_noise_last;
      return false;
    }

  }
  
}





void WamMove::find_correction_frame( const Frame& local_frame,
				     const char& axis,
				     const Eigen::Vector3d& goal_vector,
				     Frame& correction_frame,
				     double& relative_angle ){
  Vector target_axis;
  if ( axis == 'x' )      { target_axis = local_frame.M.UnitX(); }  
  if ( axis == 'y' )      { target_axis = local_frame.M.UnitY(); }  
  if ( axis == 'z' )      { target_axis = local_frame.M.UnitZ(); }  
  

  Eigen::Vector3d target_axis_eigen( target_axis[0], target_axis[1], target_axis[2] );
	    
  // goal_vector is probing_est_normal in local probing
  Eigen::Vector3d rot_axis = target_axis_eigen.cross( goal_vector );
	      
		
  // Covert rot_axis from base frame to cutter frame
  Frame inv_local_frame = local_frame.Inverse();
  Eigen::Matrix4d inv_local_frame_temp;
  for ( int i = 0; i < 4; ++i )
    for ( int j = 0; j < 4; ++j)
      inv_local_frame_temp(i, j) = inv_local_frame(i, j);

		
  Eigen::MatrixXd homo_rot_axis_base_frame(4,1);
  homo_rot_axis_base_frame << rot_axis(0), rot_axis(1), rot_axis(2), 0;

		
  Eigen::MatrixXd homo_rot_axis_local_frame = inv_local_frame_temp*homo_rot_axis_base_frame;

		
  Eigen::Vector3d temp( homo_rot_axis_local_frame(0),
			homo_rot_axis_local_frame(1),
			homo_rot_axis_local_frame(2));

	
  Eigen::Vector3d rot_axis_local_frame = temp / temp.norm();

  // calculate relative angel between estimated plane normal and cutter z axis
  relative_angle = target_axis_eigen.dot( goal_vector ) / ( target_axis_eigen.norm() * goal_vector.norm() );

  relative_angle = acos(relative_angle)*180/PI;
    
    Eigen::Matrix3d skew;
    skew << 0, -rot_axis_local_frame(2), rot_axis_local_frame(1),
      rot_axis_local_frame(2), 0, -rot_axis_local_frame(0),
      -rot_axis_local_frame(1), rot_axis_local_frame(0), 0;
	      	       		  
    double rel_angle_scaled = fabs( correction_vel(relative_angle) ); 
   
    // Rodrigues Formula
    Eigen::Matrix3d correction_rotation_temp = Eigen::Matrix3d::Identity() + skew*sin( rel_angle_scaled ) + skew*skew*( 1 - cos( rel_angle_scaled )*cos( rel_angle_scaled ) );
    
    Rotation correction_rotation;
    for ( int i = 0; i < 3; ++i )
      for ( int j = 0; j < 3; ++j)
	correction_rotation( i, j ) = correction_rotation_temp( i, j );
    
    correction_frame.M = correction_rotation;

}

