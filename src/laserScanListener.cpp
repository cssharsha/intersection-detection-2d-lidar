#include "corner_detect/laserScanListener.hh"

LaserScanListener::LaserScanListener() :
    nh_private_("~"),
    tf_(),
    scan_mutex_(),
    filter_chain_("sensor_msgs::LaserScan"),
    theta_min_(0.5f),
    scan_recv_(false)
{
    ROS_INFO("LaserScanListener::LaserScanListener: Created new node!");
    nh_private_.param("odom_frame", odom_, std::string("odom"));
    nh_private_.param("base_link_frame", base_link_, std::string("base_link"));
	nh_private_.param("reach_thresh", r_thresh_, REACH_THRESH);
	nh_private_.param("same_thresh", s_thresh_, SAME_THRESH);
	nh_private_.param("confidence_count",CONF_COUNT, 5);
	nh_private_.param("confidence_probability", CONF_PROB, 0.75f);
	nh_private_.param("max_range_thresh", MAX_RANGE_THRESH, 10.0f);
    nh_private_.param("scan_topic",scan_topic_,std::string("scan_filtered"));
    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    
    laser_sub_.subscribe(nh_,scan_topic_,10);
    laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, base_link_.c_str(), 10);
    laser_notifier_->registerCallback(
      	boost::bind(&LaserScanListener::scanCallback, this, _1));
    laser_notifier_->setTolerance(ros::Duration(0.01));
    filter_chain_.configure("scan_filter_chain");
    int_pub_ = nh_.advertise<corner_detect::MidPoint>("intersection", 0);

    inter_reach_ = false;
    left_scan_check_ = right_scan_check_ = true;
    prev_int_set_ = false;

    inf_found_ = false;
    pub_msg_ = false;
    legal_int_ = false;
}

void LaserScanListener::scanCallback (const sensor_msgs::LaserScan::ConstPtr scan_in)
{
    sensor_msgs::LaserScan scan;
    ros::Publisher output_pub_;

    filter_chain_.update (*scan_in, scan);
	if(scan.ranges.empty()) return;
    if(!convertScanToPointCloud(scan))
    {
	    ROS_WARN_STREAM_THROTTLE(1.0, "LaserScanListener::scanCallback: Error in scan to pcl conversion.");
	    return;
    }
    
    try
    {
    	if(checkLastProcessTime())
	{
    		scan_mutex_.lock();
    		detectBreakPoint(scan);
			scan_mutex_.unlock();
	}
	else
	{
		ROS_WARN_STREAM( "LaserScanListener::scanCallback: Last proc time: "
				   	<< last_proc_time_ << "Allowing time!");
	}
    }
    catch ( std::exception const &e )
    {
    	ROS_WARN_STREAM( "LaserScanListener::scanCallback: Unable to lock: " <<
				e.what());
    }
}

bool LaserScanListener::convertScanToPointCloud(sensor_msgs::LaserScan& scan)
{
  	  ROS_DEBUG_STREAM( "LaserScanListener::convertScanToPointCloud: Method enterd!" );
	  int ign_count = 0;
	  for(unsigned int i = 0;i < scan.ranges.size();i++)
	  {
		  if(std::isinf(scan.ranges[i]))
		  	ign_count++;
	  }
	  sensor_msgs::PointCloud2 cloud;

	  try
	  {
		  if(!buffer_.canTransform(base_link_,scan.header.frame_id,
					  scan.header.stamp + ros::Duration(scan.scan_time),
					  ros::Duration(TRANSFORM_WAIT_TIME)))
		  {
			  ROS_WARN_STREAM( "LaserScanListener::convertScanToPointCloud: Failed to lookup transform from '" <<
					  scan.header.frame_id << "' to '" << base_link_ << "'");
			  return false;
		  }
		  lprojector_.transformLaserScanToPointCloud(base_link_, scan, cloud, buffer_);
	  }
	  catch (const tf::TransformException &e)
	  {
		  ROS_WARN_STREAM_THROTTLE(1.0, "LaserScanListener::convertScanToPointCloud: TF returned a transform exception: " << e.what());
		  return false;
	  }

	  try
	  {
		  pcl::PCLPointCloud2 pcl_pcl2;
		  pcl_conversions::toPCL(cloud, pcl_pcl2);
		  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
                  pcl::fromPCLPointCloud2(pcl_pcl2, pcl_cloud);
		  pcl_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(pcl_cloud);
	  }
	  catch (const pcl::PCLException &e)
	  {
		  ROS_WARN_STREAM_THROTTLE( 1.0, "LaserScanListener::convertScanToPointCloud: Failed to convert a message to pcl type." << e.what());
		  return false;
	  }
	  ROS_DEBUG_STREAM( "LaserScanListener::convertScanToPointCloud: Info regarding point cloud: [" << pcl_cloud_->size() << "]. Ignore count: " << ign_count);
	  return true;
}

void LaserScanListener::confirmSpace(const sensor_msgs::LaserScan& scan, std::vector<corner_detect::MidPoint>::iterator msg)
{
    ROS_DEBUG("LaserScanListener::confirmSpace: Method entered!");
    ROS_DEBUG_STREAM("SCAN INFO:"<<scan.ranges.size());
    ROS_DEBUG("LaserScanListener::confirmSpace: Intersection to confirm: %f, %f", msg->pose.position.x, msg->pose.position.y);
    bool ret;
    int check_ang_inc = 0.92/scan.angle_increment;
    int left_space_count, left_no_space_count, right_space_count, right_no_space_count;
    left_space_count = left_no_space_count = right_space_count = right_no_space_count = 0;
    
    // check right
    for(int i = 0,si = 0;i<check_ang_inc;i++)
    {
        if(scan.ranges[si] > 3)
        {
            right_space_count++;
            right_no_space_count = 0;
        }
        else
        {
            right_no_space_count++;
            if(right_no_space_count > 3)
            {
                right_no_space_count = 0;
                if(right_space_count < (check_ang_inc/4)) right_space_count = 0;
            }
        }
        si++;
    }
    
    // check left
    for(int i = 0,si = scan.ranges.size()-1;i<check_ang_inc;i++)
    {
        if(scan.ranges[si] > 3)
        {
            left_space_count++;
            left_no_space_count = 0;
        }
        else
        {
            left_no_space_count++;
            if(left_no_space_count > 3)
            {
                left_no_space_count = 0;
                if(left_space_count<(check_ang_inc/4)) left_space_count = 0;
            }
        }
        si--;
    }
    
    int inter_name = msg->intersection_enum;
    bool to_pub = false;
    ROS_DEBUG_STREAM("Printing info HARSHA:" << msg->intersection_name << ":" << inter_name);
    
    if(inter_name == TI || inter_name == FWI)
    {
        if(left_space_count >= (check_ang_inc/4) && right_space_count >= (check_ang_inc/4))
            to_pub = true;
        else if(left_space_count >= (check_ang_inc/4))
        {
            msg->intersection_name = "LEFT_INTERSECTION";
            to_pub = true;
        }
        else if(right_space_count >= (check_ang_inc/4))
        {
            msg->intersection_name = "RIGHT_INTERSECTION";
            to_pub = true;
        }
        else
        {
            ROS_DEBUG("WRONG INTERSECTION DETECTED TI");
        }
        
    }
    else if(inter_name == LI || inter_name == LT)
    {
        if(left_space_count >= (check_ang_inc/4))
        {
            if(right_space_count >= (check_ang_inc/4))
                msg->intersection_name = "FOUR_WAY_INTERSECTION";
            to_pub = true;;
        }
        else
            ROS_DEBUG("WRONG INTERSECTION DETECTED LI/LT");
    }
    else if(inter_name == RI || inter_name == RT)
    {
        if(right_space_count >= (check_ang_inc/4))
        {
            if(left_space_count >= (check_ang_inc/4))
                msg->intersection_name = "FOUR_WAY_INTERSECTION";
            to_pub = true;
        }
        else
            ROS_DEBUG("WRONG INTERSECTION DETECTED RI/RT");
    }
    if(to_pub)
    {
        //int_pub_.publish(*msg);
        ROS_DEBUG("LaserScanListener::confirmSpace: Message published");
    }
    
}
    
void LaserScanListener::detectBreakPoint(const sensor_msgs::LaserScan& scan)
{
    ROS_DEBUG("LaserScanListener::detectBreakPoint: Method entered!");

    float dmax, delta_phi, lambda, sigma_r, prev_inten, euc_dist, prev_range;
    pcl::PointCloud<pcl::PointXYZ>::iterator start, pcl_iter;

    bool prev_range_set = 0;

    delta_phi = scan.angle_increment; lambda = 0.174f; sigma_r = 0.05;

    start = pcl_iter = pcl_cloud_->begin()+1;
    ROS_DEBUG("LaserScanListener::detectBreakPoint: length of pcl cloud: %d", (int)pcl_cloud_->size());

    if(!std::isinf(scan.ranges[0]))
    {
      	prev_range = scan.ranges[0];
      	prev_range_set = 1;
    }
    topo_feat_.clearTopoVector();

    ROS_DEBUG("FOR LOOP BEGIN");
    float angle;

    int check_ang_inc = 0.92/delta_phi;
    int left_space_count, left_no_space_count, right_space_count, right_no_space_count;
    left_space_count = left_no_space_count = right_space_count = right_no_space_count = 0;

    bool left_space, right_space, center_space;
    left_space = right_space = center_space = true;

    // check right
    for(int i = 0,si = 0;i<5;i++)
    {
        if(scan.ranges[si] > 2)
        {
            right_space_count++;
            right_no_space_count = 0;
        }
        else
        {
	    right_no_space_count++;
	    if(right_no_space_count > 3)
            {
	    	right_space = false;
		break;
            }
        }
        si++;
    }

    // check left
    for(int i = 0,si = scan.ranges.size()-1;i<5;i++)
    {
        if(scan.ranges[si] > 2)
        {
            left_space_count++; 
            left_no_space_count = 0;
        }
        else
        {
            left_no_space_count++; 
            if(left_no_space_count > 3)
            {
	    	left_space = false;
		break;
            }
        }
        si--;
    }

    // check center
    //int si = (scan.ranges.size()/2) - (check_ang_inc/2);
    /*for(int i = 0; i < check_ang_inc;i++)
    {
        if(scan.ranges[si] > 1)
        {
            center_space_count++;
            center_no_space_count = 0;
        }
        else
        {
            center_no_space_count++;
            if(center_no_space_count > 3)
            {
                center_space = false;
                break;
            }
        }
        si++;
    }*/


    ROS_INFO("Space info: left: [%d], Right: [%d]",left_space, right_space);
    corner_detect::MidPoint dum_msg;
    if(left_space || right_space)
    {
    	try
	{
		cur_tf_ = buffer_.lookupTransform(odom_, base_link_, ros::Time(0));
		 tf::Transform temp;
        	temp = tf::Transform(tf::Quaternion(cur_tf_.transform.rotation.x,
                                            cur_tf_.transform.rotation.y,
                                            cur_tf_.transform.rotation.z,
                                            cur_tf_.transform.rotation.w),
                             	     tf::Vector3(cur_tf_.transform.translation.x,
                                         cur_tf_.transform.translation.y,
                                         cur_tf_.transform.translation.z));
        	geometry_msgs::Pose tp;
        	tf::poseTFToMsg(temp, tp);
		if(inf_found_ && !pub_msg_)
		{
			if(computeDistance(cur_tf_, prev_tf_) > 0.5 && computeDistance(tp,legal_msg_.pose) <1.0)
			{
				if(legal_int_ && !pub_msg_ && inter_reach_)
				{
					
					printMessage(legal_msg_.intersection_enum, tp);
				}
				//ROS_INFO("Ideal time to print");
			}
		}
		else if(!inf_found_)
		{
			inf_found_ = true;
			ROS_INFO("Setting space here!######### Window starts left:[%d], Right:[%d]",left_space, right_space);
			prev_tf_ = cur_tf_;
			ROS_INFO("WINDOW START: (%f,%f)",cur_tf_.transform.translation.x, cur_tf_.transform.translation.y);
		}
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("LaserScanListener::detectLineSegments: Clearing topo features : %s",ex.what());
	}
    }
    else if(!left_space && !right_space && inf_found_)
    {
    	inf_found_ = false;
    	ROS_INFO("Window end. Should have detected");
		ROS_INFO("WINDOW END: (%f,%f)", cur_tf_.transform.translation.x, cur_tf_.transform.translation.y);
	pub_msg_ = false;
    }
    else if(!pub_msg_ && !inf_found_)
    {
	ROS_INFO("COMING HERE|HARSHA");
	int center_space_count, center_no_space_count;
	center_space_count = center_no_space_count = 0;
	bool center_space = false;
	for(int i=0;i<scan.ranges.size();i++)
	{
		if(scan.ranges[i] > 2)
		{
			center_space_count++;
			if(center_space_count > check_ang_inc/4)
			{
				center_space = true;
				center_no_space_count = 0;
				break;
			}
		}
		else
		{
			center_no_space_count++;
			center_space_count = 0;
		}
	}
	if(!center_space)
	{
	try
	{
		cur_tf_ = buffer_.lookupTransform(odom_, base_link_, ros::Time(0));
		
tf::Transform temp;
        	temp = tf::Transform(tf::Quaternion(cur_tf_.transform.rotation.x,
                                            cur_tf_.transform.rotation.y,
                                            cur_tf_.transform.rotation.z,
                                            cur_tf_.transform.rotation.w),
                             	     tf::Vector3(cur_tf_.transform.translation.x,
                                         cur_tf_.transform.translation.y,
                                         cur_tf_.transform.translation.z));
        	geometry_msgs::Pose tp;
        	tf::poseTFToMsg(temp, tp);
		printMessage(UNKW, tp);
	}
    	catch(tf::TransformException &ex)
    	{
        	ROS_ERROR("LaserScanListener::detectLineSegments: Clearing topo features : %s",ex.what());
    	}
	}
	
    }


    left_space_count = left_no_space_count = right_space_count = right_no_space_count = 0;
    if(inter_reach_)
    {
        ROS_DEBUG_STREAM("SCAN INFO:"<<scan.ranges.size());
        // check right
        for(int i = 0,si = 0;i<check_ang_inc;i++)
        {
                if(scan.ranges[si] > 2)
                {
                        right_space_count++;
                        right_no_space_count = 0;
                }
                else
                {
                        right_no_space_count++;
                        if(right_no_space_count > 3)
                        {
                                right_no_space_count = 0;
                                if(right_space_count < (check_ang_inc/4)) right_space_count = 0;
                                if(si<3)
                                {
                                        right_scan_check_ = false;
                                }
                        }
                }
                si++;
        }

        // check left
        for(int i = 0,si = scan.ranges.size()-1;i<check_ang_inc;i++)
        {
                if(scan.ranges[si] > 2)
                {
                        left_space_count++;
                        left_no_space_count = 0;
                }
                else
                {
                        left_no_space_count++;
                        if(left_no_space_count > 3)
                        {
                                left_no_space_count = 0;
                                if(left_space_count<(check_ang_inc/4)) left_space_count = 0;
                                if(si<3)
                                {
                                        left_scan_check_ = false;
                                }
                        }
                }
                si--;
        }

        //ROS_INFO_STREAM("HARSHA: Left space count: "<<left_space_count<<" Right space count: " << right_space_count << "check_ang_inc: " << check_ang_inc);
                int inter_name = to_pub_msg_.intersection_enum;
        ROS_DEBUG_STREAM("Printing info HARSHA:" << to_pub_msg_.intersection_name << ":" << inter_name);
        if(inter_name == TI || inter_name == FWI)
        {
                if(left_space_count >= (check_ang_inc/4) && right_space_count >= (check_ang_inc/4))
		{
			legal_int_ = true;
                        legal_msg_ = to_pub_msg_;
		}
                else if(left_space_count >= (check_ang_inc/4))
                {
                        to_pub_msg_.intersection_name = "LEFT_INTERSECTION";
						to_pub_msg_.intersection_enum = LI;
			legal_int_ = true;
			legal_msg_ = to_pub_msg_;
			legal_msg_.intersection_enum = LI;
                }
                else if(right_space_count >= (check_ang_inc/4))
                {
                        to_pub_msg_.intersection_name = "RIGHT_INTERSECTION";
						to_pub_msg_.intersection_enum = RI;
			legal_int_ = true;
			legal_msg_ = to_pub_msg_;
			legal_msg_.intersection_enum = RI;
                }
                else
                {
                        ROS_INFO("WRONG INTERSECTION DETECTED TI");
                }

        }
        else if(inter_name == LI || inter_name == LT)
        {
                if(left_space_count >= (check_ang_inc/4))
                {
                        if(right_space_count >= (check_ang_inc/4))
						{
                                to_pub_msg_.intersection_name = "FOUR_WAY_INTERSECTION";
								to_pub_msg_.intersection_enum = FWI;
								legal_msg_ = to_pub_msg_;
								legal_msg_.intersection_enum = FWI;
								legal_int_ = true;
						}
						else
						{
			legal_int_ = true;
			legal_msg_ = to_pub_msg_;
						}
			
                }
                else
                        ROS_DEBUG("WRONG INTERSECTION DETECTED LI/LT");
        }
        else if(inter_name == RI || inter_name == RT)
        {
                if(right_space_count >= (check_ang_inc/4))
                {
                        if(left_space_count >= (check_ang_inc/4))
						{
                                to_pub_msg_.intersection_name = "FOUR_WAY_INTERSECTION";
								to_pub_msg_.intersection_enum = FWI;
								legal_msg_ = to_pub_msg_;
								legal_msg_.intersection_enum = FWI;
								legal_int_ = true;
						}
						else
						{
			legal_int_ = true;
			legal_msg_ = to_pub_msg_;
						}
                }
               else
                        ROS_DEBUG("WRONG INTERSECTION DETECTED RI/RT");
        }
    }
    
    for(int i=1;i<scan.ranges.size();i++)
    {
      	angle = (i*delta_phi) + scan.angle_min;
      	if(std::abs(scan.ranges[i]) > MAX_RANGE_THRESH)
      	{
			prev_range_set = false;
			continue;
      	}

      	if(prev_range_set)
      	{
        	dmax = (prev_range*(sin(delta_phi)/sin(lambda-delta_phi)))+5*sigma_r;
        	euc_dist = computeEuclidDist(pcl_iter, pcl_iter-1);
		ROS_DEBUG_STREAM("LaserScanListener::detectBreakPoint: Euclidean distance: " <<
                                euc_dist << "dmax: " << dmax );
        	if(euc_dist > dmax)
        	{
	    		ROS_DEBUG_STREAM( "LaserScanListener::detectBreakPoint: Angle reached here: " << angle); 
	    		ROS_DEBUG_STREAM( "LaserScanListener::detectBreakPoint: Points got from laser scan :<" <<scan.ranges[i]*cos(angle) << "," << scan.ranges[i]*sin(angle) << ">");
			int s, e;
			s = std::distance(pcl_cloud_->begin(),start);
			e = std::distance(pcl_cloud_->begin(),pcl_iter-1);
	    		ROS_DEBUG_STREAM( "LaserScanListener::detectBreakPoint: Starting from : " << s << " to :" << e);
	    		detectLineSegments(start, pcl_iter-1);
	  		start = pcl_iter;
        	}
      	}
      	else
      	{
		prev_range = scan.ranges[i];
		prev_range_set = 1;
        	continue;
      	}

      	prev_range = scan.ranges[i];
      	prev_range_set = 1;
      	pcl_iter++;
    }
    
    if(start < (pcl_iter-5))
      	if ( std::abs((start->x)) < 10 )
        	detectLineSegments(start, pcl_iter-1);
    topo_feat_.printTopoFeatures();
    scan_recv_ = true;
    ROS_DEBUG("FOR LOOP END!");
    return;
}

void LaserScanListener::printMessage(int int_type, geometry_msgs::Pose p)
{

	ROS_INFO("Printing here!!!!!!!MESSAGE PRINTED: ");
	corner_detect::MidPoint msg;
	msg.pose = p;
	msg.header.stamp = ros::Time::now();
	msg.child_frame_id = odom_;
	msg.reached = "REACHED";
	msg.intersection_name = convertEnumToString(int_type);
	msg.intersection_enum = int_type;
	ROS_INFO("%s",msg.intersection_name.c_str());
	ROS_INFO("(%f,%f)",p.position.x,p.position.y);
	int_pub_.publish(msg);
	pub_msg_ = true;
	legal_int_ = false;
	inter_reach_ = false;
}

void LaserScanListener::processScan()
{
	ROS_DEBUG_STREAM( "LaserScanListener::processScan: Method entered!" );
	int rel_ret;
	inter_det::TopoFeature::intersection inter_pose;
	ros::Rate r(10);
	while( nh_.ok())
	{
	    	try
		{
		    	last_proc_time_ = ros::Time::now();
		    	if(scan_recv_)
		    	{
		    		scan_mutex_.lock();
				rel_ret = topo_feat_.buildRelation();
				ROS_DEBUG_STREAM( "LaserScanListener::processScan: Return is: " << rel_ret);
				if( rel_ret > 0 )
				{
					topo_feat_.printRelation();
					inter_pose = topo_feat_.identifyIntersection();
					publishIntersection(inter_pose);
				}
				if(rel_ret != 0)
					topo_feat_.clearTopoFeatures(std::abs(rel_ret));
				scan_mutex_.unlock();
		    	}
		}
		catch( std::exception const &e )
		{
			ROS_WARN_STREAM( "LaserScanListener::processScan: Scan mutex lock failed: " <<
							e.what());
		}
		ros::spinOnce();
		r.sleep();
	}
	if( nh_.ok() != true )
	{
		ROS_ERROR_STREAM( "LaserScanListener::processScan: node handle not ok" );
	}
}

void LaserScanListener::detectLineSegments(pcl::PointCloud<pcl::PointXYZ>::iterator begin, pcl::PointCloud<pcl::PointXYZ>::iterator end)
{
    int inter;
    ROS_DEBUG("LaserScanListener::detectLineSegments: Detecting line segments between two break points <%f,%f> and <%f,%f>.",begin->x,begin->y,end->x,end->y);

    ROS_DEBUG_STREAM( "LaserScanListener::detectLineSegments: Pcl params: " <<
    		      			"begin: " << std::distance(pcl_cloud_->begin(),begin) <<
		      			"end: " << std::distance(pcl_cloud_->begin(),end));

    if(std::distance(begin,end) < 5)
	return;

    pcl::PointCloud<pcl::PointXYZ>::iterator ppit_ls,ppit_fi,ppit_bi;

    int kf,kb;
    float euc_dist_fi, euc_dist_bi, real_dist_fi, real_dist_bi, dot_product, mag_product, mag_f, mag_b, theta_i, f_vec[2], b_vec[2], Uk;
    bool flag_fi,flag_bi;
    flag_fi = 1;flag_bi = 1;Uk = 0.1f;

    if( begin < pcl_cloud_->begin() && begin > pcl_cloud_->end())
    {
	    ROS_WARN_STREAM( "LaserScanListener:detectLineSegments: Start pointer is NULL!" );
	    return;
    }

    if( end < pcl_cloud_->begin() && end > pcl_cloud_->end() )
    {
	    ROS_WARN_STREAM( "LaserScanListener::detectLineSegments: End pointer null!");
	    return;
    }


    for(ppit_ls = begin+1;ppit_ls<=end-1;ppit_ls++)
    {
	ppit_fi = ppit_ls + 1;
	ppit_bi = ppit_ls - 1;

      	flag_fi = flag_bi = 1;
      	real_dist_fi = real_dist_bi = 0.0f;
      	kf = 0;kb = 0;

      	// STEP 1
      	while(flag_fi)
      	{
        	euc_dist_fi = computeEuclidDist(ppit_fi,ppit_ls);
		real_dist_fi += computeEuclidDist(ppit_fi,ppit_fi-1);
	
		if((euc_dist_fi < (real_dist_fi - Uk)) || (ppit_fi >= end))
		{
	  		flag_fi = 0;
			ROS_DEBUG_STREAM( "Iterator: " << std::distance(pcl_cloud_->begin(),ppit_fi));
	  		ROS_DEBUG_STREAM("LaserScanListener::detectLineSegments:[Forward] Reached the end of pcl_cloud_ or euclidean distance " 
						<< euc_dist_fi << "less than real laser distance " << real_dist_fi);
		}	
		else
			ppit_fi++;
      	}

      	while(flag_bi)
      	{
        	euc_dist_bi = computeEuclidDist(ppit_bi,ppit_ls);
		real_dist_bi += computeEuclidDist(ppit_bi,ppit_bi+1);
			
		if(std::isinf(real_dist_bi))
	  		real_dist_bi = 0.0f;

		if((euc_dist_bi < (real_dist_bi - Uk)) || (ppit_bi <= begin))
		{
	  		flag_bi = 0;
			ROS_DEBUG_STREAM("Iterator: " << std::distance(pcl_cloud_->begin(),ppit_bi));
	  		ROS_DEBUG_STREAM("LaserScanListener::detectLineSegments:[Backward] Reached the beginning of pcl_cloud_ or euclidean distance " << euc_dist_bi << " less than real laser distance" << real_dist_bi);
		}
		else
			ppit_bi--;
      	}

      	//STEP 2
      	f_vec[0] = ppit_ls->x-ppit_fi->x;f_vec[1] = ppit_ls->y-ppit_fi->y;
      	b_vec[0] = ppit_ls->x-ppit_bi->x;b_vec[1] = ppit_ls->y-ppit_bi->y;

      
      	ROS_DEBUG_STREAM( "LaserScanListener::detectLineSegments: Points:\n" << 
		      "Forward : (" << ppit_fi->x << "," << ppit_fi->y << ")\n" <<
		      "Backward: (" << ppit_bi->x << "," << ppit_bi->y << ")\n" <<
		      "Current : (" << ppit_ls->x << "," << ppit_ls->y << ")");

      	dot_product = f_vec[0]*b_vec[0] + f_vec[1]*b_vec[1];
      	mag_f = sqrt(pow(f_vec[0],2)+pow(f_vec[1],2));
      	mag_b = sqrt(pow(b_vec[0],2)+pow(b_vec[1],2));
      	mag_product = mag_f*mag_b;

      	ROS_DEBUG("LaserScanListener::detectLineSegments[Step 2]: Dot product: <%f>, Mag product: <%f>", dot_product,mag_product);
      	theta_i = acos(dot_product/mag_product);
      
      	if(std::isinf(theta_i) || std::isnan(theta_i))
       		theta_i = 0;
	
	ROS_DEBUG_STREAM( "LaserScanListener::detectLineSegments[Step 3]: theta_i is: " << theta_i );
       
      	//STEP 3
      	if(((std::abs(theta_i) <= theta_min_) || (std::abs(theta_i - M_PI) < theta_min_)) && (computeEuclidDist(ppit_fi,ppit_bi)>=LIN_SIGMA))
      	{
			int pos = (std::distance(pcl_cloud_->begin(),ppit_ls-kb) + 
					   std::distance(pcl_cloud_->begin(),ppit_ls+kf))/2;
			if(std::distance(ppit_bi,ppit_fi) > 5)
				topo_feat_.addTopoFeat(LINE, pos, 
					ppit_bi->x, ppit_bi->y, 
					ppit_fi->x,ppit_fi->y, 
					ros::Time::now().toSec());
      	}
    }
    topo_feat_.printTopoFeatures();
    return;
}

float LaserScanListener::constrainAngle(float x)
{
    x = fmod(x + M_PI, 2*M_PI);
    if(x<0)
      	x+=(2*M_PI);
    return (x - M_PI);
  }

/*void LaserScanListener::detectCurveAndCorner(std::vector<PointParamters>::iterator begin,std::vector<PointParamters>::iterator end)
{
    //ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Method entered!");
    int ie, ib;
    float Uc, sum_theta, max_theta, ci;
    int type;
    ie = ib = 2;Uc = 0.5;
    std::vector<PointParamters>::iterator temp_ppit_;
    ppit_ = begin+2;
    for(;ppit_<=end-2;ppit_++)
    {
      //ROS_INFO("Point cloud <%d> handling now.",(int)std::distance(p_param_->begin(),ppit_));
      //STEP 4 & 5
      temp_ppit_ = ppit_-ib;

      sum_theta = 0;
      max_theta = temp_ppit_->theta;
      //ROS_INFO("+++++++++++Printing thestas here!+++++++++++++");
      for(;temp_ppit_<=ppit_+ie;temp_ppit_++)
      {
	if(temp_ppit_>p_param_->end() || temp_ppit_<p_param_->begin())
	{
	  ROS_INFO("No more point clouds to handle!");
	  break;
	}
        sum_theta += temp_ppit_->theta;
	//ROS_INFO("Sum theta <%f>, Current theta <%f>",sum_theta,temp_ppit_->theta);
	if(max_theta<temp_ppit_->theta)
	  max_theta = temp_ppit_->theta;
      }
      //ROS_INFO("Max theta <%f>",max_theta);
      //ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++");

      if(!max_theta)
        ci = (sum_theta/5)/max_theta;
      else
        ci = 0.0;
      if (ci>Uc)
      {
        ppit_->pointType = CURVE;
	ppit_->ci = ci;
	//rho = 1/((sum_theta/5)*c);
	//xc = (pcl_iter_-ib)->x + rho*cos(sum_theta);
	//yc = (pcl_iter_-ib)->y + rho*cos(sum_theta);
	topo_feat_.addTopoFeat(CURVE,std::distance(p_param_->begin(),ppit_),(ppit_-ib)->x,(ppit_-ib)->y,(ppit_+ie)->x,(ppit_+ie)->y,ros::Time::now().toSec());
      }
      else
      {
        ROS_INFO("CI <%f> not greater!",ci);
      }

    }
    //ROS_INFO("----------------------AFTER DET CURVES-----------------------------");
    //printTopoFeatures();
    //ROS_INFO("-------------------------------------------------------------------");

    //STEP 6*/
    /*for(pcl_iter_=pcl_cloud_->begin(),ppit_=p_param_->begin();pcl_iter_<pcl_cloud_->end();ppit_++,pcl_iter_++)
    {
      //if(pcl_iter_ == NULL)
      //{
	//ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Null point cloud!");
	//return_;
      //}
      //if(ppit_ == NULL)
      //{
        //ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Null param detected!");
        //return_;
      //}
      if((ppit_->ci>(ppit_+1)->ci) && (ppit_->ci>(ppit_-1)->ci))
      {
        if(ppit_->theta>theta_min_)
	{
	  ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Corn_er detected!");
	  publishPoint(pcl_iter_,CORNER);
	}
      }
    }*/
    //ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Finished!");
  //}

void LaserScanListener::publishIntersection(inter_det::TopoFeature::intersection i)
{
    corner_detect::MidPoint msg;
    tf::Transform odom_tf;
    tf::Transform ct_pose, pt_pose;
    tf::Pose pp;
    geometry_msgs::Pose p;
    bool cd;
    bool to_pub = false;
    ROS_DEBUG_STREAM("LaserScanListener::publishIntersection: Detected midpoint: " <<
    				i.type << i.p.getOrigin().x() << "," <<
				i.p.getOrigin().y());
    try
    {
      cur_tf_ = buffer_.lookupTransform(odom_, base_link_, ros::Time(0));
      odom_tf = (tf::Transform(tf::Quaternion(cur_tf_.transform.rotation.x,
      					      cur_tf_.transform.rotation.y,
					      cur_tf_.transform.rotation.z,
					      cur_tf_.transform.rotation.w),
			       tf::Vector3(cur_tf_.transform.translation.x,
			       		   cur_tf_.transform.translation.y,
					   cur_tf_.transform.translation.z)));
      if(i.type != UNKW)
      {
    	ct_pose = odom_tf*i.p;
		ROS_DEBUG_STREAM("FINAL: Current inter point: " <<
				i.p.getOrigin().x() << "," <<
				i.p.getOrigin().y() << "," <<
				i.p.getOrigin().z());
    	tf::poseTFToMsg(ct_pose,p);
    	msg.header.stamp = ros::Time::now();
	msg.child_frame_id = odom_;
	if(!is_.empty())
	{
		
			ROS_DEBUG_STREAM("FINAL: Current Pose: "<<ct_pose.getOrigin().x() <<","
							<<ct_pose.getOrigin().y()<<","
							<<ct_pose.getOrigin().z());
			std::list<std::pair<geometry_msgs::Pose,std::map<int,int>>>::iterator it;
			it = is_.begin();
			bool found = false;
			std::pair<geometry_msgs::Pose,std::map<int,int>> cur;
			while(it != is_.end())
			{
				cur = *it;
				cd = computeManDistance(p,cur.first,false);
				ROS_DEBUG_STREAM("Distance inside while loop between pose-intersection" << cd);
				if(cd)
				{
					tf::Transform ave_pose;
					geometry_msgs::Pose ave_p;
					float ax, ay, az;
					ax = (2*cur.first.position.x + ct_pose.getOrigin().x())/3;
					ay = (2*cur.first.position.y + ct_pose.getOrigin().y())/3;
					az = (2*cur.first.position.z + ct_pose.getOrigin().z())/3;
					ave_pose = tf::Transform(ct_pose.getRotation(),
										tf::Vector3(ax, ay, az));
					tf::poseTFToMsg(ave_pose,ave_p);
					msg.reached = "SAME";
					msg.pose = p;
					msg.intersection_name = convertEnumToString(i.type);
					msg.intersection_enum = i.type;
					found = true;
					it->first = ave_p;
					std::map<int,int>::iterator mit;
					mit = it->second.find(i.type);
					mit->second += 1;
					ROS_DEBUG_STREAM("FINAL: =======SAME=======");
					break;
				}
				it++;
			}
			if(!found)
			{
				ROS_DEBUG_STREAM("FINAL: New pose: "<<ct_pose.getOrigin().x() << ","
						<<ct_pose.getOrigin().y() <<","
						<<ct_pose.getOrigin().z());
				std::pair<geometry_msgs::Pose,std::map<int,int>> outertemp;
				std::map<int,int> temp;
				temp[TI] = 0;
				temp[RI] = 0;
				temp[RT] = 0;
				temp[LI] = 0;
				temp[LT] = 0;
				temp[FWI] = 0;
				temp[i.type] += 1;
				outertemp = std::make_pair(p,temp);
				is_.emplace_back(outertemp);
				msg.reached = "NEW";
				msg.pose = p;
				msg.intersection_name = convertEnumToString(i.type);
				msg.intersection_enum = i.type;
				ROS_DEBUG_STREAM("FINAL: ========NEW=======");
			}
	}
	else 
	{
			ROS_DEBUG_STREAM("FINAL: New pose: "<<ct_pose.getOrigin().x() << ","
						   <<ct_pose.getOrigin().y() <<","
						   <<ct_pose.getOrigin().z());
			msg.reached = "NEW";
			msg.pose = p;
			msg.intersection_name = convertEnumToString(i.type);
			msg.intersection_enum = i.type;
			std::pair<geometry_msgs::Pose,std::map<int,int>> outertemp;
                        std::map<int,int> temp;
                        temp[TI] = 0;
                        temp[RI] = 0;
                        temp[RT] = 0;
                        temp[LI] = 0;
                        temp[LT] = 0;
                        temp[FWI] = 0;
			temp[i.type] += 1;
                        outertemp = std::make_pair(p,temp);
                        is_.emplace_back(outertemp);
			ROS_DEBUG_STREAM("FINAL: ========NEW=======");
	}
	//to_pub = true;
      }

      if(!is_.empty())
      {
		std::list<std::pair<geometry_msgs::Pose,std::map<int,int>>>::iterator it;
		for(it = is_.begin(); it != is_.end(); it++)
		{
      			geometry_msgs::Pose op;
			op.position.x = odom_tf.getOrigin().x();
			op.position.y = odom_tf.getOrigin().y();
			op.position.z = odom_tf.getOrigin().z();
			std::pair<geometry_msgs::Pose,std::map<int,int>> cur;
			cur = *it;
			bool cd = computeManDistance(op,cur.first,true);
			float ecd = computeDistance(op,cur.first);
			bool del_mp = false;
			if(cd)
			{
				//pub_msg_ = false;
				msg.reached = "REACHED";
				int sum =0;
				float prob = 0;
				int large = 0;
				int type = 0;
				std::map<int,int>::iterator mit = cur.second.begin();
				for(;mit != cur.second.end();mit++) 
				{
					ROS_INFO("Info: Type[%d]:[%d]",mit->first,mit->second);
					if(mit->second > large)
					{
						msg.intersection_name = convertEnumToString(mit->first);
						msg.intersection_enum = mit->first;
						prob = (float)mit->second;
						type = mit->first;
						large = mit->second;
						ROS_INFO("Larger than before");
						ROS_INFO("Info: Larger Type [%d]:[%f]",type,prob);
					}
					sum += mit->second;
				}
				prob = prob/sum;
				ROS_INFO("SUM: [%d]",sum);
				msg.intersection_name = convertEnumToString(type);
				msg.intersection_enum = type;
				msg.pose = cur.first;
				/*if(sum < CONF_COUNT || prob < CONF_PROB)
				{
					to_pub = false;
					del_mp = true;
				}
				else
				{*/
					ROS_INFO_STREAM("FINAL: =====REACHED PUBLISH=====\n" << 
							"Point: (" << cur.first.position.x << "," <<
							cur.first.position.y << ")\n" <<
							"Robot Position: (" << op.position.x << "," <<
							op.position.y << ")");
					del_mp = true;
					to_pub = true;
				//}
				is_.erase(it);
				break;
			}
			else if(ecd > 5 && del_mp)
			{
				ROS_INFO("Deleting the point[%f,%f] with distance : %f!",cur.first.position.x,cur.first.position.y,ecd);
				ROS_INFO("Odom position: (%f,%f)", op.position.x, op.position.y);
				is_.erase(it);
			}
		}
      }
      std::list<std::pair<geometry_msgs::Pose,std::map<int,int>>>::iterator it;
      for(it = is_.begin(); it != is_.end(); it++)
      {
        std::pair<geometry_msgs::Pose,std::map<int,int>> cur;
	cur = *it;
      	ROS_DEBUG_STREAM("FINAL: [" << std::distance(is_.begin(),it) << "]:"
				<< cur.first.position.x << ","
				<< cur.first.position.y << ","
				<< cur.first.position.z);
      }
      ROS_DEBUG_STREAM("================================");
      ROS_DEBUG_STREAM("Odom tf is: " << odom_tf.getOrigin().x() << ","
      				    << odom_tf.getOrigin().y() << ","
				     	<< odom_tf.getOrigin().z());
      
    }
    catch(tf::TransformException &ex)
    {
    	ROS_ERROR("LaserScanListener::detectLineSegments: Clearing topo features : %s",ex.what());
    }
    if(to_pub)
	{
		ROS_INFO("PUBLISHING MESSAGE!!!!!");
		to_pub_msg_ = msg;
    	inter_reach_ = true;
	}
}

float LaserScanListener::computeDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  	float dist;
	float dist_x = a.position.x - b.position.x;
	float dist_y = a.position.y - b.position.y;
	dist = sqrt(pow(dist_x,2) + pow(dist_y,2));
	return dist;
}

float LaserScanListener::computeDistance(geometry_msgs::TransformStamped a, geometry_msgs::TransformStamped b)
{
	float dist;
	float dist_x = a.transform.translation.x - b.transform.translation.x;
	float dist_y = a.transform.translation.y - b.transform.translation.y;
	dist = sqrt(pow(dist_x,2) + pow(dist_y,2));
	return dist;
}

bool LaserScanListener::computeManDistance(geometry_msgs::Pose a, geometry_msgs::Pose b,bool reach)
{
	float thresh;
	if(reach)
		thresh = r_thresh_;
	else
		thresh = s_thresh_;

	if(std::abs(a.position.x - b.position.x) < thresh &&
			std::abs(a.position.y - b.position.y) < thresh )
		return true;
	return false;
}

float LaserScanListener::computeEuclidDist(pcl::PointCloud<pcl::PointXYZ>::iterator pcl1,pcl::PointCloud<pcl::PointXYZ>::iterator pcl2)
{
    float dist;
    float dist_x = pcl1->x - pcl2->x;
    float dist_y = pcl1->y - pcl2->y;

    dist = pow(dist_x,2) + pow(dist_y,2);
    dist = sqrt(dist);
    return dist;
}

LaserScanListener::~LaserScanListener(){

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "scannode");
  ROS_INFO("Node init!");
  LaserScanListener lsListener;
  //ros::spin();
  lsListener.processScan();
  return 0;
}
