#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include "filters/filter_chain.h"
#include <math.h>
#include "std_msgs/String.h"
#include <corner_detect/topoFeatures.hh>
#include <boost/thread/mutex.hpp>
#include <corner_detect/MidPoint.h>

class LaserScanListener{



  ros::NodeHandle nh_private_, nh_;

  // Variable used but output from this not used at all.
  laser_geometry::LaserProjection lprojector_;
  const int TRANSFORM_WAIT_TIME = 3.0;
  const float PI = 3.14159265358979323846;
  float MAX_RANGE_THRESH;
  float CONF_PROB;
  int CONF_COUNT;

  float r_thresh_, s_thresh_;
  bool inter_reach_;
  bool left_scan_check_, right_scan_check_;
  corner_detect::MidPoint to_pub_msg_;
  corner_detect::MidPoint legal_msg_;
  bool legal_int_;
  bool reached_;
  
  // Still to be used!!
  tf::TransformListener tf_;
  geometry_msgs::TransformStamped cur_tf_;
  geometry_msgs::TransformStamped prev_tf_;
  bool inf_found_, pub_msg_;

  std::string odom_, base_link_, base_laser_, scan_topic_;

  // Subscriber to laser scan
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  //ros::Subscriber laser_sub_;
  ros::Subscriber odom_sub_;
  geometry_msgs::PoseStamped odom_pose_;
  bool odom_set_ = true;

  //Used to register callback but not for the actual purpose.
  tf::MessageFilter<sensor_msgs::LaserScan> *laser_notifier_;

  // Below not used until now.
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_;
  sensor_msgs::PointCloud2::Ptr cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::iterator pcl_iter_;
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
  // ====================================

  // Minimum angle to be considered as a curve
  float theta_min_;

  // Publisher commentde out as of now.
  ros::Publisher p_pub_;

  // Not used at all
  visualization_msgs::Marker p_;

  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  boost::mutex scan_mutex_;

  /*struct PoseBin
  {
	  geometry_msgs::Pose p;
	  std::map<int,int> bin;
  };*/

  std::list<std::pair<geometry_msgs::Pose,std::map<int,int>>> is_;

  enum features{BREAKPOINT,LINE,CURVE,CORNER};
  enum angle_range{UNKNOWN,acute,obtuse,per,par};
  enum intersec{TI,RI,RT,LI,LT,FWI,UNKW};
  ros::Publisher int_pub_;
  
  int seq_no_;
  inter_det::TopoFeature topo_feat_;
  bool scan_recv_;

  ros::Time last_proc_time_;
  enum DIST{NEW,SAME,REACHED};
  
    bool prev_int_set_;
    corner_detect::MidPoint prev_int_;
    std::vector<corner_detect::MidPoint> pub_buf_;

  public: LaserScanListener();
  public: ~LaserScanListener();
  protected: void scanCallback(const sensor_msgs::LaserScan::ConstPtr);
  protected: void detectBreakPoint(const sensor_msgs::LaserScan& scan);
  protected: void detectLineSegments(pcl::PointCloud<pcl::PointXYZ>::iterator,pcl::PointCloud<pcl::PointXYZ>::iterator);
  //protected: void detectCurveAndCorner(pcl::PointCloud<pcl::PointXYZ>::iterator, pcl::PointCloud<pcl::PointXYZ>::iterator);
  protected: float computeEuclidDist(pcl::PointCloud<pcl::PointXYZ>::iterator,pcl::PointCloud<pcl::PointXYZ>::iterator);
  protected: void publishPoint(pcl::PointCloud<pcl::PointXYZ>::iterator,int);
  protected: bool convertScanToPointCloud(sensor_msgs::LaserScan& scan);
  protected: float constrainAngle(float);
  protected: void publishIntersection(inter_det::TopoFeature::intersection);
  protected: float computeDistance(geometry_msgs::Pose a, geometry_msgs::Pose b);
  protected: float computeDistance(geometry_msgs::TransformStamped a, geometry_msgs::TransformStamped b);
  protected: void printMessage(int int_type, geometry_msgs::Pose p);
  protected: bool computeManDistance(geometry_msgs::Pose a, geometry_msgs::Pose b, bool reach);
protected: void confirmSpace(const sensor_msgs::LaserScan& scan, std::vector<corner_detect::MidPoint>::iterator msg);
  public: void processScan();
  protected: bool checkLastProcessTime()
  		{
			if((ros::Time::now() - last_proc_time_).toSec() < 2)
				return true;
			return false;
		}
  protected: std::string convertEnumToString(int i)
  		{
			std::string intersection_name;
			switch(i)
			{
				case TI: intersection_name = "T_INTERSECTION";
					 break;
				case RI: intersection_name = "RIGHT_INTERSECTION";
					 break;
				case RT: intersection_name = "RIGHT_TURN";
					 break;
				case LI: intersection_name = "LEFT_INTERSECTION";
					 break;
				case LT: intersection_name = "LEFT_TURN";
					 break;
				case FWI: intersection_name = "FOUR_WAY_INTERSECTION";
					  break;
				case UNKW: intersection_name = "NO_INT";
					   break;
			}
			return intersection_name;
		}

    int convertStringToEnum(std::string name)
    {
      int inter_enum;
      if(name.compare("T_INTERSECTION"))
      {
        ROS_DEBUG("HARSHA: T heR");
        inter_enum = TI;
      }
      else if(name.compare("RIGHT_INTERSECTION"))
      {
        ROS_DEBUG("HARSHA:RI heR");
        inter_enum = RI;
      }
      else if(name.compare("RIGHT_TURN"))
      {
        ROS_DEBUG("HARSHA: RT heR");
        inter_enum = RT;
      }
      else if(name.compare("LEFT_INTERSECTION"))
      {
        ROS_DEBUG("HARSHA: LI heR");
        inter_enum = LI;
      }
      else if(name.compare("LEFT_TURN"))
      {
        ROS_DEBUG("HARSHA: LT heR");
        inter_enum = LT;
      }
      else if(name.compare("FOUR_WAY_INTERSECTION"))
      {
        ROS_DEBUG("HARSHA: FWI heR");
        inter_enum = FWI;
      }
      return inter_enum;
    }
};


