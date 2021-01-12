#include "std_msgs/String.h"
#include <math.h>
#include <ros/ros.h>
#include<tf/tf.h>

const float PER_ANG = 1.57;
const float PAR_ANG = 3.14;
const float ANG_SIGMA = 0.35;
const float C_ANG_SIGMA = 0.7;
const float LIN_SIGMA = 0.5;
const float PAR_LIN_SIGMA = 0.9;
const float LIN_GAP_SIGMA = 0.3;
const float REACH_THRESH = 0.2;
const float SAME_THRESH = 0.5;
namespace inter_det
{
   struct topo
    {
      int feat;
      int position;
      float beg_x;
      float beg_y;
      float end_x;
      float end_y;
      float cur_time;
    };
   struct node
    {
            int type;
            struct topo info;
            struct relation *fr;
	    struct relation *br;
    };

    struct relation
    {
            float angle;
            float space_x, space_y;
            bool side_gap, same_gap;
            int rel_to;
	    struct node *next;
    };

    struct ret_code
    {
	    bool gap;
	    int rel;
    };
    
    struct intersection
    {
    	int type;
	tf::Transform p;
    };

  class TopoFeature
  {
    enum REL_ENT { RIGHT, LEFT, FRONT, BACK, UNKNOWN, T, FOUR, L1INL2, L2INL1, 
   			L1ENDINL2, L2BEGINL1 };
    enum REL_POI { BEG=1, END, UNK, NONE, PAR };


    struct node *node_ref_, *node_head_;

    std::vector<topo> topo_vec_;
    public:
    struct intersection
    {
    	int type;
	tf::Transform p;
    };
    struct intersection pose_;

    enum GAP_T{FORW_G,LEFT_G,RIGHT_G};
    enum SIDE{QUAD,MULT};
    enum features{BREAKPOINT,LINE,CURVE,CORNER};
    enum intersec{TI,RI,RT,LI,LT,FWI,UNKW};

    tf::Transform cur_tf_;

	  public:
    void setCurrentTf(tf::Transform t);
    void addTopoFeat(int feat, int pos, float beg_x, float beg_y, float end_x,float  end_y,float time);
    void printTopoFeatures();
    intersection identifyIntersection();
    void updateFlags(float angle,struct topo seg1,struct topo seg2,int comp_flag,int side_flag);
    void initFlags();
    void clearTopoFeatures(int);
    int buildRelation();
    int checkLineIntersection(struct topo, struct topo);
    intersection logIntersection( bool, bool, bool );
    void printRelation();
    int checkParlell(struct topo, struct topo);
    bool checkEncaps(float beg_x, float beg_y, float end_x, float end_y, float x, float y);
    bool checkProperT(struct topo l1, struct topo l2, bool side);
    void clearTopoVector() { topo_vec_.clear(); }
    float dist(float x1, float y1, float x2, float y2)
    {
	    return(sqrt(pow((x2 - x1),2) + pow((y2 - y1),2)));
    }
    float computeLength(struct topo t)
    {
    	return(sqrt(pow((t.beg_x - t.end_x),2) + pow((t.beg_y - t.end_y),2)));
    }
  };
}
