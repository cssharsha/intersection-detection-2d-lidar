#include <corner_detect/topoFeatures.hh>
//using namespace ros;
void inter_det::TopoFeature::addTopoFeat(int feat, int pos, float beg_x, float beg_y, float end_x,float  end_y,float time)
{
  ROS_DEBUG("TopoFeature::addTopoFeat: Adding topo feature: <%d,%d,(%f,%f),(%f,%f),%f>",feat,pos,beg_x,beg_y,end_x,end_y,time);
  int size_struct = sizeof(struct topo);
  struct topo temp_topo;
  bool topo_found = false;
  unsigned int ti;
  if(topo_vec_.empty())
    ti = 1;
  else
  {
    for(ti = 0;ti < topo_vec_.size();ti++)
    {
      if(topo_vec_[ti].feat == LINE && 
      	(std::abs(topo_vec_[ti].beg_x - beg_x) < LIN_SIGMA) &&
		(std::abs(topo_vec_[ti].beg_y - beg_y) < LIN_SIGMA) &&
		(std::abs(topo_vec_[ti].end_x - end_x) < LIN_SIGMA) &&
		(std::abs(topo_vec_[ti].end_y - end_y) < LIN_SIGMA))
      {
      	if(beg_y > 0 && (topo_vec_[ti].end_x < end_x))
		{
			topo_vec_[ti].end_x = end_x;
			topo_vec_[ti].end_y = end_y;
		}
		else if(beg_y < 0 && topo_vec_[ti].end_x > end_x)
		{
			topo_vec_[ti].end_x = end_x;
			topo_vec_[ti].end_y = end_y;
		}
        int prev_type = topo_vec_[ti].feat;
		if(topo_vec_[ti].feat == LINE && feat == CURVE)
	  		topo_vec_[ti].feat == feat;
        ROS_DEBUG("Topo feat already present: feature <%d> position <%d> Beginning <%f,%f> End <%f,%f>, Feature number <%d>",topo_vec_[ti].feat,topo_vec_[ti].position,topo_vec_[ti].beg_x,topo_vec_[ti].beg_y,topo_vec_[ti].end_x,topo_vec_[ti].end_y,ti);
		topo_found = true;
		break;
      }
    }
  }

  if(!topo_found)
  {
      ROS_DEBUG("Topo adding!");
      temp_topo.feat = feat;
      temp_topo.position = pos;
      temp_topo.beg_x = beg_x;
      temp_topo.beg_y = beg_y;
      temp_topo.end_x = end_x;
      temp_topo.end_y = end_y;
      temp_topo.cur_time = time;
      topo_vec_.push_back(temp_topo);
      ROS_DEBUG("New topo feat added: Feature <%d> position <%d> Beginning <%f,%f> End <%f,%f>, Feature number <%d>",temp_topo.feat,temp_topo.position,temp_topo.beg_x,temp_topo.beg_y,temp_topo.end_x,temp_topo.end_y,(int)topo_vec_.size()/size_struct);
  }
}

void inter_det::TopoFeature::setCurrentTf(tf::Transform t)
{
	ROS_DEBUG("TopoFeature::setCurrentTf: Setting the current tf to <%f,%f,%f,%f>",t.getOrigin().x(),t.getOrigin().y(),t.getOrigin().z(),tf::getYaw(t.getRotation()));
	cur_tf_ = t;
}

void inter_det::TopoFeature::clearTopoFeatures(int ret_count)
{
	ROS_DEBUG("TopoFeature::clearTopoFeatures: Clearing all the topological features!");
	struct node *temp, *temp_h;
	if(node_head_ != NULL)
		temp_h = node_head_;
	else
		return;
	int end;
	int count = 0;
	while( temp_h != NULL )
	{
		count++;
		ROS_DEBUG_STREAM( "TopoFeature::clearTopoFeatures: Deleting node: " <<
				temp_h->type);
		if(temp_h->type > 4 || temp_h->type < 1)
			break;
		
		if(count >= ret_count)
		{
			delete temp_h;
			break;
		}

		ROS_DEBUG( "TopoFeature::clearTopoFeatures: before node_ref_->fr" );
		if( temp_h->fr != NULL)
		{
			ROS_DEBUG_STREAM( "TopoFeature::clearTopoFeatures: Node relation of type: " << temp_h->fr->angle);
			temp = temp_h->fr->next;
		}
		else
			temp_h = NULL;
		delete temp;
		ROS_DEBUG( "TopoFeature::clearTopoFeatures: before assign");
		temp_h = temp;
	}
	
	topo_vec_.clear();
	ROS_DEBUG( "TopoFeature::clearTopoFeatures: Finished clearing!" );
	return;
}

void inter_det::TopoFeature::printTopoFeatures()
{
  ROS_DEBUG("------------------------printTopoFeatures----------------------------");
  for(unsigned int i = 0;i < topo_vec_.size();i++)
  {
    ROS_DEBUG("Feature <%d>:\nType <%d>\nPosition <%d>\nBeginning <%f,%f> End <%f,%f>",i,topo_vec_[i].feat,topo_vec_[i].position,topo_vec_[i].beg_x,topo_vec_[i].beg_y,topo_vec_[i].end_x,topo_vec_[i].end_y);
  }
  ROS_DEBUG("---------------------------------------------------------------------");
}

int inter_det::TopoFeature::checkLineIntersection(struct topo l1, struct topo l2)
{
	float a1 = l1.end_y - l1.beg_y;
	float b1 = l1.beg_x - l1.end_x;
	float c1 = a1*l1.beg_x + b1*l1.beg_y;

	float a2 = l2.end_y - l2.beg_y;
	float b2 = l2.beg_x - l2.end_x;
	float c2 = a2*l2.beg_x + b2*l2.beg_y;

	float det = a1*b2 - a2*b1;
	float x, y;

	if (det == 0)
	{
		ROS_DEBUG("TopoFeature::checkLineIntersection: Should not come here as not parlell");
	}
	else
	{
		x = (b2*c1 - b1*c2)/det;
		y = (a1*c2 - a2*c1)/det;
	}

	if ( ((std::min(l1.beg_x - LIN_SIGMA, l1.end_x + LIN_SIGMA) <= x && x <= std::max(l1.beg_x - LIN_SIGMA, l1.end_x + LIN_SIGMA)) &&
	      (std::min(l1.beg_y - LIN_SIGMA, l1.end_y + LIN_SIGMA) <= y && y <= std::max(l1.beg_y - LIN_SIGMA, l1.end_y + LIN_SIGMA))) ||
	     ((std::min(l2.beg_x - LIN_SIGMA, l2.end_x + LIN_SIGMA) <= x && x <= std::max(l2.beg_x - LIN_SIGMA, l2.end_x + LIN_SIGMA)) &&
	      (std::min(l2.beg_y - LIN_SIGMA, l2.end_y + LIN_SIGMA) <= y && y <= std::max(l2.beg_y - LIN_SIGMA, l2.end_y + LIN_SIGMA))) ||
	     ((x > l2.beg_x && std::min(l1.beg_x,l1.end_x) > l2.beg_x) || (x < l2.beg_x && std::min(l1.beg_x,l1.end_x))))
		return FRONT;	
	else if( x > l1.end_x || y > l1.end_y )
		if( x > l2.beg_x || y > l2.beg_y )
			return LEFT;
		else if( x < l2.beg_x || y < l2.beg_y )
			return RIGHT;
	else if( x < l1.end_x || y < l1.end_y )
		return LEFT;
}

bool inter_det::TopoFeature::checkProperT(struct topo l1, struct topo l2, bool side)
{
	float a1 = l1.end_y - l1.beg_y;
	float b1 = l1.beg_x - l1.end_x;
	float c1 = a1*l1.beg_x + b1*l1.beg_y;

	float a2 = l2.end_y - l2.beg_y;
	float b2 = l2.beg_x - l2.end_x;
	float c2 = a2*l2.beg_x + b2*l2.beg_y;

	float det = a1*b2 - a2*b1;
	float x, y;

	if (det == 0)
	{
		ROS_DEBUG("TopoFeature::checkLineIntersection: Should not come here as not parlell");
	}
	else
	{
		x = (b2*c1 - b1*c2)/det;
		y = (a1*c2 - a2*c1)/det;
	}
	// RIGHT
	if(side)
	{
		if(std::abs(x-l2.end_x) < LIN_SIGMA && std::abs(y-l2.end_y) < LIN_SIGMA)
			return true;
		else 
			return false;
	}
	// LEFT
	else
	{
		if(std::abs(x-l1.beg_x) < LIN_SIGMA && std::abs(y-l1.beg_y) < LIN_SIGMA)
			return true;
		else 
			return false;
	}
}

int inter_det::TopoFeature::checkParlell(struct topo l1, struct topo l2)
{
	ROS_DEBUG_STREAM("TopoFeature::checkParlell: Method entered!");
	float para_x, para_y, begx, begy;
	bool rev;
	bool flag = true;
	float t = 0.01;
	para_x = l1.end_x - l1.beg_x;
	para_y = l1.end_y - l1.beg_y;
	begx = l1.beg_x; begy = l1.beg_y;
	(begx > l2.beg_x)? rev = false : rev = true;
	bool l1beginl2, l1endinl2, l2beginl1, l2endinl1;
	l1beginl2 = checkEncaps(l2.beg_x, l2.beg_y, l2.end_x, l2.end_y, l1.beg_x, l1.beg_y);
	l1endinl2 = checkEncaps(l2.beg_x, l2.beg_y, l2.end_x, l2.end_y, l1.end_x,l1.end_y);
	l2beginl1 = checkEncaps(l1.beg_x, l1.beg_y, l1.end_x, l1.end_y, l2.beg_x, l2.beg_y);
	l2endinl1 = checkEncaps(l1.beg_x, l1.beg_y, l1.end_x, l1.end_y, l2.end_x, l2.end_y);
	while( flag && std::abs(begx) < 10 && std::abs(begy) < 10)
	{
		ROS_DEBUG_STREAM( "TopoFeature::checkParlell: In while loop!" << begx
				 << " parax: " << para_x << " begy: " << begy
				 << " paray: " << para_y );
		begx = begx + para_x*t;
		begy = begy + para_y*t;
		if(std::abs(begx - l2.beg_x) < PAR_LIN_SIGMA && 
		   std::abs(begy - l2.beg_y) < PAR_LIN_SIGMA)
		{
			ROS_DEBUG_STREAM( "TopoFeature::checkParlell: They meet hence front" );
			if( l1beginl2 && l1endinl2 )
				return L1INL2;
			else if( l2beginl1 && l2endinl1 )
				return L2INL1;
			else if( l1endinl2 )
				return L1ENDINL2;
			else if( l2beginl1 )
				return L2BEGINL1;
			else
			{

				ROS_DEBUG_STREAM( "TopoFeature::checkParlell: No relation " << l1beginl2 << "," <<
							l1endinl2 << "," <<
							l2beginl1 << "," <<
							l2endinl1 << "," );
				return FRONT;
			}
		}
		else if(std::abs(begx - l2.end_x) < LIN_SIGMA &&
		        std::abs(begy - l2.end_y) < LIN_SIGMA)
		{
			ROS_DEBUG_STREAM( "TopoFeature::checkParlell: Same as the next line" );
			return FRONT;
		}
		if((rev && begx > l2.beg_x) || (!rev && begx < l2.beg_x))
		{
			flag = false;
		}
	}
	if( l1beginl2 && l1endinl2 )
		return L1INL2;
	else if( l2beginl1 && l2endinl1 )
		return L2INL1;
	else if( l1endinl2 )
		return L1ENDINL2;
	else if( l2beginl1 )
		return L2BEGINL1;
	else
		return LEFT;
}

bool inter_det::TopoFeature::checkEncaps(float beg_x, float beg_y, float end_x, float end_y, float x, float y)
{
	if( (std::min(beg_x - LIN_SIGMA, end_x + LIN_SIGMA) <= x && x <= std::max(beg_x - LIN_SIGMA, end_x + LIN_SIGMA)) &&
             (std::min(beg_y - LIN_SIGMA, end_y + LIN_SIGMA) <= y && y <= std::max(beg_y - LIN_SIGMA, end_y + LIN_SIGMA)) )
        {
        	  ROS_DEBUG_STREAM( "TopoFeature::checkEncaps: Point <" << x << "," << y <<
			      			"> Inside Line [(" << beg_x << "," << beg_y 
						      << ")(" << end_x << "," << end_y << ")]" );
            return true;
        }
	return false;
}
int inter_det::TopoFeature::buildRelation()
{
	ROS_DEBUG_STREAM( "inter_det::TopoFeature::buildRelation: Method entered");
	int cur, next;
	struct relation *r;
	struct node *new_node, *next_new_node;
	int node_count = 0;
	if(topo_vec_.empty())
	{
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: No features to detect returning" );
		return 0;
	}
	else
	{
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: topo_vec_ not empty!" );
	}

	float vec1[2], vec2[2];
	float dot_prod, mag_prod;
	int temp; std::string temp_str;
	float prev_beg_x, prev_end_y;
	prev_beg_x = prev_end_y = 0;
	for (unsigned int i = 0;i < topo_vec_.size();i++)
	{
		node_count++;
		cur = i;
		bool create = true;
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Before allocating space!" );
		struct relation* fr = (struct relation*)malloc(sizeof(struct relation));
		struct relation* br = (struct relation*)malloc(sizeof(struct relation));

		if ( i == (topo_vec_.size() - 1) && topo_vec_.size()>1)
		{
			ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Last node!" );
			next = 0;
			new_node = node_ref_;
			next_new_node = node_head_;
			new_node->type = END;
			ROS_DEBUG_STREAM( "TopoFeature::buildRelation: New node type: " << new_node->type
					  << "\nnew next node type: " << next_new_node->type << "last" );
		}
		else if(i == 0)
		{
			ROS_DEBUG_STREAM( "TopoFeature::buildRelation: First node!" );
			next = i + 1;
			new_node = (struct node*)malloc(sizeof(struct node));
			next_new_node = (struct node*)malloc(sizeof(struct node));
			new_node->type = BEG;
			next_new_node->type = NONE;
			new_node->info = topo_vec_[cur];
			next_new_node->info = topo_vec_[next];
			node_ref_ = new_node;
			node_head_ = new_node;
			ROS_DEBUG_STREAM( "TopoFeature::buildRelation: New node type: " << new_node->type
                                          << "\nnew next node type: " << next_new_node->type << "first" );
		}
		else
		{
			ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Middle elem" );
			next = i + 1;
			next_new_node = (struct node*)malloc(sizeof(struct node));
			new_node = node_ref_;
			next_new_node->info = topo_vec_[next];
			next_new_node->type = NONE;
			ROS_DEBUG_STREAM( "TopoFeature::buildRelation: New node type: " << new_node->type
                                          << "\nnew next node type: " << next_new_node->type << "mid" );
		}

		fr->next = next_new_node;
		br->next = new_node;

		br->same_gap = br->side_gap = fr->same_gap = fr->side_gap = false;

		br->space_x = fr->space_x = topo_vec_[next].beg_x - topo_vec_[cur].end_x;
		br->space_y = fr->space_y = topo_vec_[next].beg_y - topo_vec_[cur].end_y;
		
		float mag;

		vec1[0] = topo_vec_[cur].beg_x - topo_vec_[cur].end_x;
		vec1[1] = topo_vec_[cur].beg_y - topo_vec_[cur].end_y;
		mag = sqrt(pow(vec1[0],2) + pow(vec1[1],2));
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Vec 1: " << vec1[0] << "," << vec1[1] );
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Mag: " << mag );
		vec1[0] = vec1[0]/mag; vec1[1] = vec1[1]/mag;

		vec2[0] = topo_vec_[next].beg_x - topo_vec_[next].end_x;
		vec2[1] = topo_vec_[next].beg_y - topo_vec_[next].end_y;
		mag = sqrt(pow(vec2[0],2) + pow(vec2[1],2));
        ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Vec 2: " << vec2[0] << "," << vec2[1] );
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Mag: " << mag );
		vec2[0] = vec2[0]/mag; vec2[1] = vec2[1]/mag;

		dot_prod = vec1[0]*vec2[0] + vec1[1]*vec2[1];
		mag_prod = sqrt(pow(vec1[0],2) + pow(vec1[1],2) *
				pow(vec2[0],2) + pow(vec2[1],2));
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: First line: <" << topo_vec_[cur].beg_x <<
								         "," << topo_vec_[cur].beg_y <<
									"><" << topo_vec_[cur].end_x << 
									 "," << topo_vec_[cur].end_y << 
									 ">");
		ROS_DEBUG_STREAM(" TopoFeature::buildRelation: Sec Line: <" << topo_vec_[next].beg_x <<
								       "," << topo_vec_[next].beg_y <<
								      "><" << topo_vec_[next].end_x <<
								       "," << topo_vec_[next].end_y <<
								       ">" );
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Vec 1: " << vec1[0] << "," << vec1[1] );
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Vec 2: " << vec2[0] << "," << vec2[1] );
		ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Dot prod: " << dot_prod <<
				                             "Mag prod: " << mag_prod );

		if(dot_prod/mag_prod < -1)
			br->angle = fr->angle = M_PI;
		else if(dot_prod/mag_prod > 1)
			br->angle = fr->angle = 0;
		else
			fr->angle = br->angle = acos(dot_prod/mag_prod);

		if(std::abs(fr->angle - PER_ANG) < C_ANG_SIGMA )
		{
			ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Found a perpendicular angle!" );
			if( checkLineIntersection(topo_vec_[cur], topo_vec_[next]) == FRONT )
			{
				fr->rel_to = FRONT;
				br->rel_to = BACK;
				new_node->fr = fr;
				next_new_node->br = br;
				temp = FRONT;
				temp_str = "FRONT";
				if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
                                                topo_vec_[next].beg_x, topo_vec_[next].beg_y) > LIN_GAP_SIGMA)
                                        br->same_gap = fr->same_gap = true;

			}
			else if( checkLineIntersection(topo_vec_[cur], topo_vec_[next]) == LEFT )
			{
				fr->rel_to = LEFT;
				br->rel_to = RIGHT;
				new_node->fr = fr;
				next_new_node->br = br;
				temp = LEFT;
				temp_str = "LEFT";
				if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
                                                topo_vec_[next].beg_x, topo_vec_[next].beg_y) > LIN_GAP_SIGMA)
                                        br->side_gap = fr->side_gap = true;
			}
			else if(checkLineIntersection(topo_vec_[cur], topo_vec_[next]) == RIGHT)
			{
				fr->rel_to = RIGHT;
				br->rel_to = LEFT;
				new_node->fr = fr;
				next_new_node->br = br;
				temp = RIGHT;
				temp_str = "RIGHT";
				if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
                                                topo_vec_[next].beg_x, topo_vec_[next].beg_y) > LIN_GAP_SIGMA)
                                        br->side_gap = fr->side_gap = true;
			}
		}
		else if( std::abs(fr->angle - PAR_ANG) < C_ANG_SIGMA || 
		         std::abs(fr->angle) < C_ANG_SIGMA )
		{
			ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Found a parlell angle!" );
			int side_check = checkParlell(topo_vec_[cur], topo_vec_[next]);
			if( side_check == FRONT )
			{
				fr->rel_to = FRONT;
				br->rel_to = BACK;
				new_node->fr = fr;
				next_new_node->br = br;
				temp = FRONT;
				temp_str = "FRONT";
				if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
						topo_vec_[next].beg_x, topo_vec_[next].beg_y) > LIN_GAP_SIGMA)
					br->same_gap = fr->same_gap = true;

			}
			else if( side_check == LEFT )
			{
				fr->rel_to = LEFT;
                                br->rel_to = RIGHT;
                                new_node->fr = fr;
                                next_new_node->br = br;
                                temp = LEFT;
                                temp_str = "LEFT";
                                if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
                                                topo_vec_[next].beg_x, topo_vec_[next].beg_y) > LIN_GAP_SIGMA)
                                        br->side_gap = fr->side_gap = true;
			}
			else if( side_check == L1INL2 )
			{
				ROS_DEBUG("Deleting the next node");
				delete fr, br;
				new_node->info.beg_x = next_new_node->info.beg_x;
				new_node->info.beg_y = next_new_node->info.beg_y;
				new_node->info.end_x = next_new_node->info.end_x;
				new_node->info.end_y = next_new_node->info.end_y;
				delete next_new_node;
				new_node->fr = NULL;
				create = false;
				node_count--;
			}
			else if( side_check == L2INL1 )
			{
				ROS_DEBUG("Deleting the next node!");
				delete fr, br;
				delete next_new_node;
				new_node->fr = NULL;
				create = false;
				node_count--;
			}
			else if( side_check == L1ENDINL2 || side_check == L2BEGINL1 )
			{
				ROS_DEBUG("Deleting the next node!");
				delete fr, br;
				new_node->info.end_x = next_new_node->info.end_x;
				new_node->info.end_y = next_new_node->info.end_y;
				delete next_new_node;
				new_node->fr = NULL;
				create = false;
				node_count--;
			}
			else
			{
				ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Parlell angles should not be different than the current" );
				return(-node_count);
			}
		}
		else
		{
			ROS_DEBUG_STREAM( "TopoFeature::buildRelation: Found a wierd angle: " << fr->angle);
			return(-node_count);
		}

		prev_beg_x = node_ref_->info.beg_x;
		prev_end_y = node_ref_->info.end_y;
		if( create )
			node_ref_ = next_new_node;
	}
	return(node_count);
}

void inter_det::TopoFeature::printRelation()
{
	ROS_DEBUG_STREAM( "TopoFeature::printRelation: Method entered!" );
	if( node_head_ == NULL )
	{
		ROS_DEBUG_STREAM( "TopoFeature::printRelation: Nothing to print" );
		return;
	}
	if(topo_vec_.size() <= 1)
		return;

	node_ref_ = node_head_;
	int ent;
	std::string rel = "NONE";
	for( unsigned int i = 0;i < topo_vec_.size();i++)
	{
		ROS_DEBUG_STREAM( "TopoFeature::printRelation: Node [" << node_ref_->type <<
				 "]: Info: <" << node_ref_->info.beg_x << "," << node_ref_->info.beg_y
				 << "><" << node_ref_->info.end_x << "," << node_ref_->info.end_y
				 << ">");
		if( node_ref_->fr->rel_to == RIGHT ) rel = "RIGHT";
		else if( node_ref_->fr->rel_to == LEFT ) rel = "LEFT";
		else if( node_ref_->fr->rel_to == FRONT ) rel = "FRONT";
		else if( node_ref_->fr->rel_to == BACK ) rel = "BACK";
		ROS_DEBUG_STREAM( "TopoFeature::printRelation: Relation [" <<
					rel << "]: angle: " << node_ref_->fr->angle <<
					" space_x: " << node_ref_->fr->space_x << 
					" space_y: " << node_ref_->fr->space_y <<
					" side_gap: " << node_ref_->fr->side_gap <<
					" same_gap: " << node_ref_->fr->same_gap );

		if( node_ref_->fr != NULL ) 
			node_ref_ = node_ref_->fr->next;

	}

	ROS_DEBUG_STREAM( "TopoFeature::printRelation: Printing backwards!");
	node_ref_ = node_head_->br->next;
	for( unsigned int i = 0;i < topo_vec_.size();i++ )
    {
        ROS_DEBUG_STREAM( "TopoFeature::printRelation: Node [" << node_ref_->type <<
                                 "]: Info: <" << node_ref_->info.beg_x << "," << node_ref_->info.beg_y
                                 << "><" << node_ref_->info.end_x << "," << node_ref_->info.end_y
                                 << ">");
        if( node_ref_->br->rel_to == RIGHT ) rel = "RIGHT";
        else if( node_ref_->br->rel_to == LEFT ) rel = "LEFT";
        else if( node_ref_->br->rel_to == FRONT ) rel = "FRONT";
        else if( node_ref_->br->rel_to == BACK ) rel = "BACK";
        ROS_DEBUG_STREAM( "TopoFeature::printRelation: Relation [" <<
                                        rel << "]: angle: " << node_ref_->br->angle <<
                                        " space_x: " << node_ref_->br->space_x <<
                                        " space_y: " << node_ref_->br->space_y <<
                                        " side_gap: " << node_ref_->br->side_gap <<
                                        " same_gap: " << node_ref_->br->same_gap );
		if( node_ref_->br != NULL )
			node_ref_ = node_ref_->br->next;
    }
}

inter_det::TopoFeature::intersection inter_det::TopoFeature::identifyIntersection()
{
	bool left_space, right_space, center_space, leftf, rightf;
	leftf = rightf = left_space = right_space = center_space = false;
	if(topo_vec_.size() <= 1)
		return(logIntersection(right_space, left_space, center_space));
	ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: Method entered!" );
	if( node_head_ == NULL )
	{
		ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: No tree to traverse!" );
		pose_.type = UNKW;
		return(pose_);
	}
	else
	{
		ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: Head type: " <<
				node_head_->type);
	}

	struct node *right, *left;
	int forw_entry, back_entry;
	right = node_head_;
	left = node_head_->br->next;

	ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: right type: " <<
				node_head_->type << "left type: " << node_head_->br->next->type );

	ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: Right Line: <" <<
                                        right->info.beg_x << "," << right->info.beg_y << "><" <<
                                        right->info.end_x << "," << right->info.end_y << ">" <<
										right->fr->rel_to );

	ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: Left Line: <" <<
                                        left->info.beg_x << "," << left->info.beg_y << "><" <<
                                        left->info.end_x << "," << left->info.end_y << ">" <<
										left->br->rel_to );


	if(right->fr != NULL && left->br != NULL)
	{
		// Easy comparison for T-Intersection
		if(right->fr->next == left->br->next)
		{
			ROS_DEBUG_STREAM("TopoFeature::identifyIntersection: Checking T-Intersection");
			//Perform check for T-Intersection
			if(std::abs(right->fr->angle - PER_ANG) < ANG_SIGMA &&
					std::abs(left->br->angle - PER_ANG) < ANG_SIGMA)
				return(logIntersection((right->fr->same_gap|right->fr->side_gap), (left->br->same_gap|left->br->side_gap), false));
		}
		// Bit of complicated comparison for other intersection
		else
		{
			ROS_DEBUG_STREAM("TopoFeature::identifyIntersection: Checking other intersections");
			if(std::abs(right->info.beg_x) <  LIN_SIGMA &&
					std::abs(left->info.end_x) < LIN_SIGMA)
			{
				float left_mag = computeLength(left->info);
				float right_mag = computeLength(right->info);
				ROS_DEBUG_STREAM("TopoFeature::identifyIntersection: Left Mag: "<< left_mag << " Right Mag: " << right_mag);
				if((right_mag - left_mag) > LIN_SIGMA)
				{
					ROS_DEBUG_STREAM("TopoFeature::identifyIntersection: Must be something to Left");
					if(left->br->same_gap && 
					   (right->fr->side_gap || right->fr->same_gap))
					{
						left_space = true;
						center_space = true;
					}
					else if(left->br->same_gap &&
						(!right->fr->side_gap && !right->fr->same_gap))
						left_space = true;
				}
				else if((right_mag - left_mag) < -LIN_SIGMA)
				{
					ROS_DEBUG_STREAM("TopoFeature::identifyIntersection: Must be something to Right");
					if(right->fr->same_gap &&
					   (left->br->side_gap || left->br->same_gap))
					{
						right_space = true;
						center_space = true;
					}
					else if(right->fr->same_gap &&
						(!left->br->same_gap && !left->br->side_gap))
					{
						right_space = true;
					}
				}
				else if(std::abs(right_mag - left_mag) < LIN_SIGMA &&
						(right->fr->same_gap || right->fr->side_gap) &&
						(left->br->same_gap || left->br->side_gap))
				{
					ROS_DEBUG_STREAM("TopoFeature::identifyIntersection: Must be on both sides!");
					left_space = center_space = right_space = true;
				}
			}
		}
		return(logIntersection(right_space,left_space,center_space));
	}
}

inter_det::TopoFeature::intersection inter_det::TopoFeature::logIntersection(bool right_space, bool left_space, bool center_space)
{
    ROS_DEBUG_STREAM("TopoFeature::identifyIntersection: center: "<< center_space <<
			"left: " << left_space << "right: " << right_space);
	
    if(left_space && right_space && center_space)
    {
        	ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: FOUR_WAY" );
            pose_.type = FWI;
    }
    else if(left_space && right_space)
    {
    	struct node *right, *left;
		struct node *n = node_head_->fr->next;
		int count = 1;
		right = node_head_;
		left = node_head_->br->next;
		if(checkProperT(right->info, right->fr->next->info, true))
		{
			ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: INTERSECTION:MF RIGHT-I");
			pose_.type =  RI;
		}
		else if(checkProperT(left->info, left->br->next->info, false))
		{
			ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: INTERSECTION:MF Left-I");
			pose_.type = LI;
		}
		else
		{
        	ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: T");
            pose_.type =  TI;
        }
    }
	else if(left_space)
    {
		if(center_space)
		{
        		ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: Left-I" );
                pose_.type = LI;
		}
		else
		{
			ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: Left-T" );
			pose_.type = LT;
		}
    }
    else if(right_space)
    {
		if(center_space)
		{
        		ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: RIGHT-I" );
                pose_.type = RI;
		}
		else
		{
			ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: RIGHT-T" );
			pose_.type = RT;
		}
    }
    else
    {
        	ROS_DEBUG_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: NOTHING" );
            pose_.type = UNKW;
    }
    if(pose_.type != UNKW)
    {
	if(node_head_ != NULL)
	{
		float mp_x, mp_y;
		if(node_head_->br != NULL)
		{
			if(pose_.type == TI)
			{
				float vec[2];
				float magr, magl;

				vec[0] = (node_head_->br->next->info.beg_x -
					  node_head_->br->next->info.end_x);
				vec[1] = (node_head_->br->next->info.beg_y -
					  node_head_->br->next->info.end_y);
				magl = sqrt(pow(vec[0],2.0f) + pow(vec[1],2.0f));
				vec[0] = vec[0] / magl;
				vec[1] = vec[1] / magl;
				mp_x = node_head_->br->next->info.beg_x;// + 0.2*vec[0];
				mp_y = node_head_->br->next->info.beg_y;// + 0.2*vec[1];

				vec[0] = (node_head_->info.beg_x -
					  node_head_->info.end_x);
				vec[1] = (node_head_->info.beg_y -
					  node_head_->info.end_y);
				magr = sqrt(pow(vec[0],2.0f) + pow(vec[1],2.0f));
				vec[0] = vec[0] / magr;
				vec[1] = vec[1] / magr;
				float temp_x = node_head_->info.end_x;// + 0.2*vec[0];
				float temp_y = node_head_->info.end_y;// + 0.2*vec[1];

				mp_x = (mp_x + temp_x)/2;
				mp_y = (mp_y + temp_y)/2;
				
				if(node_head_->br->next->br->next == node_head_->fr->next)
				{
					float a[2], b[2], proj;
					a[0] = node_head_->fr->next->info.end_x - node_head_->fr->next->info.beg_x;
					a[1] = node_head_->fr->next->info.end_y - node_head_->fr->next->info.beg_y;
					b[0] = mp_x - node_head_->fr->next->info.beg_x;
					b[1] = mp_y - node_head_->fr->next->info.beg_y;

					proj = a[0]*b[0] + a[1]*b[1];

					float mag = sqrt(pow(a[0],2.0f) + pow(a[1],2.0f));
					float temp[2];
					a[0] = a[0]/mag;
					a[1] = a[1]/mag;
					ROS_INFO("MF: Proj: %f, Midpoint:(%f,%f), Vec: (%f,%f)",proj,mp_x,mp_y,a[0],a[1]);

					temp[0] = node_head_->fr->next->info.beg_x + proj*temp[0];
					temp[1] = node_head_->fr->next->info.beg_y + proj*temp[1];

					ROS_INFO_STREAM("MF Center line : (" <<
							node_head_->fr->next->info.beg_x << "," <<
							node_head_->fr->next->info.beg_y << ")(" <<
							node_head_->fr->next->info.end_x << "," <<
							node_head_->fr->next->info.end_y << ") Point: (" <<
							temp[0] << "," << temp[1] << ")");

					mp_x = (mp_x + temp[0])/2;
					mp_y = (mp_y + temp[1])/2;
				}
				pose_.p = tf::Transform(tf::Quaternion(0.0f,0.0f,0.0f,0.0f),
						tf::Vector3(mp_x,mp_y,0.0f));

			}
			else if(pose_.type == FWI /*|| pose_.type == TI*/)
			{
				float vec[2];
				float magr, magl;

				vec[0] = (node_head_->br->next->info.beg_x -
					  node_head_->br->next->info.end_x);
				vec[1] = (node_head_->br->next->info.beg_y -
					  node_head_->br->next->info.end_y);
				magl = sqrt(pow(vec[0],2.0f) + pow(vec[1],2.0f));
				vec[0] = vec[0] / magl;
				vec[1] = vec[1] / magl;
				mp_x = node_head_->br->next->info.beg_x + 1.75*vec[0];
				mp_y = node_head_->br->next->info.beg_y + 1.75*vec[1];

				vec[0] = (node_head_->info.end_x -
					  node_head_->info.beg_x);
				vec[1] = (node_head_->info.end_y -
					  node_head_->info.beg_y);
				magr = sqrt(pow(vec[0],2.0f) + pow(vec[1],2.0f));
				vec[0] = vec[0] / magr;
				vec[1] = vec[1] / magr;
				float temp_x = node_head_->info.end_x + 1.75*vec[0];
				float temp_y = node_head_->info.end_y + 1.75*vec[1];

				mp_x = (mp_x + temp_x)/2;
				mp_y = (mp_y + temp_y)/2;
				pose_.p = tf::Transform(tf::Quaternion(0.0f,0.0f,0.0f,0.0f),
						tf::Vector3(mp_x,mp_y,0.0f));
			}
			else if(pose_.type == LI || pose_.type == LT)
			{
			  	float vec[2];
				float magr, magl;
				vec[0] = (node_head_->br->next->info.beg_x - 
					  node_head_->br->next->info.end_x);
				vec[1] = (node_head_->br->next->info.beg_y - 
					  node_head_->br->next->info.end_y);
				magl = sqrt(pow(vec[0],2.0f) + pow(vec[1],2.0f));

				vec[0] = vec[0] / magl;
				vec[1] = vec[1] / magl;

				magl = magl + 0.5;

				mp_x = node_head_->br->next->info.beg_x + 0.5*vec[0];
				mp_y = node_head_->br->next->info.beg_y + 0.5*vec[1];
				
				vec[0] = (node_head_->info.end_x - 
					  node_head_->info.beg_x);
				vec[1] = (node_head_->info.end_y - 
					  node_head_->info.beg_y);
				magr = sqrt(pow(vec[0],2.0f) + pow(vec[1],2.0f));

				vec[0] = vec[0] / magr;
				vec[1] = vec[1] / magr;
				float temp_x = node_head_->info.beg_x + magl*vec[0];
				float temp_y = node_head_->info.beg_y + magl*vec[1];

				//mp_x = node_head_->br->next->info.beg_x;
				//mp_y = node_head_->br->next->info.beg_y;

				mp_x = (mp_x + temp_x)/2;
				mp_y = (mp_y + temp_y)/2;
				
				pose_.p = tf::Transform(tf::Quaternion(0.0f,0.0f,0.0f,0.0f),
						tf::Vector3(mp_x,mp_y,0.0f));
			}
			else if(pose_.type == RI || pose_.type == RT)
			{
			  	float vec[2];
				float magr, magl;
				vec[0] = (node_head_->info.end_x - 
					  node_head_->info.beg_x);
				vec[1] = (node_head_->info.end_y - 
					  node_head_->info.beg_y);

				ROS_DEBUG("Right line: (%f,%f)-(%f,%f)",node_head_->info.beg_x,node_head_->info.beg_y,node_head_->info.end_x,node_head_->info.end_y);

				magr = sqrt(pow(vec[0],2.0f) + pow(vec[1],2.0f));

				vec[0] = vec[0] / magr;
				vec[1] = vec[1] / magr;

				magr = magr + 0.5;

				mp_x = node_head_->info.end_x + 0.5*vec[0];
				mp_y = node_head_->info.end_y + 0.5*vec[1];
				ROS_DEBUG("RIGHT F: Middle right: (%f,%f)",mp_x,mp_y);

				vec[0] = (node_head_->br->next->info.beg_x - 
					  node_head_->br->next->info.end_x);
				vec[1] = (node_head_->br->next->info.beg_y -
					  node_head_->br->next->info.end_y);
				ROS_DEBUG("Left line: (%f,%f)-(%f,%f)",node_head_->br->next->info.beg_x,node_head_->br->next->info.beg_y,node_head_->br->next->info.end_x,node_head_->br->next->info.end_y);
				magl = sqrt(pow(vec[0],2.0f) + pow(vec[1],2.0f));
				vec[0] = vec[0]/magl;
				vec[1] = vec[1]/magl;
				float temp_x = node_head_->br->next->info.end_x + magr*vec[0];
				float temp_y = node_head_->br->next->info.end_y + magr*vec[1];
				ROS_DEBUG("RIGHT F: Middle left: (%f,%f)",temp_x,temp_y); 

				//mp_x = node_head_->info.end_x;
				//mp_y = node_head_->info.end_y;

				mp_x = (mp_x + temp_x)/2;
				mp_y = (mp_y + temp_y)/2;

				ROS_DEBUG("Right F: CenterMid: (%f,%f)",mp_x,mp_y);

				pose_.p = tf::Transform(tf::Quaternion(0.0f,0.0f,0.0f,0.0f),
						tf::Vector3(mp_x,mp_y,0.0f));
			}
		}
	    }
	}
	ROS_DEBUG_STREAM(" TopoFeature::identifyIntersection: Type: " << pose_.type);
	return pose_;
}
