#ifndef _DATA_TYPE_TUBE
#define _DATA_TYPE_TUBE

#include <Eigen/Eigen>
#include <unordered_map>  
#define inf 9999999.0
using namespace std;

struct tubeNode;
typedef tubeNode * NodePtr;

struct tubeNode
{     
      Eigen::Vector3d coord;
      float radius; // radius of this node

      bool valid;
      bool best;
      bool change;

      // temporary variables, only for speedup the tree rewire procedure
      int   rel_id;
      float rel_dis;

      NodePtr preNode_ptr;
      vector<NodePtr> nxtNode_ptr;

      float g; // total cost of the shortest path from this node to the root
      float f; // heuristic value of the node to the target point
      float vol; // cost of intersection volume of adjoint spheres
      float cost;
      
      tubeNode( Eigen::Vector3d coord_, float radius_, float g_, float f_, float vol_)
      {		
		coord  = coord_;
		radius = radius_;
		g      = g_;  
		f      = f_;
		vol = vol_;
            
		rel_id  = - 2;   // status undefined
		rel_dis = - 1.0; // distance undifined

		valid  = true;
		best   = false; 
      	change = false;

		preNode_ptr = NULL;
      	nxtNode_ptr.clear();
      }

      tubeNode(){}
      ~tubeNode(){}
};

#endif