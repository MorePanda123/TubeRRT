#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include "graph_searcher.h"
#include "backward.hpp"
#include "corridor_finder.h"

using namespace std;
using namespace Eigen;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace backward {
backward::SignalHandling sh;
}

int repeat_num = 10, repeat_count = 0, false_count = 0;
vector<double> ATL_set, MSV_set, Radius_set;
int _type;

// simulation param from launch file
double _x_l, _x_h, _y_l, _y_h, _z_l, _z_h;  // For random map simulation : map boundary
double _vel_max, _path_find_limit, _sensing_range, _sample_portion, _goal_portion, _refine_portion;
double _safety_margin, _search_margin, _max_radius, _planning_rate;
int _max_samples;

double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    

// useful global variables
bool _has_map   = false, _stop_plan = false, start_pubPath = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
MatrixXd _Path;
int _max_x_id, _max_y_id, _max_z_id;
visualization_msgs::MarkerArray path_vis;

// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher  _grid_map_vis_pub, _RRTstar_path_vis_pub, _BITstar_path_vis_pub, _Informed_path_vis_pub;
ros::Publisher  _vis_corridor_pub, _RRTstar_path_pub;
ros::Timer planning_timer;

RRTstarPreparatory * _RRTstar_preparatory     = new RRTstarPreparatory();

safeRegionRrtStar _rrtPathPlaner; 

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void pathFinding(const Vector3d start_pt, const Vector3d target_pt, const int type);
void visRRTstarPath(vector<Vector3d> nodes,int type  );
void calcuMetrixs(MatrixXd path);

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{   
    // if(_stop_plan) return;
    if(!_has_map ) return;
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    _rrtPathPlaner.reset();
    _rrtPathPlaner.setPt(_start_pt, target_pt, _x_l, _x_h, _y_l, _y_h, _z_l, _z_h, _sensing_range, _max_samples, _sample_portion, _goal_portion );
    // pathFinding(_start_pt, target_pt,1); // RRT*
    pathFinding(_start_pt, target_pt,_type); //BIT*
    // _rrtPathPlaner.setStartPt(_start_pt, target_pt);  
    _stop_plan = true;
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    // _stop_plan = true;
    // if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);
    _rrtPathPlaner.setInput(cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        // a point in cloud

        // set obstalces into grid map for path planning
        _RRTstar_preparatory->setObs(pt.x, pt.y, pt.z);

        // for visualize only
        Vector3d cor_round = _RRTstar_preparatory->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "map";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
    // _stop_plan = false;
}

// Our collision checker. For this demo, our robot's state space
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {   
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state3D =
            state->as<ob::RealVectorStateSpace::StateType>();
        /**
        *
        *
        STEP 1: Extract the robot's (x,y,z) position from its state
        *
        *
        */
        auto x = (*state3D)[0];
        auto y = (*state3D)[1];
        auto z = (*state3D)[2];

        return _RRTstar_preparatory->isObsFree(x, y, z);
    }
};


class ClearanceObjective : public ob::StateCostIntegralObjective
{
    public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
    ob::StateCostIntegralObjective(si, true)
    {}
    ob::Cost stateCost(const ob::State* s) const
    {
    return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};
// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

ob::OptimizationObjectivePtr getPathMarginObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::MaximizeMinClearanceObjective(si));
}

ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));
    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 1.0);
    opt->addObjective(clearObj, 0.0);
    return ob::OptimizationObjectivePtr(opt);
}




void pathFinding(const Vector3d start_pt, const Vector3d target_pt, const int type)
{
    // Construct the robot state space in which we're planning. 
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));

    // Set the bounds of space to be in [0,1].
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, - _x_size * 0.5);
    bounds.setLow(1, - _y_size * 0.5);
    bounds.setLow(2, 0.0);

    bounds.setHigh(0, + _x_size * 0.5);
    bounds.setHigh(1, + _y_size * 0.5);
    bounds.setHigh(2, _z_size);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setup();

    // Set our robot's starting state
    ob::ScopedState<> start(space);
    /**
    *
    *
    STEP 2: Finish the initialization of start state
    *
    *
    */ //  todo can  I simply it?
    start[0] = (&start_pt)->operator[](0)  ;
    start[1] = (&start_pt)->operator[](1)  ;
    start[2] = (&start_pt)->operator[](2)  ;

    // Set our robot's goal state
    ob::ScopedState<> goal(space);
    /**
    *
    *
    STEP 3: Finish the initialization of goal state
    *
    *
    */
    goal[0] = (&target_pt)->operator[](0)  ;
    goal[1] = (&target_pt)->operator[](1)  ;
    goal[2] = (&target_pt)->operator[](2)  ;
    // Create a problem instance

    /**
    *
    *
    STEP 4: Create a problem instance, 
    please define variable as pdef
    *
    *
    */
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Set the optimization objective
    /**
    *
    *
    STEP 5: Set the optimization objective, the options you can choose are defined earlier:
    getPathLengthObjective() and getThresholdPathLengthObj()
    *
    *
    */
    // pdef->setOptimizationObjective(getPathLengthObjective(si));
    // pdef->setOptimizationObjective(getPathMarginObjective(si));
    
    // pdef->setOptimizationObjective(getThresholdPathLengthObj(si));
    if (type == 3)
    {
        pdef->setOptimizationObjective(getBalancedObjective(si));
    }else
    {
        // pdef->setOptimizationObjective(getPathLengthObjective(si));
        pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
        // pdef->setOptimizationObjective(getThresholdPathLengthObj(si));
    }

    // Construct our optimizing planner using the RRTstar algorithm.
    /**
    *
    *
    STEP 6: Construct our optimizing planner using the RRTstar algorithm, 
    please define varible as optimizingPlanner
    *
    *
    */
   ob::PlannerStatus solved;
   if (type == 1)
   {
       ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();
        solved = optimizingPlanner->solve(_path_find_limit);
   }else if (type == 2)
   {
       ob::PlannerPtr optimizingPlanner(new og::BiTRRT(si) );
       optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();
        solved = optimizingPlanner->solve(_path_find_limit);
   }else
   {
       ob::PlannerPtr optimizingPlanner(new og::InformedRRTstar(si));
       optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();
        solved = optimizingPlanner->solve(_path_find_limit);
   }
   
    // 
    
    // 

    // Set the problem instance for our planner to solve
    
   

    // attempt to solve the planning problem within one second of
    // planning time
    

  
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
        // path->print(std::cout);

        // --------------------test convert og path into vector-----------------
        // ob::RealVectorStateSpace::StateType *state1 = path->getState(0)->as<ob::RealVectorStateSpace::StateType>();
        // auto x = (*state1)[0];
        // auto y = (*state1)[1];
        // auto z = (*state1)[2];
        // Vector3d temp_mat(x,y,z);
        // std::cout << temp_mat << std::endl;
        //  --------------------------------------------------------------------------------------

        vector<Vector3d> path_points;
        _Path.resize(path->getStateCount(),3);
        
        // VectorXd radius_rviz;
        for (int path_idx = 0; path_idx < path->getStateCount(); path_idx++)
        {
            ob::RealVectorStateSpace::StateType *state = path->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>();
            auto x = (*state)[0];
            auto y = (*state)[1];
            auto z = (*state)[2];
            Vector3d temp_mat(x,y,z);
            path_points.push_back(temp_mat);
            _Path(path_idx,0) =  static_cast<double>(x);
            _Path(path_idx,1) =  static_cast<double>(y);
            _Path(path_idx,2) =  static_cast<double>(z);
        }
        visRRTstarPath(path_points,type);     
        // calcuMetrixs(_Path);
        start_pubPath = true;
    }
    
   
}

void calcuMetrixs(MatrixXd path)
{
    double ATL = 0;
    VectorXd Radius = VectorXd::Zero(path.rows());

    Vector3d temp = path.row(0);
    double radius = _rrtPathPlaner.findRadius(temp);
    Radius_set.push_back(radius);
    Radius(0) = radius;

    for (int i=0; i < path.rows()-1;i++)
    {
        MatrixXd temp = path.row(i+1) - path.row(i);
        ATL = ATL + temp.norm();
        Vector3d temp_r = path.row(i+1);
        double radius = _rrtPathPlaner.findRadius(temp_r);
        Radius_set.push_back(radius);
        Radius(i+1) = radius;
    }
    // std::cout << "[BIT*]: path found" << std::endl;
    // std::cout << "[BIT*]: ATL = " << ATL << std::endl;
    ATL_set.push_back(ATL);

    double maxRadius = Radius.minCoeff();
    double MSV = 4/3*M_PI*maxRadius*maxRadius*maxRadius;
    // std::cout << "[BIT*]: MSV = " << MSV << std::endl;
    // std::cout << "[SAFE]: CT = " << CT << std::endl;
    MSV_set.push_back(MSV);
}

void planningCb(const ros::TimerEvent& event)
{
    if(!start_pubPath ) return;

    nav_msgs::Path nav_path;
    nav_path.header.frame_id = "map";
    nav_path.header.stamp = ros::Time::now();
    for (int path_idx = 0; path_idx < _Path.rows(); path_idx++)
    {
         geometry_msgs::PoseStamped pose_stmp;
        pose_stmp.header.frame_id = "map";
        pose_stmp.header.stamp = ros::Time::now();
        pose_stmp.pose.position.x = _Path(path_idx,0);
        pose_stmp.pose.position.y = _Path(path_idx,1);
        pose_stmp.pose.position.z = _Path(path_idx,2);
        nav_path.poses.push_back(pose_stmp);
    }
    _RRTstar_path_pub.publish(nav_path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_plan");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );

    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _RRTstar_path_vis_pub         = nh.advertise<visualization_msgs::Marker>("RRTstar_path_vis",1);
    _RRTstar_path_pub         = nh.advertise<nav_msgs::Path>("RRTstar_path",1);
    _BITstar_path_vis_pub = nh.advertise<visualization_msgs::Marker>("BITstar_path_vis",1);
    _Informed_path_vis_pub = nh.advertise<visualization_msgs::Marker>("Informed_path_vis",1);
     _vis_corridor_pub             = nh.advertise<visualization_msgs::MarkerArray>("sphere_vis",1);


    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );


    nh.param("planParam/safety_margin",   _safety_margin,   0.0);
    nh.param("planParam/search_margin",   _search_margin,   0.0);
    nh.param("planParam/max_radius",      _max_radius,      3.0);
    nh.param("planParam/sensing_range",   _sensing_range,   30.0);     
    nh.param("planParam/refine_portion",  _refine_portion,  0.80);     
    nh.param("planParam/sample_portion",  _sample_portion,  0.25);     // the ratio to generate samples inside the map range
    nh.param("planParam/goal_portion",    _goal_portion,    0.05);     // the ratio to generate samples on the goal
    nh.param("planParam/path_find_limit", _path_find_limit, 1.0);     
    nh.param("planParam/max_samples",     _max_samples,     5000000);
    nh.param("planParam/plan_rate",       _planning_rate,   10.0); 
     nh.param("mapBoundary/lower_x", _x_l,  -25.0);
    nh.param("mapBoundary/upper_x", _x_h,   25.0);
    nh.param("mapBoundary/lower_y", _y_l,  -25.0);
    nh.param("mapBoundary/upper_y", _y_h,   25.0);
    nh.param("mapBoundary/lower_z", _z_l,    0.0);
    nh.param("mapBoundary/upper_z", _z_h,    5.0);
    _x_l =  - _x_size/2.0; _x_h =  _x_size/2.0;
    _y_l =  - _y_size/2.0; _y_h = _y_size/2.0;
    _z_l = 0; _z_h = _z_size;
   _rrtPathPlaner.setParam(_safety_margin, _search_margin, _max_radius, _sensing_range);
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);
    nh.param("type",_type, 1);

    _x_size = _x_size + 6;
    _y_size = _y_size + 6;
    
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    _RRTstar_preparatory  = new RRTstarPreparatory();
    _RRTstar_preparatory  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    planning_timer = nh.createTimer(ros::Duration(0.2), planningCb);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete _RRTstar_preparatory;
    return 0;
}

void visRRTstarPath(vector<Vector3d> nodes,int type )
{
    visualization_msgs::Marker Points, Line; 
    Points.header.frame_id = Line.header.frame_id = "map";
    Points.header.stamp    = Line.header.stamp    = ros::Time::now();
    Points.ns              = Line.ns              = "path_plan/RRTstarPath";
    Points.action          = Line.action          = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id   = 1;
    Points.type = visualization_msgs::Marker::POINTS;
    Line.type   = visualization_msgs::Marker::LINE_STRIP;

    Points.scale.x = _resolution/2; 
    Points.scale.y = _resolution/2;
    Line.scale.x   = _resolution/2;

    //points are green and Line Strip is blue
    Points.color.g = 1.0f;
    Points.color.a = 1.0;
    if (type == 1)
    {
        Line.color.g   = 0.0;
        Line.color.r   = 0.0;
        Line.color.b   = 0.0;
    }else if (type == 2)
    {
        Line.color.g   = 1.0;
        Line.color.r   = 0.0;
        Line.color.b   = 1.0;
    }else
    {
        Line.color.g   = 0.0;
        Line.color.r   = 1.0;
        Line.color.b   = 1.0;
    }
    
    Line.color.a   = 1.0;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Points.points.push_back(pt);
        Line.points.push_back(pt);
    }
    if(type==1)
    {
        _RRTstar_path_vis_pub.publish(Points);
        _RRTstar_path_vis_pub.publish(Line); 
    }else if (type == 2)
    {
        _BITstar_path_vis_pub.publish(Points);
        _BITstar_path_vis_pub.publish(Line); 
    }  
    else
    {
        _Informed_path_vis_pub.publish(Points);
        _Informed_path_vis_pub.publish(Line); 
    }
}

void visFlightCorridor(MatrixXd path, VectorXd radius)
{           
    for (auto & mk: path_vis.markers) 
      mk.action = visualization_msgs::Marker::DELETE;

    // _vis_corridor_pub.publish(path_vis);
    path_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "path_plan/flight_corridor";
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 0.4;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;

    for(int i = 0; i < int(path.rows()); i++){
        mk.id = i;
        mk.pose.position.x = path(i, 0); 
        mk.pose.position.y = path(i, 1); 
        mk.pose.position.z = path(i, 2); 
        mk.scale.x = 2 * radius(i);
        mk.scale.y = 2 * radius(i);
        mk.scale.z = 2 * radius(i);
        
        path_vis.markers.push_back(mk);
    }

    _vis_corridor_pub.publish(path_vis);
}