#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>

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
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include "graph_searcher.h"
#include "backward.hpp"
//new 
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
// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    
double _x_l, _x_h, _y_l, _y_h, _z_l, _z_h;  // For random map simulation : map boundary
double _vel_max, _path_find_limit, _sensing_range, _sample_portion, _goal_portion, _refine_portion;
double _safety_margin, _search_margin, _max_radius, _planning_rate;
int _max_samples;
double CT;
visualization_msgs::MarkerArray path_vis;
Vector3d _start_pos, _end_pos;

// useful global variables
bool _has_map   = false, stop_plan = false;

Vector3d _start_pt,_target_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;


Eigen::MatrixXd _Path;
Eigen::VectorXd _Radius;

// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher  _grid_map_vis_pub, _RRTstar_path_vis_pub;
ros::Publisher  _vis_corridor_pub;

RRTstarPreparatory * _RRTstar_preparatory     = new RRTstarPreparatory();
//new 
safeRegionRrtStar _rrtPathPlaner; 

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void visRRTstarPath(vector<Vector3d> nodes);
void planInitialPath();
void visFlightCorridor(MatrixXd path, VectorXd radius);
void calcuMetrixs(MatrixXd path, VectorXd radius);

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if(stop_plan) return;
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    _target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    // pathFinding(_start_pt, target_pt);
    //new 
    // _rrtPathPlaner.setStartPt(_start_pos, _end_pos);  
    _rrtPathPlaner.setStartPt(_start_pt, _target_pt);  
    planInitialPath();
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(_has_map ) return;
    stop_plan = true;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);
    _rrtPathPlaner.setInput(cloud);

    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        

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

    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
    stop_plan = false;
}



void planInitialPath()
{
    _rrtPathPlaner.reset();
    _rrtPathPlaner.setPt(_start_pt, _target_pt, _x_l, _x_h, _y_l, _y_h, _z_l, _z_h, _sensing_range, _max_samples, _sample_portion, _goal_portion );


    double begin_time = ros::Time::now().toSec();
    _rrtPathPlaner.SafeRegionExpansion(_path_find_limit);
   double end_time = ros::Time::now().toSec();
   CT = end_time - begin_time;

    tie(_Path, _Radius) = _rrtPathPlaner.getPath();
    repeat_count ++;
    for(int i = 0; i < _Radius.size(); i++)
    {
        Radius_set.push_back(_Radius(i));
    }
    if(repeat_count > repeat_num)
     {
        double sum = std::accumulate(Radius_set.begin(), Radius_set.end(),0.0);
        double mean  = sum/Radius_set.size();
        double var = 0;
        for(int i = 0; i < Radius_set.size(); i++)
        {
            double rad_tmp = Radius_set[i];
            var += (rad_tmp - mean) * (rad_tmp - mean);
        }
        var /= Radius_set.size();
        double stdvar = std::sqrt(var);

        std::cout << "-------------------------[SAFE] mean of radius = " << mean << std::endl;
        std::cout << "-------------------------[SAFE] std var of radius = " <<  stdvar << std::endl;
        repeat_count = 1;
     }



    // cout << "[SAFE]: path found:" << endl;
    // cout << "radius: " << _Radius << endl;
    vector<Vector3d> path_points;

    for (size_t path_idx = 0; path_idx < int(_Path.rows()); path_idx++)
    {
        auto x = _Path(path_idx,0);
        auto y = _Path(path_idx,1);
        auto z = _Path(path_idx,2);
        Vector3d temp_mat(x,y,z);
        path_points.push_back(temp_mat);
    }
    if(_Path.size()>6)
    {
        visRRTstarPath(path_points);
        visFlightCorridor(_Path, _Radius);
        calcuMetrixs(_Path, _Radius);
    }
}

void calcuMetrixs(MatrixXd path, VectorXd radius)
{
    double ATL = 0;
    for (int i=0; i < path.rows()-1;i++)
    {
        MatrixXd temp = path.row(i+1) - path.row(i);
        ATL = ATL + temp.norm();
    }
    std::cout << "[SAFE]: ATL = " << ATL << std::endl;

    
    double maxRadius = radius.minCoeff();
    double MSV = 4/3*M_PI*maxRadius*maxRadius*maxRadius;
    std::cout << "[SAFE]: MSV = " << MSV << std::endl;
    std::cout << "[SAFE]: CT = " << CT << std::endl;


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RRTMargin");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );

    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _RRTstar_path_vis_pub         = nh.advertise<visualization_msgs::Marker>("RRTstar_path_vis",1);
    _vis_corridor_pub             = nh.advertise<visualization_msgs::MarkerArray>("flight_corridor_vis",1);


    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );

    nh.param("planParam/safety_margin",   _safety_margin,   0.75);
    nh.param("planParam/search_margin",   _search_margin,   0.25);
    nh.param("planParam/max_radius",      _max_radius,     5.0);
    nh.param("planParam/sensing_range",   _sensing_range,   30.0);     
    nh.param("planParam/refine_portion",  _refine_portion,  0.80);     
    nh.param("planParam/sample_portion",  _sample_portion,  0.25);     // the ratio to generate samples inside the map range
    nh.param("planParam/goal_portion",    _goal_portion,    0.05);     // the ratio to generate samples on the goal
    nh.param("planParam/path_find_limit", _path_find_limit, 1.0);     
    nh.param("planParam/max_samples",     _max_samples,     5000000);
    nh.param("planParam/plan_rate",       _planning_rate,   10.0); 

    nh.param("dynamic/max_vec",   _vel_max,  3.0);

    nh.param("mapBoundary/lower_x", _x_l,  -25.0);
    nh.param("mapBoundary/upper_x", _x_h,   25.0);
    nh.param("mapBoundary/lower_y", _y_l,  -25.0);
    nh.param("mapBoundary/upper_y", _y_h,   25.0);
    nh.param("mapBoundary/lower_z", _z_l,    0.0);
    nh.param("mapBoundary/upper_z", _z_h,    5.0);
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;

    _x_l =  - _x_size/2.0; _x_h =  _x_size/2.0;
    _y_l =  - _y_size/2.0; _y_h = _y_size/2.0;
    _z_l = 0; _z_h = _z_size;

    _rrtPathPlaner.setParam(_safety_margin, _search_margin, _max_radius, _sensing_range);

    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    _RRTstar_preparatory  = new RRTstarPreparatory();
    _RRTstar_preparatory  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
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

void visRRTstarPath(vector<Vector3d> nodes )
{
    visualization_msgs::Marker Points, Line; 
    Points.header.frame_id = Line.header.frame_id = "world";
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
    Line.color.b   = 1.0;
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
    _RRTstar_path_vis_pub.publish(Points);
    _RRTstar_path_vis_pub.publish(Line); 
}

void visFlightCorridor(MatrixXd path, VectorXd radius)
{           
    for (auto & mk: path_vis.markers) 
      mk.action = visualization_msgs::Marker::DELETE;

    // _vis_corridor_pub.publish(path_vis);
    path_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
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