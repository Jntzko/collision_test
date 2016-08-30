#include <iostream>
#include <stdio.h>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_datatypes.h>

class CollisionTest{

protected:
  ros::NodeHandle node_handle;
  ros::ServiceClient planning_scene_diff_client;
  moveit::planning_interface::MoveGroup arm;

public:
  CollisionTest() :
    arm("arm")
  {
    planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
  }

  ~CollisionTest(){
  }

  // spawn cylinder attached to link link_name
  void spawnAttachedObject(std::string link_name){
    moveit_msgs::ApplyPlanningScene srv;
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.is_diff = true;

    moveit_msgs::AttachedCollisionObject attached_object;

    attached_object.object.header.frame_id = "s_model_tool0";
    attached_object.object.id = "object";
    attached_object.link_name = link_name;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.push_back(0.5);
    primitive.dimensions.push_back(0.04);

    geometry_msgs::Pose pose;
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI/2, 0.0);
    pose.position.x = primitive.dimensions[0]/2;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);

    // add object to scene
    attached_object.object.operation = attached_object.object.ADD;
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);

    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
  }

  // spawn a wall in the workspace
  void spawnObstacle(){
    moveit_msgs::ApplyPlanningScene srv;
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.is_diff = true;

    moveit_msgs::CollisionObject obstacle;
    obstacle.header.frame_id = "world";
    obstacle.id = "obstacle";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 1.0;
    primitive.dimensions[2] = 1.0;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    pose.position.x = 0.75;
    pose.position.y = 0.75;
    pose.position.z = 2.0;

    obstacle.primitives.push_back(primitive);
    obstacle.primitive_poses.push_back(pose);

    // add obstacle to scene
    obstacle.operation = obstacle.ADD;
    planning_scene.world.collision_objects.push_back(obstacle);

    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);

  }

  void moveWithAttachedObject(std::string link_name){


    spawnAttachedObject(link_name);

    arm.setNamedTarget("folded");
    if(!arm.move()){
      ROS_ERROR("failed");
    }

    arm.setNamedTarget("home");
    if(!arm.move()){
      ROS_ERROR("failed");
    }
  
  }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "CollisionTest");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  CollisionTest collision_test;

  collision_test.spawnObstacle();

  collision_test.moveWithAttachedObject("s_model_tool0");
  collision_test.moveWithAttachedObject("ur5_ee_link");

  return 0;
}
