// #pragma once

#include <iostream>
#include <vector>
 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/SetPen.h>

#include "../include/hwa.h"

#define AGENT_NAME "hwa"
#define NUMBER_OF_AGENTS 24
#define NODE_NAME "hwa_walk"
#define DEFAULT_TURTLE_NAME "turtle1"
#define LINE_WIDTH 3
#define INTEREST_NAME "interest"

hwa spawnInterest(ros::NodeHandle &node, std::string name, int mapMin, int mapMax);
void setPen(ros::NodeHandle &node, std::string name);
void killTurtle(ros::NodeHandle &node, std::string name);
std::vector<hwa> spawnAgents(ros::NodeHandle &node, int numberOfAgents, std::string name, int mapMin, int mapMax);
void updateAgents(std::vector<hwa> &hwaList, space::point interestPoint);

int main(int argc, char **argv) {
    srand(time(0));

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle node;

    killTurtle(node, INTEREST_NAME);
    hwa a1 = spawnInterest(node, INTEREST_NAME, 5, 5);
    a1.subscribe();

    killTurtle(node, DEFAULT_TURTLE_NAME);
    for (int i = 0; i < NUMBER_OF_AGENTS; i++)
        killTurtle(node, AGENT_NAME + std::to_string(i));

    std::vector<hwa> hwaList = spawnAgents(node, NUMBER_OF_AGENTS, AGENT_NAME, MAP_MIN + 1, MAP_MAX - 1);

    for (int i = 0; i < NUMBER_OF_AGENTS; i++)
        setPen(node, AGENT_NAME + std::to_string(i));

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        updateAgents(hwaList, a1.Postition());
        a1.update(hwaList, SPEED_MOVEMENT / 4);

        loop_rate.sleep();
    }
 
    return 0;
}

hwa spawnInterest(ros::NodeHandle &node, std::string name, int mapMin, int mapMax) {
    ros::ServiceClient spawnClient = node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;

    req.x = mapMin;
    req.y = mapMax;
    req.theta = 0;
    req.name = name;

    bool success = spawnClient.call(req, resp);

    if (success)
        ROS_INFO_STREAM("Spawned a interest point: " << resp.name);
    else 
        ROS_ERROR_STREAM("Failed to spawn an interest point: " << req.name);

    return hwa(node, name);
}

void setPen(ros::NodeHandle &node, std::string name) {
    ros::ServiceClient setPenClient = node.serviceClient<turtlesim::SetPen>(name + "/set_pen");
    turtlesim::SetPen::Request req;
    turtlesim::SetPen::Response resp;

    req.r = rand() % 256;
    req.g = rand() % 256;
    req.b = rand() % 256;
    req.width = LINE_WIDTH;
    bool success = setPenClient.call(req, resp);

    if (success)
        ROS_INFO_STREAM("Changed pen color of turtle " << name);
    else
        ROS_ERROR_STREAM("Couldn't change pen color of turtle " << name);
}

void killTurtle(ros::NodeHandle &node, std::string name) {
    ros::ServiceClient killClient = node.serviceClient<turtlesim::Kill>("kill");
    turtlesim::Kill::Request req;
    turtlesim::Kill::Response resp;

    req.name = name;
    bool success = killClient.call(req, resp);

    if (success)
        ROS_INFO_STREAM("Killed a turtle named " << name);
    else
        ROS_ERROR_STREAM("Failed to assassinate a turtle named " << name);
}

std::vector<hwa> spawnAgents(ros::NodeHandle &node, int numberOfAgents, std::string name, int mapMin, int mapMax) {
    std::vector<hwa> hwaList;

    ros::ServiceClient spawnClient = node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;

    for (int i = 0; i < numberOfAgents; i++) {
        req.x = rand() % (mapMax - mapMin + 1) + mapMin;
        req.y = rand() % (mapMax - mapMin + 1) + mapMin;
        req.theta = math::deg2rad(rand() % (360 + 1) - 180);
        req.name = name + std::to_string(i);

        bool success = spawnClient.call(req, resp);

        if (success) {
            ROS_INFO_STREAM("Spawned a turtle named " << resp.name);
            hwaList.push_back(hwa(node, resp.name));
        }
        else {
            ROS_ERROR_STREAM("Failed to spawn a turtle named " << req.name);
        }
    }

    for (int i = 0; i < hwaList.size(); i++) {
        hwaList[i].subscribe();
    }

    return std::move(hwaList);
}

void updateAgents(std::vector<hwa> &hwaList, space::point interestPoint) {
    for (int i = 0; i < hwaList.size(); i++) {
        hwaList[i].update(hwaList, interestPoint);
    }
}