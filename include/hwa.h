#pragma once

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <chrono>
#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"

#include "vector.h"
#include "utils.cpp"

#define SPEED_MOVEMENT 1.0
#define SPEED_TURNING 1.0
#define SPEED_TURNING_MULTIPLIER 0.2
#define ANGLE_MAX 12
#define WALK_TIME 500
#define NUMBER_OF_SENSORS 6
#define SENSOR_DISTANCE 0.4
#define DEFAULT_SIZE 0.4
#define SENSOR_FIELD 180
#define MAP_MIN 0
#define MAP_MAX 11
#define VISION_RANGE 90
#define VISION_DISTANCE 5.5
#define VISION_RADIUS 2.0
#define INTEREST_TIME_MIN 6000
#define INTEREST_TIME_MAX 12000
#define WANDER_TIME_MIN 10000
#define WANDER_TIME_MAX 40000
#define INTEREST_SIZE 0.4
#define DISTANCE_TO_INTEREST_MIN 1.5
#define DISTANCE_TO_INTEREST_MAX 4.5
#define DISTANCE_TO_INTEREST_BUMP 1.75

class hwa {
    private:
        ros::Subscriber currentRotationSubscriber;
        ros::Publisher velocityPublisher;

        geometry_msgs::Pose2D currentPose;
        geometry_msgs::Twist command;

        space::vector sensors[NUMBER_OF_SENSORS];
        space::vector sensors_rotated[NUMBER_OF_SENSORS];

        std::chrono::steady_clock::time_point timerBegin;

        double direction;
        double directionMultiplier;

        ros::NodeHandle &node;
        std::string name;

        double size;

        space::vector visionVectors[2];
        space::vector visionVectorsRotated[2];
        bool interestPointSpotted;
        bool isWandering;
        std::chrono::steady_clock::time_point interestTimer;
        std::chrono::steady_clock::time_point wanderTimer;
        int interestTime;
        int wanderTime;

        void setup() {
            command.linear.x = SPEED_MOVEMENT;
            command.linear.y = 0.0;
            command.linear.z = 0.0;
            command.angular.x = 0.0;
            command.angular.y = 0.0;
            command.angular.z = 0.0;

            double rotationPerSensor = (double)math::deg2rad(SENSOR_FIELD) / (double)(NUMBER_OF_SENSORS - 1);
            for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
                sensors[i] = space::vector(0, -SENSOR_DISTANCE, 0).rotatedAroundZ((rotationPerSensor * i) + ((180 - SENSOR_FIELD) / 2));
            }

            std::chrono::steady_clock::time_point timerStart = std::chrono::steady_clock::now();

            size = DEFAULT_SIZE;

            direction = 0.0;
            directionMultiplier = 0.0;

            visionVectors[0] = space::vector(1, 0, 0).rotatedAroundZ(math::deg2rad(VISION_RANGE / 2));
            visionVectors[1] = space::vector(1, 0, 0).rotatedAroundZ(-math::deg2rad(VISION_RANGE / 2));
            visionVectorsRotated[0] = visionVectors[0];
            visionVectorsRotated[1] = visionVectors[1];

            interestPointSpotted = false;
            isWandering = true;

            wanderTime = wanderTime = rand() % (int)(WANDER_TIME_MAX - WANDER_TIME_MIN + 1) + WANDER_TIME_MIN;
            wanderTimer =  std::chrono::steady_clock::now();

            // for (int i = -180; i <= 180; i++) {
            //     space::vector v6 (1, 0, 0);
            //     v6.rotateAroundZ(math::deg2rad(i));
            //     std::cout << i << " : " << isInVision(v6) << std::endl;
            // }

        }

        void updatePosition(const turtlesim::PoseConstPtr &currentPosePtr) {
            currentPose.x = currentPosePtr->x;
            currentPose.y = currentPosePtr->y;
            currentPose.theta = currentPosePtr->theta;

            for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
                sensors_rotated[i] = sensors[i].rotatedAroundZ(currentPose.theta) + space::vector(currentPose.x, currentPose.y, 0);
            }

            visionVectorsRotated[0] = visionVectors[0].rotatedAroundZ(currentPose.theta);
            visionVectorsRotated[1] = visionVectors[1].rotatedAroundZ(currentPose.theta);
        }

        bool fixCollision(std::vector<hwa> &hwaList, space::point interestPoint) {
            space::vector vect (0, 0, 0);

            vect += fixWallCollision();
            vect += fixHwaCollision(hwaList);
            vect += fixInterestCollision(interestPoint);
            vect = vect.normalized() + space::vector(currentPose.x, currentPose.y, 0);

            if (vect != space::vector(0, 0, 0)) {
                turnToPoint(vect);
                return true;
            }
            else {
                generateDirection(WALK_TIME, ANGLE_MAX);
                return false;
            }
        }

        bool fixCollision(std::vector<hwa> &hwaList) {
            space::vector vect (0, 0, 0);

            vect += fixWallCollision();
            vect += fixHwaCollision(hwaList);
            vect = vect.normalized() + space::vector(currentPose.x, currentPose.y, 0);

            if (vect != space::vector(0, 0, 0)) {
                turnToPoint(vect);
                return true;
            }
            else {
                generateDirection(WALK_TIME, ANGLE_MAX);
                return false;
            }
        }

        space::vector fixWallCollision() {
            space::vector vect (0, 0, 0);

            double distanceMin = std::sqrt(std::pow(currentPose.x - MAP_MIN, 2) + std::pow(currentPose.y - MAP_MIN, 2));
            double distanceMax = std::sqrt(std::pow(currentPose.x - MAP_MAX, 2) + std::pow(currentPose.y - MAP_MAX, 2));
            double multiplier = std::min(distanceMin, distanceMax);

            for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
                if (sensors_rotated[i].x < MAP_MIN || sensors_rotated[i].x > MAP_MAX || sensors_rotated[i].y < MAP_MIN || sensors_rotated[i].y > MAP_MAX) {
                    vect += (sensors_rotated[i] - space::vector(currentPose.x, currentPose.y, 0)) * (-1);
                    directionMultiplier += 1 / multiplier;
                }
            }

            return vect;
        }

        space::vector fixHwaCollision(std::vector<hwa> &hwaList) {
            space::vector vect (0, 0, 0);

            for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
                for (int j = 0; j < hwaList.size(); j++) {
                    double dx = sensors_rotated[i].x - hwaList[j].currentPose.x;
                    double dy = sensors_rotated[i].y - hwaList[j].currentPose.y;
                    double distanceLinear = std::sqrt((dx * dx) - (dy * dy));

                    if (distanceLinear <= hwaList[j].size && &hwaList[j] != this) {
                        vect += (sensors_rotated[i] - space::vector(currentPose.x, currentPose.y, 0)) * (-1);
                        directionMultiplier += 1 / distanceLinear;
                    }
                }
            }

            return vect;
        }

        space::vector fixInterestCollision(space::point interestPoint) {
            space::vector vect (0, 0, 0);

            for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
                double dx = sensors_rotated[i].x - interestPoint.x;
                double dy = sensors_rotated[i].y - interestPoint.y;
                double distanceLinear = std::sqrt((dx * dx) - (dy * dy));

                if (distanceLinear <= INTEREST_SIZE) {
                    vect += (sensors_rotated[i] - space::vector(currentPose.x, currentPose.y, 0)) * (-1);
                    directionMultiplier += 1 / distanceLinear;
                }
            }

            return vect;
        }

        void generateDirection(int walkTime, int angleMax) {
            std::chrono::steady_clock::time_point timerEnd = std::chrono::steady_clock::now();
            int timePassed = std::chrono::duration_cast<std::chrono::milliseconds>(timerEnd - timerBegin).count();

            if (timePassed >= walkTime) {
                double degrees = (double)(rand() % (2 * angleMax - 1) - angleMax) + (double)rand() / (double)RAND_MAX;
                direction = math::deg2rad(degrees);
                fixDirection(direction);
                timerBegin = std::chrono::steady_clock::now();
            }
        }

        void turnToPoint(space::point p) {
            double dx = p.x - currentPose.x;
            double dy = p.y - currentPose.y;
            double dtheta = atan2(dy, dx);
            direction = dtheta - currentPose.theta;
            fixDirection(direction);
        }

        void fixDirection(double &dir) {
            if (dir > M_PI)
                dir -= 2 * M_PI;
            else if (dir < -M_PI)
                dir += (2 * M_PI);
        }

        bool isInVision (space::point p) {
            space::vector v (p.x - currentPose.x, p.y - currentPose.y, 0);
            v.rotateAroundZ(-currentPose.theta + math::deg2rad(-90));

            if (v.length() <= VISION_RADIUS) {
                setupInterest(INTEREST_TIME_MIN, INTEREST_TIME_MAX);
                return true;
            }
            else if (v.angle(visionVectors[0]) <= 0 && v.angle(visionVectors[1]) >= 0) {
                if (v.length() <= VISION_DISTANCE) {
                    setupInterest(INTEREST_TIME_MIN, INTEREST_TIME_MAX);
                    return true;
                }
            }

            return false;
        }

        void setupInterest(double timeMin, double timeMax) {
            interestPointSpotted = true;
            interestTimer = std::chrono::steady_clock::now();
            interestTime = rand() % (int)(timeMax - timeMin + 1) + timeMin;

            // std::cout << name << " will look for " << std::to_string(interestTime / 1000) << " seconds" << std::endl;
        }

        double getHeadingError(space::point &p) {
            double dx = p.x - currentPose.x;
            double dy = p.y - currentPose.y;
            double theta = atan2(dy, dx);
            double dtheta = theta - currentPose.theta;
            fixDirection(dtheta);
            return dtheta;
        }

        void moveToPoint(space::point &p, double tolerateMin, double tolerateMax) {
            double distanceToPoint = std::sqrt(std::pow(p.x - currentPose.x, 2) + std::pow(p.y - currentPose.y, 2));
            double direction = getHeadingError(p);
            
            double distanceFromInterest = rand() % 101;
            distanceFromInterest = math::map(0, 100, tolerateMin, tolerateMax, distanceFromInterest);
            if (std::abs(distanceToPoint) > distanceFromInterest) {
                command.linear.x = distanceToPoint > 1.0 ? 1.0 : distanceToPoint;
                command.angular.z = direction > 1.0 ? 1.0 : direction;
            }
            else {
                command.linear.x = 0.0;
                command.angular.z = direction > 1.0 ? 1.0 : direction;
            }
        }

    public:
        hwa(ros::NodeHandle &_node, std::string _name) : node(_node), name(_name) {
            setup();
        }

        void subscribe() {
            currentRotationSubscriber = node.subscribe(name + "/pose", 0, &hwa::updatePosition, this);
            velocityPublisher = node.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 0);
        }

        void update(std::vector<hwa> &hwaList, space::point interestPoint) {
            if (!interestPointSpotted) {
                if (!isWandering) {
                    isInVision(interestPoint);
                }
                else {
                    std::chrono::steady_clock::time_point timerEnd = std::chrono::steady_clock::now();
                    int timePassed = std::chrono::duration_cast<std::chrono::milliseconds>(timerEnd - wanderTimer).count();

                    if (timePassed >= wanderTime) {
                        isWandering = false;
                    }
                }

                fixCollision(hwaList, interestPoint);
                command.angular.z = (SPEED_TURNING + SPEED_TURNING_MULTIPLIER * directionMultiplier) * direction;
                velocityPublisher.publish(command);
                directionMultiplier = 0.0;
            }
            else {
                moveToPoint(interestPoint, DISTANCE_TO_INTEREST_MIN, DISTANCE_TO_INTEREST_MAX);
                
                double distanceToInterest = std::sqrt(std::pow(interestPoint.x - currentPose.x, 2) + std::pow(interestPoint.y - currentPose.y, 2));
                if (distanceToInterest > DISTANCE_TO_INTEREST_BUMP) {
                    bool collision = fixCollision(hwaList, interestPoint);
                    if (collision) {
                        command.angular.z = (SPEED_TURNING + SPEED_TURNING_MULTIPLIER * directionMultiplier) * direction;
                        directionMultiplier = 0.0;
                    }
                }

                velocityPublisher.publish(command);

                std::chrono::steady_clock::time_point timerEnd = std::chrono::steady_clock::now();
                int timePassed = std::chrono::duration_cast<std::chrono::milliseconds>(timerEnd - interestTimer).count();
                
                if (timePassed >= interestTime) {
                    interestPointSpotted = false;
                    isWandering = true;
                    wanderTime = rand() % (int)(WANDER_TIME_MAX - WANDER_TIME_MIN + 1) + WANDER_TIME_MIN;
                    wanderTimer = std::chrono::steady_clock::now();
                    command.linear.x = SPEED_MOVEMENT;

                    // std::cout << name << " will wander for " << std::to_string(wanderTime / 1000) << " seconds" << std::endl;
                }
            }
        }

        void update(std::vector<hwa> &hwaList, double movementSpeed) {
            fixCollision(hwaList);
            command.linear.x = movementSpeed;
            command.angular.z = (SPEED_TURNING + SPEED_TURNING_MULTIPLIER * directionMultiplier) * direction;
            velocityPublisher.publish(command);
            directionMultiplier = 0.0;
        }

        space::point Postition() {
            return space::point(currentPose.x, currentPose.y, 0);
        }

        std::string toString(int precision = DEFAULT_PRECISION) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(precision) 
                << "current x: " << currentPose.x << std::endl
                << "current y: " << currentPose.y << std::endl
                << "current theta: rad: " << currentPose.theta << " deg: " << math::rad2deg(currentPose.theta) << std::endl << std::endl;

            ss << "Sensors relative to hwa:" << std::endl;
            for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
                space::vector tmp = sensors_rotated[i];
                tmp.x -= currentPose.x;
                tmp.y -= currentPose.y;
                ss << tmp.toString() << std::endl;
            }

            return ss.str();
        }


};