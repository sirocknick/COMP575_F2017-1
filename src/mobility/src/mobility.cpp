#include <ros/ros.h>
//
// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

// ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "RotationalController.h"
#include "TranslationalController.h"
#include "SearchController.h"
#include "Pose.h"
#include "TargetState.h"

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"

#include <signal.h>
#include <math.h>

using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;

TranslationalController translational_controller;
RotationalController rotational_controller;
RotationalController rotational_translational_controller;
SearchController search_controller;

string rover_name;
char host[128];
bool is_published_name = false;

int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

pose current_location;
pose goal;
pose alternative_location;
pose saved_location;

int claimed_target_id;
const int HOME_APRIL_TAG_ID = 256;
const int TOTAL_NUMBER_RESOURCES = 256;
vector<TargetState> targets;

int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;
double current_time_for_paper = 0.0;
int targets_home_size = 0;
float my_distance = 0.0;
vector <float> distances;
// Set true when we are inside the center circle and we need to drop the block,
// back out, and reset the boolean cascade.
// New variables for improved obstacle avoidance and pickup of targets while keeping with waypoint based search
bool obstacle_encountered = false;
bool is_resource_picked_up = false;
// central collection point has been seen (aka the nest)
bool is_home_seen = false;
std_msgs::Int16 targetDetected; // ID of detected target
bool targetsCollected[256] = {0}; // array of booleans indicating whether each target ID has been found.
// state machine states
#define SEARCH_STATE 0
#define DROPOFF_STATE 1
#define RESUME_SEARCH_STATE 2
#define OBSTACLE_STATE 3

int current_state = SEARCH_STATE;
int previous_state = SEARCH_STATE;

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher target_collected_publisher;
ros::Publisher targetPickUpPublish;
ros::Publisher targetDropOffPublish;
ros::Publisher angular_publisher;
ros::Publisher messagePublish;
ros::Publisher debug_publisher;
ros::Publisher pickup_publisher;
ros::Publisher info_for_paper_publisher;
ros::Publisher project_pickup_publisher;
ros::Publisher project_arrival_publisher;
//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber targetsCollectedSubscriber;
ros::Subscriber messageSubscriber;
ros::Subscriber pickup_subscriber;
ros::Subscriber project_pickup_subscriber;
ros::Subscriber project_arrival_subscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// OS Signal Handler
void sigintEventHandler(int signal);

// Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message);
void modeHandler(const std_msgs::UInt8::ConstPtr &message);
void targetHandler(const shared_messages::TagsImage::ConstPtr &tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr &message); // 
void odometryHandler(const nav_msgs::Odometry::ConstPtr &message);
void mobilityStateMachine(const ros::TimerEvent &);
void publishStatusTimerEventHandler(const ros::TimerEvent &event);
void killSwitchTimerEventHandler(const ros::TimerEvent &event);
void pickupHandler(const std_msgs::Int16::ConstPtr &message);
void projectArrivalHandler(const std_msgs::Float32::ConstPtr &message);
void projectPickupHandler(const std_msgs::Float32::ConstPtr &message);

//Utility functions
float distanceToHome();
void deleteFromDistanceVector(float distance);
void insertIntoDistanceVector(float distance);
double computeGoalTheta();
double computeDistanceBetweenWaypoints(pose final_location, pose start_location);
void debugWaypoints();
void debugRotate();
void debugTranslate(double distance_);
void debugRandom();
void visitRandomLocation();
bool isGoalReachedR();
bool isGoalReached();
void reportDetected(int tag_id);
void process_resources(const shared_messages::TagsImage::ConstPtr &message);
void process_home_tags(const shared_messages::TagsImage::ConstPtr &message);
pose getAlternativeLocation();
pose getCurrentLocation();
pose getHomeLocation();
pose getSavedLocation();
pose getNextSearchLocation();
pose getCurrentSearchLocation();
bool isGoalReached();
void automodeStateMachine();
void driveTowardsGoal();

int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostName(host);

    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    claimed_target_id = -1; // initialize target claimed
    targetDetected.data = -1;
    if (argc >= 2)
    {
        rover_name = argv[1];
        cout << "Welcome to the world of tomorrow " << rover_name << "!  Mobility module started." << endl;
    } else
    {
        rover_name = hostName;
        cout << "No Name Selected. Default is: " << rover_name << endl;
    }
    search_controller = SearchController(rover_name);
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (rover_name + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((rover_name + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((rover_name + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((rover_name + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((rover_name + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, odometryHandler);
    project_pickup_subscriber = mNH.subscribe("project_pickup",10, projectPickupHandler);
    project_arrival_subscriber = mNH.subscribe("project_arrival",10, projectArrivalHandler);
    pickup_subscriber = mNH.subscribe("pickup", 10, pickupHandler );
    status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    target_collected_publisher = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((rover_name + "/targetPickUpImage"), 1, true);
    targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((rover_name + "/targetDropOffImage"), 1, true);
    angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);
    debug_publisher = mNH.advertise<std_msgs::String>("/debug", 1, true);
    pickup_publisher = mNH.advertise<std_msgs::Int16>(("pickup"), 1, true);
    project_pickup_publisher = mNH.advertise<std_msgs::Float32>(("project_pickup"),6,true);
    project_arrival_publisher = mNH.advertise<std_msgs::Float32>(("project_arrival"),6,true);
    info_for_paper_publisher = mNH.advertise<std_msgs::String>("info_for_paper", 10, true);

    ros::spin();
    return EXIT_SUCCESS;
}

pose getAlternativeLocation() {
    return alternative_location;
}
pose getNextSearchLocation() {
    return search_controller.getNextWaypoint(current_location);
}
pose getCurrentSearchLocation() {
    return search_controller.getCurrentWaypoint(current_location);
}
pose getHomeLocation() {
    pose home_location;
    home_location.x = 0;
    home_location.y = 0;
    home_location.theta = 0;
    return home_location;
}
pose getCurrentLocation() {
    return current_location;
}
pose getSavedLocation() {
    return saved_location;
}

void driveTowardsGoal() {
    double angular_velocity = rotational_translational_controller.calculateVelocity(current_location, goal);
    double linear_velocity = 0.00;
    if (!isGoalReachedR() && !obstacle_encountered && (current_state == SEARCH_STATE || current_state == RESUME_SEARCH_STATE)) {
        linear_velocity = 0.05;
    }
    else if (!isGoalReached()) {
        linear_velocity = translational_controller.calculateVelocity(current_location, goal);
    }
    setVelocity(linear_velocity, angular_velocity);
}

void automodeStateMachine() {
    std_msgs::String state_machine_msg;
    if (obstacle_encountered) {
        obstacle_encountered = false;
        goal = getAlternativeLocation();
    }
    else {
    switch(current_state) {
        case SEARCH_STATE:
            state_machine_msg.data = "SEARCH";
            goal = getCurrentSearchLocation();
            if (is_resource_picked_up) {
                saved_location = current_location;
                saved_location.x += 0.75*cos(current_location.theta);
                saved_location.y += 0.75*sin(current_location.theta);
                goal = getHomeLocation();
                my_distance = distanceToHome();
                std_msgs::Float32 msg;
                msg.data = my_distance;
                project_pickup_publisher.publish(msg);
                current_state = DROPOFF_STATE;
            } else if (isGoalReached()) {
                goal = getNextSearchLocation();
            }
            break;
        case DROPOFF_STATE:
            state_machine_msg.data = "DROPOFF";
            if (find(distances.begin(), distances.end(), my_distance) == distances.begin()) {
                   goal = getHomeLocation();
            } else {
                goal = current_location;
            }

            if (is_home_seen) {
                goal = getSavedLocation();
                is_resource_picked_up = false;
                is_home_seen = false;
                std_msgs::Float32 msg;
                msg.data = my_distance;
                project_arrival_publisher.publish(msg);
                current_state = RESUME_SEARCH_STATE;
            }
            break;
        case RESUME_SEARCH_STATE:
            state_machine_msg.data = "RESUME";
            goal = getSavedLocation();
            if (is_resource_picked_up) {
                goal = getHomeLocation();
                my_distance = distanceToHome();
                std_msgs::Float32 msg;
                msg.data = my_distance;
                project_pickup_publisher.publish(msg);
                current_state = DROPOFF_STATE;
            } else if (isGoalReached()) {
                goal = getCurrentSearchLocation();
                current_state = SEARCH_STATE;
            }
            break;
    }
    }
    driveTowardsGoal();
    stateMachinePublish.publish(state_machine_msg);
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    std_msgs::String state_machine_msg;
    std_msgs::String info_for_paper;

    if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
    {
        if (transitions_to_auto == 0)
        {
            // This is the first time we have clicked the Autonomous Button. Log the time and increment the counter.
            transitions_to_auto++;
            time_stamp_transition_to_auto = ros::Time::now().toSec();
        }
        automodeStateMachine();
    }
    else
    { // mode is NOT auto

        // publish current state for the operator to seerotational_controller
        std::stringstream converter;
        converter <<"CURRENT MODE: " << simulation_mode;

        state_machine_msg.data = "WAITING, " + converter.str();
    }
   // stateMachinePublish.publish(state_machine_msg);
    current_time_for_paper = ros::Time::now().toSec();
    std::stringstream ss_converter;
    ss_converter << rover_name << ", " << current_time_for_paper-time_stamp_transition_to_auto << ", " << targets_home_size << ", " << current_location.x << ", " << current_location.y << ", " << current_location.theta;
    info_for_paper.data = ss_converter.str();
    //info_for_paper_publisher.publish(info_for_paper);
}

void setVelocity(double linearVel, double angularVel)
{
    geometry_msgs::Twist velocity;
    // Stopping and starting the timer causes it to start counting from 0 again.
    // As long as this is called before the kill switch timer reaches kill_switch_timeout seconds
    // the rover's kill switch wont be called.
    killSwitchTimer.stop();
    killSwitchTimer.start();

    velocity.linear.x = linearVel * 1.5;
    velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
    velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/
void targetHandler(const shared_messages::TagsImage::ConstPtr &message) {
    // If in manual mode do not try to automatically pick up the target
    if (simulation_mode == 1 || simulation_mode == 0) return;

    if (targetDetected.data == -1) {
        process_resources(message);
    }
    else if (current_state==DROPOFF_STATE && !is_home_seen)
    {
        process_home_tags(message);
    }
}

void process_resources(const shared_messages::TagsImage::ConstPtr &message) {
    for (int i = 0; i < message->tags.data.size(); i++) {
        //check if target has not yet been collected
        if (message->tags.data[i] != 256 && !targetsCollected[message->tags.data[i]]) {
            is_resource_picked_up = true;
            //copy target ID to class variable
            targetDetected.data = message->tags.data[i];
            //publish detected target
            target_collected_publisher.publish(targetDetected);
            //publish to scoring code
            targetPickUpPublish.publish(message->image);
            std_msgs::Int16 claimed_id;
            claimed_id.data = message->tags.data[i];
            pickup_publisher.publish(claimed_id);
            return;
        }
    }
}

void process_home_tags(const shared_messages::TagsImage::ConstPtr &message) {
    for (int i = 0; i < message->tags.data.size(); i++) {
        int tag_id = message->tags.data[i];
        if (tag_id == 256) {
            is_home_seen = true;
            if (targetDetected.data != -1) {
                //publish to scoring code
                targetDropOffPublish.publish(message->image);
                targetDetected.data = -1;
                return;
            }
        }
    }
    is_home_seen = false;
}

float distanceToHome() {
    return hypotf(0-current_location.x, 0-current_location.y);
}

void projectArrivalHandler(const std_msgs::Float32::ConstPtr &message) {
    deleteFromDistanceVector(message->data);
}

void projectPickupHandler(const std_msgs::Float32::ConstPtr &message) {
     insertIntoDistanceVector(message->data);
}

void deleteFromDistanceVector(float distance) {
    distances.erase(remove(distances.begin(), distances.end(), distance), distances.end());
}

void insertIntoDistanceVector(float distance) {
    if (find(distances.begin(), distances.end(), distance) == distances.end()) {
        distances.insert( upper_bound(distances.begin(),distances.end(),distance), distance);
        std_msgs::String info_for_paper;
        std::stringstream ss_converter;
        ss_converter << "INSERTED: vector size = " << distances.size();
        info_for_paper.data = ss_converter.str();
        info_for_paper_publisher.publish(info_for_paper);
    }
}

void pickupHandler(const std_msgs::Int16::ConstPtr &message) {
    targetsCollected[message->data] = true;
}

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
    simulation_mode = message->data;
    setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
    const double PI_OVER_4 = 0.78539816339;

    if ((message->data > 0)) { // before this had & !target_collected
        double distance_to_move = 1;//rng->uniformReal(0, 1);
        if (message->data == 1) {
            // select new heading 0.2 radians to the left
            //goalLocation.theta = current_location.theta + 0.6;
            alternative_location.x = current_location.x + distance_to_move*cos(current_location.theta + PI_OVER_4);
            alternative_location.y = current_location.y + distance_to_move*sin(current_location.theta + PI_OVER_4);
            alternative_location.theta = current_location.theta + PI_OVER_4;
                  setVelocity(-0, 0.3);
        }
        // obstacle in front or on left side
        else if (message->data == 2) {
            // select new heading 0.2 radians to the right
            //goalLocation.theta = current_location.theta + 0.6;
            alternative_location.x = current_location.x + distance_to_move*cos(current_location.theta - PI_OVER_4);
            alternative_location.y = current_location.y + distance_to_move*sin(current_location.theta - PI_OVER_4);
            alternative_location.theta = current_location.theta - PI_OVER_4;
                setVelocity(-0, -0.3);
        }
        obstacle_encountered = true;
         //double angular_velocity = rotational_translational_controller.calculateVelocity(current_location, goal);

    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message)
{
    //Get (x,y) location directly from pose
    current_location.x = message->pose.pose.position.x;
    current_location.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                     message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_location.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message)
{
    if (simulation_mode == 0 || simulation_mode == 1)
    {
        setVelocity(message->linear.x, message->angular.z);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
    if (!is_published_name)
    {
        std_msgs::String name_msg;
        name_msg.data = "I ";
        name_msg.data = name_msg.data + rover_name;
        messagePublish.publish(name_msg);
        is_published_name = true;
    }

    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent &t)
{
    // No movement commands for killSwitchTime seconds so stop the rover
    setVelocity(0.0, 0.0);
    double current_time = ros::Time::now().toSec();
    ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.",
             current_time);
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

double computeGoalTheta()
{
    return atan2(goal.y - current_location.y, goal.x - current_location.x);
}
/***********************
 * CUSTOM MESSAGE HANDLERS
 ************************/
bool isGoalReachedR()
{
    goal.theta = computeGoalTheta();
    float distance_to_goal = fabs(angles::shortest_angular_distance(current_location.theta, goal.theta));
    return distance_to_goal < 0.25; // 0.25 radians around goal heading
}

bool isGoalReached()
{
    float distance_to_goal = hypotf(goal.x-current_location.x, goal.y-current_location.y);
    return distance_to_goal < 0.5; // 0.5 meter circle around target waypoint.
}
