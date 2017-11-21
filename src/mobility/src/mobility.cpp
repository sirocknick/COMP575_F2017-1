#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

// ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "Pose.h"
#include "TargetState.h"
#include "RotationalController.h"
// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"

#include <signal.h>
#include <string>
#include <math.h>

using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;
string rover_name;
map<string,pose> rover_locations;
set<string> rover_names;
char host[128];
bool is_published_name = false;
int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

pose current_location;
pose goal_location;

RotationalController rotational_controller;
int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;

// state machine states
#define STATE_MACHINE_TRANSLATE 0
#define COUNT_STATE 1
int state_machine_state = COUNT_STATE;
int count_same = 0;
//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher target_collected_publisher;
ros::Publisher angular_publisher;
ros::Publisher messagePublish;
ros::Publisher debug_publisher;
ros::Publisher pose_publisher;
ros::Publisher name_publisher;
ros::Publisher global_average_heading_publisher;
ros::Publisher local_average_heading_publisher;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber pose_subscriber;
ros::Subscriber name_subscriber;
ros::Subscriber messageSubscriber;

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
void messageHandler(const std_msgs::String::ConstPtr &message);
void poseHandler(const std_msgs::String::ConstPtr &message);
void nameHandler(const std_msgs::String::ConstPtr &message);
double computeAndPublishLocalAverageHeading();
double computeAndPublishGlobalAverageHeading();

int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostName(host);

    rotational_controller = RotationalController();
    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    if (argc >= 2)
    {
        rover_name = argv[1];
        cout << "Welcome to the world of tomorrow " << rover_name << "!  Mobility module started." << endl;
    } else
    {
        rover_name = hostName;
        cout << "No Name Selected. Default is: " << rover_name << endl;
    }
    rover_names.insert(rover_name);
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (rover_name + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((rover_name + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((rover_name + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((rover_name + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((rover_name + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, odometryHandler);
    messageSubscriber = mNH.subscribe(("messages"), 10, messageHandler);
    pose_subscriber = mNH.subscribe("pose",10,poseHandler);
    name_subscriber = mNH.subscribe("name",10,nameHandler);

    status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    target_collected_publisher = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);
    debug_publisher = mNH.advertise<std_msgs::String>("/debug", 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10 , true);
    pose_publisher = mNH.advertise<std_msgs::String>("pose",10, true);
    name_publisher = mNH.advertise<std_msgs::String>("name",10, true);
    global_average_heading_publisher = mNH.advertise<std_msgs::String>("gah",10,true);
    local_average_heading_publisher =  mNH.advertise<std_msgs::String>("lah",10,true);
    
    ros::spin();
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    std_msgs::String state_machine_msg;
    std_msgs::String pose_msg;
    std::stringstream pose_converter;
    pose_converter << rover_name << " " << current_location.x << " " << current_location.y << " " << current_location.theta;
    pose_msg.data = pose_converter.str();
    pose_publisher.publish(pose_msg);
    std_msgs::String name_msg;
    stringstream name_converter;
    name_converter << rover_name << " " << rover_names.size();
    name_msg.data = name_converter.str();
    name_publisher.publish(name_msg);
    if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
    {
        if (transitions_to_auto == 0)
        {
            // This is the first time we have clicked the Autonomous Button. Log the time and increment the counter.
            transitions_to_auto++;
            time_stamp_transition_to_auto = ros::Time::now().toSec();
        }
        switch (state_machine_state)
        {
        case COUNT_STATE:
        {
            if (count_same >= 100) {
                state_machine_state = STATE_MACHINE_TRANSLATE;
            }
            else {

            }
            break;
        }
        case STATE_MACHINE_TRANSLATE:
        {
            state_machine_msg.data = "TRANSLATING";//, " + converter.str();
            //float angular_velocity = 0.2;
            //float linear_velocity = 0.1;
            goal_location = current_location;
            goal_location.theta = computeAndPublishGlobalAverageHeading();
            if (rover_name != "achilles") {
            setVelocity(0.1, rotational_controller.calculateVelocity(current_location,goal_location));
            }
            else {
                setVelocity(0.1, 0.2);
            }
            break;
        }
        default:
        {
            state_machine_msg.data = "DEFAULT CASE: SOMETHING WRONG!!!!";
            break;
        }
        }

    }
    else
    { // mode is NOT auto

        // publish current state for the operator to seerotational_controller
        std::stringstream converter;
        converter <<"CURRENT MODE: " << simulation_mode;

        state_machine_msg.data = "WAITING, " + converter.str();
    }
    stateMachinePublish.publish(state_machine_msg);
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
    // Only used if we want to take action after seeing an April Tag.
}

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
    simulation_mode = message->data;
    setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
    if ( message->data > 0 )
    {
        if (message->data == 1)
        {
            // obstacle on right side
        }
        else
        {
            //obstacle in front or on left side
        }
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

void messageHandler(const std_msgs::String::ConstPtr& message)
{
}

double parseDouble(string double_string) {
    stringstream stream;
    stream << double_string;
    double double_value;
    stream >> double_value;
    return double_value;
}

int parseInt(string int_string) {
    stringstream stream;
    stream << int_string;
    int int_value;
    stream >> int_value;
    return int_value;
}

void parsePoseMessage(string msg_string){
    int name_position = msg_string.find_first_of(" ");
    string message_rover_name = msg_string.substr(0,name_position);
    msg_string = msg_string.substr(name_position+1);
    int x_position = msg_string.find_first_of(" ");
    string x_message = msg_string.substr(0,x_position);
    double x = parseDouble(x_message);
    msg_string = msg_string.substr(x_position+1);
    int y_position = msg_string.find_first_of(" ");
    string y_message = msg_string.substr(0,y_position);
    double y = parseDouble(y_message);
    msg_string = msg_string.substr(y_position+1);
    double theta = parseDouble(msg_string);
    pose location;
    location.x = x;
    location.y = y;
    location.theta = theta;
    rover_locations[message_rover_name] = location;
}

double computeAndPublishGlobalAverageHeading() {

    double global_average_heading;
    if (rover_locations.empty()) {
        global_average_heading = current_location.theta;
    }
    else {
        double average_cosine = 0;
        double average_sine = 0;
        for (map<string, pose>::iterator rover_iterator = rover_locations.begin(); rover_iterator != rover_locations.end(); ++rover_iterator) {
            average_cosine += cos(rover_iterator->second.theta);
            average_sine += sin(rover_iterator->second.theta);
        }
        global_average_heading = atan2(average_sine,average_cosine);
    }
    return global_average_heading;
}
double computeAndPublishLocalAverageHeading() {
    double average_cosine = 0;
    double average_x = 0;
    double average_y = 0;
    double average_sine = 0;
    int neighbor_count = 0;
    for (map<string, pose>::iterator rover_iterator = rover_locations.begin(); rover_iterator != rover_locations.end(); ++rover_iterator) {
        if (rover_iterator->first != rover_name) {
            double delta_x = current_location.x - rover_iterator->second.x;
            double delta_y = current_location.y - rover_iterator->second.y;
            double distance = hypot(delta_x,delta_y);

            if (distance <= 2.0) {
                average_x += (rover_iterator->second.x - current_location.x);
                average_y += (rover_iterator->second.y - current_location.y);
                average_cosine += cos(rover_iterator->second.theta);
                average_sine += sin(rover_iterator->second.theta);
                neighbor_count += 1;
            }
        }
    }
    double local_average_heading;
    if (neighbor_count > 0) {
        average_x = current_location.x + (average_x/neighbor_count);
        average_y = current_location.y + (average_y/neighbor_count);
        local_average_heading = atan2(average_y,average_x);
    }
    else {
        local_average_heading = current_location.theta;
    }
    return local_average_heading;
}

void poseHandler(const std_msgs::String::ConstPtr &message)
{
    parsePoseMessage(message->data);
    computeAndPublishGlobalAverageHeading();
    computeAndPublishLocalAverageHeading();
}



void nameHandler(const std_msgs::String::ConstPtr &message)
{
    if (state_machine_state == COUNT_STATE) {
    string msg_string = message->data;
    int name_position = msg_string.find_first_of(" ");
    string message_rover_name = msg_string.substr(0,name_position);
    pair<set<string>::iterator,bool> ret = rover_names.insert(message_rover_name);
    if (ret.second) {
        count_same=0;
    }
    else {
        count_same++;
    }
    }
}
