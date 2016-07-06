// Include GLEW
#include <GL/glew.h>
// Include sfml
#include <SFML/Window.hpp>
// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "markertracker.hpp"
#include "model.hpp"

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

using namespace std;

typedef enum
{
    Initialize,
    Track,
    Render,
    Publish
} ActionState;

static std::map<ActionState, std::string> state_strings = {
        { Initialize,               "Trying to find Marker" },
        { Track,                    "Tracking Marker" },
        { Render,                   "Render screen" },
        { Publish,                  "Publish the pose" }
};

ActionState NextState(ActionState s)
{
    ActionState newstate;
    switch (s)
    {
        case Initialize:
            newstate = Track;
            break;
        case Track:
            newstate = Render;
            break;
        case Render:
            newstate = Publish;
            break;
        case Publish:
            newstate = Track;
            break;
    }
    return newstate;
}


int main(int argc, char *argv[]) {
    // create sfml window
    sf::ContextSettings settings;
    settings.depthBits = 24;
    settings.stencilBits = 8;
    sf::Window window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT, 32), "Marker Tracker", sf::Style::Titlebar | sf::Style::Close, settings);

    // Initialize GLEW
    glewExperimental = GL_TRUE;
    glewInit();

    // initialize marker tracker for webcams with device IDs 0 and 1
    std::vector<int> devices = {1};
    MarkerTracker markerTracker(devices);

    for(uint device=0;device<markerTracker.webcam.size(); device++) {
        cv::namedWindow(markerTracker.webcam[device].name, CV_WINDOW_AUTOSIZE);
        cv::moveWindow(markerTracker.webcam[device].name, 640*(device+1), 0);
        cv::createTrackbar("threshold",markerTracker.webcam[device].name,&markerTracker.webcam[device].threshold_value,255);
    }

    Model model("/home/letrend/workspace/markertracker","markermodel.dae");
    Vector3f cameraPosition(0,0,0);
    Vector3f point(0,0,1);
    // first person camera
    model.lookAt(point,cameraPosition);

    Matrix4f pose_sphere = Matrix4f::Identity();

    ActionState currentState = Initialize;

    ros::init(argc, argv, "markertracker");
    ros::NodeHandle nh;
    ros::Publisher rviz_marker_pub;
    rviz_marker_pub=nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);
    // Publish the marker
    while (rviz_marker_pub.getNumSubscribers() < 1)
    {
        ros::Duration d(1.0);
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        d.sleep();
    }
    ROS_INFO_ONCE("Found subscriber");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    char c;
    bool running = true;
    while (running){
        // handle events
        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
                // window closed
                case sf::Event::Closed:
                    running = false;
                    break;
                case sf::Event::Resized:
                    glViewport(0, 0, event.size.width, event.size.height);
                    break;
            }
        }

        switch(currentState)
        {
            case Initialize:
            {
                cout << state_strings[currentState].c_str() << endl;
                if(markerTracker.init()) // on success continue, else repeat initialization
                    currentState = NextState(currentState);
                break;
            }
            case Track:
            {
                markerTracker.poseEstimation();
                currentState = NextState(currentState);
                break;
            }
            case Render: {
                pose_sphere = markerTracker.ModelMatrix.cast<float>();
                model.updateViewMatrix(window);
                // render the object with the updated modelmatrix
                Mat img;
                model.render(pose_sphere,img);
                window.display();

                currentState = NextState(currentState);
                break;
            }
            case Publish: {
                static uint next_id = 0;
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.header.seq = next_id++;
                marker.ns = "markertracker";
                marker.id = next_id;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                Vector3f position = pose_sphere.topRightCorner(3,1);
                if(ros::Time::now().sec%10==0)
                    printf("( %.4f, %.4f, %.4f )\n", position(0), position(1), position(2));
                Matrix3f pose = pose_sphere.topLeftCorner(3,3);
                Quaternionf q(pose);
                marker.pose.position.x = position(0);
                marker.pose.position.y = position(1);
                marker.pose.position.z = position(2);
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();
                marker.pose.orientation.w = q.w();
                marker.scale.x = 0.01f;
                marker.scale.y = 0.01f;
                marker.scale.z = 0.01f;
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 0;
                marker.color.a = 1;
                marker.lifetime = ros::Duration(30);
                rviz_marker_pub.publish(marker);
                currentState = NextState(currentState);
            }
        }
    }
    markerTracker.stopPoseTracking = true;
    if(markerTracker.pose_estimation_thread!=nullptr){
        markerTracker.pose_estimation_thread->join();
    }

    window.close();

    return 0;
}
