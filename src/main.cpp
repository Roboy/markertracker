#define GLM_FORCE_RADIANS
// Include GLEW
#include <GL/glew.h>

// Include sfml
#include <SFML/Window.hpp>

#include "markertracker.hpp"
#include "model.hpp"

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

using namespace std;

typedef enum
{
    Initialize,
    Track,
    Render
} ActionState;

static std::map<ActionState, std::string> state_strings = {
        { Initialize,               "Trying to find Marker" },
        { Track,                    "Tracking Marker" },
        { Render,                    "Render screen" }
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
    std::vector<int> devices = {0};
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
                Vector3f position = pose_sphere.topRightCorner(3,1);
                printf("( %.4f, %.4f, %.4f )\n", position(0), position(1), position(2));
                model.updateViewMatrix(window);
                // render the object with the updated modelmatrix
                Mat img;
                model.render(pose_sphere,img);
                window.display();

                currentState = NextState(currentState);
                break;
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