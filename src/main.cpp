#define GLM_FORCE_RADIANS
// Include GLEW
#include <GL/glew.h>

// Include sfml
#include <SFML/Window.hpp>

#include "markertracker.hpp"
#include "renderer.hpp"

#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 1090

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

    // create opengl renderer
    char file_path[] = "/home/letrend/workspace/markertracker";
    string obj = "object";
    Renderer renderer(file_path, obj);

    // load a mesh
    Mesh mesh;
    mesh.LoadMesh("/home/letrend/workspace/poseestimator/models/Iron_Man_mark_6/Iron_Man_mark_6.dae");

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
                Matrix4f pose = markerTracker.ModelMatrix.cast<float>();
                cout << "pose: \n" << pose << endl;
                renderer.updateViewMatrix(obj, window);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                renderer.renderColor(obj, pose, &mesh);
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