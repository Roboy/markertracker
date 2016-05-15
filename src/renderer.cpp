#include <SFML/Window/Keyboard.hpp>
#include "renderer.hpp"

Renderer::Renderer(const char *file_path, const char *object) {
    char file[200];
    // create color program
    string src;
    sprintf(file, "%s/shader/color.vertexshader", file_path);
    loadShaderCodeFromFile(file, src);
    compileShader(src, GL_VERTEX_SHADER, shader["color_vertex"]);

    sprintf(file, "%s/shader/color.fragmentshader", file_path);
    loadShaderCodeFromFile(file, src);
    compileShader(src, GL_FRAGMENT_SHADER, shader["color_fragment"]);

    if (createRenderProgram(shader["color_vertex"], shader["color_fragment"], program[object], MatrixID[object],
                            ViewMatrixID[object], ModelMatrixID[object], LightPositionID[object]) == GL_FALSE)
        return;

    ModelMatrix[object] = Matrix4f::Identity();
    ViewMatrix[object] = Matrix4f::Identity();
    Matrix3f rot;
    rot = Eigen::AngleAxisf(degreesToRadians(180), Vector3f::UnitY());
    ViewMatrix[object].topLeftCorner(3, 3) = rot;
    ViewMatrix[object].topRightCorner(3, 1) << 0,0,0;

    Mat cameraMatrix, distCoeffs;
    sprintf(file, "%s/intrinsics.xml", file_path);
    cv::FileStorage fs(file, cv::FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    // calculate undistortion mapping
    Mat img_rectified, map1, map2;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(WIDTH, HEIGHT), 1,
                                                      cv::Size(WIDTH, HEIGHT), 0),
                            cv::Size(WIDTH, HEIGHT), CV_16SC2, map1, map2);

    float n = 0.01; // near field
    float f = 100; // far field
    ProjectionMatrix << cameraMatrix.at<double>(0, 0) / cameraMatrix.at<double>(0, 2), 0.0, 0.0, 0.0,
            0.0, cameraMatrix.at<double>(1, 1) / cameraMatrix.at<double>(1, 2), 0.0, 0.0,
            0.0, 0.0, -(f + n) / (f - n), (-2.0f * f * n) / (f - n),
            0.0, 0.0, -1.0, 0.0;
    K << cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(0, 1), cameraMatrix.at<double>(0, 2),
            cameraMatrix.at<double>(1, 0), cameraMatrix.at<double>(1, 1), cameraMatrix.at<double>(1, 2),
            cameraMatrix.at<double>(2, 0), cameraMatrix.at<double>(2, 1), cameraMatrix.at<double>(2, 2);
    cout << "K\n" << K << endl;
    Kinv = K.inverse();

    cout << "ProjectionMatrix\n" << ProjectionMatrix << endl;
    cout << "ModelMatrix\n" << ModelMatrix[object] << endl;
    cout << "ViewMatrix\n" << ViewMatrix[object] << endl;

    // background ccolor
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);
    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);
}

Renderer::~Renderer() {
    for (auto p:program)
        glDeleteProgram(p.second);
    for (auto t:tbo)
        glDeleteBuffers(1, &t.second);
}

Mat Renderer::renderColor(string &object, Matrix4f &pose, Mesh *mesh ) {
    ModelMatrix[object] = pose;

    Matrix4f MVP = ProjectionMatrix * ViewMatrix[object] * ModelMatrix[object];
    glUseProgram(program[object]);
    glUniformMatrix4fv(ViewMatrixID[object], 1, GL_FALSE, &ViewMatrix[object](0, 0));
    glUniformMatrix4fv(MatrixID[object], 1, GL_FALSE, &MVP(0, 0));
    glUniformMatrix4fv(ModelMatrixID[object], 1, GL_FALSE, &ModelMatrix[object](0, 0));
    Vector3f lightPosition(0, 1, 1);
    glUniform3fv(LightPositionID[object], 1, &lightPosition(0));

    mesh->Render();

    // get the image from opengl buffer
    GLubyte *data = new GLubyte[3 * WIDTH * HEIGHT];
    glReadPixels(0, 0, WIDTH, HEIGHT, GL_BGR, GL_UNSIGNED_BYTE, data);
    Mat img = cv::Mat(HEIGHT, WIDTH, CV_8UC3, data);
    flip(img, img, -1);
    return img;
}

void Renderer::updateViewMatrix(string &object, sf::Window &window){
    float speed_trans = 0.2f , speed_rot = 0.001f;

    // Get mouse position
    sf::Vector2i windowsize = sf::Vector2i(window.getSize().x, window.getSize().y);
    double xpos, ypos;
    Matrix3f rot = Matrix3f::Identity();
    static bool sticky = false;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)){
        sticky = !sticky;
    }
    if(sticky) {
        sf::Vector2i mousepos = sf::Mouse::getPosition(window);
        sf::Vector2i delta = windowsize/2-mousepos;
        if(delta.x != 0 || delta.y != 0)
        {
            // set cursor to window center
            sf::Mouse::setPosition(windowsize/2, window);
            // Compute new orientation
            float horizontalAngle = speed_rot * float(delta.x);
            float verticalAngle = speed_rot * float(delta.y);

            rot = Eigen::AngleAxisf(horizontalAngle, ViewMatrix[object].block<1, 3>(1, 0)) *
                  Eigen::AngleAxisf(verticalAngle, ViewMatrix[object].block<1, 3>(0, 0));
        }
    }

    Vector3f direction = ViewMatrix[object].block<1,3>(2,0);
    Vector3f right = ViewMatrix[object].block<1,3>(0,0);

    Vector3f dcameraPos(0,0,0);
    // Move forward
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)){
        dcameraPos += direction  * speed_trans;
    }
    // Move backward
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)){
        dcameraPos -= direction * speed_trans;
    }
    // Strafe right
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)){
        dcameraPos -= right * speed_trans;
    }
    // Strafe left
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)){
        dcameraPos += right * speed_trans;
    }

    Matrix4f RT = Matrix4f::Identity();
    RT.topLeftCorner(3,3) = rot;
    RT.topRightCorner(3,1) = dcameraPos;

    ViewMatrix[object] = ViewMatrix[object]*RT;
}

bool Renderer::loadShaderCodeFromFile(const char *file_path, string &src) {
    src.clear();
    ifstream VertexShaderStream(file_path, ios::in);
    if (VertexShaderStream.is_open()) {
        string Line = "";
        while (getline(VertexShaderStream, Line))
            src += "\n" + Line;
        VertexShaderStream.close();
        return true;
    } else {
        printf("Cannot read %s\n", file_path);
        getchar();
        return false;
    }
}

void Renderer::compileShader(string src, GLenum type, GLuint &shader) {
    shader = glCreateShader(type);
    const char *c_str = src.c_str();
    glShaderSource(shader, 1, &c_str, nullptr);
    glCompileShader(shader);
}

GLint Renderer::createRenderProgram(GLuint &vertex_shader, GLuint &fragment_shader, GLuint &program, GLint &MatrixID,
                                    GLint &ViewMatrixID, GLint &ModelMatrixID, GLint &LightPositionID) {
    // Link the program
    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    int params = -1;
    glGetProgramiv(program, GL_LINK_STATUS, &params);
    printf("GL_LINK_STATUS = %i\n", params);

    glGetProgramiv(program, GL_ATTACHED_SHADERS, &params);
    printf("GL_ATTACHED_SHADERS = %i\n", params);

    glGetProgramiv(program, GL_ACTIVE_ATTRIBUTES, &params);
    printf("GL_ACTIVE_ATTRIBUTES = %i\n", params);

    GLint Result = GL_FALSE;
    int InfoLogLength;
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(vertex_shader, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
        glGetShaderInfoLog(vertex_shader, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
        printf("%s\n", &VertexShaderErrorMessage[0]);
        return Result;
    }

    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(fragment_shader, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
        glGetShaderInfoLog(fragment_shader, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
        printf("%s\n", &VertexShaderErrorMessage[0]);
        return Result;
    }

    glDetachShader(program, vertex_shader);
    glDetachShader(program, fragment_shader);

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    MatrixID = glGetUniformLocation(program, "MVP");
    ViewMatrixID = glGetUniformLocation(program, "ViewMatrix");
    ModelMatrixID = glGetUniformLocation(program, "ModelMatrix");
    LightPositionID = glGetUniformLocation(program, "LightPosition_worldspace");

    return Result;
}