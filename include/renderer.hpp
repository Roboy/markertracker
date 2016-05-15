// glew
#include <GL/glew.h>
// sfml
#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
// glm
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
// std
#include <cstring>
#include <map>
#include <fstream>

#include "mesh.hpp"

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * (float)M_PI / 180.0f)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0f / (float)M_PI)

#define WIDTH 640
#define HEIGHT 480

using namespace std;
using namespace Eigen;
using cv::Mat;

struct PackedVertex {
    glm::vec3 position;
    glm::vec2 uv;
    glm::vec3 normal;

    bool operator<(const PackedVertex that) const {
        return memcmp((void *) this, (void *) &that, sizeof(PackedVertex)) > 0;
    };
};

class Renderer {
public:
    Renderer(const char *file_path, const char *object);

    ~Renderer();

    Mat renderColor(string &object, Matrix4f &pose, Mesh *mesh);

    void updateViewMatrix(string &object, sf::Window &window);

    map<string, vector<glm::vec3>> vertices, normals;
    Matrix3f K, Kinv; // intrinsics matrix
private:
    bool loadShaderCodeFromFile(const char *file_path, string &src);

    void compileShader(string src, GLenum type, GLuint &shader);

    void createTransformProgram(GLuint &shader, const GLchar *feedbackVaryings[], uint numberOfVaryings,
                                GLuint &program);

    GLint createRenderProgram(GLuint &vertex_shader, GLuint &fragment_shader, GLuint &program,GLint &MatrixID,
                              GLint &ViewMatrixID, GLint &ModelMatrixID, GLint &LightPositionID);

    map<string, GLuint> shader, program, vertexbuffer, uvbuffer, normalbuffer, elementbuffer;
    map<string, vector<glm::vec2>> uvs;
    map<string, vector<unsigned short>> indices;
    map<string, GLuint> tbo;
    map<string, GLint> ViewMatrixID, MatrixID, ModelMatrixID, KID, LightPositionID;
    map<string, GLsizei> numberOfVertices, numberOfIndices;
    map<string, Matrix4f> ViewMatrix, ModelMatrix;
    Matrix4f ProjectionMatrix;
};
