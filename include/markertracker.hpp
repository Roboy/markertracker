#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <algorithm>
#include <thread>
#include <future>
#include <glm/detail/type_mat.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "timer.hpp"
#define MARKER 4
using namespace std;
using namespace Eigen;

typedef Matrix<double, 4, MARKER> Matrix4xMARKERd;
typedef Matrix<double, 3, MARKER> Matrix3xMARKERd;
typedef Matrix<double, 3, 4> Matrix3x4d;

// Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }

    // you should define that in the subclass :
//  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};

struct WebcamMarkerModel:Functor<double>{
    WebcamMarkerModel(void): Functor<double>(6,2*MARKER){
        pose = VectorXd(6);
        markerIDs.resize(MARKER);
        Trafo2FirstCamera = Matrix4d::Identity();

        cv::Mat cameraMatrix, distCoeffs;
        cv::FileStorage fs("/home/letrend/workspace/markertracker/intrinsics.xml",cv::FileStorage::READ);
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        fs.release();

        // calculate undistortion mapping
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(640,480), 1, cv::Size(640,480), 0),
                                cv::Size(640,480), CV_16SC2, map1, map2);

        std::cout<< "cameraMatrix: \n" << cameraMatrix << "\ndistCoeffs: \n" << distCoeffs << std::endl;

        // camera intrinsic matrix
        K = Matrix3d((double*)cameraMatrix.data).transpose();
        cout << "camera instrinsics:\n" << K << endl;
    };
    void initializeModel();
    void checkCorrespondence();
    void projectInto2D(Matrix3xMARKERd &position2d, Matrix4xMARKERd &position3d, Matrix3x4d &RT);
    void getRTmatrix(VectorXd &x, Matrix3x4d &RT);
    void getRTmatrix(Matrix4d &RT);
    void getRTmatrix(Matrix4f &RT);
    bool track();
    VectorXd pose;
    Matrix4d ModelMatrix;
    Matrix4d Trafo2FirstCamera;
    Matrix4xMARKERd pos3D;// ((x y z 1)' (x y z 1)' ... ),
    Vector4d origin3D;
    Vector3d origin2D;
    Matrix3xMARKERd pos2D;// ((x y 1)' (x y 1)' ... )
    cv::Mat img, img_rectified;
    cv::Mat map1, map2;
    cv::VideoCapture capture;
    uint id;
    char name[20];
    Matrix3d K;
    double FocalLegth = 525;
    double horizontalFOV = 62.0*M_PI/180.0;
    double verticalFOV = 48.6*M_PI/180.0;
    int threshold_value = 235;
    std::vector<uint> markerIDs;
    NumericalDiff<WebcamMarkerModel> *numDiff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<WebcamMarkerModel>, double> *lm;
    double reprojectionError;
    int operator()(const VectorXd &x, VectorXd &fvec) const
    {
        Matrix3xMARKERd projectedPosition2D(3,MARKER);
        Matrix3x4d RT = MatrixXd::Identity(3,4);
        // construct quaternion (cf unit-sphere projection Terzakis paper)
        double alpha_squared = pow(pow(x(0),2.0)+pow(x(1),2.0)+pow(x(2),2.0),2.0);
        Quaterniond q((1-alpha_squared)/(alpha_squared+1),
                        2.0*x(0)/(alpha_squared+1),
                        2.0*x(1)/(alpha_squared+1),
                        2.0*x(2)/(alpha_squared+1));
        // construct RT matrix
        RT.topLeftCorner(3,3) = q.toRotationMatrix();
        RT.topRightCorner(3,1) << x(3), x(4), x(5);

        projectedPosition2D = K * RT * pos3D;
        for(uint col = 0; col<MARKER; col ++){
            projectedPosition2D(0,col)/=projectedPosition2D(2,col);
            projectedPosition2D(1,col)/=projectedPosition2D(2,col);
        }

        Matrix<double,2,MARKER> difference;
        difference = projectedPosition2D.block<2,MARKER>(0,0)-pos2D.block<2,MARKER>(0,0);
        fvec << difference(0,0),difference(1,0),difference(0,1),difference(1,1),difference(0,2),difference(1,2),difference(0,3),difference(1,3);
//        cout << "K\n" << K << endl;
//        cout << "RT\n" << RT << endl;
//        cout << "position3D\n" <<  pos3D << endl;
//        cout << "position2D\n" <<  pos2D << endl;
//        cout << "projectedPosition2D\n" <<  projectedPosition2D << endl;
//        cout << "difference : " << difference <<endl;
//        cout << "error : " << difference.squaredNorm() <<endl;
//        cout << "x : " << x <<endl;
        return 0;
    }
};

class MarkerTracker{
public:
    /**
     * Constructor
     * @param devices vector of camera ids
     */
    MarkerTracker(std::vector<int> &devices);
    ~MarkerTracker();

    void poseEstimation();
    bool init();
    bool findTrafoBetweenCameras();

    void predictMarkerPosition3D();
    void updateWithMeasurement(Vector3d &z);

    Matrix4d ModelMatrix;
    Vector3d pos3D_old, pos3D_new;

    std::vector<WebcamMarkerModel> webcam;
    bool stopPoseTracking = false;
    std::thread* pose_estimation_thread = nullptr;
private:
    Matrix3d Q,M,P,R,Kgain;
    Timer timer;
};

