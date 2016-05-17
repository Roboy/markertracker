#include "markertracker.hpp"

void WebcamMarkerModel::initializeModel(){
    // this is the representation of the marker
    pos3D(0,0)=0;       pos3D(1,0)=-0.125;   pos3D(2,0)=0;       pos3D(3,0)=1;
    pos3D(0,1)=0;       pos3D(1,1)=0;       pos3D(2,1)=-0.083;   pos3D(3,1)=1;
    pos3D(0,2)=-0.104;   pos3D(1,2)=0;       pos3D(2,2)=0;       pos3D(3,2)=1;
    pos3D(0,3)=0.183;  pos3D(1,3)=0;       pos3D(2,3)=0;  pos3D(3,3)=1;

    origin3D << 0,0,0,1;
    origin2D << 0,0,1;

    pose << 0,0,0,0,0,0.01;

    ModelMatrix = Matrix4d::Identity();

    vector<cv::Point2f> centers(MARKER);
    while(true) {
        capture.read(img);
        // undistort
        cv::remap(img, img, map1, map2, cv::INTER_CUBIC);
        cv::flip(img, img, 1);
        if (img.empty())
            continue;
        cv::Mat img_gray;
        cv::cvtColor(img, img_gray, CV_BGR2GRAY);

        cv::Mat filtered_img;
        cv::threshold(img_gray, filtered_img, threshold_value, 255, 3);

        cv::Mat erodeElement = cv::getStructuringElement(2, cv::Size(3, 3), cv::Point(1, 1));
        cv::Mat dilateElement = cv::getStructuringElement(2, cv::Size(5, 5), cv::Point(3, 3));

        erode(filtered_img, filtered_img, erodeElement);
        erode(filtered_img, filtered_img, erodeElement);
        dilate(filtered_img, filtered_img, dilateElement);
        dilate(filtered_img, filtered_img, dilateElement);

        // find contours in result, which hopefully correspond to a found object
        vector<vector<cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;
        findContours(filtered_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,
                     cv::Point(0, 0));

        // filter out tiny useless contours
        double min_contour_area = 60;
        for (auto it = contours.begin(); it != contours.end();) {
            if (contourArea(*it) < min_contour_area) {
                it = contours.erase(it);
            }
            else {
                ++it;
            }
        }

        if (contours.size() == MARKER) {
            for (int idx = 0; idx < contours.size(); idx++) {
                drawContours(img, contours, idx, cv::Scalar(255, 0, 0), 4, 8, hierarchy, 0,
                             cv::Point());
                float radius;
                minEnclosingCircle(contours[idx], centers[idx], radius);
            }
            // sort the centers wrt x coordinates
            sort(centers.begin(), centers.end(),
                 [](const cv::Point2f &a, const cv::Point2f &b) -> bool {
                     return a.x <= b.x;
                 });
            char str[1];
            sprintf(str, "%d", 2);
            circle(img, centers[0], 10, cv::Scalar(0, 255, 0), 4);
            putText(img, str, centers[0], cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1,
                    cv::Scalar::all(255));
            sprintf(str, "%d", 3);
            circle(img, centers[3], 10, cv::Scalar(0, 255, 0), 4);
            putText(img, str, centers[3], cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1,
                    cv::Scalar::all(255));
            // sort the remaining two centers wrt y coordinates
            sprintf(str, "%d", 0);
            circle(img,
                   centers[1].y < centers[2].y ? centers[2] : centers[1], 10,
                   cv::Scalar(0, 255, 0), 4);
            putText(img, str,
                    centers[1].y < centers[2].y ? centers[1] : centers[2],
                    cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar::all(255));
            sprintf(str, "%d", 1);
            circle(img,
                   centers[1].y > centers[2].y ? centers[2] : centers[1], 10,
                   cv::Scalar(0, 255, 0), 4);
            putText(img, str,
                    centers[1].y < centers[2].y ? centers[2] : centers[1],
                    cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar::all(255));
            break;
        } else {
            // draw detected contours onto output image
            for (int idx = 0; idx < contours.size(); idx++) {
                drawContours(img, contours, idx, cv::Scalar(0, 0, 255), 4, 8, hierarchy, 0,
                             cv::Point());
            }
        }
//        imshow("intialization", img);
//        cv::waitKey(1);
    }

    // write those positions to the model
    pos2D(0,0) = centers[2].x; pos2D(1,0) = centers[2].y; pos2D(2,0) = 1;
    pos2D(0,1) = centers[1].x; pos2D(1,1) = centers[1].y; pos2D(2,1) = 1;
    pos2D(0,2) = centers[0].x; pos2D(1,2) = centers[0].y; pos2D(2,2) = 1;
    pos2D(0,3) = centers[3].x; pos2D(1,3) = centers[3].y; pos2D(2,3) = 1;
//    cout << "pos2D: \n" << pos2D << endl;

    for(uint id=0; id<MARKER; id++)
        markerIDs[id]=id;

    // find the pose
    pose << 0,0,0,0,0,0.6;

    numDiff = new NumericalDiff<WebcamMarkerModel>(*this);
    lm = new LevenbergMarquardt<NumericalDiff<WebcamMarkerModel>, double> (*numDiff);
    lm->parameters.maxfev = 2000;
    lm->parameters.xtol = 1.0e-10;
    int ret = lm->minimize(pose);
//    cout << "iterations: " << lm->iter << endl;
//    cout << "x that minimizes the function: \n" << pose << endl;

    Matrix3x4d RT;
    getRTmatrix(pose,RT);

    Matrix3xMARKERd projectedPosition2D = K * RT * pos3D;
    origin2D = K * RT * origin3D;
    origin2D(0)/=origin2D(2);
    origin2D(1)/=origin2D(2);
    char str[1];
    for(uint col = 0; col<MARKER; col ++){
        projectedPosition2D(0,col)/=projectedPosition2D(2,col);
        projectedPosition2D(1,col)/=projectedPosition2D(2,col);

        float radius=10;
        cv::Point2f center(projectedPosition2D(0,col), projectedPosition2D(1,col));
        circle(img, center, radius, cv::Scalar(255, 255, 0), 4);
        line(img, cv::Point2f(origin2D(0),origin2D(1)), center, cv::Scalar::all(255),4);
        sprintf(str,"%d",markerIDs[col]);
        putText(img,str,center,cv::FONT_HERSHEY_SCRIPT_SIMPLEX,1,cv::Scalar::all(255));
    }
//    cout << "projectedPosition2D: \n" << projectedPosition2D << endl;

    delete numDiff;
    delete lm;
}

void WebcamMarkerModel::projectInto2D(Matrix3xMARKERd &position2d, Matrix4xMARKERd &position3d, Matrix3x4d &RT) {
    position2d = K * RT * position3d;
    for(uint col = 0; col<MARKER; col ++){
        position2d(0,col)/=position2d(2,col);
        position2d(1,col)/=position2d(2,col);
    }
}

void WebcamMarkerModel::getRTmatrix(VectorXd &x, Matrix3x4d &RT){
    RT = MatrixXd::Identity(3,4);
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(pose(0),2.0)+pow(pose(1),2.0)+pow(pose(2),2.0),2.0);
    Quaterniond q((1-alpha_squared)/(alpha_squared+1),
                  2.0*pose(0)/(alpha_squared+1),
                  2.0*pose(1)/(alpha_squared+1),
                  2.0*pose(2)/(alpha_squared+1));
    // construct RT matrix
    RT.topLeftCorner(3,3) = q.toRotationMatrix();
    RT.topRightCorner(3,1) << pose(3), pose(4), pose(5);
}
void WebcamMarkerModel::getRTmatrix(Matrix4d &RT){
    RT = Matrix4d::Identity();
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(pose(0),2.0)+pow(pose(1),2.0)+pow(pose(2),2.0),2.0);
    Quaterniond q((1-alpha_squared)/(alpha_squared+1),
                  2.0*pose(0)/(alpha_squared+1),
                  2.0*pose(1)/(alpha_squared+1),
                  2.0*pose(2)/(alpha_squared+1));
    // construct RT matrix
    RT.topLeftCorner(3,3) = q.toRotationMatrix();
    RT.topRightCorner(3,1) << pose(3), pose(4), pose(5);
}
void WebcamMarkerModel::getRTmatrix(Matrix4f &RT){
    RT = Matrix4f::Identity();
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    float alpha_squared = powf(powf(pose(0),2.0f)+powf(pose(1),2.0f)+powf(pose(2),2.0f),2.0f);
    Quaternionf q((1-alpha_squared)/(alpha_squared+1),
                  2.0*pose(0)/(alpha_squared+1),
                  2.0*pose(1)/(alpha_squared+1),
                  2.0*pose(2)/(alpha_squared+1));
    // construct RT matrix
    RT.topLeftCorner(3,3) = q.toRotationMatrix();
    RT.topRightCorner(3,1) << pose(3), pose(4), pose(5);
}

void WebcamMarkerModel::checkCorrespondence(){
    Matrix3x4d RT;
    getRTmatrix(pose,RT);
    Matrix4xMARKERd pos3D_backup = pos3D;
    
    vector<uint> perm = {0,1,2,3};
    double minError = 1e10;
    vector<uint> bestPerm = {0,1,2,3};
    cv::Mat img = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);

    Matrix3xMARKERd projectedPosition2D(3,MARKER);
    do {
        pos3D <<
        pos3D_backup(0,perm[0]), pos3D_backup(0,perm[1]), pos3D_backup(0,perm[2]), pos3D_backup(0,perm[3]),
                pos3D_backup(1,perm[0]), pos3D_backup(1,perm[1]), pos3D_backup(1,perm[2]), pos3D_backup(1,perm[3]),
                pos3D_backup(2,perm[0]), pos3D_backup(2,perm[1]), pos3D_backup(2,perm[2]), pos3D_backup(2,perm[3]),
                1,1,1,1;
        projectInto2D(projectedPosition2D, pos3D, RT);
        Matrix<double,2,MARKER> difference;
        difference = projectedPosition2D.block<2,MARKER>(0,0)-pos2D.block<2,MARKER>(0,0);
        double error = difference.squaredNorm();

        if (error < minError) {
            bestPerm = perm;
            minError = error;
        }
    } while (next_permutation(perm.begin(), perm.end()));

    vector<uint> markerIDs_temp = markerIDs;
    for(uint id=0; id<MARKER; id++)
        markerIDs[id] = markerIDs_temp[bestPerm[id]];
//    printf("assignement: %d %d %d %d, error: %f\n", bestPerm[0], bestPerm[1], bestPerm[2], bestPerm[3], minError);

    pos3D <<
            pos3D_backup(0, bestPerm[0]), pos3D_backup(0, bestPerm[1]), pos3D_backup(0, bestPerm[2]), pos3D_backup(0, bestPerm[3]),
            pos3D_backup(1, bestPerm[0]), pos3D_backup(1, bestPerm[1]), pos3D_backup(1, bestPerm[2]), pos3D_backup(1, bestPerm[3]),
            pos3D_backup(2, bestPerm[0]), pos3D_backup(2, bestPerm[1]), pos3D_backup(2, bestPerm[2]), pos3D_backup(2, bestPerm[3]),
            1, 1, 1, 1;
}

bool WebcamMarkerModel::track(){
    capture.retrieve(img);
    if (!img.empty()) {
        // undistort
        cv::remap(img, img, map1, map2, cv::INTER_CUBIC);
        cv::flip(img,img,1);
        cv::Mat img2;
        cv::cvtColor(img, img2, CV_BGR2GRAY);

        cv::Mat filtered_img;
        cv::threshold(img2, filtered_img, threshold_value, 255, 3);

        cv::Mat erodeElement = cv::getStructuringElement(2, cv::Size(3, 3), cv::Point(1, 1));
        cv::Mat dilateElement = cv::getStructuringElement(2, cv::Size(5, 5), cv::Point(3, 3));

        erode(filtered_img, filtered_img, erodeElement);
        erode(filtered_img, filtered_img, erodeElement);
        dilate(filtered_img, filtered_img, dilateElement);
        dilate(filtered_img, filtered_img, dilateElement);

        // find contours in result, which hopefully correspond to a found object
        vector<vector<cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;
        findContours(filtered_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,
                     cv::Point(0, 0));

        // filter out tiny useless contours
        double min_contour_area = 60;
        for (auto it = contours.begin(); it != contours.end();) {
            if (contourArea(*it) < min_contour_area) {
                it = contours.erase(it);
            }
            else {
                ++it;
            }
        }

        // detect circles in filtered image
        cv::Mat circle_out = filtered_img.clone();
        cv::cvtColor(circle_out, circle_out, CV_GRAY2BGR);

        reprojectionError = 1000000;

        if (contours.size() == 4) {
            // draw detected contours onto output image
            for (int idx = 0; idx < contours.size(); idx++) {
                drawContours(img, contours, idx, cv::Scalar(0, 255, 0), 4, 8, hierarchy, 0,
                             cv::Point());
                float radius;
                cv::Point2f center;
                minEnclosingCircle(contours[idx], center, radius);
                circle(circle_out, center, radius, cv::Scalar(255, 255, 255), 4);

                pos2D(0, idx) = center.x;
                pos2D(1, idx) = center.y;
                pos2D(2, idx) = 1;
            }
            checkCorrespondence();

            // need to instantiate new lm every iteration so it will be using the new positions
            numDiff = new NumericalDiff<WebcamMarkerModel>(*this);
            lm = new Eigen::LevenbergMarquardt<Eigen::NumericalDiff<WebcamMarkerModel>, double> (*numDiff);
            lm->parameters.maxfev = 2000;
            lm->parameters.xtol = 1.0e-10;
            int ret = lm->minimize(pose);
//                    cout << "iterations: " << lm->iter << endl;
//                    cout << "x that minimizes the function: \n" << pose << endl;

            Matrix4d RT;
            getRTmatrix(RT);
//                    cout << "RT: \n" << RT << endl;
            pos3D = RT*pos3D;
            ModelMatrix = RT*ModelMatrix;
//            cout << "ModelMatrix : \n" << ModelMatrix  << endl;

            Matrix3xMARKERd projectedPosition2D = K * pos3D.block<3,MARKER>(0,0);
            origin2D = K * ModelMatrix.topRightCorner(3,1);
            origin2D(0)/=origin2D(2);
            origin2D(1)/=origin2D(2);
            char name[1];
            for(uint col = 0; col<MARKER; col ++){
                projectedPosition2D(0,col)/=projectedPosition2D(2,col);
                projectedPosition2D(1,col)/=projectedPosition2D(2,col);

                float radius=10;
                cv::Point2f center(projectedPosition2D(0,col), projectedPosition2D(1,col));
                circle(img, center, radius, cv::Scalar(255, 0, 0), 4);
                line(img, cv::Point2f(origin2D(0),origin2D(1)), center, cv::Scalar::all(255),4);
                sprintf(name,"%d",markerIDs[col]);
                putText(img,name,center,cv::FONT_HERSHEY_SCRIPT_SIMPLEX,1,cv::Scalar::all(255));
            }

            reprojectionError = lm->fnorm;

            delete numDiff;
            delete lm;
            return true;
        }else{
            // draw detected contours onto output image
            for (int idx = 0; idx < contours.size(); idx++) {
                drawContours(img, contours, idx, cv::Scalar(0, 0, 255), 4, 8, hierarchy, 0,
                             cv::Point());
            }
        }
    }
    return false;
}

MarkerTracker::MarkerTracker(vector<int> &devices){
    webcam.resize(devices.size());
    for(int i = 0; i<devices.size(); i++){
        if(!webcam[i].capture.open(devices[i])){
            cout << "could not open camera " << devices[i] << endl;
        }
        webcam[i].id = devices[i];
        sprintf(webcam[i].name, "camera %d", webcam[i].id);
    }

    // Covariance process noise
    Q = Matrix3d::Identity()*0.1;
    // Covariance matrix representing errors in state estimates (i.e. variance of  truth minus estimate)
    P = Matrix3d::Identity()*10;
    // Covariance of measurement noise
    R = Matrix3d::Identity()*0.1;
}

MarkerTracker::~MarkerTracker() {
//    for(auto thread:pose_estimation_threads)
//        thread->join();
}

void MarkerTracker::poseEstimation(){
    static uint counter=0;
    // grap camera images
    for (uint device=0; device < webcam.size(); device++) {
        webcam[device].capture.grab();
    }
    // start tracking threads
    vector<packaged_task<bool(void)>> tracking_tasks;
    vector<future<bool>> tracking_ret;
    vector<thread> tracking_threads;
    for (uint device=0; device < webcam.size(); device++) {
        tracking_tasks.push_back(packaged_task<bool(void)>(bind(&WebcamMarkerModel::track, &webcam[device])));   // set up packaged_task
        tracking_ret.push_back(tracking_tasks.back().get_future());            // get future
        tracking_threads.push_back(thread(move(tracking_tasks.back())));   // spawn tracking thread
    }
    // wait for results and fuse estimates if available
    Matrix4d pose;
    Vector3d pos;
    for (uint device=0; device < webcam.size(); device++) {
        if(tracking_ret[device].get()) {// wait for the tracking tasks to finish
            // transform into first camera frame
            Matrix4d pose = webcam[device].Trafo2FirstCamera*webcam[device].ModelMatrix;
            Vector3d pos = pose.topRightCorner(3,1);
            updateWithMeasurement(pos);
        }else{
            predictMarkerPosition3D();
        }
        tracking_threads[device].join();
        cv::imshow(webcam[device].name, webcam[device].img);
        cv::waitKey(1);
    }
    // use orientation with minimal reprojection error
    double minimalReprojectionError = webcam[0].reprojectionError;
    ModelMatrix = webcam[0].Trafo2FirstCamera*webcam[0].ModelMatrix;
    for (uint device=0; device < webcam.size(); device++) {
        if(webcam[device].reprojectionError<minimalReprojectionError){
            minimalReprojectionError = webcam[device].reprojectionError;
            ModelMatrix = webcam[device].Trafo2FirstCamera*webcam[device].ModelMatrix;
        }
    }
    // use fused position
    ModelMatrix.topRightCorner(3,1) = pos3D_new;
    counter ++;
}

bool MarkerTracker::init() {
    cout << "please stand in front of cameras" << endl;
    for(uint device=0; device<webcam.size(); device++) {
        webcam[device].initializeModel();
        cv::imshow(webcam[device].name, webcam[device].img);
        cv::waitKey(1);
    }
    return findTrafoBetweenCameras();
}

bool MarkerTracker::findTrafoBetweenCameras(){
    // measure for a few seconds until estimates are good enough to get transform between cameras, otherwise escape
    vector<Matrix4d> bestPose(webcam.size());
    uint goodEnough;
    timer.start();
    do{
        poseEstimation();
        goodEnough = 0;
        for(uint device=0; device<webcam.size(); device++){
            if(webcam[device].reprojectionError<1.0){
                bestPose[device] = webcam[device].ModelMatrix;
                goodEnough += 1;
            }
//            cout << webcam[device].name << " reprojection error " << webcam[device].reprojectionError << " goodEnough " << goodEnough << endl;
        }
        if(timer.elapsedTime()>1.0){
            cout << "could not get accurate enough pose estimates" << endl;
            return false;
        }
    }while(goodEnough != webcam.size());
    for(uint device=0; device<webcam.size(); device++) {
        cout << "minimal reprojection error " << webcam[device].name << " " << webcam[device].reprojectionError << endl;
        cout << bestPose[device] << endl;
        webcam[device].ModelMatrix = bestPose[device];
    }
    // find transformations between cameras wrt to first camera
    for (uint device = 1; device < webcam.size(); device++){
        webcam[device].Trafo2FirstCamera = webcam[0].ModelMatrix*webcam[device].ModelMatrix.inverse();
        cout << "trafo " << webcam[device].name << " to " << webcam[0].name << endl << webcam[device].Trafo2FirstCamera << endl;
    }
    return true;
}

void MarkerTracker::predictMarkerPosition3D(){
    pos3D_old = pos3D_new;
    P += Q;
}

void MarkerTracker::updateWithMeasurement(Vector3d &z){
    // STEP 1: time update -> prediction
    pos3D_old = pos3D_new;
    M = P + Q;

   //STEP 2: measurement update -> correction
    Kgain = M*(P + R).inverse();
    pos3D_new = pos3D_old + Kgain*(z - pos3D_old);
    P = M - Kgain*M;
}



