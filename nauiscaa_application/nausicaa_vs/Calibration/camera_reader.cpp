#include "camera_reader.h"


#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/calib3d/calib3d.hpp>


#define RECTIFY_FIRST


using namespace cv;

#include <iostream>
#include <fstream>
#include <time.h>
using namespace std;


int radiusCircle = 10;
cv::Scalar colorCircle1(255,255,255);
int thicknessCircle1 = 2;
int ax;
int NP = 5;

string type2str(int type);

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    Camera * cam =  (Camera*)userdata;
     if  ( event == EVENT_LBUTTONDOWN  ){
         if(flags & EVENT_FLAG_CTRLKEY)
             {
                  cam->origin = cv::Point(x,y);
             }
         else
             if(flags & EVENT_FLAG_SHIFTKEY)
             {
                 cam->axis_points[ax] = cv::Point(x,y);
                 ax=(ax+1)%NP;
             }
     }

     else if  ( event == EVENT_RBUTTONDOWN )
     {
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
     }
     else if ( event == EVENT_MOUSEMOVE )
     {

     }
}

void Camera::init(uint port, std::string camera_intrinsics_file, bool scaramuzza){
    /////////////Scaramuzza camera parameter read///////////////////
    origin = cv::Point(-1, -1);
    for (int i = 0; i < 3; ++i) axis_points[i] = cv::Point(-1, -1);
    if (scaramuzza)
    {
        cv::Size imageSize(cv::Size(1948, 1096));
        cv::Mat mapx_persp = cv::Mat(imageSize, CV_32FC1);
        cv::Mat mapy_persp = cv::Mat(imageSize, CV_32FC1); //CV_32FC1
        map1 = mapx_persp.clone();
        map2 = mapy_persp.clone();
        float r, l, t, b, f = 0.5;
      
        get_ocam_model(&o, camera_intrinsics_file.c_str());
        //create_perspecive_undistortion_LUT(map1, map2, &o, scaleFactor);

       

       /* cameraMatrix  = computeOpenCVMatrixFromScaraMuzza(o);
        int len = o.length_pol;
        distCoeffs = cv::Mat(1, 4, CV_32F);
        for (int i = 1; i < len; i++)
            distCoeffs.at<float>(0, i - 1) = o.pol[i];
        std::cout << "M = " << std::endl << " " << cameraMatrix << std::endl << std::endl;
        cv::Mat  new_camera_matrix;
        new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(1948, 1096), 1, cv::Size(1948, 1096), 0);
        cameraMatrix = new_camera_matrix;
        std::cout << "M_after = " << std::endl << " " << cameraMatrix << std::endl << std::endl;
        cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), new_camera_matrix, imageSize, CV_32F, map1, map2);*/

        Eigen::Vector2d img_size(o.width, o.height);
        Eigen::Matrix3f K_out;
        std::array<float, 5> D_out;
        float xi_out;
        Eigen::Vector2d principal_point{ o.yc, o.xc };
        std::vector<double>  invpoly;
        invpoly.assign(o.invpol, o.invpol + o.length_invpol);
        // 
        calib_converter::convertOcam2Mei(invpoly, principal_point, img_size, o.c, o.d, o.e, K_out, D_out, xi_out);
       // cv::Mat dst,newMat;
        
        //cv::Mat d = cv::Mat(1, 4, CV_32F);
        //cv::Mat K(3, 3, cv::DataType<float>::type);
        cameraMatrix = cv::Mat(3, 3, CV_32F);// computeOpenCVMatrixFromScaraMuzza(o);
        //d = getDistCoeffiecient(o);
        distCoeffs = cv::Mat(1, 4, CV_32F);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                cameraMatrix.at<float>(i, j) = K_out(i, j);
            }


        }

        for (int i = 0; i < 4; ++i)
        {
            distCoeffs.at<float>(i) = D_out[i];
        }
       // std::cout << cameraMatrix << std::endl;
        std::cout << "distortion coefficients:" << distCoeffs << std::endl;
        cv::Mat new_camera_matrix = cv::Mat(cv::Matx33f(-o.pol[0], 0, cameraMatrix.at<float>(0, 2),
            0, -o.pol[0], cameraMatrix.at<float>(1, 2),
            0, 0, 1));

        std::cout << "Camera Intrinsics for rectification:" << new_camera_matrix << std::endl;
       cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
        omnidir::initUndistortRectifyMap(cameraMatrix, distCoeffs, xi_out, R, new_camera_matrix, imageSize,
            CV_32F, map1, map2, cv::omnidir::RECTIFY_PERSPECTIVE);
        //std::cout << "map1:" << map1 << std::endl;
        cameraMatrix = new_camera_matrix;

        cameraMatrix44.SetIdentity();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                cameraMatrix44[i][j] = cameraMatrix.at<float>(i, j);

        computeOpenGLFrustrumScaramuzza(cameraMatrix, 1948, 1096, f, r, l, t, b);
        vcg_cam.SetFrustum(l, r, b, t, f, vcg::Point2i(1948, 1096));
        return;
    }


    cv::Mat  new_camera_matrix;

    cameraMatrix = cv::Mat(3,3,CV_32F);
    distCoeffs = cv::Mat(1,4,CV_32F);
    FILE *ip=fopen(camera_intrinsics_file.c_str(),"r");
    for(int i=0; i < 9; ++i) {
        float v;
        fscanf(ip,"%f",&v);
        cameraMatrix.at<float>(i/3,i%3) = v;
    }
    for(int i=0; i < 4; ++i) {
        float v;
        fscanf(ip,"%f",&v);
        distCoeffs.at<float>(0,i) = v; // <--- remove dist
    }
    fclose(ip);
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;




    cv::Mat dst;
    cv::Size imageSize(cv::Size(1948,1096));

#ifdef RECTIFY_FIRST
    cv::Mat distCoeffs1 = cv::Mat(1,4,CV_32F);

    for(int i=0; i < 4; ++i)distCoeffs1.at<float>(0,i)=0.f;

    // Refining the camera matrix using parameters obtained by calibration
    new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);
    std::cout << "distCoeffs UND : " << distCoeffs << std::endl;

    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),new_camera_matrix,imageSize, CV_16SC2, map1, map2);

    std::cout <<  type2str(map1.type()) << std::endl;

//    cv::imshow("map1",map1);
//    cv::waitKey(0);

//    cv::imshow("map2",map2);
//    cv::waitKey(0);

    std::cout << "new cameraMatrix : " << new_camera_matrix << std::endl;
    cameraMatrix = new_camera_matrix;
#endif

    cameraMatrix44.SetIdentity();
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            cameraMatrix44[i][j] = cameraMatrix.at<float>(i,j);

    // copy to vcgcam
    float r, l, t, b, f = 0.5;
  
   
    this->opencv2opengl_camera_params(cameraMatrix,1948,1096,f,r,l,t,b);
    vcg_cam.SetFrustum(l,r,b,t,f,vcg::Point2i(1948,1096));
}
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void Camera::start_reading(){
    VideoCapture cap("udpsrc port=5000 caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink",
            CAP_GSTREAMER);

    latest_frame_mutex.lock();
    reading = true;
    namedWindow("receiver",1);
    setMouseCallback("receiver", CallBackFunc, this);
    while (reading) {
        latest_frame_mutex.unlock();
        Mat frame;

#ifdef RECTIFY_FIRST
        cap.read(frame);
        //Mat temp(frame.size(), frame.type());
        //dst = frame.clone();
        //create_perspecive_undistortion_LUT(map1, map2, &o, scaleFactor);
        //std::cout << " Iam done with undistortion:" << std::endl;
        cv::remap(frame, dst, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        //cv::imwrite("D:/CamImages/rectified_streamed_output.jpg", dst);
#else
        cap.read(dst);
#endif
        // drawing
        if(origin!=cv::Point2f(-1,-1))
            cv::circle(dst, origin, radiusCircle, cv::Scalar(255,255,255), thicknessCircle1);
        for(int i=0; i < NP; ++i)
            if(axis_points[i]!=cv::Point2f(-1,-1)){
                cv::Scalar col(255*(i==2),255*(i==1),255*(i==0));
                cv::circle(dst, axis_points[i], radiusCircle, col, thicknessCircle1);
                cv::line(dst, origin, axis_points[i], col, 2);
            }

//        cv::circle(dst, cv::Point2f(vcg_cam.CenterPx.X(),vcg_cam.CenterPx.Y()), radiusCircle*2, cv::Scalar(255,255,0), 3);
//        cv::line(dst, cv::Point2f(vcg_cam.CenterPx.X(),0),cv::Point2f(vcg_cam.CenterPx.X(),vcg_cam.ViewportPx.Y()), cv::Scalar(255,255,0), 2);

        //
        imshow("receiver", dst);

        if (waitKey(1) == 'b') {
             break;
        }
        latest_frame_mutex.lock();
    }
    latest_frame_mutex.unlock();
    cv::destroyAllWindows();
}
void Camera::stop_reading(){
   latest_frame_mutex.lock();
   reading = false;
   latest_frame_mutex.unlock();
}

void Camera::export_camera_match(CameraMatch & c){
    const vcg::Point2f & cn = vcg_cam.CenterPx;
    const int h = vcg_cam.ViewportPx[1];

    c.o = vcg::Point3f(-(origin.x-cn.X()),-((h-origin.y)-cn.Y()), vcg_cam.FocalMm);
    c.o_im  = vcg::Point2f(origin.x,(h-origin.y));

    for(int i = 0; i < 3; ++i){
       c.axis_im[i] = vcg::Point2f(this->axis_points[i].x,(h-this->axis_points[i].y))-cn;
       c.axis_im[i].Normalize();
    }
    c.xax = vcg::Point2f(this->axis_points[0].x,this->axis_points[0].y);
    c.camera =  vcg_cam ;
}


vcg::Shotf Camera::SolvePnP(std::vector<vcg::Point3f> p3vcg){
    cv::Mat cm,dc;
    cv::Mat r,t;
    r =  cv::Mat(3,1,CV_32F);
    t =  cv::Mat(4,1,CV_32F);
    dc=  cv::Mat(4,1,CV_32F);
    dc=0;
//    dc.at<float>(0,0)=0.f;
    std::vector<cv::Point3d> p3;
    std::vector<cv::Point2d> p2, p2_auto;

    std::vector<cv::Mat> rvecs, tvecs;

    p2_auto.push_back(origin);
    for(int i=0; i < NP; ++i) p2_auto.push_back(axis_points[i]);
    

    // temporary hard-coded image pixel values
    p2.push_back(cv::Point2d(831,354));
    p2.push_back(cv::Point2d(1719, 158));
    p2.push_back(cv::Point2d(681,401));
    p2.push_back(cv::Point2d(1418,5));
    p2.push_back(cv::Point2d(519,1008));
    p2.push_back(cv::Point2d(863,1025));

// DEBUG
//    for(int i=0; i < 3; ++i) p2[i].y = 1096-p2[i].y;
//___


    for(int i=0; i < NP+1; ++i) p3.push_back(cv::Point3f(p3vcg[i].X(),p3vcg[i].Y(),p3vcg[i].Z()));

    cv::Mat rot_cv(3,3,CV_32F);

    try{

        /* SOLVE P3P */
//        cv::solveP3P(p3,p2,cameraMatrix,dc,rvecs,tvecs,/* SOLVEPNP_EPNP SOLVEPNP_AP3P*/ SOLVEPNP_P3P);
//        cv::Rodrigues(rvecs[0],rot_cv);
//        rot_cv.convertTo(rot_cv,CV_32F);
//        std::cout<< "rotation vector" << rot_cv << std::endl;
//        std::cout<< "rotation vector" << tvecs[0] << std::endl;
//        tvecs[0].convertTo(t,CV_32F);


#ifdef RECTIFY_FIRST
//        p3.erase(p3.begin()+4,p3.end());
//        p2.erase(p2.begin()+4,p2.end());
        bool status = cv::solvePnP(p3,p2,cameraMatrix,dc,r,t,false, SOLVEPNP_EPNP /*SOLVEPNP_AP3P   SOLVEPNP_P3P*/);
        std::cout << " Status of PnP" << status << std::endl;
        cv::TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, 20, 1e-8);
        cv::solvePnPRefineLM(p3, p2, cameraMatrix, dc, r, t, criteria);
#else
        cv::solvePnP(p3,p2,cameraMatrix,this->distCoeffs,r,t,false,SOLVEPNP_EPNP);
#endif
        std::cout<< "rotation vector" << r << std::endl;

        cv::Rodrigues(r,rot_cv);
        std::cout<< "rotation matrix" << rot_cv << std::endl;
        std::cout<< "translations" << t << std::endl;
       /* cv::Mat J;
        std::vector<Point2f> p;
        projectPoints(p3, r, t, cameraMatrix, Mat(), p, J);
        for (int i = 0; i < p.size(); ++i)
        {
            std::cout << p[i] << std::endl;
        }*/

    }catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }

    cv::Mat p3m;
    p3m=cv::Mat(3,1,CV_32F);
    std::vector < cv::Mat> camSp;
    camSp.resize(6);
    std::cout << "projection check "<< std::endl;
    for(int i=0; i < 6; ++i){
           p3m.at<float>(0,0)=p3[i].x;
           p3m.at<float>(1,0)=p3[i].y;
           p3m.at<float>(2,0)=p3[i].z;
           camSp[i] = rot_cv*p3m+t;
           //camSp[i] /=camSp[i].at<float>(2,0);

           cv::Mat p_ = cameraMatrix*camSp[i];
           //std::cout << p_ << std::endl;
           //std::cout << "3D: "<<p3m<< std::endl;
          /* std::cout << "3DCS: "<<camSp[i]<< std::endl;
           std::cout<< "2D: "<<p_ << std::endl;*/
           std::cout<< "2Dnr: "<<p_/p_.at<float>(2,0) << std::endl;
           cv::Point2f p(p_.at<float>(0, 0), p_.at<float>(1, 0));
           //std::cout<< p2[i]  << std::endl <<  std::endl;

    }


    vcg::Shotf shot;
    shot.Intrinsics = this->vcg_cam;
    vcg::Matrix44f rot;
    rot.SetIdentity();

    for(int i=0; i < 3;i++)
        for(int j=0; j < 3;j++)
            rot[i][j] = rot_cv.at<float>(i,j);

//----------------------------------------------------
    extrinsics.SetIdentity();
    for(int i=0; i < 3;i++)
        for(int j=0; j < 3;j++)
            extrinsics[i][j] = rot_cv.at<float>(i,j);

    extrinsics[0][3] = t.at<float>(0,0);
    extrinsics[1][3] = t.at<float>(1,0);
    extrinsics[2][3] = t.at<float>(2,0);

    vcg::Point4f _p = extrinsics*vcg::Point4f(0,0,0,1);
     _p = cameraMatrix44*_p;
     _p/=_p.Z();
// ---------------------------------------------------


    rot_cv = rot_cv.t();

// DEBUG back projection (OK)
//     for(int i=0; i < 4; ++i){
//        std::cout << "cam space" << camSp[i] << std::endl;
//       std::cout << "p3 retransf" << rot_cv*camSp[i]-rot_cv*t << std::endl;
//       std::cout << "p3 input " << p3[i] << std::endl;
//     }

    // rotate by 180° around X for converting opencv to opengl convention
    rot.transposeInPlace();
    rot.SetColumn(1,rot.GetColumn3(1)*-1);
    rot.SetColumn(2,rot.GetColumn3(2)*-1);
    rot.transposeInPlace();

    shot.Extrinsics.SetRot(rot);

    cv::Mat vp = -rot_cv*t;
    std::cout << "camera position "<< vp << std::endl;
    shot.SetViewPoint(vcg::Point3f(vp.at<float>(0,0),vp.at<float>(1,0),vp.at<float>(2,0)));


    // TEST VCG CAMERA
    for(int i = 0 ; i < NP+1; ++i){

        vcg::Point2f pr = shot.Project(p3vcg[i]);

        printf("%f %f %f ->",p3vcg[i][0],p3vcg[i][1],p3vcg[i][2]);
        printf("%f %f --- %f %f\n",pr[0],pr[1],p2[i].x,p2[i].y);
    }
    return shot;

}
