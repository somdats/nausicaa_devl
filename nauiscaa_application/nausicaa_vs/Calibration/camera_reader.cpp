#include "camera_reader.h"


#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/calib3d/calib3d.hpp>


#define RECTIFY_FIRST

//#undef SCENE_REPLAY
//#undef FAKE_IMAGE




#include <iostream>
#include <fstream>
#include <time.h>

#include"Logger.h"

#if SAVE_IMG
std::chrono::system_clock::time_point timeCamera1;
FILE* fi1 = nullptr;
FILE* fi2 = nullptr;
FILE* fi3 = nullptr;
FILE* fi4 = nullptr;
FILE* fi5 = nullptr;
FILE* fi6 = nullptr;
#endif // SAVE_IMG

using namespace cv;

using namespace std;



int radiusCircle = 10;
cv::Scalar colorCircle1(255,255,255);
int thicknessCircle1 = 2;



string type2str(int type);

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    Camera * cam =  (Camera*)userdata;
     if  ( event == EVENT_LBUTTONDOWN  ){
         if(flags & EVENT_FLAG_CTRLKEY)
             {
                 cam->p2i[cam->ax] = cv::Point(x,y);
                 cam->ax=(cam->ax+1)%cam->p3.size();
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

void Camera::init(uint port, std::string camera_intrinsics_file, int cameraID, bool scaramuzza){
    /////////////Scaramuzza camera parameter read///////////////////
    camID = cameraID;
    inStPort = port;
#if SAVE_IMG
    timeCamera1 = std::chrono::system_clock::now();
#endif
#ifdef SCENE_REPLAY
    
    std::string toFolder = DUMP_FOLDER_PATH + "\\Images\\" + std::to_string(cameraID) + "\\";
    std::string timestamps = toFolder + "timestamps.txt";
    FILE* ft = fopen(timestamps.c_str(), "r");
    while (!feof(ft)) {
        char time_alfanumeric[20];
        fscanf(ft, "%s", time_alfanumeric);
        std::string ta = std::string(time_alfanumeric);

        timed_images.push_back(std::make_pair(std::stoull(ta), toFolder + ta + ".jpeg"));
    }
    fclose(ft);

    std::sort(timed_images.begin(), timed_images.end());

  


#endif
  

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
        //K_out = K_out * Eigen::Vector3f(0.5f, 0.5f, 1.0f).asDiagonal();
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
#ifdef SCENE_REPLAY
    int i = 0;
    int first_i;
    while (timed_images[i].first < start_time)++i;
    first_i = i;
#else
    std::string Camerafull0args;
    //VideoCapture cap;
    std::string args = "! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! decodebin ! videoconvert !  appsink";
  /*  if ("5000" == std::to_string(inStPort))
    {*/
        std::string port = " udpsrc port = " + std::to_string(inStPort);
        Camerafull0args = port + " " + args;
    /*}*/
   
    
    //VideoCapture cap;

   /* VideoCapture cap("udpsrc port=5000 caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink",
            CAP_GSTREAMER);*/
    cap.open(Camerafull0args, CAP_GSTREAMER);
#endif

    latest_frame_mutex.lock();
    reading = true;
    namedWindow(std::to_string(this->camID).c_str(),1);
    setMouseCallback(std::to_string(this->camID).c_str(), CallBackFunc, this);

#if SAVE_IMG
    if (logger::isExistDirectory(DUMP_FOLDER_PATH))
    {
        std::string imgDir = DUMP_FOLDER_PATH + "Images";
       
        if (!logger::isExistDirectory(imgDir))
            fs::create_directory(imgDir);

        std::string camDir = imgDir + "/" + std::to_string(inStPort);
        if (!logger::isExistDirectory(camDir))
            fs::create_directory(camDir);
      
        std::string timeStampFileName = camDir + "/" + "timestamps.txt";
        if (fs::exists(timeStampFileName))
            fs::remove(timeStampFileName);

        if (inStPort == 5000 && std::string(timeStampFileName).find(std::to_string(inStPort)) != std::string::npos)
            fi1 = fopen(timeStampFileName.c_str(), "wb");
        if (inStPort == 5001 && std::string(timeStampFileName).find(std::to_string(inStPort)) != std::string::npos)
            fi2 = fopen(timeStampFileName.c_str(), "wb");

        // remove condition when all the camera functions
        if (CameraCount > 2) {
            if (inStPort == 5002 && std::string(timeStampFileName).find(std::to_string(inStPort)) != std::string::npos)
                fi3 = fopen(timeStampFileName.c_str(), "wb");
            if (inStPort == 5003 && std::string(timeStampFileName).find(std::to_string(inStPort)) != std::string::npos)
                fi4 = fopen(timeStampFileName.c_str(), "wb");
            if (inStPort == 5004 && std::string(timeStampFileName).find(std::to_string(inStPort)) != std::string::npos)
                fi5 = fopen(timeStampFileName.c_str(), "wb");
            if (inStPort == 5005 && std::string(timeStampFileName).find(std::to_string(inStPort)) != std::string::npos)
                fi6 = fopen(timeStampFileName.c_str(), "wb");
        }
    }
    else
    {
        std::cout << "root path does not exist:" << DUMP_FOLDER_PATH << std::endl;

    }
#endif // SAVE_IMG

    while (reading) {
        latest_frame_mutex.unlock();
        Mat frame;



#ifdef RECTIFY_FIRST
    #ifdef SCENE_REPLAY
        unsigned long long delta = timed_images[i % timed_images.size()].first - start_time;
        unsigned long long delta1 = clock() - restart_time + partial_time;

        int ii = first_i;
        if (time_running) {
            while ((ii < timed_images.size()) && virtual_time > timed_images[ii].first - start_time) ++ii;
            if (ii != i)
            {
                i = ii;
                std::this_thread::sleep_for(10ms);
                latest_frame_mutex.lock();
                dst = cv::imread(timed_images[i].second.c_str());
                latest_frame_mutex.unlock();
            }
        }
    #else 
        cap.read(frame); 
    
        //Mat temp(frame.size(), frame.type());
        //dst = frame.clone();
        //create_perspecive_undistortion_LUT(map1, map2, &o, scaleFactor);
        //std::cout << " Iam done with undistortion:" << std::endl;
        if (!frame.empty())
            cv::remap(frame, dst, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        else
        {
            std::cout << "Warning: camera frame empty\n" << std::endl;
        }


#if SAVE_IMG

        std::string tCam;
        bool stat = logger::getTimeStamp(timeCamera1, tCam, false);
        if (stat)
        {
            logger::saveImages(DUMP_FOLDER_PATH, tCam, dst, std::to_string(inStPort));
            if (inStPort == 5000)
                fprintf(fi1, "%s\n", tCam.c_str());
            if (inStPort == 5001)
                fprintf(fi2, "%s\n", tCam.c_str());

            // Remove the condition when all the cameras are functional
            if (CameraCount > 2)
            {
                if (inStPort == 5002)
                    fprintf(fi3, "%s\n", tCam.c_str());
                if (inStPort == 5003)
                    fprintf(fi4, "%s\n", tCam.c_str());
                if (inStPort == 5004)
                    fprintf(fi5, "%s\n", tCam.c_str());
                if (inStPort == 5005)
                    fprintf(fi6, "%s\n", tCam.c_str());
            }
            
        }
#endif // SAVE_IMG 

        //cv::imwrite("D:/CamImages/rectified_streamed_output.jpg", dst);
    #endif

#else
    #ifdef SCENE_REPLAY
            dst = cv::imread("image.jpg");
    #else 
            cap.read(dst);

    #endif
#endif

        // drawing
        if (dst.rows > 0) {
            for (int i = 0; i < this->p3.size(); ++i)
                if (p2i[i] != cv::Point2f(-1, -1)) {
                    cv::Scalar col(255, 255, 255);
                    cv::circle(dst, p2i[i], radiusCircle, col, thicknessCircle1);
                    cv::putText(dst, cv::String(std::to_string(i)), p2i[i], cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2, false);
                }
            imshow(std::to_string(this->camID).c_str(), dst);

        }
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
#if SAVE_IMG
   fclose(fi1);
   fclose(fi2);
   // remove when all the cameras are functional
   if (CameraCount > 2)
   {
       fclose(fi3);
       fclose(fi4);
       fclose(fi5);
       fclose(fi6);
   }
#endif
   latest_frame_mutex.unlock();
}

void Camera::export_camera_match(CameraMatch & c){
    //const vcg::Point2f & cn = vcg_cam.CenterPx;
    //const int h = vcg_cam.ViewportPx[1];

    //c.o = vcg::Point3f(-(origin.x-cn.X()),-((h-origin.y)-cn.Y()), vcg_cam.FocalMm);
    //c.o_im  = vcg::Point2f(origin.x,(h-origin.y));

    //for(int i = 0; i < 3; ++i){
    //   c.axis_im[i] = vcg::Point2f(this->axis_points[i].x,(h-this->axis_points[i].y))-cn;
    //   c.axis_im[i].Normalize();
    //}
    //c.xax = vcg::Point2f(this->axis_points[0].x,this->axis_points[0].y);
    //c.camera =  vcg_cam ;
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

 
    for(int i=0; i < p3vcg.size(); ++i) p2_auto.push_back(p2i[i]);

    for(int i=0; i < p3vcg.size(); ++i) p3.push_back(cv::Point3f(p3vcg[i].X(),p3vcg[i].Y(),p3vcg[i].Z()));

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
        bool status = cv::solvePnP(p3, p2_auto,cameraMatrix,dc,r,t,false, SOLVEPNP_EPNP /*SOLVEPNP_AP3P   SOLVEPNP_P3P*/);
        std::cout << " Status of PnP:" << status << std::endl;
        cv::TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, 20, 1e-8);
        cv::solvePnPRefineLM(p3, p2_auto, cameraMatrix, dc, r, t, criteria);
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
    camSp.resize(this->p3.size());
    std::cout << "projection check "<< std::endl;
    for(int i=0; i < this->p3.size(); ++i) {
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
    //for(int i = 0 ; i < this->p3.size(); ++i) {

    //    vcg::Point2f pr = shot.Project(p3vcg[i]);

    //    printf("%f %f %f ->",p3vcg[i][0],p3vcg[i][1],p3vcg[i][2]);
    //    printf("%f %f --- %f %f\n",pr[0],pr[1],p2[i].x,p2[i].y);
    //}
    return shot;

}
