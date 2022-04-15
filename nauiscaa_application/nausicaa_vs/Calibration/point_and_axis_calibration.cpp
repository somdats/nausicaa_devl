#include "point_and_axis_calibration.h"
#include "newuoa.h"
#include <vcg/math/matrix44.h>
#include <vcg/math/shot.h>


static CameraMatch cm;
double  best_or[4];


vcg::Shotf shot_rot_from_params(double *p){
    vcg::Matrix44f Ra,Rb,D;

    vcg::Shotf shot;
    shot.Intrinsics = cm.camera;

    Ra.SetRotateRad(p[0],vcg::Point3f(0,1,0));
    Rb.SetRotateRad(p[1],vcg::Point3f(1,0,0));
    D = Ra*Rb;

    // o_cs: projection vector of the 3D origin in camera space (equals D*vcg::Point3f(0,0,1) in world space)
    vcg::Point3f o_cs = cm.o;
    o_cs.Normalize();

    // rotate the camera so that the axis -z (in world space) match the axis o_cs ( in camera space )
    vcg::Matrix44f Rp;
    float angle = acos(o_cs[2]);
    vcg::Point3f rot_axis = (o_cs  ^ vcg::Point3f(0,0,1)).Normalize();
    Rp.SetRotateRad(angle,rot_axis);
    Rp.transposeInPlace();

    vcg::Matrix44f Rcam;
    Rcam.SetRotateRad(p[2],vcg::Point3f(0,0,1));

    shot.Extrinsics.SetRot(D*Rcam*Rp);
    shot.Extrinsics.SetTra(D*vcg::Point3f(0,0,p[3]));

    return shot;
}

struct Error{
    double operator () ( int n,double *p ){

        std::cout << "par "<< p[0] << " "<<p[1] << " "<<p[2]<< std::endl;
    // p[0],p[1]: yaw, pitch
    // p[2]: roll
    // p[3]: distance along the transformed z axis

    vcg::Shotf shot = shot_rot_from_params(p);

    // projection and error computation
    double err = 0.0;
    vcg::Point2f axis_pr;
    for(int i=0; i < 3; ++i){
        axis_pr = shot.Project(vcg::Point3f(i==0,i==1,i==2));
        axis_pr -= cm.o_im;
        if(axis_pr.Norm() < 0.00001)
            err+=1.0;
        else{
            axis_pr.Normalize();
            err += fabs(axis_pr*cm.axis_im[i]-1.0);
        }
    }
    std::cout<< "err 0 "<< err << std::endl;
    return err;
}
};

struct ErrorD{
    double operator () ( int n,double *p ){

    best_or[3] = *p;
    // p : distance along the transformed z axis

    vcg::Shotf shot = shot_rot_from_params(best_or);

    if(shot.GetViewPoint().Norm() < 1.0)
        return 1000000;

    // projection and error computation
    double err = 0.0;
    vcg::Point2f pr;

    pr = shot.Project(vcg::Point3f(0,0,0));
    err +=  (pr-cm.o_im).Norm();

    pr = shot.Project(vcg::Point3f(1,0,0));
    err +=  (pr-cm.xax).Norm();

    return err;
    }
};


vcg::Shotf find_extrinsics(CameraMatch _cm){
    cm = _cm;
    double  p;
    int n=3;
    Error e;
    ErrorD ed;
    double err,errd;

    best_or[3] = 10.0;
    err = min_newuoa<double,Error>(n,&best_or[0],e);
    for(int i=0; i < 3; ++i)
        best_or[i] = best_or[i]- floor(best_or[i]/M_PI)*M_PI;

    vcg::Shotf ss = shot_rot_from_params(best_or);

    errd = min_newuoa<double,ErrorD>(1,&p,ed);

    best_or[3] = p;

    return shot_rot_from_params(best_or);
}
