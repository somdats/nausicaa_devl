/****************************************************************************
 * VCGLib                                                            o o     *
 * Visual and Computer Graphics Library                            o     o   *
 *                                                                _   O  _   *
 * Copyright(C) 2007                                                \/)\/    *
 * Visual Computing Lab                                            /\/|      *
 * ISTI - Italian National Research Council                           |      *
 *                                                                    \      *
 * All rights reserved.                                                      *
 *                                                                           *
 * This program is free software; you can redistribute it and/or modify      *
 * it under the terms of the GNU General Public License as published by      *
 * the Free Software Foundation; either version 2 of the License, or         *
 * (at your option) any later version.                                       *
 *                                                                           *
 * This program is distributed in the hope that it will be useful,           *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 * GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
 * for more details.                                                         *
 *                                                                           *
 ****************************************************************************/

 /**
  * Minimal   trimesh viewer made with AntTweakBar and freglut
  *
  * This sample shows how to use togheter:
  * - the trimesh loading and initialization
  * - basic usage of the default manipulators (the "Trackball")
  */


#include "defines.h"
#include <GL/glew.h>
#include "shader_basic.h"
#include "fbo.h"
#include <stdio.h>
#include <GL/freeglut.h>
#include <AntTweakBar.h>

  /// vcg imports
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/create/platonic.h>

/// wrapper imports
#include <wrap/io_trimesh/import.h>
#include <wrap/gl/trimesh.h>
#include <wrap/gui/trackball.h>
#include <wrap/gl/shot.h>
#include <wrap/gl/camera.h>


#include "velodyne_reader.h"
#include "camera_reader.h"
#include <opencv2/imgproc.hpp>

///server header
#include "..\..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\server.h"
#include"..\..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\nausicaa_api_server.h"
#include"..\..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\Common.h"

#include"Logger.h"

#ifdef SCENE_REPLAY

unsigned long long start_time;
unsigned long long end_time;
unsigned long long restart_time;
unsigned int partial_time;
bool time_running = false;
unsigned int  virtual_time;
#endif

int CameraCount;
int NUMCAM;

#if VIDEO_STREAM

#include"OutputStream.h"
#include"VideoCodec.h"
#include"FrameProcessor.h"

#include"Config.h"


using namespace ffmpegCodec;

int fps = 30, bitrate = 3000000;
std::string h264profile = "high444";
std::string outputServer = "udp://146.48.84.241:83";

// encoder name
std::string codecName = "h264";

VideoCodec vCodec(codecName.c_str());

// instantiate output-stream
std::string codecFormat = "H264";

OutputStream outStream(codecFormat.c_str(), outputServer.c_str(), codecName.c_str());

AVFormatContext* fmtContext = nullptr;

AVCodecContext* cdcCntx = nullptr;

FrameProcessor fmProcessor;

#endif


std::mutex buff_mutex;

static GLubyte* pixelData = NULL;
size_t format_nchannels = 3;


struct PlaneC : public vcg::Plane3f {
    PlaneC() :valid(false) {};
    vcg::Point3f o;
    bool valid;
};

struct LineC : public vcg::Line3f {
    LineC() :valid(false) {};
    bool valid;
};

// instantiate server
Server servC, servS;
::Camera  cameras[6];

int currentLidar = 0;
int currentPlane = 0;
int currentCamera = 0;
int ax;
bool showPlanes = true;
bool showCameras = false;
bool showfromcamera = false;
bool drawAllLidars = false;
bool enable_proj = false;
std::vector<vcg::Point3f> selected;
PlaneC planes[2][3];
LineC  axis[2][3];
vcg::Matrix44f transfLidar[2];
std::mutex mesh_mutex;
float lid_col[2][3] = { {0.2,0.8,0.3},{0.2,0.3,0.8} };
unsigned int texture;
Shader point_shader, shadow_shader,texture_shader;
FBO shadowFBO, cameraFBO;
TwBar* bar; // Pointer to the tweak bar

//selection
vcg::Point2f corners_sel_2D[2];
bool selecting = false;
bool escapemode = true;
struct GLERR {
    GLERR() {
        int err = glGetError();
        if (err) {
            printf((const char*)gluErrorString(err));
            exit(0);
        }
    }
};


using namespace vcg;

class CFace;
class CVertex;
struct MyUsedTypes : public UsedTypes<	Use<CVertex>		::AsVertexType,
    Use<CFace>			::AsFaceType> {};

/// compositing wanted proprieties
class CVertex : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags> {};
class CFace : public vcg::Face<  MyUsedTypes, vcg::face::VertexRef, vcg::face::Normal3f, vcg::face::BitFlags > {};
class CMesh : public vcg::tri::TriMesh< std::vector<CVertex>, std::vector<CFace> > {};

#define NL 16
#define N_LIDARS 2
static std::string camIniFile = "../calib_results_25032022.txt";
struct LidarRender {

    Lidar  lidar;

    std::vector < float > samples;
    std::vector < float > distances;
    GLuint buffers[3];

    std::vector<GLuint> iTriangles;

    int n_strips;
    int n_verts;
    float deltaA;

    void fillGrid() {
        int az = lidar.latest_frame.azimuth[0];
        int idStrip = floor(az / deltaA);
        for (unsigned int i = 0; i < lidar.latest_frame.x.size(); ++i)
            if (lidar.latest_frame.laser_id[i] < 16)
            {

                idStrip = floor(lidar.latest_frame.azimuth[i] / deltaA);

                int j = lidar.latest_frame.laser_id[i];
                // laser_id are interleaved -15° to 0, 1 to 15°
                // Engineears suck
                j = (j % 2) ? 7 + j / 2 + 1 : j / 2;

                samples[(idStrip * NL + j) * 3] = lidar.latest_frame.x[i];
                samples[(idStrip * NL + j) * 3 + 1] = lidar.latest_frame.y[i];
                samples[(idStrip * NL + j) * 3 + 2] = lidar.latest_frame.z[i];
                distances[(idStrip * NL + j)] = lidar.latest_frame.distance[i];
                assert(idStrip < n_strips);
                assert((idStrip * NL + j) * 3 + 2 < samples.size());
            }

        glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
        glBufferData(GL_ARRAY_BUFFER, n_verts * 3 * sizeof(float), &(*samples.begin()), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
        glBufferData(GL_ARRAY_BUFFER, n_verts * sizeof(float), &(*distances.begin()), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void init() {
        int ns = 0;
        for (unsigned int i = 0; i < lidar.latest_frame.azimuth.size() - 1; ++i)
            if (lidar.latest_frame.azimuth[i + 1] - lidar.latest_frame.azimuth[i] > 0) {
                deltaA += lidar.latest_frame.azimuth[i + 1] - lidar.latest_frame.azimuth[i];
                ns++;
            }
        deltaA /= ns;

        n_strips = ceil(36000 / deltaA);
        n_verts = ceil(n_strips * NL);

        samples.resize(n_verts * 3);

        distances.resize(n_verts, 0.f);

        glCreateBuffers(3, buffers);
        GLERR();
        fillGrid();
        GLERR();

        glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
        glBufferData(GL_ARRAY_BUFFER, n_verts * 3 * sizeof(float), &(*samples.begin()), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
        glBufferData(GL_ARRAY_BUFFER, n_verts * sizeof(float), &(*distances.begin()), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        for (int i = 0; i < n_strips - 1; ++i)
            for (int j = 0; j < NL - 1; ++j)
            {
                iTriangles.push_back(i * NL + j);
                iTriangles.push_back((i + 1) * NL + j);
                iTriangles.push_back(i * NL + j + 1);

                iTriangles.push_back(i * NL + j + 1);
                iTriangles.push_back((i + 1) * NL + j);
                iTriangles.push_back((i + 1) * NL + j + 1);
            }

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[1]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, iTriangles.size() * sizeof(int), &*iTriangles.begin(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    }

};

LidarRender lidars[2];

/// the active mesh instance
CMesh mesh;


/// the active mesh opengl wrapper
vcg::GlTrimesh<CMesh> glWrap;
/// the active manipulator
vcg::Trackball track[2];

/// window size
int width, height;

/// we choosed a subset of the avaible drawing modes
enum DrawMode { PERPOINTS = 0, SMOOTH, WIRE, FLATWIRE, HIDDEN, FLAT };

/// the current drawmode
DrawMode drawmode;

/// Takes a GLUT MouseButton and returns the equivalent Trackball::Button
static vcg::Trackball::Button GLUT2VCG(int glut_button, int)
{
    int vcgbt = vcg::Trackball::BUTTON_NONE;

    switch (glut_button) {
    case GLUT_LEFT_BUTTON: vcgbt |= vcg::Trackball::BUTTON_LEFT;	break;
    case GLUT_MIDDLE_BUTTON: vcgbt |= vcg::Trackball::BUTTON_RIGHT;	break;
    case GLUT_RIGHT_BUTTON: vcgbt |= vcg::Trackball::BUTTON_MIDDLE;	break;
    }

    int modifiers = glutGetModifiers();

    if (modifiers & GLUT_ACTIVE_SHIFT)	vcgbt |= vcg::Trackball::KEY_SHIFT;
    if (modifiers & GLUT_ACTIVE_CTRL)	vcgbt |= vcg::Trackball::KEY_CTRL;
    if (modifiers & GLUT_ACTIVE_ALT)	vcgbt |= vcg::Trackball::KEY_ALT;

    return vcg::Trackball::Button(vcgbt);
}

void initMesh()
{
    // update bounding box
    vcg::tri::UpdateBounding<CMesh>::Box(mesh);
    // update Normals
    // Initialize the opengl wrapper
    glWrap.m = &mesh;
    glWrap.Update();
}



void updatePC(int il) {
    static bool first = true;
    lidars[il].lidar.latest_frame_mutex.lock();

    mesh.Clear();
    mesh.C() = vcg::Color4b(255, 244, 255, 255);
    if (lidars[il].lidar.latest_frame.x.empty()) {
        lidars[il].lidar.latest_frame_mutex.unlock();
        return;
    }

    if (first) {
        lidars[0].init();
        lidars[1].init();
    }


    glWrap.m = &mesh;
    vcg::tri::Allocator<CMesh>::AddVertices(*glWrap.m, lidars[il].lidar.latest_frame.x.size());

    for (unsigned int i = 0; i < lidars[il].lidar.latest_frame.x.size(); ++i) {
        mesh.vert[i].P()[0] = lidars[il].lidar.latest_frame.x[i];
        mesh.vert[i].P()[1] = lidars[il].lidar.latest_frame.y[i];
        mesh.vert[i].P()[2] = lidars[il].lidar.latest_frame.z[i];
    }

    lidars[il].fillGrid();

    if (first) {
        initMesh();
        first = false;
    }
    lidars[il].lidar.latest_frame_mutex.unlock();
}

vcg::Point3f win2NDC(vcg::Point2f in) {
    return  vcg::Point3f(-1.f + 2.f * in[0] / float(::width), -1 + 2 * in[1] / float(::height), 0);
}

vcg::Point3f mostOrtho(vcg::Point3f p) {
    int minID = (fabs(p[0]) < fabs(p[1])) ?
        (fabs(p[0]) < fabs(p[2]) ? 0 : 2) :
        (fabs(p[1]) < fabs(p[2]) ? 1 : 2);
    return vcg::Point3f(minID == 0, minID == 1, minID == 2);
}
void drawPlane(PlaneC pl, vcg::Point2f min, vcg::Point2f max) {
    vcg::Point3f u, v, up, n;
    n = pl.Direction();

    up = mostOrtho(n);
    //std::cout<< n*up <<std::endl;
    vcg::GetUV(n, u, v, up);

    vcg::Point3f p[4];
    p[0] = pl.o + u * min[0] + v * min[1];
    p[1] = pl.o + u * max[0] + v * min[1];
    p[2] = pl.o + u * max[0] + v * max[1];
    p[3] = pl.o + u * min[0] + v * max[1];

    glBegin(GL_QUADS);
    for (int i = 0; i < 4; ++i)
        glVertex3f(p[i][0], p[i][1], p[i][2]);
    glEnd();
}

void draw_frame(vcg::Matrix44f m) {
    glBegin(GL_LINES);
    for (int i = 0; i < 3; ++i) {
        glColor3f((i == 0), (i == 1), (i == 2));
        glVertex3f(0, 0, 0);
        vcg::Point3f a = m.GetRow3(i);
        glVertex3f(a[0], a[1], a[2]);
    }
    glEnd();
}


void initializeGLStuff() {

    if (point_shader.SetFromFile("./Calibration/Shaders/points.vs", "./Calibration/Shaders/triangles.gs"/* "points.gs" */,
        "./Calibration/Shaders/points.fs") < 0)
    {
        printf("SHADER ERR");
    }
    point_shader.Validate();
    GLERR();

    glUseProgram(point_shader.pr);
    point_shader.bind("mm");
    point_shader.bind("pm");
    point_shader.bind("toCam");
    point_shader.bind("camTex");
    point_shader.bind("camDepth");
    glUniform1i(point_shader["camTex"], 5);
    glUniform1i(point_shader["camDepth"], 6);
    glUseProgram(0);
    GLERR();

    if (shadow_shader.SetFromFile("./Calibration/Shaders/shadow_map.vs",
        "./Calibration/Shaders/shadow_map.gs", "./Calibration/Shaders/shadow_map.fs") < 0)
    {
        printf("shadow SHADER ERR");
    }
    shadow_shader.Validate();
    GLERR();
    shadow_shader.bind("toCam");



    if (texture_shader.SetFromFile("./Calibration/Shaders/texture.vs",
       0, "./Calibration/Shaders/texture.fs") < 0)
    {
        printf("texture SHADER ERR");
    }
    texture_shader.Validate();
    GLERR();
    texture_shader.bind("uTexture");

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glColor3f(1, 1, 1);
    glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_REPLACE);

 
    shadowFBO.Create(1948, 1096);

    cameraFBO.Create(1280, 720);
 
}

void drawScene() {
    /* BEGIN DRAW SCENE */
    vcg::Matrix44f toCamera;
    GLfloat mm[16], pm[16];

    for (int il = 0; il < N_LIDARS; ++il)if (currentLidar == il || drawAllLidars) {
        float mm1[16];
        glPushMatrix();
        glGetFloatv(GL_MODELVIEW_MATRIX, mm1);

        glMultMatrix(transfLidar[il]);
        toCamera.SetIdentity();

        GLERR();
        // draw axis
        for (int il = 0; il < 3; ++il)
            if (axis[currentLidar][il].valid) {
                glColor3f(il == 0, il == 1, il == 2);
                glBegin(GL_LINES);
                glVertex(axis[currentLidar][il].Origin());
                glVertex(axis[currentLidar][il].Origin() + axis[currentLidar][il].Direction());
                glEnd();
            }
        GLERR();


        if (enable_proj) {
            drawmode = SMOOTH;
            glUseProgram(point_shader.pr);


            GLERR();
            //toCamera = cameras[currentCamera].cameraMatrix44*cameras[currentCamera].extrinsics;

            glActiveTexture(GL_TEXTURE5);
            glBindTexture(GL_TEXTURE_2D, texture);


            static unsigned char* _ = 0;
            if (_ == 0) {
                _ = (unsigned char*)malloc(1948 * 1096 * 3);
                cameras[currentCamera].latest_frame_mutex.lock();
                memcpy(_, cameras[currentCamera].dst.ptr(), 1948 * 1096 * 3);
                cameras[currentCamera].latest_frame_mutex.unlock();
            }
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1948, 1096, 0, GL_RGB, GL_UNSIGNED_BYTE, _);


            GLERR();

            glGetFloatv(GL_PROJECTION_MATRIX, pm);
            glUniformMatrix4fv(point_shader["pm"], 1, GL_FALSE, pm);

            glGetFloatv(GL_MODELVIEW_MATRIX, mm);
            glUniformMatrix4fv(point_shader["mm"], 1, GL_FALSE, mm);

            // toCamera = cameras[currentCamera].cameraMatrix44*cameras[currentCamera].extrinsics*transfLidar[il];
            toCamera = cameras[currentCamera].opencv2opengl_camera(cameras[currentCamera].cameraMatrix, 1948, 1096, 0.5, 10) * cameras[currentCamera].opengl_extrinsics() * transfLidar[il];
            glUniformMatrix4fv(point_shader["toCam"], 1, GL_TRUE, &toCamera[0][0]);

            glActiveTexture(GL_TEXTURE6);
            glBindTexture(GL_TEXTURE_2D, shadowFBO.id_tex);
        }

       // glColor3f(il == 0, il == 0, il == 1);
        glColor3f(il == 0 || 1, il == 0, 0);

        GLERR();
        glBindBuffer(GL_ARRAY_BUFFER, lidars[il].buffers[0]);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

        glBindBuffer(GL_ARRAY_BUFFER, lidars[il].buffers[2]);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 1, GL_FLOAT, false, 0, 0);


        if (drawmode == PERPOINTS)
            glDrawArrays(GL_POINTS, 0, lidars[il].samples.size() / 3);
        if (drawmode == SMOOTH) {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lidars[il].buffers[1]);
            glDrawElements(GL_TRIANGLES, lidars[il].iTriangles.size(), GL_UNSIGNED_INT, 0);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        }

        GLERR();

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);


        glUseProgram(0);
        glPopMatrix();




        if (!showfromcamera && showCameras) {
            // draw cameras
            glColor3f(0, 1, 0);
            glPushMatrix();
            vcg::Point3f p = cameras[currentCamera].calibrated.GetViewPoint();
            glTranslatef(p[0], p[1], p[2]);
            gluSphere(gluNewQuadric(), 0.02, 10, 10);
            draw_frame(cameras[currentCamera].calibrated.Extrinsics.Rot());
            glPopMatrix();
        }
    }
    /* END DRAW SCENE */
}

void Display() {
    static bool init = true;
    if (init) {
        init = false;
        initializeGLStuff();
        pixelData = reinterpret_cast<GLubyte*>(malloc(3 * sizeof(GLubyte) * cameraFBO.w * cameraFBO.h));
    }

    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();



    vcg::Box3f boxsel;
    boxsel.SetNull();

    if (selecting && corners_sel_2D[0][0] != -1) {
        vcg::Point3f p0 = win2NDC(corners_sel_2D[0]);
        vcg::Point3f p1 = win2NDC(corners_sel_2D[1]);
        glBegin(GL_LINES);
        glVertex3f(p0[0], p0[1], p0[2]);
        glVertex3f(p1[0], p1[1], p1[2]);
        glEnd();

        boxsel.Add(vcg::Point3f(corners_sel_2D[0][0], corners_sel_2D[0][1], 0));
        boxsel.Add(vcg::Point3f(corners_sel_2D[1][0], corners_sel_2D[1][1], 0));
        boxsel.Offset(vcg::Point3f(0, 0, 0.1));
    }

    if (lidars[currentLidar].lidar.reading)
        updatePC(currentLidar);


    if (!mesh.vert.empty()) {
        vcg::Matrix44f toCamera;
        GLfloat mm[16], pm[16];

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(40, width / (float)height, 0.1, 100);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);

        track[currentLidar].center = vcg::Point3f(0, 0, 0);
        track[currentLidar].radius = 1;
        track[currentLidar].GetView();
        track[currentLidar].Apply();

        glPushMatrix();
        float d = 1.0f / mesh.bbox.Diag();
        vcg::glScale(d);
        glTranslate(-glWrap.m->bbox.Center());


        glUseProgram(0);
        glDisable(GL_LIGHTING);
        if (selecting) {
            vcg::Point3f p;
            GLdouble mm[16], pm[16];
            GLint view[4];
            double x, y, z;
            glGetDoublev(GL_MODELVIEW_MATRIX, mm);
            glGetDoublev(GL_PROJECTION_MATRIX, pm);
            glGetIntegerv(GL_VIEWPORT, view);


            if (!escapemode) {


                selected.clear();
                glPointSize(3.0);
                glColor3f(1, 0, 0);
                glBegin(GL_POINTS);
                for (unsigned int i = 0; i < mesh.vert.size(); ++i) {
                    p = mesh.vert[i].cP();
                    gluProject(p[0], p[1], p[2], mm, pm, view, &x, &y, &z);
                    if (boxsel.IsIn(vcg::Point3f(x, y, 0))) {
                        selected.push_back(vcg::Point3f(p[0], p[1], p[2]));
                        glVertex3f(selected.back()[0], selected.back()[1], selected.back()[2]);
                    }
                }
                glEnd();
                glPointSize(1.0);

                if (selected.size() > 4) {
                    vcg::FitPlaneToPointSet(selected, planes[currentLidar][currentPlane]);
                    PlaneC& pl = planes[currentLidar][currentPlane];

                    pl.o = selected[0];
                    for (uint i = 1; i < selected.size(); ++i)
                        pl.o += selected[i];
                    pl.o /= selected.size();
                    pl.o = pl.Projection(pl.o);
                    pl.valid = true;
                }
            }
        }

        if (showPlanes)
            for (int ip = 0; ip < 3; ++ip)
                if (planes[currentLidar][ip].valid) {
                    glColor3f(ip == 0, ip == 1, ip == 2);
                    drawPlane(planes[currentLidar][ip], vcg::Point2f(-1, -1), vcg::Point2f(1, 1));
                }
 
        bool virtualCamerasExist = !virtualCameras.empty();
        GLERR();


        for (int il = 0; il < N_LIDARS; ++il)
            updatePC(il);

        


        if (enable_proj) {
            // create shadow maps
            glViewport(0, 0, shadowFBO.w, shadowFBO.h); // shadowFBO will be one for each camera
            glBindFramebuffer(GL_FRAMEBUFFER, shadowFBO.id_fbo);
            glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);

            glUseProgram(shadow_shader.pr);
            vcg::Matrix44f oglP = cameras[currentCamera].opencv2opengl_camera(cameras[currentCamera].cameraMatrix, 1948, 1096, 0.5, 10.0);
 

            // these shadow maps will have to be created for each camera
            for (int il = 0; il < 2; ++il) {
                
                toCamera = oglP * cameras[currentCamera].opengl_extrinsics() * transfLidar[il];  // opengl matrices
                //toCamera = cameras[currentCamera].cameraMatrix44*cameras[currentCamera].extrinsics*transfLidar[il]; (opencv matrices)

                glUniformMatrix4fv(shadow_shader["toCam"], 1, GL_TRUE, &toCamera[0][0]);

                GLERR();
                glBindBuffer(GL_ARRAY_BUFFER, lidars[il].buffers[0]);
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

                glBindBuffer(GL_ARRAY_BUFFER, lidars[il].buffers[2]);
                glEnableVertexAttribArray(1);
                glVertexAttribPointer(1, 1, GL_FLOAT, false, 0, 0);

                glBindBuffer(GL_ARRAY_BUFFER, 0);

                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lidars[il].buffers[1]);
                glDrawElements(GL_TRIANGLES, lidars[il].iTriangles.size(), GL_UNSIGNED_INT, 0);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
            }
            glUseProgram(0);

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            //          cv::Mat ima(1096,1948,CV_8UC3);
            //          glGetTexImage(GL_TEXTURE_2D,0,GL_RGB,GL_UNSIGNED_BYTE,ima.ptr());
            //          cv::imwrite("depth_ogl.png",ima);

        }

        glViewport(0, 0, width, height);
        glClearColor(0.0, 0.0, 0.0, 1.0);      
        drawScene();

        if (showfromcamera) {
            // branch show from one of the real cameras. It is just camera 0 for now but this will change to 0-6
            GlShot<vcg::Shotf>::SetView(cameras[currentCamera].calibrated, 0.5, 10);
            glViewport(width/2, height/2, width/2, height/2);
            drawScene();
            GlShot<vcg::Shotf>::UnsetView(); 
        }
        if (streamON && virtualCamerasExist)
        {
            //streaming branch that write to framebuffer   
            glBindFramebuffer(GL_FRAMEBUFFER, cameraFBO.id_fbo);
            glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);
            glClearDepth(1.0);
            glViewport(0, 0, cameraFBO.w, cameraFBO.h);

            GlShot<vcg::Shotf>::SetView(virtualCameras[activeCamera], 0.5, 10);
           // GlShot<vcg::Shotf>::SetView(cameras[currentCamera].calibrated, 0.5, 10);

            drawScene();
            GlShot<vcg::Shotf>::UnsetView(); ;

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glBindTexture(GL_TEXTURE_2D, cameraFBO.id_tex);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, pixelData);

            
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();

            glViewport(0, 0, width, height);
            glDisable(GL_DEPTH_TEST);

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, cameraFBO.id_tex);
            glUseProgram(texture_shader.pr);
            glUniform1i(texture_shader["uTexture"], 0);
            glBegin(GL_QUADS);
            glVertexAttrib2f(1, 0.0, 0.0);
            glVertex3f(0.0, -1, 0.0);
            glVertexAttrib2f(1, 1.0, 0.0);
            glVertex3f(1.0, -1, 0.0);
            glVertexAttrib2f(1, 1.0, 1.0);
            glVertex3f(1.0, 0.0, 0.0);
            glVertexAttrib2f(1, 0.0, 1.0);
            glVertex3f(0.0, 0.0, 0.0);
            glEnd();
            glUseProgram(0);
            glEnable(GL_DEPTH_TEST);

            glMatrixMode(GL_PROJECTION);
            glPopMatrix();
            glMatrixMode(GL_MODELVIEW);
            glPopMatrix();

        }

        glPopMatrix();

          
          track[currentLidar].DrawPostApply();
    }

    glUseProgram(0);
    


    glActiveTexture(GL_TEXTURE0);
    TwRefreshBar(bar);
    TwDraw();
    ///////////////// send streaming data
;

    /////////////////////////////

    // Present frame buffer
    glutSwapBuffers();

    // Recall Display at next frame
    glutPostRedisplay();
    
#ifdef SCENE_REPLAY
    if (time_running) {
        virtual_time = clock() - restart_time + partial_time;
        if (virtual_time > end_time - start_time)
        {
            restart_time = clock();
            partial_time = virtual_time = 0;
        }
    }
#endif
}

void Reshape(int _width, int _height) {
    width = _width;
    height = _height;
  
 //  

    TwWindowSize(width, height);
}

void TW_CALL rotateAxis(void*) {
    vcg::Matrix44f R;
    R.SetRotateDeg(90, axis[currentLidar][1].Direction());
    axis[currentLidar][0].SetDirection(R * axis[currentLidar][0].Direction());
    axis[currentLidar][2].SetDirection(R * axis[currentLidar][2].Direction());

}
void TW_CALL alignCamera(void*) {
    cameras[currentCamera].calibrated = cameras[currentCamera].SolvePnP(cameras[currentCamera].p3);
}
void TW_CALL computeTranformation(void*) {

    vcg::Matrix44f T;
    for (int il = 0; il < 2; il++) {
        T.SetIdentity();
        T.SetColumn(0, axis[il][0].Direction());
        T.SetColumn(1, axis[il][1].Direction());
        T.SetColumn(2, axis[il][2].Direction());

        const vcg::Point3f& _o = axis[il][0].Origin();

        T.SetColumn(3, _o);
        T[3][3] = 1.0;

        T = vcg::Inverse(T);
        transfLidar[il] = T;
    }

}
void TW_CALL computeFrame(void*) {
    for (int ip = 0; ip < 3; ++ip)
        if (!planes[currentLidar][ip].valid)
            return;

    for (int il = 0; il < 3; ++il) {
        vcg::IntersectionPlanePlane(planes[currentLidar][(il + 2) % 3], planes[currentLidar][il], axis[currentLidar][il]);
        axis[currentLidar][il].valid = true;
    }
    vcg::Point3f o;
    IntersectionLinePlane(axis[currentLidar][1], planes[currentLidar][2], o);
    for (int il = 0; il < 3; ++il)
    {
        axis[currentLidar][il].SetOrigin(o);
        axis[currentLidar][il].Normalize();
    }

    if (axis[currentLidar][1].Direction()[2] < 0)
        axis[currentLidar][1].Flip();
    const vcg::Point3f& n0 = axis[currentLidar][0].Direction();
    const vcg::Point3f& n1 = axis[currentLidar][1].Direction();
    const vcg::Point3f& n2 = axis[currentLidar][2].Direction();
    if ((n0 ^ n1) * n2 < 0)
        axis[currentLidar][2].Flip();

}


void TW_CALL saveAxis(void*) {
    std::string calibration_file = std::string("Calib.bin");
#ifdef SCENE_REPLAY
    calibration_file = DUMP_FOLDER_PATH + "\\PointClouds\\" + calibration_file;
#endif
    FILE* fo = fopen(calibration_file.c_str(), "wb");
    for (int il = 0; il < 2; il++)
        for (int a = 0; a < 3; a++)
            fwrite(&axis[il][a], 1, sizeof(LineC), fo);
    fclose(fo);
}

void TW_CALL saveImPoints(void*) {
    std::string correspondences_file = std::to_string(cameras[currentCamera].camID) + "_correspondences.txt";
#ifdef SCENE_REPLAY
    correspondences_file = DUMP_FOLDER_PATH + "\\Images\\" + std::to_string(cameras[currentCamera].camID) + "\\" + correspondences_file;
#endif

    FILE* fo = fopen(correspondences_file.c_str(), "w");
    if (fo) {
        char trash[1000];
        fprintf(fo,"#correspondences x3d y3d z3d x2d y2d (x2d == y2s == -1 means not defined yet)\n");

        for( int i = 0; i < cameras[currentCamera].p3.size(); ++i)
        {
            fprintf(fo, "%f %f %f %f %f\n", cameras[currentCamera].p3[i].X(),
                cameras[currentCamera].p3[i].Y(), cameras[currentCamera].p3[i].Z(),
                cameras[currentCamera].p2i[i].x, cameras[currentCamera].p2i[i].y);
        }
        fclose(fo);
    }
}

void TW_CALL loadImPoints(void*) {
    std::string correspondences_file = std::to_string(cameras[currentCamera].camID) + "_correspondences.txt";
#ifdef SCENE_REPLAY
    correspondences_file = DUMP_FOLDER_PATH + "\\Images\\" + std::to_string(cameras[currentCamera].camID) + "\\" + correspondences_file;
#endif

    FILE* fo = fopen(correspondences_file.c_str(), "r");
    if (fo) {
        char trash[1000];
        fgets(trash, 1000, fo);
        cameras[currentCamera].p3.resize(0);

        int i = 0;
        float x, y, z, u, v;

        while (!feof(fo)) {
            int r = fscanf(fo, "%f %f %f %f %f", &x, &y, &z, &u, &v);
            if (r == 5) {
                cameras[currentCamera].p3.push_back(vcg::Point3f(x, y, z));
                cameras[currentCamera].p2i.push_back(cv::Point2f(u, v));
            }
        }
        fclose(fo);
    }
}

void TW_CALL loadAxis(void*) {
    std::string calibration_file = std::string("Calib.bin");
#ifdef SCENE_REPLAY
    calibration_file = DUMP_FOLDER_PATH + "\\PointClouds\\" + calibration_file;
#endif
    FILE* fo = fopen(calibration_file.c_str(), "rb");
    for (int il = 0; il < 2; il++)
        for (int a = 0; a < 3; a++)
            fread(&axis[il][a], 1, sizeof(LineC), fo);
    fclose(fo);
}




void   keyReleaseEvent(unsigned char k, int x, int y)
{
    int modifiers = glutGetModifiers();
    if (modifiers & GLUT_ACTIVE_CTRL)
        track[currentLidar].ButtonUp(Trackball::Button::KEY_CTRL);
    if (modifiers & GLUT_ACTIVE_SHIFT)
        track[currentLidar].ButtonUp(Trackball::Button::KEY_SHIFT);
    if (modifiers & GLUT_ACTIVE_ALT)
        track[currentLidar].ButtonUp(Trackball::Button::KEY_ALT);
}

void   keyPressEvent(unsigned char k, int x, int  y)
{

    if (k == 's') {
        ::corners_sel_2D[0][0] = -1;
        selecting = !selecting;
        escapemode = !selecting;
    }
    else if (k == 'e') {
        escapemode = !escapemode;
    }

    int modifiers = glutGetModifiers();
    if (modifiers & GLUT_ACTIVE_CTRL)
        track[currentLidar].ButtonDown(Trackball::Button::KEY_CTRL);
    if (modifiers & GLUT_ACTIVE_SHIFT)
        track[currentLidar].ButtonDown(Trackball::Button::KEY_SHIFT);
    if (modifiers & GLUT_ACTIVE_ALT)
        track[currentLidar].ButtonDown(Trackball::Button::KEY_ALT);

    TwEventKeyboardGLUT(k, x, y);
}

void mousePressEvent(int bt, int state, int x, int y) {
    if (TwEventMouseButtonGLUT(bt, state, x, y))
        return;

    if (selecting && !escapemode) {
        if (state == GLUT_DOWN) {
            ::corners_sel_2D[0] = vcg::Point2f(x, height - y);

        }
    }
    else
    {
        if (state == GLUT_DOWN)
            track[currentLidar].MouseDown(x, height - y, GLUT2VCG(bt, state));
        else
            track[currentLidar].MouseUp(x, height - y, GLUT2VCG(bt, state));

    }

};

void mouseMoveEvent(int x, int y)
{
    if (selecting && !escapemode) {
        if (::corners_sel_2D[0][0] != -1)
            ::corners_sel_2D[1] = vcg::Point2f(x, height - y);
    }
    else
        if (!TwEventMouseMotionGLUT(x, y))
            track[currentLidar].MouseMove(x, height - y);
}



void wheelEvent(int wheel, int direction, int x, int y) {
    track[currentLidar].MouseWheel(wheel * direction);
}


void TW_CALL CopyCDStringToClient(char** destPtr, const char* src)
{
    size_t srcLen = (src != NULL) ? strlen(src) : 0;
    size_t destLen = (*destPtr != NULL) ? strlen(*destPtr) : 0;

    // Alloc or realloc dest memory block if needed
    if (*destPtr == NULL)
        *destPtr = (char*)malloc(srcLen + 1);
    else if (srcLen > destLen)
        *destPtr = (char*)realloc(*destPtr, srcLen + 1);

    // Copy src
    if (srcLen > 0)
        strncpy(*destPtr, src, srcLen);
    (*destPtr)[srcLen] = '\0'; // null-terminated string
}
std::thread  tComm, tStream, tacc, taccSt;
std::thread tLidars[2],tCameras[6];

void start_lidar(int iL) {
    lidars[iL].lidar.start_reading();
}

void start_camera(int iC) {
    cameras[iC].start_reading();
}

void acceptCommunicationThread() {
    while (true)
        serverComm.accepting_connections();
}
void acceptStreamThread() {
    while (true)
        // serv.accepting_connections_stream();
        serverStream.accepting_connections();
}
void start_Communication_thread() {
    int portComm = 81;
    serverComm.start_server(portComm);
    std::string msg;
    while (true) {
        if (!serverComm.incoming_message(msg))
        {
            std::cout << msg << std::endl;
            call_API_function(msg);
        }
    }

}

void start_Streaming_thread() {
    int portStream = 82;
    serverStream.start_server(portStream);
    std::string  msg;
    //serv.wait_for_start(msg);
    while (true) {
        /* if (streamON)
         {*/
        buff_mutex.lock();
        int fbSize = format_nchannels * sizeof(GLubyte) * cameraFBO.w * cameraFBO.h;
        cv::Mat frame = cv::Mat(cameraFBO.h, cameraFBO.w, CV_8UC3);
        cv::Mat dstFrame;
        frame.data = pixelData;
        cv::flip(frame, dstFrame, 0);  
        
        cv::imwrite("streamed.jpg", dstFrame);
        
        std::vector<uchar>buf;
        cv::imencode(".jpg", dstFrame, buf);
        size_t szbuf = buf.size();
        //serverStream.send(reinterpret_cast<char*>(buf.data()));
        serverStream.send(reinterpret_cast<char*>(buf.data()), buf.size());
#if  VIDEO_STREAM
        outStream.createStream(fmProcessor, *cdcCntx, dstFrame);
#endif //  VIDEO_STREAM

     

      

        buff_mutex.unlock();
        cv::waitKey(10);
        /* }*/
    }

}

void TW_CALL initLidars(void*) {

    transfLidar[0].SetIdentity();
    transfLidar[1].SetIdentity();
    lidars[0].lidar.init(2368, "./Calibration/32db.xml");
    lidars[1].lidar.init(2369, "./Calibration/16db.xml");

    tLidars[0] = std::thread(&start_lidar,0);
    tLidars[1] = std::thread(&start_lidar,1);
    cv::waitKey(1000);
}

void TW_CALL initCameras(void*) {
//#if SAVE_IMG
//     timeCamera1 = std::chrono::system_clock::now();
//#endif
    for (int i = 0; i < NUMCAM; ++i)
    {
        cameras[i].init(5000 + i, camIniFile, 5000 + i, true);
        CameraCount++; // temporary hack to get the number of activated cameras
    }
    // disable the condition when all the camera functions

    for(int i = 0 ; i < CameraCount;++i)
        tCameras[i] = std::thread(&start_camera, i);

    //#if SAVE_IMG
//    std::string tCam;
//    bool stat =  getTimeStamp(timeCamera1, tCam);
//    if (stat)
//        saveImages(DUMP_FOLDER_PATH, tCam, cameras[0].dst);
//#endif
    tComm = std::thread(&start_Communication_thread);
    tStream = std::thread(&start_Streaming_thread);
    tacc = std::thread(&acceptCommunicationThread);
    taccSt = std::thread(&acceptStreamThread);

}

void TW_CALL runTest(void*) {
    ::initLidars(0);
    ::loadAxis(0);
    ::computeTranformation(0);
    ::initCameras(0);
    ::loadImPoints(0);
    ::alignCamera(0);
}
// to be used 
void TW_CALL startVideoStreaming(void*) {
    int vs = 1;
}

#ifdef SCENE_REPLAY
void TW_CALL time_startstop(void*) {
    time_running = !time_running;
    if (time_running) {
        restart_time = clock();
        virtual_time = partial_time;
    }
    else
        partial_time += clock() - restart_time;
    TwSetParam(bar, "start_stop", "label", TW_PARAM_CSTRING, 1, (time_running)?"stop":"play");
}
#endif

void Terminate() {
    if (lidars[0].lidar.reading)  lidars[0].lidar.stop_reading();
    if (lidars[1].lidar.reading)  lidars[1].lidar.stop_reading();

    for(int i = 0; i < NUMCAM; ++i)
        if (cameras[i].reading) cameras[i].stop_reading();

    for(int i = 0; i < 2;++i)
       if (tLidars[i].joinable())tLidars[i].join();
   
    for (int i = 0; i < CameraCount;++i)
            if (tCameras[i].joinable()) tCameras[i].join();

    if (tComm.joinable())tComm.join();
    if (tStream.joinable())tStream.join();

    free(pixelData);
}

void TW_CALL stop(void*) {
    if (lidars[0].lidar.reading) { lidars[0].lidar.stop_reading(); }
    if (lidars[1].lidar.reading) { lidars[1].lidar.stop_reading(); }
    for(int i = 0; i < 6; ++i)  
       if (cameras[i].reading) { cameras[i].stop_reading(); }
}

void read_first_and_last_timestamp(std::string path, unsigned long long &f, unsigned long long&l) {
    std::string timestamps = DUMP_FOLDER_PATH + "\\"+ path;
    FILE* ft = fopen(timestamps.c_str(), "r");
    char time_alfanumeric[20];
    unsigned long long first = -1;
    std::string ta, ms;
    

    while (!feof(ft)) {
        fscanf(ft, "%s", time_alfanumeric);
        ta = std::string(time_alfanumeric);
        
        if (first == -1) {
            first = std::stoull(ta.c_str());
            f = std::max(first, f);
        }
    }
    l   = std::min(std::stoull(ta.c_str()),l);
    fclose(ft);
}


int main(int argc, char* argv[])
{
    std::string inFile = "config.txt";
    vecPair configData = logger::readConfigFile(inFile);
    NUMCAM = stoi(configData[0].second);
    DUMP_FOLDER_PATH = configData[1].second; // "D:/CamImages/CamData";  //C:\\Users\\Fabio Ganovelli\\Documents\\GitHub\\nausicaa_devl\\data

    /*PacketDecoder::HDLFrame lidarFrame2;
  
    logger::LoadPointCloudBinary("D:\\Personal\\PointClouds\\2369\\1654782937525.bin", lidarFrame2);
    logger::savePointCloudASCII("D:/CamImages", lidarFrame2);*/
#if VIDEO_STREAM
    ////streaming class instantiation///////////////
    fmtContext = outStream.getFormatContext();
    int wd = 1280, he = 720;
    vCodec.setCodecParameter(wd, he, fps, bitrate, fmtContext->oformat->flags);
    vCodec.InitializeCodecStream(*outStream.getStream(), h264profile);

    cdcCntx = vCodec.getCodecContext();

    outStream.setExtras(cdcCntx->extradata, cdcCntx->extradata_size);

    av_dump_format(fmtContext, 0, outputServer.c_str(), 1);


    outStream.WriteHeader();
    outStream.setStreamFrameRate(cdcCntx->framerate);
    //instantiate frame-processor
    fmProcessor.resetDimension(wd, he);
    fmProcessor.initializeFrame(*cdcCntx);
    fmProcessor.initializeSampleScaler(*cdcCntx);
    fmProcessor.StreamInformation(*(outStream.getStream()));
#endif

    //activeCamera = -1;
   

    // Initialize AntTweakBar
    // (note that AntTweakBar could also be intialized after GLUT, no matter)
    if (!TwInit(TW_OPENGL, NULL))
    {
        // A fatal error occured    
        fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
        return 1;
    }


    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutCreateWindow("AntTweakBar simple example using GLUT");
    glutCreateMenu(NULL);

    /*glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);*/

    // Set GLUT callbacks
    glutDisplayFunc(Display);
    glutReshapeFunc(Reshape);
    atexit(Terminate);  // Called after glutMainLoop ends

        // Set GLUT event callbacks
    // - Directly redirect GLUT mouse button events to AntTweakBar
    glutMouseFunc((GLUTmousebuttonfun)mousePressEvent);
    // - Directly redirect GLUT mouse motion events to AntTweakBar
    glutMotionFunc((GLUTmousemotionfun)mouseMoveEvent);
    // - Directly redirect GLUT mouse "passive" motion events to AntTweakBar (same as MouseMotion)
    glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
    // - Directly redirect GLUT key events to AntTweakBar
    glutKeyboardFunc((GLUTkeyboardfun)TwEventKeyboardGLUT);
    // - Directly redirect GLUT special key events to AntTweakBar
    glutSpecialFunc((GLUTspecialfun)TwEventSpecialGLUT);

    glutKeyboardFunc(keyPressEvent);
    glutKeyboardUpFunc(keyReleaseEvent);


    glutMouseWheelFunc(wheelEvent);
    TwGLUTModifiersFunc(glutGetModifiers);
    bar = TwNewBar("Controls");
    TwDefine(" Controls size='240 480' "); // resize bar

    TwCopyCDStringToClientFunc(CopyCDStringToClient);

    TwAddButton(bar, "Init LIDAR", initLidars, 0, " label='start lidars' group=Input help=`initialize LIDARS` ");
    TwAddButton(bar, "Init Camera", initCameras, 0, " label='start cameras' group=Input help=`initialize Cameras` ");
    TwAddButton(bar, "stop", ::stop, 0, " label='stop reading' group=Input help=`stop input` ");


    TwAddVarRW(bar, "showplanes", TW_TYPE_BOOL8, &showPlanes, " label='Show Planes' group=`Register Lidars` help=` select` ");
    TwAddButton(bar, "compute frame", ::computeFrame, 0, " label='Compute Frame' group=`Register Lidars` help=`compute frame` ");
    TwAddButton(bar, "rotate", ::rotateAxis, 0, " label='rotate frame' group=`Register Lidars` help=`rotate frame` ");
    TwAddVarRW(bar, "Current Lidar", TW_TYPE_UINT32, &currentLidar, " label='currrent LIdar' min=0 max=1 group=`Register Lidars` help=` current lidar` ");
    TwAddVarRW(bar, "Current Plane", TW_TYPE_UINT32, &currentPlane, " label='currrent Plane' min=0 max=2 group=`Register Lidars` help=` current plane` ");
    TwAddVarRW(bar, "Current Axis", TW_TYPE_UINT32, &ax, " label='currrent Axis on the image' min=0 max=2 group=`Register Lidars` help=` current axis` ");
    TwAddButton(bar, "align lidar", ::computeTranformation, 0, " label='Align LIdars' group=`Register Lidars` help=`Align` ");
    TwAddButton(bar, "save", ::saveAxis, 0, " label='saveAxis' group=`Register Lidars` help=`rotate frame` ");
    TwAddButton(bar, "load", ::loadAxis, 0, " label='loadAxis' group=`Register Lidars` help=`rotate frame` ");

    TwAddButton(bar, "align camera", ::alignCamera, 0, " label='Align Camera' group=`Align Cameras` help=`Align` ");
    TwAddButton(bar, "saveIP", ::saveImPoints, 0, " label='saveImPoints' group=`Align Cameras` help=` ` ");
    TwAddButton(bar, "loadIP", ::loadImPoints, 0, " label='loadImPoints' group=`Align Cameras` help=` ` ");

    TwAddVarRW(bar, "showcameras", TW_TYPE_BOOL8, &showCameras, " label='showcameras' group=Rendering help=` select` ");
    TwAddVarRW(bar, "showfromcamera", TW_TYPE_BOOL8, &showfromcamera, " label='showfromcamera' group=`Rendering` help=` draw all` ");
    TwAddVarRW(bar, "mapcolor", TW_TYPE_BOOL8, &enable_proj, " label='map color' group=`Rendering` help=` draw all` ");
    TwAddVarRW(bar, "drawall", TW_TYPE_BOOL8, &drawAllLidars, " label='draw All' group=`Rendering` help=` draw all` ");

    TwAddButton(bar, "test", ::runTest, 0, " label='RUNTEST' group='Test' help=`test` ");
    TwAddButton(bar, "Stream", ::startVideoStreaming, 0, "label='VIDEOSTREAM' group='Streaming' help=`streaming` ");


    // ShapeEV associates Shape enum values with labels that will be displayed instead of enum values
    TwEnumVal drawmodes[6] = { {SMOOTH, "Smooth"}, {PERPOINTS, "Per Points"}, {WIRE, "Wire"}, {FLATWIRE, "FlatWire"},{HIDDEN, "Hidden"},{FLAT, "Flat"} };
    // Create a type for the enum shapeEV
    TwType drawMode = TwDefineEnum("DrawMode", drawmodes, 6);
    // add 'g_CurrentShape' to 'bar': this is a variable of type ShapeType. Its key shortcuts are [<] and [>].
    TwAddVarRW(bar, "Draw Mode", drawMode, &drawmode, " keyIncr='<' keyDecr='>' group=`Rendering` help='Change draw mode.' ");

#ifdef SCENE_REPLAY
    end_time = std::numeric_limits<unsigned long long >::max();
    start_time = 0;
    read_first_and_last_timestamp(std::string("PointClouds") + "\\2368\\timestamps.txt", start_time, end_time);
    read_first_and_last_timestamp(std::string("PointClouds") + "\\2369\\timestamps.txt", start_time, end_time);
    read_first_and_last_timestamp(std::string("Images") + "\\5000\\timestamps.txt", start_time, end_time);
    read_first_and_last_timestamp(std::string("Images") + "\\5001\\timestamps.txt", start_time, end_time);


    TwAddButton(bar, "start_stop", ::time_startstop, 0, " label='startstop' group=`Streaming` help=`Align` ");
    TwAddVarRW(bar, "virtualtime", TW_TYPE_UINT32, &virtual_time, " keyIncr='<' keyDecr='>' group=`Streaming` help='Change draw mode.' ");
#endif

    std::cout << "OpenGL version supported by this platform (%s): " << glGetString(GL_VERSION) << std::endl;

    int maxi;
    glGetIntegerv(GL_MAX_ELEMENTS_INDICES, &maxi);

    glewInit();


    glutMainLoop();
}
