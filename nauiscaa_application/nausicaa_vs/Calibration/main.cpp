﻿/****************************************************************************
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

#include <crtdbg.h>

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
#include "vcg_mesh.h" 

/// wrapper imports
#include <wrap/io_trimesh/import.h>
#include <wrap/gl/trimesh.h>
#include <wrap/gui/trackball.h>
#include <wrap/gl/shot.h>
#include <wrap/gl/camera.h>


#include "velodyne_reader.h"
#include "camera_reader.h"
#include "detect_3d_marker.h">
#include <opencv2/imgproc.hpp>

///server header
#include "..\..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\server.h"
#include "..\..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\nausicaa_api_server.h"
#include "..\..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\Common.h"

#include "Logger.h"
#include "state.h"

#ifdef MJPEG_WRITE

#include"..\headers\mjpeg_streamer.hpp"
using MJPEGStreamer = nadjieb::MJPEGStreamer;
MJPEGStreamer streamer;

#endif

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
int autoLaunch = 0;

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

VideoCodec vCodec(codecName.c_str(), outputServer);

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
    PlaneC(vcg::Plane3f p) { *(vcg::Plane3f*)this = p;valid = true; };
    vcg::Point3f o;
    bool valid;
};

struct LineC : public vcg::Line3f {
    LineC() :valid(false) {};
    bool valid;
};

// instantiate server
Server servC, servS;
std::vector<::Camera>  cameras;

int currentLidar = 0;
int currentPlane = 0;
int currentCamera = 0;
int ax = 0;
bool showPlanes = true;
bool showCameras = false;
bool showfromcamera = false;
bool drawAllLidars = false;
bool enable_proj = false;
bool drawBackground = false;

// values to define the boatFrame
float alphaFrame, betaFrame, gammaFrame;
float xFrame, yFrame, zFrame;

// values to define the boatFrameWS (set by the client)

float  LatDecimalDegrees    = 0.f;
float  LonDecimalDegrees   = 0.f;
float  ElevationMeters      = 0.f;
float  pitchDegrees         = 0.f;
float  rollDegrees          = 0.f;
float  bowDirectionDegrees  = 0.f;

// picking
bool pick_point = false;
int pick_x, pick_y;
float picked_point[6];



std::vector<vcg::Point3f> selected;
vcg::Point3f closest_sel;
PlaneC planes[2][3];
LineC  axis[2][3]; 
std::vector<vcg::Line3f> lines3d;
std::vector<vcg::Matrix44f> frames[2];
std::vector<vcg::Point3f>  points;
vcg::Point3f marker3D;

bool usePoint[100];
MarkerDetector md;

float xCoord, yCoord, zCoord;
int idFrame;

// markers
// a marker cannot appear smaller than
float min_size_on_screen = 100.f; 
float max_size_on_screen = 200.f;


// this matrix brings the point clouds to a common (local) reference system
vcg::Matrix44f transfLidar[2]; 

// this matrix brings the point clouds from a  common reference system to (local) one on the same point by rotated by pitch and roll
// in order to compensate for the floating
vcg::Matrix44f toSteadyFrame;

// this matrix brings the point clouds from the (local) steadyFrame to one with origin in wgs84 latlong and with z oriented with the compass
vcg::Matrix44f toGeoFrame;


// local reference frame for the data
vcg::Matrix44f boatFrame;   

// boatFrame in world space
vcg::Matrix44f boatFrameWS;


std::mutex mesh_mutex;
float lid_col[2][3] = { {0.2,0.8,0.3},{0.2,0.3,0.8} };
unsigned int *textures;
unsigned int markersTextureID;
int markers_pos_x = 0, markers_pos_y = 0;

Shader point_shader, shadow_shader,texture_shader,flat_shader;
std::vector<FBO> shadowFBO;
FBO cameraFBO;
TwBar* bar, // Pointer to the tweak bar
* frameBar, 
* pointsBar;

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

struct Tracker {
    Tracker() :active3D(false), active2D(false) {}
    bool active3D, active2D;
    vcg::Point3f currect_marker3D;
};
Tracker tracker;

#define NL 16
#define N_LIDARS 2
static std::string camIniFile; // "../calib_results_30062022.txt";
struct LidarRender {

    Lidar  lidar;

    vcg::Box3f bbox;
    std::vector < float > samples;
    std::vector < float > distances;
    GLuint buffers[3];

    std::vector<GLuint> iTriangles;

    int n_strips;
    int n_verts;
    float deltaA;

    void fillGrid() {
        memset(&samples[0],0,sizeof(float)*3*n_verts);
        memset(&distances[0], 0, sizeof(float) *  n_verts);

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
        if (lidar.latest_frame.azimuth.empty())
            return;
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
        lidars[il].bbox = mesh.bbox;
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
    vcg::Point3f o = m.GetColumn3(3);
    glBegin(GL_LINES);
    for (int i = 0; i < 3; ++i) {
        glColor3f((i == 0), (i == 1), (i == 2));
        glVertex3f(o[0], o[1], o[2]);
        vcg::Point3f a = m.GetColumn3(i);
        glVertex3f(o[0]+a[0], o[1] + a[1], o[2] + a[2]);
    }
    glEnd();
}
void drawLine(vcg::Line3f l) {
    vcg::Point3f p0 = l.Origin() - l.Direction();
    vcg::Point3f p1 = l.Origin() + l.Direction();
    glBegin(GL_LINES);

    glColor3f(1,1,0);
    glVertex3f(p0[0], p0[1], p0[2]);
    glVertex3f(p1[0], p1[1], p1[2]);
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
    assert(_CrtCheckMemory());
    glUseProgram(point_shader.pr);
    point_shader.bind("mm");
    point_shader.bind("lidarToWorld");
    point_shader.bind("pm");
    point_shader.bind("toCam");
    point_shader.bind("camTex");
    point_shader.bind("camDepth");
    point_shader.bind("aligned");
    point_shader.bind("used");
    GLint  texs[6] = { 5,6,7,8,9,10};
    GLint  dpts[6] = { 11,12,13,14,15,16};
    glUniform1iv(point_shader["camTex"],6, texs);
    glUniform1iv(point_shader["camDepth"],6, dpts);
    glUseProgram(0);
    GLERR();
    assert(_CrtCheckMemory());
    if (shadow_shader.SetFromFile("./Calibration/Shaders/shadow_map.vs",
        "./Calibration/Shaders/shadow_map.gs", "./Calibration/Shaders/shadow_map.fs") < 0)
    {
        printf("shadow SHADER ERR");
    }
    shadow_shader.Validate();
    GLERR();
    shadow_shader.bind("toCam");
    assert(_CrtCheckMemory());


    if (texture_shader.SetFromFile("./Calibration/Shaders/texture.vs",
       0, "./Calibration/Shaders/texture.fs") < 0)
    {
        printf("texture SHADER ERR");
    }
    texture_shader.Validate();
    GLERR();
    texture_shader.bind("uTexture");
    texture_shader.bind("mm");
    texture_shader.bind("pm");
    GLERR();
    assert(_CrtCheckMemory());
    if (flat_shader.SetFromFile("./Calibration/Shaders/flat.vs",
        "./Calibration/Shaders/flat.gs", "./Calibration/Shaders/flat.fs") < 0)
    {
        printf("flat SHADER ERR");
    }
    flat_shader.Validate();
    GLERR();
    
    flat_shader.bind("mm");
    flat_shader.bind("pm");
   
    GLERR();
    assert(_CrtCheckMemory());
    glGenTextures(1, &markersTextureID);
    textures = new unsigned int[NUMCAM];
    glGenTextures(NUMCAM, textures);
    assert(_CrtCheckMemory());
    for (int i = 0; i < NUMCAM; ++i) {
        glActiveTexture(GL_TEXTURE5 + i);
        glBindTexture(GL_TEXTURE_2D, textures[i]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glColor3f(1, 1, 1);
        glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_REPLACE);
        GLERR();
        assert(_CrtCheckMemory());
    }
    shadowFBO.resize(NUMCAM);
    
    for (int i = 0; i < NUMCAM; ++i) {
        shadowFBO[i].Create(1948, 1096);
        assert(_CrtCheckMemory());
    }
    cameraFBO.Create(1280, 720);
    GLERR();
    glActiveTexture(GL_TEXTURE5 + NUMCAM);
    glBindTexture(GL_TEXTURE_2D, markersTextureID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2048, 2048, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    GLERR();
}

void updateBoatFrame() {
        boatFrame = vcg::Matrix44f().SetRotateDeg(alphaFrame, vcg::Point3f(1, 0, 0))*
                    vcg::Matrix44f().SetRotateDeg(betaFrame, vcg::Point3f(0, 1, 0))*
                    vcg::Matrix44f().SetRotateDeg(gammaFrame, vcg::Point3f(0, 0, 1));
        boatFrame.SetColumn(3, vcg::Point3f(xFrame,yFrame,zFrame));
}
void updateToSteadyFrame() {
    vcg::Matrix44f rollMat;
    toSteadyFrame.SetRotateDeg(pitchDegrees, vcg::Point3f(1.0, 0.0, 0.0));
    rollMat.SetRotateDeg(rollDegrees, vcg::Point3f(0.0, 0.0, 1.0));

    toSteadyFrame = toSteadyFrame * rollMat;
    toSteadyFrame = vcg::Inverse(toSteadyFrame);
}
void updateToGeoFrame() {
    toGeoFrame.SetIdentity();
    vcg::Matrix44f bowRot;
    bowRot.SetRotateDeg(bowDirectionDegrees, vcg::Point3f(0.0, 1.0, 0.0));
    toSteadyFrame.SetRotateDeg(pitchDegrees, vcg::Point3f(1.0, 0.0, 0.0));
    bowRot.SetRotateDeg(rollDegrees, vcg::Point3f(0.0, 0.0, 1.0));
    toGeoFrame = bowRot * toGeoFrame;
    toGeoFrame.SetColumn(3, vcg::Point3f(LatDecimalDegrees, LonDecimalDegrees, ElevationMeters));
    toGeoFrame = vcg::Inverse(toGeoFrame);
}

//GLubyte _dat[4194304*4];


void drawString(vcg::Point3f p, const char* string, int size) {
    void *  fonts[4] = { GLUT_BITMAP_HELVETICA_10,GLUT_BITMAP_HELVETICA_12,GLUT_BITMAP_HELVETICA_18,GLUT_BITMAP_TIMES_ROMAN_24 };
    int sizes[4] = { 10,12,18,24 };
    glRasterPos3f(p[0], p[1], p[2]);
    glColor3f(1, 1, 1);
    int fontsize = size / (1 + std::string(string).length());
    void  * font = fonts[3];
    for(int fi = 0; fi < 4; fi)
        if (fontsize < sizes[fi])
        {
            font = fonts[fi];
            break;
        }

    for (const char* c = string; *c != '\0'; c++) {
        glutBitmapCharacter(font, *c);  // Updates the position
    }
}
//char* _data[2048 * 2048 * 4];

void drawScene() {
    vcg::Matrix44f toCamera[6];
    GLfloat mm[16], pm[16],mm1[16];
    GLint aligned[6];
    GLint used[6];

    glGetFloatv(GL_MODELVIEW_MATRIX, mm1);
    glGetFloatv(GL_MODELVIEW_MATRIX, mm);
    glGetFloatv(GL_PROJECTION_MATRIX, pm);

    if (enable_proj) {
        drawmode = SMOOTH;
        glUseProgram(point_shader.pr);
        for (int ic = 0; ic < NUMCAM; ++ic)
            if (cameras[ic].aligned) {
                glActiveTexture(GL_TEXTURE5 + ic);
                glBindTexture(GL_TEXTURE_2D, textures[ic]);

                cameras[ic].latest_frame_mutex.lock();
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1948, 1096, 0, GL_BGR, GL_UNSIGNED_BYTE, cameras[ic].dst.ptr());
                cameras[ic].latest_frame_mutex.unlock();

                // toCamera[ic] = cameras[ic].opencv2opengl_camera(cameras[ic].cameraMatrix, 1948, 1096, 0.5, 50) * cameras[ic].opengl_extrinsics() * toSteadyFrame * transfLidar[il];
                toCamera[ic] = cameras[ic].opencv2opengl_camera(cameras[ic].cameraMatrix, 1948, 1096, 0.1, 150) * cameras[ic].opengl_extrinsics();
                //glUniformMatrix4fv(point_shader["toCam"], 1, GL_TRUE, &toCamera[0][0]);         
                GLERR();
                aligned[ic] = 1;
                used[ic] = cameras[ic].used;
            }
            else
                aligned[ic] = 0;
        glUniformMatrix4fv(point_shader["toCam"], NUMCAM, GL_TRUE, &toCamera[0][0][0]);
        glUniform1iv(point_shader["aligned"], NUMCAM, aligned);  
        glUniform1iv(point_shader["used"], NUMCAM, used);

        glGetFloatv(GL_PROJECTION_MATRIX, pm);
        glUniformMatrix4fv(point_shader["pm"], 1, GL_FALSE, pm);
        glUseProgram(0);
    }
    for (int il = 0; il < lines3d.size(); ++il)
        drawLine(lines3d[il]);

    for (int il = 0; il < N_LIDARS; ++il)if (currentLidar == il || drawAllLidars) {
        glPushMatrix();
       
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

        glMultMatrix(transfLidar[il]);
        glMultMatrix(toSteadyFrame);
        for(int ic=0; ic < NUMCAM; ++ic) toCamera[ic].SetIdentity();

        if (enable_proj) {
            drawmode = SMOOTH;
            glUseProgram(point_shader.pr);

            GLERR();
 
            glGetFloatv(GL_MODELVIEW_MATRIX, mm);
            glUniformMatrix4fv(point_shader["mm"], 1, GL_FALSE, mm);

            vcg::Matrix44f l2w = toSteadyFrame* transfLidar[il];
            glUniformMatrix4fv(point_shader["lidarToWorld"], 1, GL_TRUE, &l2w[0][0]);

            // THIS ONLY NEED TO BE DONE ONCE
            for (int i = 0; i < NUMCAM; ++i) {
                glActiveTexture(GL_TEXTURE11 + i); // here we need to pass all the cameras
                glBindTexture(GL_TEXTURE_2D, shadowFBO[i].id_tex);
            }
        }
        else 
            if (drawmode == SMOOTH)
            {
            glUseProgram(flat_shader.pr);
            glGetFloatv(GL_PROJECTION_MATRIX, pm);
            glUniformMatrix4fv(flat_shader["pm"], 1, GL_FALSE, pm);

            glGetFloatv(GL_MODELVIEW_MATRIX, mm);
            glUniformMatrix4fv(flat_shader["mm"], 1, GL_FALSE, mm);

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

        glPopMatrix();
        glUseProgram(0);
    }

    
        if (markers.empty()) {
            markers_pos_x = 0;
            markers_pos_y = 0;
        } else
         if (!virtualCameras.empty()) {
            glUseProgram(texture_shader.pr);

            glActiveTexture(GL_TEXTURE0 + NUMCAM);
            glBindTexture(GL_TEXTURE_2D, markersTextureID);
            glUniform1i(texture_shader["uTexture"], NUMCAM);
            glUseProgram(0);

            for (std::map<unsigned int, Marker>::iterator im = markers.begin(); im != markers.end(); ++im)
                if ((*im).second.visible)
                {
                    if ((*im).second.png_data != 0) {
                        GLERR();
                        (*im).second.tc[0] = markers_pos_x;
                        (*im).second.tc[1] = markers_pos_y;

                        //texture needs to be created
                        glBindTexture(GL_TEXTURE_2D, markersTextureID);
                        cv::Mat matImg;
                        matImg = cv::imdecode(cv::Mat(1, (*im).second.png_data_length, CV_8UC1, (*im).second.png_data), cv::IMREAD_UNCHANGED);
                        cv::flip(matImg, matImg, 0);
                        glTexSubImage2D(GL_TEXTURE_2D, 0, (*im).second.tc[0], (*im).second.tc[1], 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, matImg.ptr());

                        // DBG show to whole texture    
                        //glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA,GL_UNSIGNED_BYTE,_data);
                        //cv::Mat FT(2048, 2048, CV_8UC4, _data);
                        //cv::imwrite("texture.png", FT);

                        GLERR();
                        delete[](*im).second.png_data;
                        (*im).second.png_data = 0;

                        // define the next square where to copy
                        if (markers_pos_x < 1984) // before the last 64x64 square of the row
                            markers_pos_x += 64;
                        else {// make a new row
                            markers_pos_y += 64;
                            markers_pos_x = 0;
                        }
                        if (markers_pos_y == 2048) {
                            printf("more than 32x32 images");
                            exit(0);
                        }
                    }
                }

            vcg::Matrix44f billboard_frame = virtualCameras[activeCamera].Extrinsics.Rot();
            vcg::Point3f pov = virtualCameras[activeCamera].GetViewPoint();
            billboard_frame.transposeInPlace();
            vcg::Point3f z_ax = billboard_frame.GetColumn3(2);
            float sx, dx, bt, tp, n;
            virtualCameras[activeCamera].Intrinsics.GetFrustum(sx, dx, bt, tp, n);
            float met_2_pixels =  virtualCameras[activeCamera].Intrinsics.ViewportPx[0]/ (dx - sx);

            std::vector<Marker> sorted_markers;
            for (std::map<unsigned int, Marker>::iterator im = markers.begin(); im != markers.end(); ++im)
                if ((*im).second.visible) {
                    (*im).second.z = (pov - (*im).second.pos) * z_ax;
                    sorted_markers.push_back(im->second);
                }

            std::sort(sorted_markers.begin(), sorted_markers.end());

            for (std::vector<Marker> ::iterator im = sorted_markers.begin(); im != sorted_markers.end(); ++im)
                if ((*im).visible){
                    // Compute the frame for the billboard
                    billboard_frame.SetColumn(3, (*im).pos);

                    //draw_frame(billboard_frame);

                    glGetFloatv(GL_MODELVIEW_MATRIX, mm);
                    vcg::Matrix44f vm_(mm);
                    vm_.transposeInPlace();
                    vcg::Matrix44f mm_ = vm_ * billboard_frame;

                    glUseProgram(texture_shader.pr);
                    glUniformMatrix4fv(texture_shader["mm"],1,true, &mm_[0][0]);
                    glUniformMatrix4fv(texture_shader["pm"], 1, false, pm);

                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

                    float size_on_viewplane = (*im).width * n / (*im).z;
                    float size_on_screen = size_on_viewplane  * met_2_pixels;
                    float ratio = 1.f;
                    if (size_on_screen < min_size_on_screen) {
                        ratio = min_size_on_screen / size_on_screen;
                        size_on_screen = min_size_on_screen;
                    }else
                        if (size_on_screen < max_size_on_screen) {
                            ratio = max_size_on_screen / size_on_screen;
                            size_on_screen = max_size_on_screen;
                        }


                    float h_size  = (*im).width* ratio /2.0;


                    glBegin(GL_TRIANGLES);
                    glVertexAttrib2f(1, (*im).tc[0] / 2048.f, (*im).tc[1] / 2048.f);
                    glVertex3f(-h_size, h_size, 0.0);
                    glVertexAttrib2f(1, ((*im).tc[0] + 64) / 2048.f, (*im).tc[1] / 2048.f);
                    glVertex3f(h_size, h_size, 0.0);
                    glVertexAttrib2f(1, ((*im).tc[0] + 64) / 2048.f, ((*im).tc[1] + 64) / 2048.f);
                    glVertex3f(h_size, h_size*2.f, 0.0);

                    glVertexAttrib2f(1, (*im).tc[0] / 2048.f, (*im).tc[1] / 2048.f);
                    glVertex3f(-h_size, h_size , 0.0);
                    glVertexAttrib2f(1, ((*im).tc[0] + 64) / 2048.f, ((*im).tc[1] + 64) / 2048.f);
                    glVertex3f(h_size, h_size * 2.f, 0.0);
                    glVertexAttrib2f(1,  (*im).tc[0]   / 2048.f, ((*im).tc[1] + 64) / 2048.f);
                    glVertex3f(-h_size, h_size * 2.f, 0.0);
                    glEnd();
                    glUseProgram(0);
                    GLERR();
                    glDisable(GL_BLEND);
                    drawString((*im).pos, (*im).label.c_str(), size_on_screen);
                }
        
        GLERR();
    }

    if(drawBackground)
    if (drawmode == SMOOTH && enable_proj) {
        glUseProgram(point_shader.pr);
        glUniformMatrix4fv(point_shader["lidarToWorld"], 1, GL_TRUE, &vcg::Matrix44f::Identity()[0][0]);
        glUniformMatrix4fv(point_shader["mm"], 1, GL_FALSE, mm1);
        gluSphere(gluNewQuadric(), 100.0, 10, 10);
        glUseProgram(0);
    }
    /* END DRAW SCENE */
}
void TW_CALL detectMarker(void*);
void Display() {
    assert(_CrtCheckMemory());
    static bool init = true;
    if (init) {
        init = false;
        initializeGLStuff();
       
        pixelData = reinterpret_cast<GLubyte*>(malloc(3 * sizeof(GLubyte) * cameraFBO.w * cameraFBO.h));
    }
    assert(_CrtCheckMemory());
    updateBoatFrame();
    updateToSteadyFrame();
    updateToGeoFrame();
    assert(_CrtCheckMemory());
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    vcg::Box3f boxsel;
    boxsel.SetNull();
    assert(_CrtCheckMemory());
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
        vcg::Matrix44f toCurrCamera;
        GLfloat mm[16], pm[16];

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(40, width / (float)height, 0.1, 100);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(1, 1, 5, 0, 0, 0, 0, 1, 0);


        track[currentLidar].center = vcg::Point3f(0, 0, 0);
        track[currentLidar].radius = 1.0;
        track[currentLidar].GetView();
        track[currentLidar].Apply();


        glPushMatrix();

/*         float d = 1.0f / mesh.bbox.Diag();
        vcg::glScale(d);
        glTranslate(-glWrap.m->bbox.Center());
*/
        float d = 1.0f / lidars[currentLidar].bbox.Diag();
        vcg::glScale(d);
        glTranslate(-lidars[currentLidar].bbox.Center());

        draw_frame(boatFrame);

        glUseProgram(0);
        glDisable(GL_LIGHTING);
        if (selecting) {
            vcg::Point3f p;
            GLdouble mm[16], pm[16];
            GLint view[4];
            double x, y, z,zmin = std::numeric_limits<float>::max();
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
                    p = transfLidar[currentLidar] * p;
                    gluProject(p[0], p[1], p[2], mm, pm, view, &x, &y, &z);
                    if (boxsel.IsIn(vcg::Point3f(x, y, 0))) {
                        selected.push_back(vcg::Point3f(p[0], p[1], p[2]));
                        glVertex3f(selected.back()[0], selected.back()[1], selected.back()[2]);
                        if (z < zmin) {
                            zmin = z;
                            closest_sel = p;
                        }
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

        if (tracker.active3D)
            detectMarker(0);

        if (enable_proj) {
            for (int iCam = 0; iCam < NUMCAM;++iCam)
                if(cameras[iCam].aligned)
            {
                // create shadow maps
                glViewport(0, 0, shadowFBO[iCam].w, shadowFBO[iCam].h); // shadowFBO will be one for each camera
                glBindFramebuffer(GL_FRAMEBUFFER, shadowFBO[iCam].id_fbo);
                glClearColor(1.0, 1.0, 1.0, 1.0);
                glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
                glEnable(GL_DEPTH_TEST);

                glUseProgram(shadow_shader.pr);
                vcg::Matrix44f oglP = cameras[iCam].opencv2opengl_camera(cameras[iCam].cameraMatrix, 1948, 1096, 0.1, 150.0);
               
                for (int il = 0; il < N_LIDARS; ++il) {

                    toCurrCamera = oglP * cameras[iCam].opengl_extrinsics() * toSteadyFrame * transfLidar[il];  // opengl matrices

                    glUniformMatrix4fv(shadow_shader["toCam"], 1, GL_TRUE, &toCurrCamera[0][0]);

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
                //if (iCam == 0) {
                //   glBindTexture(GL_TEXTURE_2D, shadowFBO[iCam].id_tex);
                //   cv::Mat ima(1096,1948,CV_8UC3);
                //   glGetTexImage(GL_TEXTURE_2D,0,GL_BGR,GL_UNSIGNED_BYTE,ima.ptr());
                //   cv::flip(ima, ima, 0);
                //   cv::imwrite((std::string("depth_ogl")+std::to_string(iCam)+".png").c_str(), ima);
                //}
            }


        }

        glViewport(0, 0, width, height);
        glClearColor(0.0, 0.0, 0.0, 1.0);      
       // glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        drawScene();

        if (!showfromcamera && showCameras) {
            // draw cameras
            for (int ic = 0; ic < cameras.size(); ++ic) {
                glColor3f(0, 1, 0);
                glPushMatrix();
                vcg::Point3f p = cameras[ic].calibrated.GetViewPoint();
                glTranslatef(p[0], p[1], p[2]);
                gluSphere(gluNewQuadric(), 0.2, 10, 10);
                draw_frame(cameras[ic].calibrated.Extrinsics.Rot());
                glPopMatrix();
            }
        }

        for (int ip = 0; ip < points.size(); ++ip)
            if(::usePoint[ip])
                {
                    glColor3f(0, 0, 1);
                    glPushMatrix();
                    vcg::Point3f p = points[ip];
                    glTranslatef(p[0], p[1], p[2]);
                    gluSphere(gluNewQuadric(), 0.02, 10, 10);
                    glPopMatrix();
                }
        {
            glColor3f(1, 0, 0);
            glPushMatrix();
            vcg::Point3f p = marker3D;
            glTranslatef(p[0], p[1], p[2]);
            gluSphere(gluNewQuadric(), 0.02, 10, 10);
            glPopMatrix();
        }
        {
            glColor3f(0, 1, 0);
            glPushMatrix();
            vcg::Point3f p = closest_sel;
            glTranslatef(p[0], p[1], p[2]);
            gluSphere(gluNewQuadric(), 0.02, 10, 10);
            glPopMatrix();
        }
        for (int fi = 0; fi < frames[0].size(); ++fi)
            if (::idFrame == fi)
                draw_frame(frames[0][fi]);

        if (showfromcamera) {
            // branch show from one of the currentCamera
            GlShot<vcg::Shotf>::SetView(cameras[currentCamera].calibrated, 0.1, 10);
            glViewport(width/2, height/2, width/2, height/2);
            drawScene();
            GlShot<vcg::Shotf>::UnsetView(); 
            glViewport(0,0, width, height);
        }
        if (streamON && virtualCamerasExist)
        {
            //streaming branch that write to framebuffer   
            glBindFramebuffer(GL_FRAMEBUFFER, cameraFBO.id_fbo);
            glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);
            glClearDepth(1.0);
            glViewport(0, 0, cameraFBO.w, cameraFBO.h);

            float sx, dx, bt, tp, n;
            virtualCameras[activeCamera].Intrinsics.GetFrustum(sx, dx, bt, tp, n);


            GlShot<vcg::Shotf>::SetView(virtualCameras[activeCamera], n, 3000);

            drawScene();
            if (::pick_point) {
                double  x3d, y3d, z3d;
                float z;
                GLdouble mm[16], pm[16];
                GLint view[4];
                glGetDoublev(GL_MODELVIEW_MATRIX, mm);
                glGetDoublev(GL_PROJECTION_MATRIX, pm);
                glGetIntegerv(GL_VIEWPORT, view);

                glReadPixels(pick_x, pick_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);

                gluUnProject(pick_x, pick_y, z, mm, pm, view, &x3d, &y3d, &z3d);

                picked_point[0] = x3d;
                picked_point[1] = y3d;
                picked_point[2] = z3d;
                vcg::Point4f xyzGeo = toGeoFrame* vcg::Point4f(x3d, y3d, z3d, 1.0);
                picked_point[3] = xyzGeo[0];
                picked_point[4] = xyzGeo[1];
                picked_point[5] = xyzGeo[2];

                {
                    std::lock_guard lk(m);
                    picked  = true;
                    ::pick_point = false;
                    condv.notify_one();
                }
            }

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
            glUniformMatrix4fv(texture_shader["mm"], 1, false, &vcg::Matrix44f::Identity()[0][0]);
            glUniformMatrix4fv(texture_shader["pm"], 1, false, &vcg::Matrix44f::Identity()[0][0]);
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

void TW_CALL rotateAxis(void*a) {
    int ax = *(int*)a;
    vcg::Matrix44f R;
    R.SetRotateDeg(90, axis[currentLidar][ax].Direction());
    axis[currentLidar][(ax+1)%3].SetDirection(R * axis[currentLidar][(ax + 1) % 3].Direction());
    axis[currentLidar][(ax+2)%3].SetDirection(R * axis[currentLidar][(ax + 2) % 3].Direction());
}

void addPointToGUI(vcg::Point3f p,int i) {
    std::string pn = std::to_string(p.X()) + "," + std::to_string(p.Y()) + "," + std::to_string(p.Z());
    std::string n = pn;

    TwAddVarRW(pointsBar, n.c_str(), TW_TYPE_BOOL8, &usePoint[i], "");
}
void TW_CALL addPoint(void*) {
    vcg::Point3f newPoint = frames[currentLidar][idFrame] * vcg::Point3f(xCoord, yCoord, zCoord); 
    addPointToGUI(newPoint, points.size());
    points.push_back(newPoint);
}
void TW_CALL setMarker(void*) {
    for (unsigned int i = 0; i < points.size(); ++i)
        if (usePoint[i])
            marker3D = points[i];
}

std::vector<TwEnumVal> frame_item_dd;
vcg::Matrix44f  axis2frame() {   
    vcg::Matrix44f F;
    F.SetIdentity();
    F.SetColumn(0, axis[currentLidar][0].Direction());
    F.SetColumn(1, axis[currentLidar][1].Direction());
    F.SetColumn(2, axis[currentLidar][2].Direction());

    const vcg::Point3f& _o = axis[currentLidar][0].Origin();

    F.SetColumn(3, _o);
    F[3][3] = 1.0;
    return F;
}

 
TwEnumVal listframes[20] = { {0,"0"},{1,"1"},{2,"2"},{3,"3"},{4,"4"},{5,"5"},{6,"6"},{7,"7"},{8,"8"},{9,"9"},
                             {10,"10"},{11,"11"},{12,"12"},{13,"13"},{14,"14"},{15,"15"},{16,"16"},{17,"17"},{18,"18"},{19,"19"}};

void addFrameToGUI() {
    TwType frames_dd = TwDefineEnum("frames", listframes, frames[currentLidar].size());
    TwRemoveVar(frameBar, "Frame");
    TwAddVarRW(frameBar, "Frame", frames_dd, &idFrame, " keyIncr='<' keyDecr='>' label = 'in frame' help='reference frame.' ");
}

void TW_CALL addFrame(void*) {
    frames[currentLidar].push_back(axis2frame());
    addFrameToGUI();
}

void   alignCamera(int ic) {
        if(cameras[ic].p3.size() == cameras[ic].p2i.size() && cameras[ic].p3.size()>3)
            cameras[ic].calibrated = cameras[ic].SolvePnP(cameras[ic].p3);
}

void TW_CALL alignCamera(void*) {
    if (cameras[currentCamera].p3.size() == cameras[currentCamera].p2i.size() && cameras[currentCamera].p3.size() > 3)
        cameras[currentCamera].calibrated = cameras[currentCamera].SolvePnP(cameras[currentCamera].p3);
}

void TW_CALL assignPointsToCamera(void*) {
//    cameras[currentCamera].p3.clear();
    for (int ip = 0; ip < points.size();++ip)
        if (usePoint[ip])
            cameras[currentCamera].p3.push_back(points[ip]);
//    cameras[currentCamera].p2i.resize(cameras[currentCamera].p3.size());
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

void TW_CALL addMarkerToPoints(void*) {
    addPointToGUI(::marker3D, points.size());
    points.push_back(marker3D);
}
void TW_CALL computeFrame(void*);
void TW_CALL detectMarker(void*) {
    
    lidars[currentLidar].lidar.latest_frame_mutex.lock();
   
    md.points.clear();
    for (unsigned int i = 0; i < lidars[currentLidar].lidar.latest_frame.x.size();++i) {
        vcg::Point3f p = transfLidar[currentLidar]*vcg::Point3f(lidars[currentLidar].lidar.latest_frame.x[i], lidars[currentLidar].lidar.latest_frame.y[i], lidars[currentLidar].lidar.latest_frame.z[i]);
        if (vcg::Distance<float>(p, marker3D) < 1.0)
            md.points.push_back(p);
    }

    lidars[currentLidar].lidar.latest_frame_mutex.unlock();
   
    vcg::Point3f res;
    vcg::Plane3f p0, p1, p2;
    if (md.detect_corner(marker3D, p0,p1,p2) ){
        planes[currentLidar][0] = PlaneC(p0);
        planes[currentLidar][1] = PlaneC(p1);
        planes[currentLidar][2] = PlaneC(p2);
        computeFrame((void*) 0);
        tracker.currect_marker3D = marker3D;
    }
}


void TW_CALL computeLine(void*) {
    for (int ip = 0; ip < 2; ++ip)
        if (!planes[currentLidar][ip].valid)
            return;
    vcg::Line3f l; 
    vcg::IntersectionPlanePlane(planes[currentLidar][0], planes[currentLidar][1], l);
    float pr0 = l.Projection(planes[currentLidar][0].o);
    float pr1 = l.Projection(planes[currentLidar][1].o);
    l.SetOrigin(l.Origin() + l.Direction()* (pr0 + pr1) * 0.5);
    lines3d.push_back(l);
}
void TW_CALL  setClosMarker(void*) {
    marker3D = closest_sel;
    tracker.currect_marker3D = closest_sel;
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

    if (axis[currentLidar][1].Direction()[1] < 0.0)
        axis[currentLidar][1].Flip();
    const vcg::Point3f& n0 = axis[currentLidar][0].Direction();
    const vcg::Point3f& n1 = axis[currentLidar][1].Direction();
    const vcg::Point3f& n2 = axis[currentLidar][2].Direction();
    if ((n0 ^ n1) * n2 < 0)
        axis[currentLidar][2].Flip();
}


void TW_CALL saveFrames(void*) {
    std::string  file = std::string("frames.bin");
#ifdef SCENE_REPLAY
     file = DUMP_FOLDER_PATH + "\\PointClouds\\" + file;
#endif
    FILE* fo = fopen( file.c_str(), "wb");
    for (int il = 0; il < 2; il++)
        for (int fi = 0; fi < frames[il].size(); fi++) {
            printf("write frame \n");
            fwrite(&(frames[il][fi][0][0]),  sizeof(vcg::Matrix44f),1, fo);
        }
    fclose(fo);
}
void TW_CALL loadFrames(void*) {
    std::string  file = std::string("frames.bin");
#ifdef SCENE_REPLAY
    file = DUMP_FOLDER_PATH + "\\PointClouds\\" + file;
#endif
    frames[0].clear();
    FILE* fo = fopen(file.c_str(), "rb");
    fseek(fo, 0, SEEK_END);
    int l = ftell(fo);
    fseek(fo, 0, SEEK_SET);
    for (int i = 0; i < l / sizeof(vcg::Matrix44f); ++i)
    {
        size_t nr = 0;
        frames[0].push_back(vcg::Matrix44f());
        nr = fread(& (frames[0].back()[0][0]),  sizeof(vcg::Matrix44f),1, fo);
        addFrameToGUI();
    }
    fclose(fo);
}

void TW_CALL savePoints(void*) {
    std::string  file = std::string("points.bin");
#ifdef SCENE_REPLAY
    file = DUMP_FOLDER_PATH + "\\PointClouds\\" + file;
#endif
    FILE* fo = fopen(file.c_str(), "wb");
    fwrite(&(points[0][0]), sizeof(vcg::Point3f), points.size(), fo);
    fclose(fo);
}
void TW_CALL loadPoints(void*) {
    std::string  file = std::string("points.bin");
#ifdef SCENE_REPLAY
    file = DUMP_FOLDER_PATH + "\\PointClouds\\" + file;
#endif
    FILE* fo = fopen(file.c_str(), "rb");
    fseek(fo, 0, SEEK_END);
    int l = ftell(fo);
    fseek(fo, 0, SEEK_SET);
    points.resize(l / sizeof(vcg::Point3f));
    fread(&(points[0][0]), sizeof(vcg::Point3f), points.size(), fo);
    fclose(fo);
    for (int i = 0; i < points.size(); ++i)
        addPointToGUI(points[i],i);
}


void TW_CALL saveAxis(void*) {
    if(MessageBoxW(NULL, (LPCWSTR)L"Are you sure? This will ovrewrite the lidaf calibration", (LPCWSTR)L"message", MB_ICONEXCLAMATION | MB_YESNO) != IDYES)
        return;
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
void TW_CALL saveAlignment(void*) {
    if (MessageBoxW(NULL, (LPCWSTR)L"Are you sure? This will ovrewrite the lidaf calibration", (LPCWSTR)L"message", MB_ICONEXCLAMATION | MB_YESNO) != IDYES)
        return;
    std::string calibration_file = std::string("Aln.bin");
#ifdef SCENE_REPLAY
    calibration_file = DUMP_FOLDER_PATH + "\\PointClouds\\" + calibration_file;
#endif
    FILE* fo = fopen(calibration_file.c_str(), "wb");
    for (int il = 0; il < 2; il++)
            fwrite(&transfLidar[il], 1, sizeof(vcg::Matrix44f), fo);
    fclose(fo);
}

void TW_CALL saveImPoints(void*) {
    if (cameras[currentCamera].p3.size() != cameras[currentCamera].p2i.size())
    {
        int ret = MessageBoxW(NULL, (LPCWSTR)L"nope", (LPCWSTR)L"message", MB_ICONEXCLAMATION | MB_YESNO);

        return;

    }
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


void TW_CALL setFrame(void*) {
    for (int i = 0; i < N_LIDARS; ++i)
        transfLidar[i] = vcg::Inverse(boatFrame) * transfLidar[i];
    alphaFrame = betaFrame = gammaFrame = xFrame = yFrame = zFrame = 0.0;
    boatFrame.SetIdentity();
}

void   loadImPoints(int iCam) {
    std::string correspondences_file = std::to_string(cameras[iCam].camID) + "_correspondences.txt";
#ifdef SCENE_REPLAY
    correspondences_file = DUMP_FOLDER_PATH + "\\Images\\" + std::to_string(cameras[iCam].camID) + "\\" + correspondences_file;
#endif

    FILE* fo = fopen(correspondences_file.c_str(), "r");
    if (fo) {
        cameras[iCam].p3.clear();
        cameras[iCam].p2i.clear();

        char trash[1000];
        fgets(trash, 1000, fo);
 
        int i = 0;
        float x, y, z, u, v;

        while (!feof(fo)) {
            int r = fscanf(fo, "%f %f %f %f %f", &x, &y, &z, &u, &v);
            if (r == 5) {
                cameras[iCam].p3.push_back(vcg::Point3f(x, y, z));
                cameras[iCam].p2i.push_back(cv::Point2f(u, v));
                ++i;
            }
        }
        fclose(fo);
    }
}

void TW_CALL loadImPoints(void*) {
    loadImPoints(currentCamera);
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

void TW_CALL loadAlignment(void*) {
    std::string calibration_file = std::string("aln.bin");
#ifdef SCENE_REPLAY
    calibration_file = DUMP_FOLDER_PATH + "\\PointClouds\\" + calibration_file;
#endif
    FILE* fo = fopen(calibration_file.c_str(), "rb");
    for (int il = 0; il < 2; il++)
            fread(&transfLidar[il], 1, sizeof(vcg::Matrix44f), fo);
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
    while (!serverComm.stop_signal)
        serverComm.accepting_connections();
}
void acceptStreamThread() {
    while (!serverStream.stop_signal)
        // serv.accepting_connections_stream();
        serverStream.accepting_connections();
}
void start_Communication_thread() {
    int portComm = 81;
    serverComm.start_server(portComm);
    std::string msg;
    while (!serverComm.stop_signal) {
        if (!serverComm.incoming_message(msg))
        {
            std::cout << msg << std::endl;
            call_API_function(msg);
        }
    }
    serverComm.close();
}

void start_Streaming_thread() {
    int portStream = 82;
    serverStream.start_server(portStream);
    std::string  msg;
    //serv.wait_for_start(msg);
#ifdef MJPEG_WRITE
    streamer.start(8084);
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 90 };
#endif // MJPEG_WRITE

    while (!serverStream.stop_signal/* && streamer.isRunning()*/)  
        if (streamON)
        {
            buff_mutex.lock();
            int fbSize = format_nchannels * sizeof(GLubyte) * cameraFBO.w * cameraFBO.h;
            cv::Mat frame = cv::Mat(cameraFBO.h, cameraFBO.w, CV_8UC3);
            cv::Mat dstFrame;
            frame.data = pixelData;
            cv::flip(frame, dstFrame, 0);  
        
         //   cv::imwrite("streamed.jpg", dstFrame);
        
            std::vector<uchar>buf;
            cv::imencode(".jpg", dstFrame, buf);
            size_t szbuf = buf.size();
            //serverStream.send(reinterpret_cast<char*>(buf.data()));
            serverStream.send(reinterpret_cast<char*>(buf.data()), buf.size());
    #ifdef MJPEG_WRITE

            streamer.publish("/bgr", std::string(buf.begin(), buf.end()));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    #endif // MJPEG_WRITE

    #if  VIDEO_STREAM
            outStream.createStream(fmProcessor, *cdcCntx, dstFrame);
    #endif //  VIDEO_STREAM

  
            buff_mutex.unlock();
            cv::waitKey(10);
            /* }*/
        }
    serverStream.close();
#ifdef MJPEG_WRITE
    streamer.stop();
#endif // MJPEG_WRITE

}
void TW_CALL start_server(void*) {
    tComm = std::thread(&start_Communication_thread);
    tStream = std::thread(&start_Streaming_thread);
    tacc = std::thread(&acceptCommunicationThread);
    taccSt = std::thread(&acceptStreamThread);
}

void TW_CALL initLidars(void*) {

    transfLidar[0].SetIdentity();
    transfLidar[1].SetIdentity();
    lidars[0].lidar.init(2368, "./Calibration/16db.xml");
    lidars[1].lidar.init(2369, "./Calibration/16db.xml");

    tLidars[0] = std::thread(&start_lidar,0);
    tLidars[1] = std::thread(&start_lidar,1);
    cv::waitKey(1000);
    //start_server();
}

void TW_CALL initCameras(void*) {
//#if SAVE_IMG
//     timeCamera1 = std::chrono::system_clock::now();
//#endif
    cameras.resize(NUMCAM);
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
    TwSetParam(bar, "start_stop", "label", TW_PARAM_CSTRING, 1, (time_running) ? "stop" : "play");
}
#endif

void TW_CALL runTest(void*) {
    ::initLidars(0);
#ifdef SCENE_REPLAY
    time_startstop(0);
#endif
    //::loadAxis(0);
    //::computeTranformation(0);
    loadAlignment(0);

    ::initCameras(0);
    
    for (int i = 0; i < NUMCAM;++i) {
        ::loadImPoints(i);
        ::alignCamera(i);
    }
     
    currentCamera = 0;
    ::enable_proj = true;
}
// to be used 
void TW_CALL startVideoStreaming(void*) {
    int vs = 1;
}



void Terminate() {
    return;
    if (lidars[0].lidar.reading)  lidars[0].lidar.stop_reading();
    if (lidars[1].lidar.reading)  lidars[1].lidar.stop_reading();

    for(int i = 0; i < NUMCAM; ++i)
        if (cameras[i].reading) cameras[i].stop_reading();

    for(int i = 0; i < 2;++i)
       if (tLidars[i].joinable())tLidars[i].join();
   
    //  for (int i = 0; i < CameraCount;++i)
    //          if (tCameras[i].joinable()) 
    //              tCameras[i].join();
    serverComm.stop_server();
    serverStream.stop_server();

    if (tComm.joinable())tComm.join();
    if (tStream.joinable())tStream.join();

    free(pixelData);
    State::save_state();
}

void TW_CALL stop(void*) {
    if (lidars[0].lidar.reading) { lidars[0].lidar.stop_reading(); }
    if (lidars[1].lidar.reading) { lidars[1].lidar.stop_reading(); }
    for(int i = 0; i < CameraCount; ++i)  
       if (cameras[i].reading) { cameras[i].stop_reading(); }
}
void TW_CALL setMapColor(const void* v,void *) {
    if(!cameras.empty())
        cameras[currentCamera].used = *(bool*)v;
}
void TW_CALL getMapColor(  void*v, void*) {
    if (!cameras.empty())
        *(bool*)v  = cameras[currentCamera].used ;
}

#ifdef SCENE_REPLAY
void TW_CALL setVirtualTime(const void* v, void*) {
    virtual_time = partial_time = *(int*)v;
}
void TW_CALL getVirtualTime(void* v, void*) {
    *(int*)v = virtual_time;
}
#endif

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
    assert(_CrtCheckMemory());
    
    std::string inFile = "config.txt";
    vecPair configData = logger::readConfigFile(inFile);
    NUMCAM = stoi(configData[0].second);
    DUMP_FOLDER_PATH = configData[1].second; // "D:/CamImages/CamData";  //C:\\Users\\Fabio Ganovelli\\Documents\\GitHub\\nausicaa_devl\\data
    camIniFile = configData[2].second; 
    meiConverterFile = configData[3].second;
    autoLaunch = stoi(configData[4].second);
    State::set_filename("state.txt");
    State::load_state();
    /*PacketDecoder::HDLFrame lidarFrame2;
  
    logger::LoadPointCloudBinary("D:\\Personal\\PointClouds\\2369\\1654782937525.bin", lidarFrame2);
    logger::savePointCloudASCII("D:/CamImages", lidarFrame2);*/
#if VIDEO_STREAM
    ////streaming class instantiation///////////////
    fmtContext = outStream.getFormatContext();
    int wd = 1280, he = 720;
    vCodec.setCodecParameter(wd, he, fps, bitrate, fmtContext->oformat->flags);
    vCodec.InitializeCodecStream(*outStream.getStream());

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
    glutCloseFunc(Terminate);

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

    int axes[3] = { 0,1,2 };
    TwAddVarRW(bar, "showplanes", TW_TYPE_BOOL8, &showPlanes, " label='Show Planes' group=`Register Lidars` help=` select` ");
    TwAddButton(bar, "compute frame", ::computeFrame, 0, " label='Compute Frame' group=`Register Lidars` help=`compute frame` ");
    TwAddButton(bar, "compute line", ::computeLine, 0, " label='Compute Line' group=`Register Lidars` help=`compute line` ");
    TwAddButton(bar, "closest as marker", ::setClosMarker, 0, " label='set closest as marker' group=`Register Lidars` help=`compute line` ");
    TwAddButton(bar, "detect marker", ::detectMarker, 0, " label='Detect Marker' group=`Register Lidars` help=`compute line` ");
    TwAddButton(bar, "add marker to points", ::addMarkerToPoints, 0, " label='Add Marker to P' group=`Register Lidars` help=`compute line` ");
    TwAddVarRW(bar, "tracking 3D", TW_TYPE_BOOL8, &tracker.active3D, " label='tracking3D ' group=`Register Lidars` help=` track3D` ");

    TwAddButton(bar, "rotateX", ::rotateAxis, (void*)&axes[0], " label='rotate frame X' group=`Register Lidars` help=`rotate frame X` ");
    TwAddButton(bar, "rotateY", ::rotateAxis, (void*)&axes[1], " label='rotate frame Y' group=`Register Lidars` help=`rotate frame Y` ");
    TwAddButton(bar, "rotateZ", ::rotateAxis, (void*)&axes[2], " label='rotate frame Z' group=`Register Lidars` help=`rotate frame Z` ");

    TwAddButton(bar, "add", ::addFrame, 0, " label='add  frame' group=`Register Lidars` help=`add current frame` ");
    TwAddVarRW(bar, "Current Lidar", TW_TYPE_UINT32, &currentLidar, " label='currrent LIdar' min=0 max=1 group=`Register Lidars` help=` current lidar` ");
    TwAddVarRW(bar, "Current Plane", TW_TYPE_UINT32, &currentPlane, " label='currrent Plane' min=0 max=2 group=`Register Lidars` help=` current plane` ");
    TwAddVarRW(bar, "Current Axis", TW_TYPE_UINT32, &ax, " label='currrent Axis on the image' min=0 max=2 group=`Register Lidars` help=` current axis` ");
    TwAddButton(bar, "align lidar", ::computeTranformation, 0, " label='Align LIdars' group=`Register Lidars` help=`Align` ");
    TwAddButton(bar, "save", ::saveAxis, 0, " label='saveAxis' group=`Register Lidars` help=`rotate frame` ");
    TwAddButton(bar, "load", ::loadAxis, 0, " label='loadAxis' group=`Register Lidars` help=`rotate frame` ");
    TwAddButton(bar, "saveAln", ::saveAlignment, 0, " label='saveAlignemnt' group=`Register Lidars` help=` ` ");
    TwAddButton(bar, "loadAln", ::loadAlignment, 0, " label='loadAlignment' group=`Register Lidars` help=` ` ");

    TwAddVarRW(bar, "Current Camera", TW_TYPE_UINT32, &currentCamera, std::string(" label='currrent Camera' min=0 max=2 group=`Align Cameras` help = ` current camera` min=0 max =" +std::to_string(NUMCAM)).c_str());
    TwAddVarCB(bar, "Map Color", TW_TYPE_BOOL8, setMapColor, getMapColor, (void*)0, " label='map color' group=`Align Cameras` help=`map color` ");
    TwAddButton(bar, "assign points", ::assignPointsToCamera, 0, " label='assign 3D points' group=`Align Cameras` help=`copy points` ");
    TwAddButton(bar, "align camera", ::alignCamera, 0, " label='Align Camera' group=`Align Cameras` help=`Align` ");
    TwAddButton(bar, "saveIP", ::saveImPoints, 0, " label='saveImPoints' group=`Align Cameras` help=` ` ");
    TwAddButton(bar, "loadIP", ::loadImPoints, 0, " label='loadImPoints' group=`Align Cameras` help=` ` ");



    TwAddButton(bar, "setFrame", ::setFrame, 0, " label='setFrame' group=`Change Reference Frame` help=` ` ");
    TwAddSeparator(bar, NULL,"group=`Change Reference Frame` ");
    TwAddVarRW(bar, "alpha", TW_TYPE_FLOAT, &alphaFrame, " value = 0 label='alpha' group='Change Reference Frame' help=` alpha angle` ");
    TwAddVarRW(bar, "beta", TW_TYPE_FLOAT, &betaFrame, " value = 0  label='beta' group='Change Reference Frame' help=` beta angle` ");
    TwAddVarRW(bar, "gamma", TW_TYPE_FLOAT, &gammaFrame, " value = 0  label='gamma' group='Change Reference Frame' help=` gamma angle` ");
    TwAddSeparator(bar, NULL, "group=`Change Reference Frame` ");
    TwAddVarRW(bar, "x", TW_TYPE_FLOAT, &xFrame, "value = 0  step = 0.01 label='x' group='Change Reference Frame' help=` select` ");
    TwAddVarRW(bar, "y", TW_TYPE_FLOAT, &yFrame, " value = 0 step = 0.01  label='y' group='Change Reference Frame' help=` select` ");
    TwAddVarRW(bar, "z", TW_TYPE_FLOAT, &zFrame, "value = 0  step = 0.01  label='z' group='Change Reference Frame' help=` select` ");

    TwAddVarRW(bar, "showcameras", TW_TYPE_BOOL8, &showCameras, " label='showcameras' group=Rendering help=` select` ");
    TwAddVarRW(bar, "showfromcamera", TW_TYPE_BOOL8, &showfromcamera, " label='showfromcamera' group=`Rendering` help=` draw all` ");
    TwAddVarRW(bar, "mapcolor", TW_TYPE_BOOL8, &enable_proj, " label='map color' group=`Rendering` help=` draw all` ");
    TwAddVarRW(bar, "drawall", TW_TYPE_BOOL8, &drawAllLidars, " label='draw All' group=`Rendering` help=` draw all` ");
    TwAddVarRW(bar, "drawbackground", TW_TYPE_BOOL8, &drawBackground, " label='draw background' group=`Rendering` help=` draw all` ");
    

    TwAddButton(bar, "test", ::runTest, 0, " label='RUNTEST' group='Test' help=`test` ");
    TwAddButton(bar, "Stream", ::startVideoStreaming, 0, "label='VIDEOSTREAM' group='Streaming' help=`streaming` ");
    TwAddButton(bar, "Server", ::start_server, 0, "label='Start Server' group='Streaming' help=`streaming` ");

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
//    TwAddVarRW(bar, "virtualtime", TW_TYPE_UINT32, &virtual_time, " keyIncr='<' keyDecr='>' group=`Streaming` help='Change draw mode.' ");
    TwAddVarCB(bar, "virtualtime", TW_TYPE_UINT32, setVirtualTime, getVirtualTime, (void*)0, " label='virtual time' group=`Streaming` help=`virtual` ");
#endif
    
    frameBar = TwNewBar("Frames");
    TwAddVarRW(frameBar, "xCoord", TW_TYPE_FLOAT, &xCoord, " value = 0.0 label='x'   help=` x coord` ");
    TwAddVarRW(frameBar, "yCoord", TW_TYPE_FLOAT, &yCoord, " value = 0.0 label='y'   help=` y coord` ");
    TwAddVarRW(frameBar, "zCoord", TW_TYPE_FLOAT, &zCoord, " value = 0.0 label='z'   help=` z coord` ");
    TwAddButton(frameBar, "setMarker", ::setMarker, 0, " label='set as marker' help=`set this point as the 3D marker` ");
    TwAddButton(frameBar, "addPoint", ::addPoint, 0, " label='add this point' help=`add this point to the list of known points` ");
    TwAddButton(frameBar, "saveFrames", ::saveFrames, 0, " label='saveFrames'  help=`save frames` ");
    TwAddButton(frameBar, "loadFrames", ::loadFrames, 0, " label='loadFrames'  help=`load frames` ");

    vcg::Matrix44f I;I.SetIdentity();
    frames[currentLidar].push_back(I);
    addFrameToGUI();


    pointsBar = TwNewBar("Points");
    TwAddButton(pointsBar, "savePoints", ::savePoints, 0, " label='savePoints'  help=`save points` ");
    TwAddButton(pointsBar, "loadPoints", ::loadPoints, 0, " label='loadPoints'  help=`load points` ");


    std::cout << "OpenGL version supported by this platform (%s): " << glGetString(GL_VERSION) << std::endl;

    int maxi;
    glGetIntegerv(GL_MAX_ELEMENTS_INDICES, &maxi);

    glewInit();
    assert(_CrtCheckMemory());

    if (autoLaunch)
    {
        runTest(0);
        start_server(0);
    }

    glutMainLoop();
}
