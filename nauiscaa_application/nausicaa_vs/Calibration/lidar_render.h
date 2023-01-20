#pragma once

#include "defines.h"
#include <GL/glew.h>
#include <vcg/space/box3.h>
#include "velodyne_reader.h"
#include "common.h"

#define NL 16

struct LidarRender {

    Lidar  lidar;

    vcg::Box3f bbox;

    uint64_t epochtime;

    int cs;
    std::vector< std::vector < float > >  samples_mem;
    std::vector< std::vector < float > > distances_mem;

    std::vector < float > samples;
    std::vector < float > distances;
    GLuint buffers[3];
    GLuint sub_buffer;

    vcg::Matrix44f  transfLidar;

    std::vector<GLuint> iTriangles;

    int subset_size;
    int n_strips;
    int n_verts;
    float deltaA;

    void fillSubset(std::vector<vcg::Point3f> & points) {
        glBindBuffer(GL_ARRAY_BUFFER, sub_buffer);
        glBufferData(GL_ARRAY_BUFFER, points.size() * 3 * sizeof(float), &(*points.begin()), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        subset_size = points.size();
    }


    void fillGrid() {
        memset(&samples[0], 0, sizeof(float) * 3 * n_verts);
        memset(&distances[0], 0, sizeof(float) * n_verts);

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

                int offset = (idStrip * NL + j);
                samples_mem[cs][offset * 3    ] = samples[offset * 3    ] = lidar.latest_frame.x[i];
                samples_mem[cs][offset * 3 + 1] = samples[offset * 3 + 1] = lidar.latest_frame.y[i];
                samples_mem[cs][offset * 3 + 2] = samples[offset * 3 + 2] = lidar.latest_frame.z[i];
                distances_mem[cs][offset] = distances[offset] = lidar.latest_frame.distance[i];
   
                assert(idStrip < n_strips);
                assert((idStrip * NL + j) * 3 + 2 < samples.size());
            }
        //if(0)
        for(int  i = 0; i < distances.size(); ++i)
            if (distances[i] < 0.1) {
                // no value, check in the recent history
                for (int ip = 0; ip < distances_mem.size() - 1; ++ip) {
                    int iM = (cs + distances_mem.size()) % distances_mem.size();
                    if (distances_mem[iM][i] > 0.f) {
                        memcpy(&samples[i * 3], &samples_mem[iM][i * 3], sizeof(float) * 3);
                        distances[i] = distances_mem[iM][i];
                    }
                }
            }

        glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
        glBufferData(GL_ARRAY_BUFFER, n_verts * 3 * sizeof(float), &(*samples.begin()), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
        glBufferData(GL_ARRAY_BUFFER, n_verts * sizeof(float), &(*distances.begin()), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        cs = (cs+1)%distances_mem.size();
    }

    void init(int memsize = 5) {


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
        cs = 0;
        samples_mem.resize(memsize);
        for (int i = 0; i < samples_mem.size(); ++i)
            samples_mem[i].resize(n_verts * 3);
        distances_mem.resize(memsize);
        for (int i = 0; i < distances_mem.size(); ++i)
            distances_mem[i].resize(n_verts * 3,0.f);

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

        // init for the selection
        glCreateBuffers(1, &sub_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, sub_buffer);
        glBufferData(GL_ARRAY_BUFFER, n_verts * 3 * sizeof(float), &(*samples.begin()), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

};

struct BoxRender {
    GLuint buffers[3];

    void init(float size) {
        GLfloat corners[24] = {
          -size, -size, size,
            size, -size, size,
            -size, size, size,
            size, size, size,
            -size, -size, -size,
            size, -size, -size,
            -size, size, -size,
            size, size, -size
        };

        GLfloat dis[8] = { 10,10,10,10,10,10,10,10 };

        GLuint tris[36] = {
                0, 1, 2, 2, 1, 3,  // front
                5, 4, 7, 7, 4, 6,  // back
                4, 0, 6, 6, 0, 2,  // left
                1, 5, 3, 3, 5, 7,  // right
                2, 3, 6, 6, 3, 7,  // top
                4, 5, 0, 0, 5, 1   // bottom
        };

       
        glCreateBuffers(3, buffers);

        glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
        glBufferData(GL_ARRAY_BUFFER, 8 * 3 * sizeof(float), corners, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
        glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float), dis, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[1]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 366 * sizeof(int), tris, GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    }

};