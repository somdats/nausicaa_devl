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
    std::vector < float > samples;
    std::vector < float > distances;
    GLuint buffers[3];

    std::vector<GLuint> iTriangles;

    int n_strips;
    int n_verts;
    float deltaA;

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
