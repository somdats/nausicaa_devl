#include "state.h"
#include "..\..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\Common.h"
#include "..\..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\Deserialize.h"
#include "..\..\..\NAUSICAA_VR_API\NAUSICAA_API\header\Serialize.h"

#include <stdio.h>

void State::set_filename(std::string fn) { filename() = fn; }

void State::save_state() {
	FILE* fs = fopen(filename().c_str(), "w");
	if (fs) {
		message msg;
		msg[std::string("N_virtual_cameras")][(int)virtualCameras.size()];
		fprintf(fs, "%s\n", msg.msg.c_str());
		for (int i = 0; i < virtualCameras.size();++i) {
			msg.msg.clear();
			float sx, dx, tp, bt, n;
			virtualCameras[i].Intrinsics.GetFrustum(sx, dx, bt, tp, n);
			msg["intrinsics"][sx][dx][bt][tp][n];
			fprintf(fs, "%s\n", msg.msg.c_str());

			msg.msg.clear();
			vcg::Point2i vp=virtualCameras[i].Intrinsics.ViewportPx;
			msg["viewport"][vp[0]][vp[1]];
			fprintf(fs, "%s\n", msg.msg.c_str());

			msg.msg.clear();
			vcg::Point3f p = virtualCameras[i].GetViewPoint();
			msg["viewpoint"][p[0]][p[1]][p[2]];
			fprintf(fs, "%s\n", msg.msg.c_str());

			msg.msg.clear();
			vcg::Matrix44f rot = virtualCameras[i].Extrinsics.Rot();
			msg["rot"]	[rot[0][0]][rot[0][1]][rot[0][2]][rot[0][3]]
						[rot[1][0]][rot[1][1]][rot[1][2]][rot[1][3]]
						[rot[2][0]][rot[2][1]][rot[2][2]][rot[2][3]]
						[rot[3][0]][rot[3][1]][rot[3][2]][rot[3][3]]
				;
			fprintf(fs, "%s\n", msg.msg.c_str());
		}
		fclose(fs);
	}

};

 
void State::load_state() {
	FILE* fs = fopen(filename().c_str(), "r");
	if (fs) {
		while (!feof(fs)) {
			char line[1000];line[0] = '\0';
			
			fgets(line, 1000, fs); 
			std::string message = std::string(line);
			std::string field = func_name(message);
			int n_virtual_cameras = 0;
			float sx, dx, tp, bt, n, x, y, z;
			vcg::Point2i vp;

			vcg::Matrix44f m;
			if (field == std::string("N_virtual_cameras")) {
				n_virtual_cameras = deserialize_int(message);
				for (int i = 0; i < n_virtual_cameras;++i) {
					fgets(line, 1000, fs);
					message = std::string(line);
					field = func_name(message);// intrinsics
					sx = deserialize_float(message);
					dx = deserialize_float(message);
					bt = deserialize_float(message);
					tp = deserialize_float(message);
					n = deserialize_float(message);

					fgets(line, 1000, fs); 
					message = std::string(line);
					field = func_name(message);// viewport
					vp[0] = deserialize_int(message);
					vp[1] = deserialize_int(message);


					fgets(line, 1000, fs);
					message = std::string(line);
					field = func_name(message);// viewpoint
					x = deserialize_float(message);
					y = deserialize_float(message);
					z = deserialize_float(message);

					fgets(line, 1000, fs); 
					message = std::string(line);
					field = func_name(message);// rot
					for (int ii = 0; ii < 4; ++ii)
						for (int jj = 0; jj < 4; ++jj)
							m[ii][jj] = deserialize_float(message);

					virtualCameras[i].Intrinsics.SetFrustum(sx, dx, bt, tp, n, vp);
					virtualCameras[i].SetViewPoint(vcg::Point3f(x, y, z));
					virtualCameras[i].Extrinsics.SetRot(m);
				}

			}
		}
		fclose(fs);
	}
};