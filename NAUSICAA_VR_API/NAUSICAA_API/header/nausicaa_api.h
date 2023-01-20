#pragma once
#pragma warning( disable : 4091 )
#pragma warning( disable : 4251 )
#include "config.h"

#ifdef _MSC_VER
#pragma once
#endif  // _MSC_VER
// c++ headers
#include <vector>
#include<algorithm>
#include<array>
#include<iostream>



#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

	constexpr int MAX_POLY_LENGTH = 64;
	constexpr int arr_size = 3;

	//! unique identifier of a camera
	typedef int cameraID;

	//! unique identifier of a lidar sensor
	typedef int lidarID;

	//! unique identifier of a virtual camera
	typedef int VirtualCameraID;

	//! unique identifier of a virtual camera
	typedef int* VirtualCameraIDs;

	//! unique identifier of a type of gliph
	typedef int  markerID;

	//! image buffer
	typedef char* image_buffer;

	//! points buffer
	typedef float* points_buffer;

	//! enum direction
	typedef enum NAUSICAA_VR_API  changeDir
	{
		xdir = 0,
		ydir,
		zdir

	};
	typedef enum NAUSICAA_VR_API angleDir {
		yaw = 0,
		pitch,
		roll
	};

	namespace VRSubsystem
	{
		/** @name Setup
		 * functions to set up the system and get the stream going
		*/
		///@{

		//! read the configuration file (not necessary, discontinued)
		/*!
			 \param func callback function
		*/
		void NAUSICAA_VR_API setCallback(void (*func)(const char*));
		 

		//! read the configuration file (not necessary, discontinued)
		/*!
			 \param path path to the configuration file
		*/
		void NAUSICAA_VR_API readConfigFile(const char* path);

		//! connect to the VR server
		/*!
		*/
		int NAUSICAA_VR_API connectToVRServer(const char* ip_addr);

		//! disconnect to the VR server
		/*!
		*/
		void NAUSICAA_VR_API disconnectToVRServer();

		//! start streaming
		void NAUSICAA_VR_API startStreaming();

		//! stop streaming
		void NAUSICAA_VR_API stopStreaming();
		///@}

		/** @name Readback from the server
		 * functions that read data from the server
		*/
		///@{

		//! read one frame (discontinued, frames are streamed)
		image_buffer  NAUSICAA_VR_API readFrame(int* byteCount);

		//! return a selection of the point clouds
		/*!
		  \param n_points number of points returned
		  \param bottom only points with y coordinates greater than this value
		  \param top only points with y coordinates smaller than this value
		  \param innerRadius only points farther than this values from the origing
		  \param outerRadius only points closer than this values to the origing
		*/
		points_buffer getPointCloud(int* n_points, float bottom, float top, float innerRadius, float outerRadius);
		///@}

		/** @name Set the reference systems
		 *  function to specify the reference systems.
		 *  NausicaaVR uses three reference system: Boat Frame, Steady Frame (shown in the figure below) and World Frame 
		 *  \image html  boat_frames.jpg
		 * The Boat Frame is fixed with the boat, its Y axis orthogonal to the boat floor, its Z axis pointing towards the stern and its X axis pointing towards the right of the boat's starboard.
		 * The origin of the Boat Frame (to be determined in the system calibration phase) is somewhere in the middle of the boat.
		 * The point clouds are originally expressed in the Boat Frame. This means that if the boat rolls clockwise, the point cloud of an object fixed in space will rotate counterclockwise.
		 * The Steady Frame is centered in the same point as the Boat Frame but its orientation is unaffected by picthing and rolling of the boat.
		 * In other words it's axes are just the canonical axes X = (1,0,0,) Y = (0,1,0) and Z = (0,0,1).
		 * Note that if the roll and pitch angles are 0, the Boat Frame and the Steady Frame are identical.
		 * By specifying pitch and roll axes, we tell the API how to transform the input so that is is expressed in the Steady Frame,
		 * that is, how to compensate for the boat rotations.
		 * 
		 * The World Frame origin is expressed in WGS84 coordinate system and it corresponds to the Steady Frame transated to the actual positoin of the boat,
		 * and rotated so the the -Z axis points to the bow of the boat. The -Z axis is defined by specifying the boat orientation in degrees as specified in the function updateBowDirection.
		*/
		///@{

		//! update position WGS84 plus elevation
		/*!
		  \param LatDecimalDegrees
		  \param LongDecimalDegrees
		  \param ElevationMeters
		*/
		void NAUSICAA_VR_API updatePositionWGS84(float LatDecimalDegrees,float LongDecimalDegrees, float ElevationMeters);

		//! update pitch and roll
		/*!
		  \param pitchDegrees: pitch is the rotation angle around the axis crossing the sides of the boat, from left to right. Positive angle means to bow is higher
		  \param rollDegrees: roll is the  rotation angle around the axis crossing the   boat from the bow to the stern. Positive angle means the boat leans on its left   
		*/
		void NAUSICAA_VR_API updatePitchRoll(float pitchDegrees, float rollDegrees);

		//! update compass
		/*!
		  \param angleDegrees: value between 0 and 360 (north) running clockwise (90-est, 180-south..). How the boat is oriented
		*/
		void NAUSICAA_VR_API updateBowDirection(float angleDegrees);
		///@}

		/** @name Rendering
		* Definition and manipulation of virtual cameras
		*/
		///@{
		//! Add a virtual camera
		/*!
			 \return camera unique identifier
		*/
		VirtualCameraID NAUSICAA_VR_API addVirtualCamera();


		//! get list of virtual cameras
		/*!

		*/
		VirtualCameraIDs NAUSICAA_VR_API getVirtualCameraList(int* n_cameras);

		//! set a virtual camera as the one to use for rendering
		/*!
			 \param id virtual camera unique identifier
		*/

		void NAUSICAA_VR_API renderFromCamera(VirtualCameraID id);

		//! set a virtual camera as the one to use for rendering
		/*!
			 \param show or not the distance map
		*/
		void NAUSICAA_VR_API renderDistanceMap(bool yes_no);

		//! Rendering quality [NOT YET IMPLEMENTED]
		/*!  Lowest qualiy corresponds to an output-sensitive rendering
		 *   which guarantees a constant frame rate of 25-30 frames per sencond.
		 *   With higher quality you can have more accurate reconstructions
		 *   with a less fluid rendering.
			 \param val value  between 0.0 e 1.0
		*/
		void NAUSICAA_VR_API selectQuality(float val);

		////! Sample geometry
		///*!
		//	 \param xi pixel coordinates (xi)
		//	 \param yi pixel coordinates (yi)
		//	 \param x 3D point (x)
		//	 \param y 3D point (y)
		//	 \param z 3D point (z)
		//*/
		//void sampleGeometry(float xi, float yi, float& x, float& y, float& z);

		//! Sample geometry
		/*!
			 \param xi pixel coordinates x of the point clicked
			 \param yi pixel coordinates y of the point clicked
			 \param xLoc x position in the Steady Frame
			 \param yLoc y position in the Steady Frame
			 \param zLoc z position in the Steady Frame
			 \param longitude
			 \param latitude
			 \param height above the sea
		*/
		void NAUSICAA_VR_API sampleGeometry(int xi, int yi, float *xLoc,float *yLoc, float *zLoc,float* longitude, float* latitude, float* height);

		//! include lidarID in the reconstruction process
		/*!
			 \param id unique identifier of the lidar to be included
		*/
		void NAUSICAA_VR_API enableLidar(lidarID id);


		//! enable all lidars in the reconstruction process
		/*!
			
		*/
		void NAUSICAA_VR_API enableAllLidars();

		//! exclude lidarID from the rendering process
		/*!
			 \param id unique identifier of the lidar to be excluded
		*/
		void NAUSICAA_VR_API disableLidar(lidarID id);

		//! exclude lidarID from the rendering process
		/*!
			
		*/
		void NAUSICAA_VR_API disableAllLidars();

		//! include cameraID
		/*!
			 \param id unique identifier of the camera to used in the rendering
		*/
		void NAUSICAA_VR_API enableCamera(cameraID id);

		//! enable all cameras
		/*!
		*/
		void NAUSICAA_VR_API enableAllCameras();

		//! exclude cameraID
		/*!
			 \param id unique identifier of the camera to be excluded from the rendering
		*/
		void NAUSICAA_VR_API disableCamera(cameraID id);

		//! exclude all cameras 
		/*!
			  
		*/
		void NAUSICAA_VR_API disableAllCameras();

		//! exclude background from the scene
		/*!

		*/
		void NAUSICAA_VR_API disableBackground();

		//! include background in the scene
		/*!

		*/
		void NAUSICAA_VR_API enableBackground();
		///@}

		/** @name Gliphs
		 * functions to set up the system and get the stream going
		*/
		///@{
		//! define a marker
		/*!
			 \param png_image image in binary format
			 \param size_in_bytes size of the image in bytes
			 \param width_mt with of the icon in meters (heigth is inferred from proportion)
			 \param label label to be shown
			 \param label_length length of the label 
			 \return unique identifier of the added marker
		*/
		markerID NAUSICAA_VR_API addMarker(char * png_image, int size_in_bytes, float width_mt, const char * label, int label_length);

		//! place a previously added marker in a specific position
		/*!
			 \param id marker ID
			 \param longitude longitude in WGS84 decimal degrees
			 \param latitude latitude in WGS84 decimal degrees
			 \param altitude altitude in meters (above the sea level)
		*/
		void NAUSICAA_VR_API placeMarker(markerID id, float longitude, float latitude, float altitude);

		//! enable/disable the visibility of a marker
		/*!
			 \param id marker ID
			 \param invisible_visible 0: invisible; 1: visible (default is 1) 
		*/
		void NAUSICAA_VR_API showMarker(markerID id, int invisible_visible);

		//! remove all previously added markers
		/*!
		*/
		void NAUSICAA_VR_API clearMarkers();
		///@}
	}

	
	//!  Virtual Camera
	/*!
		definition of a virtual camera
	*/
	namespace VirtualCamera
	{

		/// specify the intrinsics parameters of the virtual camera
		void NAUSICAA_VR_API setOrtho(VirtualCameraID cId, float left, float right, float bottom, float up,int viewportX, int viewportY);

		/// specify the intrinsics parameters of the virtual camera
		void NAUSICAA_VR_API setPerspective(VirtualCameraID cId, float AngleDeg, float AspectRatio, float Focal, int viewportX, int viewportY);

		/// update (change) viewport
		void NAUSICAA_VR_API setViewport(VirtualCameraID cId, int new_viewportX, int new_viewportY);

		/// update (change) focal length
		void NAUSICAA_VR_API setFocalLength(VirtualCameraID cId, float newFocalLength);

		/// update (change) angle
		void NAUSICAA_VR_API setAngle(VirtualCameraID cId, float newAngle);

		/// update (change) aspect ratio
		void NAUSICAA_VR_API setAspectratio(VirtualCameraID cId, float newAspectRatio);

		/// specify the extrinsics parameters of the camera
		void NAUSICAA_VR_API lookTowards(VirtualCameraID cId, float eyeX, float eyeY, float eyeZ, float dirX, float dirY, float dirZ, float upX, float upY, float upZ);

		/// update (change) camera position
		void NAUSICAA_VR_API setPosition(VirtualCameraID cId, float eyeX, float eyeY, float eyeZ);

		/// update (change) view direction (up vector is assumed 0,1,0)
		void NAUSICAA_VR_API setViewDirection(VirtualCameraID cId, float dirX, float dirY, float dirZ);

		/// update (change) view direction
		void NAUSICAA_VR_API setViewDirectionUp(VirtualCameraID cId, float dirX, float dirY, float dirZ, float upX, float upY, float upZ);

		/// move camera along the specified axis (0:x, 1:y, 2:z) by the specified amount (in meters)
		void NAUSICAA_VR_API move(VirtualCameraID cId, changeDir xyz, float amount);

		/// rotate camera along the specified angles (0:yaw, 1:pitch, 2:rool ) by the specified amount (in degrees)
		void NAUSICAA_VR_API rotate(VirtualCameraID cId, angleDir yaw_pitch_roll, float amount);

		/// reset camera to  original position
		//void resetCameraPosition(bool reset);

		/// set the frustrum of a virtual camera
		void NAUSICAA_VR_API setCameraFrustrum(VirtualCameraID cId, float sx, float  dx, float bt, float tp, float focalLength, int ViewPortX, int viewPortY);

		/// get the camera frustum
		void NAUSICAA_VR_API getFrustum(VirtualCameraID cId, float* sx, float* dx, float* bt, float* tp, float* nr);

		//! get the camera position and view frame
		/*!
			\param cId  unique identifier of the camera
			\param eyeX unique x position of the eye
			\param eyeY unique y position of the eye
			\param eyeZ unique z position of the eye

			\param  dirX x component of the view direction
			\param  dirY y component of the view direction
			\param  dirZ z component of the view direction

			\param  upX x component of the up direction
			\param  upY y component of the up direction
			\param  upZ z component of the up direction
		*/
		void NAUSICAA_VR_API getPositionAndDirection(VirtualCameraID cId, float *eyeX, float *eyeY, float *eyeZ, float *dirX, float *dirY, float *dirZ, float *upX, float *upY, float *upZ);
	}





#ifdef __cplusplus
}
#endif