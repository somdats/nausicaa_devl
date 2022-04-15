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


	//! unique identifier of a type of gliph
	typedef int  gliphID;

	//! unique identifier of an instance of a gliph
	typedef int  gliphTypeID;

	//! image buffer
	typedef char *   image_buffer;

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

		//! read the configuration file
		/*!
			 \param callback function
		*/
		void NAUSICAA_VR_API setCallback(void (*ft)(const char*));


		//! read the configuration file
		/*!
			 \param path to the configuration file
		*/
		void NAUSICAA_VR_API readConfigFile(const char*);

		//! connect to the VR server
		/*!
		*/
		int NAUSICAA_VR_API connectToVRServer(const char * ip_addr);

		//! disconnect to the VR server
		/*!
		*/
		void NAUSICAA_VR_API disconnectToVRServer();

		//! start streaming
		void NAUSICAA_VR_API startStreaming();

		//! read frame
		image_buffer  NAUSICAA_VR_API readFrame();

		//! stop streaming
		void NAUSICAA_VR_API stopStreaming();

		/** @name Rendering
		* Il sistema offre la possibilità di visualizzare il dato ricostruito  definendo
		* una videocamera virtuale, con parametri intrinseci (FOV, zoom) ed estrinseci (posizione e orientamento)
		* definiti dall’utente. Per maggior praticità di prevedere che l’utente possa definire più camera
		* virtuali per passare da una all’altra (o per affiancarle sullo schermo).
		* In ogni momento, i parametri della camera virtual selezionata potranno essere cambiati
		* tramite un'apposita GUI
		*/
		///@{
		//! Add a virtual camera
		/*!
			 \param camera specification
			 \return camera unique identifier
		*/
		VirtualCameraID NAUSICAA_VR_API addVirtualCamera();


		//! get a camera from the list with a unique camera-id
		/*!
			 \param virtual camera unique identifier
		*/
		void NAUSICAA_VR_API getVirtualCamera(VirtualCameraID id);

		//! get list of virtual cameras
		/*!

		*/
		void NAUSICAA_VR_API getVirtualCameraList();

		//! set a virtual camera as the one to use for rendering
		/*!
			 \param virtual camera unique identifier
		*/

		void NAUSICAA_VR_API renderFromCamera(VirtualCameraID id);

		//! set a virtual camera as the one to use for rendering and add it to the set of virutal cameras
		/*!
			 \param virtual camera
		*/
		VirtualCameraID NAUSICAA_VR_API renderCamera();

		//! Selezione la qualità del rendering
		/*!  Lowest qualiy corresponds to an output-sensitive rendering
		 *   which guarantees a constant frame rate of 25-30 frames al sencondo.
		 *   With higher quality you can have more accurate reconstructions
		 *   with a less fluid rendering.
			 \param valore tra 0.0 e 1.0
		*/
		void NAUSICAA_VR_API selectQuality(float val);

		////! Sample geometry
		///*!
		//	 \param pixel coordinates (xi)
		//	 \param pixel coordinates (yi)
		//	 \param 3D point (x)
		//	 \param 3D point (y)
		//	 \param 3D point (z)
		//*/
		//void sampleGeometry(float xi, float yi, float& x, float& y, float& z);

		//! Sample geometry
		/*!
			 \param pixel coordinates (xi)
			 \param pixel coordinates (yi)
			 \param longitude
			 \param latitude
		*/
		void NAUSICAA_VR_API sampleGeometry(float xi, float yi, float& longitude, float& latitude);

		//! include lidarID in the reconstruction process
		/*!
			 \param unique identifier of the lidar to be included
		*/
		void NAUSICAA_VR_API enableLidar(lidarID id);

		//! exclude lidarID from the rendering process
		/*!
			 \param unique identifier of the lidar to be excluded
		*/
		void NAUSICAA_VR_API disableLidar(lidarID id);

		//! include cameraID
		/*!
			 \param unique identifier of the camera to used in the rendering
		*/
		void NAUSICAA_VR_API enableCamera(cameraID id);

		//! exclude cameraID
		/*!
			 \param unique identifier of the camera to be excluded from the rendering
		*/
		void NAUSICAA_VR_API disableCamera(cameraID id);
		///@}

		/** @name Gliphs
		 * functions to set up the system and get the stream going
		*/
		///@{
		//! place 3D gliph
		/*!
			 \param unique identifier of the 3D gliph
			 \param x position on the screen in [0,x size]
			 \param y position on the screen in [0,y size]
			 \param d distance along the ray passing through (x,y)
		*/
		gliphID NAUSICAA_VR_API place3DGliph(gliphTypeID, float x, float y, float d);

		//! remove a previously inserted 3D gliph
		/*!
			 \param unique identifier of the 3D gliph instance
		*/
		void NAUSICAA_VR_API remove3DGliph(gliphID);

		//! return the type of an instance of a gliph
		/*!
			 \param unique identifier of the 3D gliph instance
		*/
		gliphTypeID NAUSICAA_VR_API typeOf(gliphID);

		//! return all the gliphs of a given type
		/*!
			 \param gliphs type
		*/
		//std::vector<gliphID> getInstancesOfGliphType(gliphTypeID);

		//! include the layer of gliphs in the rendering
		/*!
		*/
		void NAUSICAA_VR_API enableGliphs();

		//! disable the layer od gliphs in the rendering
		/*!
		*/
		void NAUSICAA_VR_API disableGliphs();

		///@}
	}

		///@}
		//!  Virtual Camera
		/*!
			definition of a virtual camera
		*/
	namespace VirtualCamera
	{

		/// specify the intrinsics parameters of the virtual camera
		void NAUSICAA_VR_API setPerspective(VirtualCameraID cId,float AngleDeg, float AspectRatio, float Focal, int viewportX, int viewportY);

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

		/// update (change) view direction
		void NAUSICAA_VR_API setViewDirection(VirtualCameraID cId, float eyeX, float eyeY, float eyeZ);

		/// move camera along the specified axis (0:x, 1:y, 2:z) by the specified amount (in meters)
		void NAUSICAA_VR_API move(VirtualCameraID cId, changeDir xyz, float amount);

		/// rotate camera along the specified angles (0:yaw, 1:pitch, 2:rool ) by the specified amount (in degrees)
		void NAUSICAA_VR_API rotate(VirtualCameraID cId, angleDir yaw_pitch_roll, float amount);

		/// reset camera to  original position
		//void resetCameraPosition(bool reset);

		/// set the frustrum of a virtual camera
		void NAUSICAA_VR_API setCameraFrustrum(VirtualCameraID cId, float sx, float  dx, float bt, float tp, float focalLength, int ViewPortX, int viewPortY);

		/// get the id of a virtual camera
		//VirtualCameraID virtualID()const;

		
		/// get the camera frustum
		void NAUSICAA_VR_API getFrustum(VirtualCameraID cId, int& sx, int& dx, int& bt, int& tp, float& nr);
		
	}





#ifdef __cplusplus
}
#endif