#pragma once
#include <GL/glew.h>
#include <vector>
#include<algorithm>
#include<array>
#include<iostream>

#include "wrap\gl\shot.h"

   


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

	//! enum direction
	 enum   changeDir
	{
		xdir = 0,
		ydir,
		zdir

	};
	 enum  angleDir {
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
		void  setCallback(void (*ft)(const char*));


		//! read the configuration file
		/*!
			 \param path to the configuration file
		*/
		void  readConfigFile(const char*);

		//! connect to the VR server
		/*!
		*/
		void  connectToVRServer(const char* ip_addr);

		//! disconnect to the VR server
		/*!
		*/
		void  disconnectToVRServer();

		//! start streaming
		void  startStreaming();

		//! stop streaming
		void  stopStreaming();

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
		VirtualCameraID  addVirtualCamera();


		//! get a camera from the list with a unique camera-id
		/*!
			 \param virtual camera unique identifier
		*/
		void  getVirtualCamera(VirtualCameraID id);

		//! get list of virtual cameras
		/*!

		*/
		VirtualCameraID* getVirtualCameraList( int* n_cameras);

		//! set a virtual camera as the one to use for rendering
		/*!
			 \param virtual camera unique identifier
		*/

		void  renderFromCamera(VirtualCameraID id);

		//! set a virtual camera as the one to use for rendering and add it to the set of virutal cameras
		/*!
			 \param virtual camera
		*/
		VirtualCameraID  renderCamera();

		//! Selezione la qualità del rendering
		/*!  Lowest qualiy corresponds to an output-sensitive rendering
		 *   which guarantees a constant frame rate of 25-30 frames al sencondo.
		 *   With higher quality you can have more accurate reconstructions
		 *   with a less fluid rendering.
			 \param valore tra 0.0 e 1.0
		*/
		void  selectQuality(float val);

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
		void  sampleGeometry(float xi, float yi, float* longitude, float* latitude, float * height);

		//! include lidarID in the reconstruction process
		/*!
			 \param unique identifier of the lidar to be included
		*/
		void  enableLidar(lidarID id);

		//! exclude lidarID from the rendering process
		/*!
			 \param unique identifier of the lidar to be excluded
		*/
		void  disableLidar(lidarID id);

		//! include cameraID
		/*!
			 \param unique identifier of the camera to used in the rendering
		*/
		void  enableCamera(cameraID id);

		//! exclude cameraID
		/*!
			 \param unique identifier of the camera to be excluded from the rendering
		*/
		void  disableCamera(cameraID id);
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
		gliphID  place3DGliph(gliphTypeID, float x, float y, float d);

		//! remove a previously inserted 3D gliph
		/*!
			 \param unique identifier of the 3D gliph instance
		*/
		void  remove3DGliph(gliphID);

		//! return the type of an instance of a gliph
		/*!
			 \param unique identifier of the 3D gliph instance
		*/
		gliphTypeID  typeOf(gliphID);

		//! return all the gliphs of a given type
		/*!
			 \param gliphs type
		*/
		//std::vector<gliphID> getInstancesOfGliphType(gliphTypeID);

		//! include the layer of gliphs in the rendering
		/*!
		*/
		void  enableGliphs();

		//! disable the layer od gliphs in the rendering
		/*!
		*/
		void  disableGliphs();

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
		void  setPerspective(VirtualCameraID cId, float AngleDeg, float AspectRatio, float Focal, int viewportX, int viewportY);

		/// update (change) viewport
		void  setViewport(VirtualCameraID cId, int new_viewportX, int new_viewportY);

		/// update (change) focal length
		void  setFocalLength(VirtualCameraID cId, float newFocalLength);

		/// update (change) angle
		void  setAngle(VirtualCameraID cId, float newAngle);

		/// update (change) aspect ratio
		void  setAspectratio(VirtualCameraID cId, float newAspectRatio);

		/// specify the extrinsics parameters of the camera
		void  lookTowards(VirtualCameraID cId, float eyeX, float eyeY, float eyeZ, float dirX, float dirY, float dirZ, float upX, float upY, float upZ);

		/// update (change) camera position
		void  setPosition(VirtualCameraID cId, float eyeX, float eyeY, float eyeZ);

		/// update (change) view direction
		void  setViewDirection(VirtualCameraID cId, float xDir, float yDir, float zDir);

		/// move camera along the specified axis (0:x, 1:y, 2:z) by the specified amount (in meters)
		void  move(VirtualCameraID cId, changeDir xyz, float amount);

		/// rotate camera along the specified angles (0:yaw, 1:pitch, 2:rool ) by the specified amount (in degrees)
		void  rotate(VirtualCameraID cId, angleDir yaw_pitch_roll, float amount);

		/// reset camera to  original position
		//void resetCameraPosition(bool reset);

		/// set the frustrum of a virtual camera
		void  setCameraFrustrum(VirtualCameraID cId, int sx, int dx, int bt, int tp, float focalLength, int ViewPortX, int viewPortY);;

		/// get the id of a virtual camera
		//VirtualCameraID virtualID()const;


		/// get the camera frustum
		void  getFrustum(VirtualCameraID cId, int& sx, int& dx, int& bt, int& tp, float& nr);

	}

	void call_API_function(std::string message);
