#ifndef PICOZENSE_API_H
#define PICOZENSE_API_H

/**
* @file picozense_api.h
* @brief Pico Zense AP header file.
* Copyright (c) 2018-2019 Pico Interactive, Inc.
*/

/*! \mainpage Pico Zense API Documentation
*
* \section intro_sec Introduction
*
* Welcome to the Pico Zense API documentation. This documentation enables you to quickly get started in your development efforts to programmatically interact with the Pico Zense TOF RGBD Camera (DCAM710).
*/

#include "PicoZense_define.h"

/**
*  @brief 	Initializes the API on the device specified by <code>deviceIndex</code>. This function must be invoked before any other Pico Zense APIs.
*  @return ::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsInitialize();

/**
*  @brief 	Shuts down the API on the device specified by <code>deviceIndex</code> and clears all resources allocated by the API. After invoking this function, no other Pico Zense APIs can be invoked.
*  @return ::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsShutdown();

/**
*  @brief 		Returns the number of camera devices currently connected.
*  @param[out] pDeviceCount	Pointer to a 32-bit integer variable in which to return the device count.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetDeviceCount(int32_t* pDeviceCount);

/**
*  @brief 		Opens the device specified by <code>deviceIndex</code>. The device must be subsequently closed using PsCloseDevice().
*  @param[in] 	deviceIndex	The index of the device to open. Device indices range from 0 to device count - 1.
*  @return: 	PsReturnStatus ::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsOpenDevice(int32_t deviceIndex);

/**
*  @brief 		Closes the device specified by <code>deviceIndex</code> that was opened using PsOpenDevice.
*  @param[in] 	deviceIndex	The index of the device to close. Device indices range from 0 to device count - 1.
*  @return: 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsCloseDevice(int32_t deviceIndex);

/**
*  @brief 		Starts capturing the image stream indicated by <code>frameType</code> on the device specified by <code>deviceIndex</code>. Invoke PsStopFrame() to stop capturing the image stream.
*  @param[in] 	deviceIndex	The index of the device on which to start capturing the image stream. Device indices range from 0 to device count - 1.
*  @param[in] 	frameType	The type of image stream to start capturing.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsStartFrame(int32_t deviceIndex, PsFrameType frameType);

/**
*  @brief 		Stops capturing the image stream on the device specified by <code>deviceIndex</code>, that was started using PsStartFrame.
*  @param[in] 	deviceIndex	The index of the device on which to stop capturing the image stream. Device indices range from 0 to device count - 1.
*	@param[in] 	frameType	The type of image stream to stop capturing.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsStopFrame(int32_t deviceIndex, PsFrameType frameType);

/**
*  @brief 		Captures the next image frame from the device specified by <code>deviceIndex</code>. This API must be invoked before capturing frame data using PsGetFrame().
*  @param[in] 	deviceIndex The index of the device on which to read the next frame. Device indices range from 0 to device count - 1.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsReadNextFrame(int32_t deviceIndex);

/**
*  @brief 		Returns the image data for the current frame from the device specified by <code>deviceIndex</code>. Before invoking this API, invoke PsReadNextFrame() to capture one image frame from the device.
*  @param[in] 	deviceIndex The index of the device to capture an image frame from. Device indices range from 0 to device count - 1.
*	@param[in] 	frameType 	The image frame type.
*	@param[out] pPsFrame 	Pointer to a buffer in which to store the returned image data.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetFrame(int32_t deviceIndex, PsFrameType frameType, PsFrame* pPsFrame);

/**
* @brief 		Returns IMU data from the device specified by <code>deviceIndex</code> (the update rate is 500hz),
* @param[in] 	deviceIndex The index of the device from which to get the IMU data. Device indices range from 0 to device count - 1.
* @param[out] 	pImuData	Pointer to a PsImu variable in which to store the IMU data.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetImu(int32_t deviceIndex, PsImu* pImuData);

/**
* @brief 		Returns IMU with parameters data from the device specified by <code>deviceIndex</code> (the update rate is 500hz).
* @param[in] 	deviceIndex 		The index of the device from which to get the IMU data. Device indices range from 0 to device count - 1.
* @param[out] 	pImuDataWithParams 	Pointer to a ::PsImuWithParams variable in which to store the IMU data.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetImuWithParams(int32_t deviceIndex, PsImuWithParams* pImuDataWithParams);

/**
* @brief 		Returns audio data from the device specified by <code>deviceIndex</code>.
* @param[in] 	deviceIndex 	The index of the device from which to get audio. Device indices range from 0 to device count - 1.
* @param[out] 	pAudio 			Pointer to a ::PsAudioFrame variable in which to store the audio data.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetAudio(int32_t deviceIndex, PsAudioFrame* pAudio);

/**
*  @brief 		Returns the image frame mode corresponding to the frame type specified by <code>frameType</code>.
*  @param[in] 	deviceIndex The index of the device from which to get the frame mode. Device indices range from 0 to device count - 1.
*	@param[in] 	frameType 	The type of image frame for which to get the frame mode.
*	@param[out] pFrameMode 	Pointer to a ::PsFrameMode variable in which to store the image frame mode.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetFrameMode(int32_t deviceIndex, PsFrameType frameType, PsFrameMode* pFrameMode);

/**
*  @brief 		Sets the image frame mode on the device specified by <code>deviceIndex</code>.
*  @param[in] 	deviceIndex The index of the device for which to set the frame mode. Device indices range from 0 to device count - 1.
*  @param[in] 	frameType 	The type of image frame for which to set the frame mode.
*  @param[in] 	pFrameMode 	Pointer to a ::PsFrameMode variable specifying the image frame mode.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetFrameMode(int32_t deviceIndex, PsFrameType frameType, PsFrameMode* pFrameMode);

/**
*  @brief  	Sets the output data mode.
*  @param[in]	deviceIndex  The index of the device for which to set the data mode. Device indices range from 0 to device count - 1.
*  @param[in]	dataMode  	 The output data mode. See ::PsDataMode for more information.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetDataMode(int32_t deviceIndex, PsDataMode dataMode);

/**
*  @brief 		Returns the depth range mode for the device specified by <code>deviceIndex</code>.
*  @param[in] 	deviceIndex The index of the device from which to get the depth range. Device indices range from 0 to device count - 1.
*	@param[out] pDepthRange Pointer to a ::PsDepthRange variable in which to store the returned depth range mode.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetDepthRange(int32_t deviceIndex, PsDepthRange* pDepthRange);

/**
*  @brief 		Sets the depth range mode for the device specified by <code>deviceIndex</code>.
*  @param[in] 	deviceIndex The index of the device on which to set the depth range. Device indices range from 0 to device count - 1.
*  @param[in] 	depthRange 	Specifies the depth range mode.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetDepthRange(int32_t deviceIndex, PsDepthRange depthRange);

/**
* @brief 		Returns the threshold value for the background filter on the device specified by <code>deviceIndex</code>. The value represents the\n
cut-off point for distant data that the filter should ignore. For example, if 20.0 is specified, data with 20% or less\n
confidence will be dropped.
* @param[in] 	deviceIndex The index of the device from which to get the threshold. Device indices range from 0 to device count - 1.
* @param[out] 	pThreshold 	Pointer to a 16-bit unsigned integer variable in which to return the threshold value.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetThreshold(int32_t deviceIndex, uint16_t* pThreshold);

/**
* @brief 		Sets the threshold value for the background filter on the device specified by <code>deviceIndex</code>. The value represents the cut-off\n
point for distant data that the filter should ignore. For example, if 20.0 is specified, data with 20% or less confidence will be dropped.
* @param[in] 	deviceIndex 	The index of the device on which to set the threshold. Device indices range from 0 to device count - 1.
* @param[in] 	threshold 		The threshold value to set. 0 will attempt to keep all point data but may not be accurate further away; 100 or higher will reject almost all point data leaving only the closest points.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetThreshold(int32_t deviceIndex, uint16_t threshold);

/**
* @brief 		Returns the pulse count for the device specified by <code>deviceIndex</code>.
* @param[in] 	deviceIndex The index of the device on which to set the pulse count. Device indices range from 0 to device count - 1.
* @param[out] 	pPulseCount Pointer to a 16-bit unsigned integer variable in which to store the pulse count value.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetPulseCount(int32_t deviceIndex, uint16_t* pPulseCount);

/**
* @brief 		Sets the pulse count for the device specified by <code>deviceIndex</code>.
* @param[in] 	deviceIndex The index of the device on which to set the pulse count. Device indices range from 0 to device count - 1.
* @param[in] 	pulseCount 	The pulse count value to set.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetPulseCount(int32_t deviceIndex, uint16_t pulseCount);

/**
*  @brief 		Returns a specific property value for the device specified by <code>deviceIndex</code>.
*  @param[in] 	deviceIndex 	The index of the device from which to get the property value. Device indices range from 0 to device count - 1.
*  @param[in] 	propertyType 	The type of property to get from the device. See ::PsPropertyType for more information.
*  @param[out] pData 			Pointer to a buffer to store the returned property value.
*  @param[out] pDataSize 		The size, in bytes, of the property value returned in <code>pData</code>.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetProperty(int32_t deviceIndex, int32_t propertyType, void* pData, int32_t* pDataSize);

/**
*  @brief 		Set the corresponding property value
*  @param[in]  deviceIndex 	Sets a property value for the device specified by <code>deviceIndex</code>.
*  @param[in]  propertyType	The type of property to set on the device.
*  @param[in]	pData 			Pointer to a buffer containing the property value.
*  @param[in]  dataSize 		The size, in bytes, of the property value contained in <code>pData</code>.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetProperty(int32_t deviceIndex, int32_t propertyType, const void* pData, int32_t dataSize);

/**
* @brief 		Returns the internal intrinsic and distortion coefficient parameters of the device specified by <code>deviceIndex</code>.
* @param[in] 	deviceIndex 		The index of the device from which to get the internal parameters. Device indices range from 0 to device count - 1.
* @param[in] 	sensorType 			The type of sensor (depth or RGB) from which to get parameter information. Pass in the applicable value defined by ::PsSensorType.
* @param[out] 	pCameraParameters 	Pointer to a PsCameraParameters variable in which to store the parameter values.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetCameraParameters(int32_t deviceIndex, PsSensorType sensorType, PsCameraParameters* pCameraParameters);

/**
* @brief 		Returns the camera rotation and translation coefficient parameters for the device specified by <code>deviceIndex</code>.
* @param[in] 	deviceIndex 				The index of the device from which to get the extrinsic parameters. Device indices range from 0 to device count - 1.
* @param[out] 	pCameraExtrinsicParameters 	Pointer to a ::PsGetCameraExtrinsicParameters variable in which to store the parameters.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetCameraExtrinsicParameters(int32_t deviceIndex, PsCameraExtrinsicParameters* pCameraExtrinsicParameters);

/**
* @brief 		Sets the color image pixel format on the device specified by <code>deviceIndex</code>. Currently only RGB and BGR formats are supported.
* @param[in] 	deviceIndex	The index of the device to set the pixel format. Device indices range from 0 to device count - 1.
* @param[in] 	pixelFormat	The color pixel format to use. Pass in one of the values defined by ::PsPixelFormat. Currently only <code>PsPixelFormatRGB888</code> and <code>PsPixelFormatBGR888</code> are supported.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetColorPixelFormat(int32_t deviceIndex, PsPixelFormat pixelFormat);

/**
*	@brief 		Enables or disables a filter on the device specified by <code>deviceIndex</code>.
*	@param[in] 	deviceIndex	The index of the device on which to enable or disable the filter. Device indices range from 0 to device count - 1.
*	@param[in] 	filterType	Specifies the type of filter to enable or disable.
*	@param[in] 	bEnabled	Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
*	@return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetFilter(int32_t deviceIndex, PsFilterType filterType, bool bEnabled);

/**
*	@brief 		Returns the Boolean value of whether the filter feature specified by <code>filterType</code> is enabled or disabled.
*	@param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*	@param[in] 	filterType	Specifies the type of filter.
*	@param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetFilter(int32_t deviceIndex, PsFilterType filterType, bool *bEnabled);

/**
*  @brief 		Enables or disables the depth distortion correction feature.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[in] 	bEnabled	Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetDepthDistortionCorrectionEnabled(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the depth distortion correction feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetDepthDistortionCorrectionEnabled(int32_t deviceIndex, bool *bEnabled);

/**
*  @brief 		Enables or disables the Ir distortion correction feature.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[in] 	bEnabled	Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetIrDistortionCorrectionEnabled(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the Ir distortion correction feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetIrDistortionCorrectionEnabled(int32_t deviceIndex, bool *bEnabled);

/**
*  @brief 		Enables or disables the RGB distortion correction feature.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[in] 	bEnabled	Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetRGBDistortionCorrectionEnabled(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the RGB distortion correction feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetRGBDistortionCorrectionEnabled(int32_t deviceIndex, bool *bEnabled);

/**
* @brief 		Converts the input points from world coordinate space to depth coordinate space.
* @param[in]	deviceIndex 	The index of the device on which to perform the operation. Device indices range from 0 to device count - 1.
* @param[in]	pWorldVector 	Pointer to a buffer containing the x, y, and z values of the input world coordinates to be converted, measured in millimeters.
* @param[out]	pDepthVector 	Pointer to a buffer in which to output the converted x, y, and z values of the depth coordinates. \n
*								x and y are measured in pixels, where 0, 0 is located at the top left corner of the image. \n
*								z is measured in millimeters, based on the ::PsPixelFormat depth frame.
* @param[in]	pointCount 		The number of coordinates to convert.
* @param[in]	pCameraParam	The intrinsic camera parameters for the depth camera. See ::PsGetCameraParameters.
* @return ::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsConvertWorldToDepth(int32_t deviceIndex, PsVector3f* pWorldVector, PsDepthVector3* pDepthVector, int32_t pointCount, PsCameraParameters* pCameraParam);

/**
* @brief 		Converts the input points from depth coordinate space to world coordinate space.
* @param[in] 	deviceIndex 	The index of the device on which to perform the operation. Device indices range from 0 to device count - 1.
* @param[in] 	pDepthVector 	Pointer to a buffer containing the x, y, and z values of the depth coordinates to be converted. \n
*       	                    x and y are measured in pixels, where 0, 0 is located at the top left corner of the image. \n
*	                            z is measured in millimeters, based on the ::PsPixelFormat depth frame.
* @param[out] 	pWorldVector 	Pointer to a buffer in which to output the converted x, y, and z values of the world coordinates, measured in millimeters.
* @param[in] 	pointCount 		The number of points to convert.
* @param[in]	pCameraParam	The intrinsic camera parameters for the depth camera. See ::PsGetCameraParameters.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsConvertDepthToWorld(int32_t deviceIndex, PsDepthVector3* pDepthVector, PsVector3f* pWorldVector, int32_t pointCount, PsCameraParameters* pCameraParam);

/**
* @brief 		Converts the input Depth frame from depth coordinate space to world coordinate space on the device specified by <code>deviceIndex</code>.
* @param[in] 	deviceIndex 	The index of the device on which to perform the operation. Device indices range from 0 to device count - 1.
* @param[out] 	pWorldVector 	Pointer to a buffer in which to output the converted x, y, and z values of the world coordinates, measured in millimeters.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsConvertDepthFrameToWorldVector(int32_t deviceIndex, const PsFrame& depthFrame, PsVector3f* pWorldVector);

/**
* @brief 		Enables or disables mapping of the depth image to RGB space on the device specified by <code>deviceIndex</code>. When enabled, PsGetFrame() can\n
*         		be invoked passing ::PsDepthFrame as the frame type, to get the depth frame that is mapped to RGB space. The resolution of\n
*         		the mapped depth frame is the same as that of the RGB image.
* @param[in] 	deviceIndex 	The index of the device on which to enable or disable mapping. Device indices range from 0 to device count - 1.
* @param[in] 	bEnabled 		Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetMapperEnabledDepthToRGB(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the mapping of the depth image to RGB space feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetMapperEnabledDepthToRGB(int32_t deviceIndex, bool *bEnabled);


/**
* @brief 		Enables or disables mapping of the RGB image to depth space on the device specified by <code>deviceIndex</code>. When enabled, PsGetFrame()\n
can be invoked passing ::PsRGBFrame as the frame type, to get the RGB frame that is mapped to depth space. The resolution\n
of the mapped depth frame is the same as that of the RGB image.
* @param[in] 	deviceIndex 	The index of the device on which to enable or disable mapping. Device indices range from 0 to device count - 1.
* @param[in] 	bEnabled 		Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/

PICOZENSE_C_API_EXPORT PsReturnStatus PsSetMapperEnabledRGBToDepth(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the mapping of the RGB image to depth space feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetMapperEnabledRGBToDepth(int32_t deviceIndex, bool *bEnabled);

/**
* @brief		Enables or disables mapping of the RGB image to IR on the device specified by <code>deviceIndex</code>. When enabled,\n
PsGetFrame() can be invoked passing ::PsRGBFrame as the frame type, to get the RGB frame that is mapped to IR space.
* @param[in] 	deviceIndex 	The index of the device on which to enable or disable mapping. Device indices range from 0 to device count - 1.
* @param[in] 	bEnabled 		Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetMapperEnabledRGBToIR(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the mapping of the RGB image to IR space feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetMapperEnabledRGBToIR(int32_t deviceIndex, bool *bEnabled);


/**
* @brief 		Sets the WDR output mode.
* @param[in]	deviceIndex 	The index of the device on which to set the mode. Device indices range from 0 to device count - 1.
* @param[in]	pWDRMode 		The WDR output mode to set. See ::PsWDROutputMode for more information.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetWDROutputMode(int32_t deviceIndex, PsWDROutputMode* pWDRMode);

/**
* @brief		Gets the current WDR output mode.
* @param[in]	deviceIndex 	The index of the device on which to get the mode from. Device indices range from 0 to device count - 1.
* @param[out]	pWDRMode 		A pointer to a ::PsWDROutputMode variable in which to store the current WDR output mode.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetWDROutputMode(int32_t deviceIndex, PsWDROutputMode* pWDRMode);

/**
* @brief 		Sets the fusion threshold for WDR mode.
* @param[in]	deviceIndex	The index of the device on which to set the fusion threshold. Device indices range from 0 to device count - 1.
* @param[in]	threshold1	The first threshold value.
* @param[in]	threshold2	The second threshold value.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetWDRFusionThreshold(int32_t deviceIndex, uint16_t threshold1, uint16_t threshold2);

/**
* @brief 		Sets the WDR style on the device specified by <code>deviceIndex</code>.
* @param[in] 	deviceIndex The index of the device on which to set the WDR style. Device indices range from 0 to device count - 1.
* @param[in] 	wdrStyle 	The wide dynamic range merge style to use. See ::PsWDRStyle for more information.
* @return 		::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetWDRStyle(int32_t deviceIndex, PsWDRStyle wdrStyle);

/**
*  @brief		Enables or disables the syncronize feature.
*  @param[in]	deviceIndex The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[in]  bEnabled Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetSynchronizeEnabled(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the syncronize feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count -1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetSynchronizeEnabled(int32_t deviceIndex, bool *bEnabled);

/**
*  @brief 			Returns the the device's GMM gain.
*  @param[in]		deviceIndex The index of the device from which to get the GMM gain. Device indices range from 0 to device count - 1.
*  @param[out] 	gmmgain 	Pointer to a variable in which to store the returned GMM gain.
*  @return			::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetGMMGain(int32_t deviceIndex, uint16_t* gmmgain);

/**
*  @brief 			Sets the device GMM gain on a device.
*  @param[in]		deviceIndex The index of the device on which to set the GMM gain. Device indices range from 0 to device count - 1.
*  @param[in] 		gmmgain		The GMM gain value to set. See ::PsGMMGain for more information.
*  @return			::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetGMMGain(int32_t deviceIndex, PsGMMGain gmmgain);

/**
*  @brief		Enables or disables the ComputeRealDepth feature.
*  @param[in]	deviceIndex The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[in]  bEnabled Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetComputeRealDepthCorrectionEnabled(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the ComputeRealDepth feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetComputeRealDepthCorrectionEnabled(int32_t deviceIndex, bool *bEnabled);

/**
*  @brief		Enables or disables the SmoothingFilter feature.
*  @param[in]	deviceIndex The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[in]  bEnabled Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetSmoothingFilterEnabled(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the SmoothingFilter feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetSmoothingFilterEnabled(int32_t deviceIndex, bool *bEnabled);

/**
*  @brief 			Sets the RGB frame Resolution on a device specified by <code>deviceIndex</code>.
*  @param[in]		deviceIndex The index of the device on which to set the GMM gain. Device indices range from 0 to device count - 1.
*  @param[in] 		resolution	The resolution value to set. See ::PsResolution for more information.
*  @return			::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetResolution(int32_t deviceIndex, PsResolution resolution);

/**
*  @brief 			Returns the the RGB frame Resolution.
*  @param[in]		deviceIndex The index of the device from which to get the GMM gain. Device indices range from 0 to device count - 1.
*  @param[out] 	resolution 	Pointer to a variable in which to store the returned resolution.
*  @return			::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetResolution(int32_t deviceIndex, uint16_t* resolution);

/**
*  @brief		Enables or disables the SpatialFilter feature.
*  @param[in]	deviceIndex The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[in]  bEnabled Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetSpatialFilterEnabled(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the SpatialFilter feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetSpatialFilterEnabled(int32_t deviceIndex, bool *bEnabled);

/**
*  @brief		Enables or disables the TimeFilter feature.
*  @param[in]	deviceIndex The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[in]  bEnabled Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsSetTimeFilterEnabled(int32_t deviceIndex, bool bEnabled);

/**
*  @brief 		Returns the Boolean value of whether the TimeFilter feature is enabled or disabled.
*  @param[in]	deviceIndex	The index of the device on which to enable or disable the feature. Device indices range from 0 to device count - 1.
*  @param[out] bEnabled	Pointer to a variable in which to store the returned Boolean value.
*  @return 	::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
PICOZENSE_C_API_EXPORT PsReturnStatus PsGetTimeFilterEnabled(int32_t deviceIndex, bool *bEnabled);

#endif /* PICOZENSE_API_H */
