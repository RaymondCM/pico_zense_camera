#ifndef PICOZENSE_TYPES_H
#define PICOZENSE_TYPES_H

#include <stdint.h>
#include "PicoZense_enums.h"

typedef uint16_t PsDepthPixel;  //!< Depth image pixel type in 16-bit
typedef uint16_t PsGray16Pixel; //!< Gray image pixel type in 16-bit
typedef uint8_t PsGray8Pixel;   //!< Gray image pixel type in 8-bit

#pragma pack (push, 1)
/**
 * @brief Color image pixel type in 24-bit RGB format.
 */
typedef struct
{
	uint8_t r;	//!< Red
	uint8_t g;	//!< Green
	uint8_t b;	//!< Blue
} PsRGB888Pixel;

/**
 * @brief Color image pixel type in 24-bit BGR format.
 */
typedef struct
{
	uint8_t b;	//!< Blue
	uint8_t g;	//!< Green
	uint8_t r;	//!< Red
} PsBGR888Pixel;

/**
 * @brief Specifies the frame mode including the pixel format, resolution, and frame rate.
 */
typedef struct
{
	PsPixelFormat pixelFormat;			//!< The pixel format used by a frame.
	int32_t       resolutionWidth;		//!< The width of the image, in pixels.
	int32_t       resolutionHeight;		//!< The height of the image, in pixels.
	int32_t       fps;					//!< The image stream frame rate.
}PsFrameMode;

/**
 * @brief Stores the x, y, and z components of a 3D vector.
 */
typedef struct  
{
	float x, y, z;	//!< The x, y, and z components of the vector.
}PsVector3f;

/**
 * @brief Contains depth information for a given pixel.
 */
typedef struct
{
	int          depthX;    //!< The x coordinate of the pixel.
	int          depthY;    //!< The y coordinate of the pixel.
	PsDepthPixel depthZ;    //!< The depth of the pixel, in millimeters.
}PsDepthVector3;

/**
 * @brief Specifies the IMU data.
 */
typedef struct
{
	PsVector3f acc;      //!< The accelerometer sensor measurement (m/s^2).
	PsVector3f gyro;     //!< The gyroscope sensor measurement (rad/s).
	uint8_t    frameNo;  //!< The IMU frame number in the range of 0 to 255.
}PsImu;

/**
 * @brief Specifies the IMU with extra parameters.
 */
typedef struct
{
	PsVector3f acc;      //!< The accelerometer sensor measurement (m/s^2).
	PsVector3f gyro;     //!< The gyroscope sensor measurement (rad/s).
	float      temp;     //!< The temperature, in degrees Celsius.
	uint8_t    frameNo;  //!< The IMU frame number in the range of 0~255.
}PsImuWithParams;

/**
 * @brief Camera intrinsic parameters and distortion coefficients.
 */
typedef struct
{
	double	fx;  //!< Focal length x (pixel)
	double	fy;  //!< Focal length y (pixel)
	double	cx;  //!< Principal point x (pixel)
	double	cy;  //!< Principal point y (pixel)
	double	k1;  //!< Radial distortion coefficient, 1st-order
	double	k2;  //!< Radial distortion coefficient, 2nd-order
	double	p1;  //!< Tangential distortion coefficient
	double	p2;  //!< Tangential distortion coefficient
	double	k3;  //!< Radial distortion coefficient, 3rd-order
	double	k4;  //!< Radial distortion coefficient, 4st-order
	double	k5;  //!< Radial distortion coefficient, 5nd-order
	double	k6;  //!< Radial distortion coefficient, 6rd-order
}PsCameraParameters;

/** 
 * @brief Specifies the camera’s location and orientation extrinsic parameters.
 */
typedef struct
{
	double rotation[9];     //!< Orientation stored as an array of 9 double representing a 3x3 rotation matrix.
	double translation[3];  //!< Location stored as an array of 3 double representing a 3-D translation vector.
}PsCameraExtrinsicParameters;

/**
 * @brief Depth/IR/RGB image frame data.
 */
typedef struct
{
	uint32_t       frameIndex;    //!< The index of the frame.
	PsFrameType    frameType;     //!< The type of frame. See ::PsFrameType for more information.
	PsPixelFormat  pixelFormat;   //!< The pixel format used by a frame. See ::PsPixelFormat for more information.
	uint8_t        imuFrameNo;    //!< Used to synchronize with IMU, in the range of 0 to 255.
	uint8_t*       pFrameData;    //!< A buffer containing the frame’s image data.
	uint32_t       dataLen;       //!< The length of pFrame, in bytes.
	float          exposureTime;  //!< The exposure time, in milliseconds.
	PsDepthRange   depthRange;    //!< The depth range mode of the current frame. Used only for depth frames.
	uint16_t       width;		  //!< The width of the frame, in pixels.
	uint16_t       height;        //!< The height of the frame, in pixels.
}PsFrame;

/**
 * @brief Contains a frame of audio data.
 */
typedef struct
{
	uint8_t  audioFormat;    //!< The audio format. 0:PCM.
	uint8_t  numChannels;    //!< The number of audio channels. 1:mono; 2:stereo.
	uint8_t  bitsPerSample;  //!< The number of bits per sample (16 bits).
	uint32_t sampleRate;     //!< The sampling rate (16 kHz).
	uint8_t* pData;          //!< The frame buffer pointer.
	uint32_t dataLen;        //!< The frame data size, in bytes.
}PsAudioFrame;

/** 
 * @brief WDR (Wide Dynamic Range) output mode settings (e.g. Near/Far range fusion).
 */
typedef struct
{	 
	PsWDRTotalRange totalRange;  //!< The number of ranges supported. Currently only two or three ranges are supported (e.g. Near/Far or Near/Mid/Far).
	PsDepthRange    range1;      //!< The first range.
	uint8_t         range1Count; //!< The count of successive <code>range1</code> frames.
	PsDepthRange    range2;      //!< The second range.
	uint8_t         range2Count; //!< The count of successive <code>range2</code> frames.
	PsDepthRange    range3;      //!< Third range. This range only takes effect when <code>totalRange</code> is set to <code>3</code>.
	uint8_t         range3Count; //!< The count of successive <code>range3</code> frames. This only takes effect when <code>totalRange</code> is set to <code>3</code>.
}PsWDROutputMode;

/**
 * @brief Specifies the GMMGain including the gain value and option type.
 */
typedef struct
{
	uint16_t gain;	//!< The GMM gain value of the device.
	uint8_t	option;	//!< The option type of setting the GMM gain effective time. 0:Immediate effect, invalid after camera closure; 1:Permanent entry into force.
}PsGMMGain;
#pragma pack (pop)

#endif /* PICOZENSE_TYPES_H */
