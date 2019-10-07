#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "PicoZense_api.h"

using namespace std;
using namespace cv;

static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t*pData, cv::Mat& dispImg)
{
	dispImg = cv::Mat(height, width, CV_16UC1, pData);
	Point2d pointxy(width / 2, height / 2);
	int val = dispImg.at<ushort>(pointxy);
	char text[20];
#ifdef _WIN32
	sprintf_s(text, "%d", val);
#else
	snprintf(text, sizeof(text), "%d", val);
#endif
	dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
	applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
	int color;
	if (val > 2500)
		color = 0;
	else
		color = 4096;
	circle(dispImg, pointxy, 4, Scalar(color, color, color), -1, 8, 0);
	putText(dispImg, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));
}

int main(int argc, char *argv[])
{
	PsReturnStatus status;
	int32_t deviceIndex = 0;
	int32_t deviceCount = 0;
	uint32_t slope = 1450;
	uint32_t wdrSlope = 4400;
	PsDepthRange depthRange = PsNearRange;
	int32_t dataMode = PsDepthAndRGB_30;

	status = PsInitialize();
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsInitialize failed!" << endl;
		system("pause"); 
		return -1;
	}

	status = PsGetDeviceCount(&deviceCount);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsGetDeviceCount failed!" << endl;
		system("pause");
		return -1;
	}
	cout << "Get device count: " << deviceCount << endl;

	//Set the Depth Range to Near through PsSetDepthRange interface
	status = PsSetDepthRange(deviceIndex, PsNearRange);
	if (status != PsReturnStatus::PsRetOK)
		cout << "PsSetDepthRange failed!" << endl;
	else
		cout << "Set Depth Range to Near" << endl;

	//Enable the Depth and RGB synchronize feature
	//PsSetSynchronizeEnabled(deviceIndex, true);
	
	status = PsOpenDevice(deviceIndex);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "OpenDevice failed!" << endl;
		system("pause");
		return -1;
	}

	//Set PixelFormat as PsPixelFormatBGR888 for opencv display
	PsSetColorPixelFormat(deviceIndex, PsPixelFormatBGR888);
	
	//Set to DepthAndRGB_30 mode
	status = PsSetDataMode(deviceIndex, (PsDataMode)dataMode);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "Set DataMode Failed failed!" << endl;
	}

	PsCameraParameters cameraParameters;
	status = PsGetCameraParameters(deviceIndex, PsDepthSensor, &cameraParameters);

	cout << "Get PsGetCameraParameters status: " << status << endl;
	cout << "Depth Camera Intinsic: " << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	cout << "Depth Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "K4: " << cameraParameters.k4 << endl;
	cout << "K5: " << cameraParameters.k5 << endl;
	cout << "K6: " << cameraParameters.k6 << endl;

	status = PsGetCameraParameters(deviceIndex, PsRgbSensor, &cameraParameters);

	cout << "Get PsGetCameraParameters status: " << status << endl;
	cout << "RGB Camera Intinsic: " << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	cout << "RGB Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;

	PsCameraExtrinsicParameters CameraExtrinsicParameters;
	status = PsGetCameraExtrinsicParameters(deviceIndex, &CameraExtrinsicParameters);

	cout << "Get PsGetCameraExtrinsicParameters status: " << status << endl;
	cout << "Camera rotation: " << endl;
	cout << CameraExtrinsicParameters.rotation[0] << " " 
		 << CameraExtrinsicParameters.rotation[1] << " " 
		 << CameraExtrinsicParameters.rotation[2] << " " 
		 << CameraExtrinsicParameters.rotation[3] << " " 
		 << CameraExtrinsicParameters.rotation[4] << " " 
		 << CameraExtrinsicParameters.rotation[5] << " " 
		 << CameraExtrinsicParameters.rotation[6] << " " 
		 << CameraExtrinsicParameters.rotation[7] << " " 
		 << CameraExtrinsicParameters.rotation[8] << " " 
		 << endl;
 
	cout << "Camera transfer: " << endl;
	cout << CameraExtrinsicParameters.translation[0] << " "  
		 << CameraExtrinsicParameters.translation[1] << " " 
		 << CameraExtrinsicParameters.translation[2] << " " << endl;

	cv::Mat imageMat;
	const string irImageWindow = "IR Image";
	const string rgbImageWindow = "RGB Image";
	const string depthImageWindow = "Depth Image";
	const string mappedDepthImageWindow = "MappedDepth Image";
	const string mappedRgbImageWindow = "MappedRGB Image";
	const string mappedIRWindow = "MappedIR Image";
	const string wdrDepthImageWindow = "WDR Depth Image";

	ofstream PointCloudWriter;
	PsDepthVector3 DepthVector = { 0, 0, 0 };
	PsVector3f WorldVector = { 0.0f };

	bool f_bDistortionCorrection = false;
	bool f_bFilter = false;
	bool f_bMappedRGB = true;
	bool f_bMappedIR = true;
	bool f_bMappedDepth = true;
	bool f_bWDRMode = false;
	bool f_bInvalidDepth2Zero = false;
	bool f_bDustFilter = false;
	bool f_bSync = true;
	cout << "\n--------------------------------------------------------------------" << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << "Press following key to set corresponding feature:" << endl;
	cout << "M/m: Change data mode: input corresponding index in terminal:" << endl;
	cout << "                    0: Output Depth and RGB in 30 fps" << endl;
	cout << "                    1: Output IR and RGB in 30 fps" << endl;
	cout << "                    2: Output Depth and IR in 30 fps" << endl;
	cout << "                    3: Output Depth and IR and RGB in 30 fps" << endl;
	cout << "                    4: Output Depth/IR frames alternatively in 15fps, and RGB in 30fps" << endl;
	cout << "                    5: Output WDR_Depth and RGB in 30 fps" << endl;
	cout << "0/1/2...: Change depth range Near/Middle/Far..." << endl;
	cout << "R/r: Change the RGB resolution: input corresponding index in terminal:" << endl;
	cout << "                             0: 1920*1080" << endl;
	cout << "                             1: 1280*720" << endl;
	cout << "                             2: 640*480" << endl;
	cout << "                             3: 640*360" << endl;
	cout << "P/p: Save point cloud data into PointCloud.txt in current directory" << endl;
	cout << "T/t: Change background filter threshold value" << endl;
	cout << "U/u: Enable or disable the distortion correction feature" << endl;
	cout << "F/f: Enable or disable the smoothing filter feature" << endl;
	cout << "Q/q: Enable or disable the mapped RGB in Depth space" << endl;
	cout << "L/l: Enable or disable the mapped Depth in RGB space" << endl;
	cout << "I/i: Enable or disable the mapped IR in RGB space" << endl;
	cout << "V/v: Enable or disable the WDR depth fusion feature " << endl;
	cout << "S/s: Enable or disable the Depth and RGB synchronize feature " << endl;
	cout << "Esc: Program quit " << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << "--------------------------------------------------------------------\n" << endl;

	for (;;)
	{
		PsFrame depthFrame = { 0 };
		PsFrame irFrame = { 0 };
		PsFrame rgbFrame = { 0 };
		PsFrame mappedDepthFrame = { 0 };
		PsFrame wdrDepthFrame = { 0 };
		PsFrame mappedRGBFrame = { 0 };
		PsFrame mappedIRFrame = { 0 };
		
		// Read one frame before call PsGetFrame
		status = PsReadNextFrame(deviceIndex);
		if (status != PsRetOK)
		{
			goto KEY;
		}

		//Get depth frame, depth frame only output in following data mode
		if (dataMode == PsDepthAndRGB_30 || dataMode == PsDepthAndIR_30 || dataMode == PsDepthAndIRAndRGB_30 || dataMode == PsDepthAndIR_15_RGB_30)
		{
			PsGetFrame(deviceIndex, PsDepthFrame, &depthFrame);
			
			if (depthFrame.pFrameData != NULL)
			{
				//Display the Depth Image
				Opencv_Depth(slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);
				cv::imshow(depthImageWindow, imageMat);
			}
			
		}

		//Get IR frame, IR frame only output in following data mode
		if (dataMode == PsIRAndRGB_30 || dataMode == PsDepthAndIR_30 || dataMode == PsDepthAndIRAndRGB_30 || dataMode == PsDepthAndIR_15_RGB_30)
		{
			PsGetFrame(deviceIndex, PsIRFrame, &irFrame);
			
			if (irFrame.pFrameData != NULL)
			{
				//Display the IR Image
				imageMat = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);

				// Convert 16bit IR pixel (max pixel value is 3840) to 8bit for display
				imageMat.convertTo(imageMat, CV_8U, 255.0 / 3840);
				cv::imshow(irImageWindow, imageMat);
			}
		}

		//Get RGB frame, RGB frame only output in following data mode
		if (dataMode == PsDepthAndRGB_30 || dataMode == PsIRAndRGB_30 || dataMode == PsDepthAndIRAndRGB_30 || dataMode == PsWDR_Depth || dataMode == PsDepthAndIR_15_RGB_30)
		{
			PsGetFrame(deviceIndex, PsRGBFrame, &rgbFrame);
			
			if (rgbFrame.pFrameData != NULL)
			{
				//Display the RGB Image
				imageMat = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
				cv::imshow(rgbImageWindow, imageMat);
			}
		}

		//Get WDR depth frame(fusion or alternatively, determined by PsSetWDRStyle, default in fusion)
		//WDR depth frame only output in PsWDR_Depth data mode
		if (dataMode == PsWDR_Depth)
		{
			PsGetFrame(deviceIndex, PsWDRDepthFrame, &wdrDepthFrame);
			if (wdrDepthFrame.pFrameData != NULL)
			{
				//Display the WDR Depth Image
				Opencv_Depth(wdrSlope, wdrDepthFrame.height, wdrDepthFrame.width, wdrDepthFrame.pFrameData, imageMat);
				cv::imshow(wdrDepthImageWindow, imageMat);
			}
		}

		//Get mapped depth frame which is mapped to rgb camera space
		//Mapped depth frame only output in following data mode
		//And can only get when the feature is enabled through api PsSetMapperEnabledRGBToDepth
		//When the key "L/l" pressed, this feature enable or disable
		if (dataMode == PsDepthAndRGB_30 || dataMode == PsDepthAndIRAndRGB_30 || dataMode == PsWDR_Depth || dataMode == PsDepthAndIR_15_RGB_30)
		{
			PsGetFrame(deviceIndex, PsMappedDepthFrame, &mappedDepthFrame);

			if (mappedDepthFrame.pFrameData != NULL)
			{
				//Display the MappedDepth Image
				imageMat = cv::Mat(mappedDepthFrame.height, mappedDepthFrame.width, CV_16UC1, mappedDepthFrame.pFrameData);
				cv::Mat mappedDepthMat;
				imageMat.convertTo(mappedDepthMat, CV_8U, 255.0 / (f_bWDRMode? wdrSlope:slope));
				cv::applyColorMap(mappedDepthMat, mappedDepthMat, cv::COLORMAP_RAINBOW);
				cv::imshow(mappedDepthImageWindow, mappedDepthMat);
			}
		}

		//Get mapped IR frame which is mapped to RGB camera space
		//Mapped IR frame only output in following data mode
		//And can only get when the feature is enabled through api PsSetMapperEnabledRGBToIR
		//When the key "I/i" pressed, this feature enable or disable
		if (dataMode == PsDepthAndIRAndRGB_30)
		{
			PsGetFrame(deviceIndex, PsMappedIRFrame, &mappedIRFrame);

			if (mappedIRFrame.pFrameData != NULL)
			{
				//Display the MappedIR Image
				imageMat = cv::Mat(mappedIRFrame.height, mappedIRFrame.width, CV_16UC1, mappedIRFrame.pFrameData);
				// Convert 16bit IR pixel (max pixel value is 3840) to 8bit for display
				imageMat.convertTo(imageMat, CV_8U, 255.0 / 3840);
				cv::imshow(mappedIRWindow, imageMat);
			}
		}

		//Get mapped rgb frame which is mapped to depth camera space
		//Mapped rgb frame only output in following data mode
		//And can only get when the feature is enabled through api PsSetMapperEnabledDepthToRGB
		//When the key "Q/q" pressed, this feature enable or disable
		if (dataMode == PsDepthAndRGB_30 || dataMode == PsDepthAndIRAndRGB_30 || dataMode == PsWDR_Depth || dataMode == PsDepthAndIR_15_RGB_30)
		{
			PsGetFrame(deviceIndex, PsMappedRGBFrame, &mappedRGBFrame);

			if (mappedRGBFrame.pFrameData != NULL)
			{
				//Display the MappedRGB Image
				imageMat = cv::Mat(mappedRGBFrame.height, mappedRGBFrame.width, CV_8UC3, mappedRGBFrame.pFrameData);
				cv::imshow(mappedRgbImageWindow, imageMat);
			}
		}
KEY:
		unsigned char key = waitKey(1);
		imageMat.release();

		if (key == 'M' || key == 'm')
		{
			cout << "Selection: 0:DepthAndRgb_30; 1:IrAndRgb_30; 2:DepthAndIR_30; 3:DepthAndIRAndRGB_30; 4:DepthAndIR_15_RGB_30; 5:WDR_Depth" << endl;
			int index = -1;
			cin >> index;
			//clear buffer and cin flag. if not, cin will not get input anymore;
			if (cin.fail())
			{
				std::cout << "Unexpected input\n";
				cin.clear();
				cin.ignore(1024,'\n');
				continue;
			}
			else
			{
				cin.clear();
				cin.ignore(1024,'\n');
			}

			switch (index)
			{
			case 0:
				dataMode = PsDepthAndRGB_30;
				break;
			case 1:
				dataMode = PsIRAndRGB_30;
				break;
			case 2:
				dataMode = PsDepthAndIR_30;
				break;
			case 3:
				dataMode = PsDepthAndIRAndRGB_30;
				break;
			case 4:
				dataMode = PsDepthAndIR_15_RGB_30;
				break;
			case 5:
				dataMode = PsWDR_Depth;
				break;
			default:
				cout << "Unsupported data mode!" << endl;
				continue;
			}

			PsSetDataMode(deviceIndex, (PsDataMode)dataMode);

			if(dataMode == PsWDR_Depth)
			{
				//Set WDR Output Mode, two ranges Near/Far output from device every one frame, no care for range3 and range3Count in PsWDRTotalRange_Two
				PsWDROutputMode wdrMode = { PsWDRTotalRange_Two, PsNearRange, 1, PsFarRange, 1, PsNearRange, 1 };
				//Set WDR fusion threshold, no care for threshold2 in PsWDRTotalRange_Two
				PsSetWDRFusionThreshold(deviceIndex, 1000, 2500);
				PsSetWDROutputMode(deviceIndex, &wdrMode);
				f_bWDRMode = true;
			}
			else
			{
				f_bWDRMode = false;
			}
		}
		else if ((key == '0') || (key == '1') || (key == '2') || (key == '3') || (key == '4') || (key == '5') || (key == '6') || (key == '7') || (key == '8'))
		{
			switch (key)
			{
			case '0':
				depthRange = PsNearRange;
				slope = 1450;
				break;
			case '1':
				depthRange = PsMidRange;
				slope = 3000;
				break;
			case '2':
				depthRange = PsFarRange;
				slope = 4400;
				break;
			case '3':
				depthRange = PsXNearRange;
				slope = 4800;
				break;
			case '4':
				depthRange = PsXMidRange;
				slope = 5600;
				break;
			case '5':
				depthRange = PsXFarRange;
				slope = 7500;
				break;
			case '6':
				depthRange = PsXXNearRange;
				slope = 9600;
				break;
			case '7':
				depthRange = PsXXMidRange;
				slope = 11200;
				break;
			case '8':
				depthRange = PsXXFarRange;
				slope = 15000;
				break;
			default:
				cout << "Unsupported Range!" << endl;
				continue;
			}
			status = PsSetDepthRange(deviceIndex, depthRange);
			if (depthRange == PsNearRange)
				cout << "Set depth range to Near," << " status: " << status << endl;
			else if (depthRange == PsMidRange)
				cout << "Set depth range to Mid," << " status: " << status << endl;
			else if (depthRange == PsFarRange)
				cout << "Set depth range to Far," << " status: " << status << endl;
			else if (depthRange == PsXNearRange)
				cout << "Set depth range to XNearRange," << " status: " << status << endl;
			else if (depthRange == PsXMidRange)
				cout << "Set depth range to XMidRange," << " status: " << status << endl;
			else if (depthRange == PsXFarRange)
				cout << "Set depth range to XFarRange," << " status: " << status << endl;
			else if (depthRange == PsXXNearRange)
				cout << "Set depth range to XXNearRange," << " status: " << status << endl;
			else if (depthRange == PsXXMidRange)
				cout << "Set depth range to XXMidRange," << " status: " << status << endl;
			else if (depthRange == PsXXFarRange)
				cout << "Set depth range to XXFarRange," << " status: " << status << endl;

			if (status != PsRetOK)
			{
				cout << "Set depth range failed! " << endl;
			}

			status = PsGetDepthRange(deviceIndex, &depthRange);
			cout << "Get depth range," << " depthRange: " << depthRange << endl;
		}

		else if (key == 'R' || key == 'r')
		{
			cout << "please select RGB resolution to set: 0:1080P; 1:720P; 2:480P; 3:360P" << endl;
			int index = 0;
			cin >> index;
			//clear buffer and cin flag. if not, cin will not get input anymore;
			if (cin.fail())
			{
				std::cout << "Unexpected input\n";
				cin.clear();
				cin.ignore(1024,'\n');
				continue;
			}
			else
			{
				cin.clear();
				cin.ignore(1024,'\n');
			}

			PsFrameMode frameMode;
			frameMode.fps = 30;
			frameMode.pixelFormat = PsPixelFormatBGR888;
			if (1 == index)
			{
				frameMode.resolutionWidth = 1280;
				frameMode.resolutionHeight = 720;
			}
			else if (2 == index)
			{
				frameMode.resolutionWidth = 640;
				frameMode.resolutionHeight = 480;
			}
			else if (3 == index)
			{
				frameMode.resolutionWidth = 640;
				frameMode.resolutionHeight = 360;
			}
			else
			{
				frameMode.resolutionWidth = 1920;
				frameMode.resolutionHeight = 1080;
			}
			cout << "Set RGB width:" << frameMode.resolutionWidth << " height:" << frameMode.resolutionHeight << endl;
			PsSetFrameMode(deviceIndex, PsRGBFrame, &frameMode);
		}

		else if (key == 'P' || key == 'p')
		{
			//Save the pointcloud
			if (depthFrame.pFrameData != NULL|| wdrDepthFrame.pFrameData != NULL)
			{
				PointCloudWriter.open("PointCloud.txt");
				PsFrame &srcFrame = (f_bWDRMode ? wdrDepthFrame : depthFrame);
				const int len = srcFrame.width * srcFrame.height;
				PsVector3f* worldV = new PsVector3f[len];

				PsConvertDepthFrameToWorldVector(deviceIndex, srcFrame, worldV); //Convert Depth frame to World vectors.

				for (int i = 0; i < len; i++)
				{ 
					if (worldV[i].z == 0 || worldV[i].z == 0xFFFF) 
						continue; //discard zero points
					PointCloudWriter << worldV[i].x << "\t" << worldV[i].y << "\t" << worldV[i].z << std::endl;
				}
				delete[] worldV;
				worldV = NULL;
				std::cout << "Save point cloud successful in PointCloud.txt" << std::endl;
				PointCloudWriter.close();
			}
			else
			{
				std::cout << "Current mode do not surpport point cloud " << endl;
			}
		}
		else if (key == 'T' || key == 't')
		{
			//Set background filter threshold
			static uint16_t threshold = 0;
			threshold += 10;
			if (threshold > 100)
				threshold = 0;
			PsSetThreshold(deviceIndex, threshold);
			cout << "Set background threshold value: " << threshold << endl;
		}
		else if (key == 'U' || key == 'u')
		{
			PsSetDepthDistortionCorrectionEnabled(deviceIndex, f_bDistortionCorrection);
			PsSetIrDistortionCorrectionEnabled(deviceIndex, f_bDistortionCorrection);
			PsSetRGBDistortionCorrectionEnabled(deviceIndex, f_bDistortionCorrection);
			cout << "Set DistortionCorrection " << (f_bDistortionCorrection ? "Enabled." : "Disabled.") << endl;
			f_bDistortionCorrection = !f_bDistortionCorrection;
		}
		else if (key == 'F' || key == 'f')
		{
			status = PsSetFilter(deviceIndex, PsSmoothingFilter, f_bFilter);
			if (status == PsRetOK)
			{
				cout << "Set SmoothingFilter " << (f_bFilter ? "Enabled." : "Disabled.") << endl;
				f_bFilter = !f_bFilter;
			}
		}
		else if (key == 'I' || key == 'i')
		{
			status = PsSetMapperEnabledRGBToIR(deviceIndex, f_bMappedIR);
			if (status == PsRetOK)
			{
				cout << "Set Mapper IR " << (f_bMappedIR ? "Enabled." : "Disabled.") << endl;
				f_bMappedIR = !f_bMappedIR;
			}
		}
		else if (key == 'L' || key == 'l')
		{
			status = PsSetMapperEnabledRGBToDepth(deviceIndex, f_bMappedDepth);
			if (status == PsRetOK)
			{
				cout << "Set Mapper RGBToDepth " << (f_bMappedDepth ? "Enabled." : "Disabled.") << endl;
				f_bMappedDepth = !f_bMappedDepth;
			}
		}
		else if (key == 'Q' || key == 'q')
		{
			status = PsSetMapperEnabledDepthToRGB(deviceIndex, f_bMappedRGB);
			if (status == PsRetOK)
			{
				cout << "Set Mapper DepthToRGB " << (f_bMappedRGB ? "Enabled." : "Disabled.") << endl;
				f_bMappedRGB = !f_bMappedRGB;
			}
		}
		else if (key == 'S' || key == 's')
		{
			status = PsSetSynchronizeEnabled(deviceIndex, f_bSync);
			if (status == PsRetOK)
			{
				cout << "Set Synchronize " << (f_bSync ? "Enabled." : "Disabled.") << endl;
				f_bSync = !f_bSync;
			}
		}
		else if(key == 'V' || key == 'v')
		{
			if (f_bWDRMode)
			{
				static bool bWDRStyle = true;
				status = PsSetWDRStyle(deviceIndex, bWDRStyle ? PsWDR_ALTERNATION : PsWDR_FUSION);
				if (PsRetOK == status)
				{
					cout << "WDR image output " << (bWDRStyle ? "alternatively in multi range." : "Fusion.") <<endl;
					bWDRStyle = !bWDRStyle;
				}
			}
		}
		else if (key == 27)	//ESC Pressed
		{
			break;
		}
	}

    status = PsCloseDevice(deviceIndex);
    cout << "CloseDevice status: " << status << endl;

    status = PsShutdown();
    cout << "Shutdown status: " << status << endl;
	cv::destroyAllWindows();
    return 0;
}
