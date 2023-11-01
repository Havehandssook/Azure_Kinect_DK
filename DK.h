#pragma once  //��֤��ͷ�ļ�ֻ����һ��

#include <cstdlib>
#include <stdio.h>
#include <k4a/k4a.h>
#include <iostream>
#include <stdlib.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <opencv2/opencv.hpp>
#include <k4a/k4a.hpp>
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

using namespace cv;
using namespace std;


/*��ɫͼ���õ�ö��*/
typedef enum _DKColorType
{
	/*BGRA*/
	COLOR_BGRA_1280x720_30Hz,
	COLOR_BGRA_1920x1080_30Hz,
	COLOR_BGRA_2560x1440_30Hz,
	COLOR_BGRA_3840x2160_30Hz,
	COLOR_BGRA_2048x1536_30Hz,	
	COLOR_BGRA_4096x3072_15Hz,
	
	/*MJPG*/
	COLOR_MJPG_1280x720_30Hz,
	COLOR_MJPG_1920x1080_30Hz,
	COLOR_MJPG_2560x1440_30Hz,
	COLOR_MJPG_3840x2160_30Hz,
	COLOR_MJPG_2048x1536_30Hz,
	COLOR_MJPG_4096x3072_15Hz,

	/*NV12*/
	COLOR_NV12_1280x720_30Hz,

	/*YUY2*/
	COLOR_YUY2_1280x720_30Hz,

}DKColorType;

/*���ͼ���õ�ö��*/
typedef enum _DKDepthType
{
	DEPTH_NFOV_320x288_30HZ,
	DEPTH_NFOV_640x576_30HZ,
	DEPTH_WFOV_512x512_30HZ,
	DEPTH_WFOV_1024x1024_15HZ,

}DKDepthType;

/*ͼ������õ���ö��*/
typedef enum _Align {
	NOUSE,                 // ��ʹ�ö���
	DEPTH_ALIGN_TO_COLOR,  // ��ȶ��뵽��ɫ
	COLOR_ALIGN_TO_DEPTH   // ��ɫ���뵽���
}Align;


/*���ͼ��������*/
typedef struct _pinhole_t
{
	float px;
	float py;
	float fx;
	float fy;

	int width;
	int height;
} pinhole_t;

typedef struct _coordinate_t
{
	int x;
	int y;
	float weight[4];
} coordinate_t;

typedef enum
{
	INTERPOLATION_NEARESTNEIGHBOR, /**< Nearest neighbor interpolation */
	INTERPOLATION_BILINEAR,        /**< Bilinear interpolation */
	INTERPOLATION_BILINEAR_DEPTH   /**< Bilinear interpolation with invalidation when neighbor contain invalid
												 data with value 0 */
} interpolation_t;

/*ĳһ�����ά����*/
typedef struct {
	short W;
	short H;
	unsigned short D;
}Point3D;

class KinectDK 
{
public:
	KinectDK();//���캯�� ��ʼ������
	~KinectDK();//�������� �������

		/***********************    ͼ���趨    ***********************/
	void colorInit(DKColorType ColorType);//��ʼ����ɫ���
	void depthInit(DKDepthType DepthType);//��ʼ��������

	    /***********************    ͼ������    ***********************/
	void Configuration();

	    /***********************    ���ݸ���    ***********************/
	void updateFrame();					

		/***********************    ͼ�񲶻�    ***********************/
	void get_colorImage(Mat& colorImage, Align Align);
	void get_depthImage(Mat& depthImage, Align Align);
	void get_irImage(Mat& irImage);
	void point2D_to_Point3D(k4a_float2_t point2D, int distance, k4a_float3_t& point3D);// ���ĳһ�����ά����
	    /***********************    �����ͷ�    ***********************/
	void Release();
     	/***********************    �����ǰ�    ***********************/
	void imu_start();
	void get_imu(k4a_imu_sample_t& imu_sample);
	

private:

	typedef struct {
		k4a_image_format_t                   format;        // Mat ���������
		k4a_color_resolution_t				 resolution;    // �ֱ���
		k4a_fps_t                            fps;           // ֡��
	}ColorCfg;

	typedef struct {    
		k4a_depth_mode_t				     mode;          //ģʽ
		k4a_fps_t                            fps;           // ֡��
	}DepthCfg;


	/* ����ö�ٶ�Ӧ�Ŀ��Ʋ��� */ /*�ո�ո�*/
	void ProduceColorCfg(DKColorType Colortype, ColorCfg& _ColorCfg);
	void ProduceDepthCfg(DKDepthType DepthType, DepthCfg& _DepthCfg);

	k4a_image_t rbgimage;
	k4a_image_t depthimage;
	k4a_image_t irimage;

	ColorCfg _ColorCfg; // ��ɫ���Ʋ���
	DepthCfg _DepthCfg; // ��ȿ��Ʋ���

	k4a_device_t device;
	k4a_capture_t sensor_capture; // ���崫����������
	k4a_wait_result_t get_capture_result;
	k4a_device_configuration_t cfg = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;


	k4a_imu_sample_t imu_sample;
	/*����*/
	k4a_calibration_t calibration;
	k4a_transformation_t transformation = NULL;	
	k4a_image_t transformed_color_image = NULL;
	k4a_image_t transformed_depth_image = NULL;

	bool Flg_InitColor = false;							 // �򿪲�ɫͼ��־
	bool Flg_InitDepth = false;							 // �����ͼ��־
	bool Flg_Color_to_Depth = false;					 // �����־
	bool Flg_Depth_to_Color = false;

#define INVALID INT32_MIN
	pinhole_t pinhole;
	k4a_image_t undistorted = NULL;
	k4a_image_t lut = NULL;
	interpolation_t interpolation_type = INTERPOLATION_NEARESTNEIGHBOR;

	int valid;

};


/*������������*/
extern void create_undistortion_lut(const k4a_calibration_t* calibration,
									const k4a_calibration_type_t camera,
									const pinhole_t* pinhole,
									k4a_image_t lut,
									interpolation_t type);

extern void compute_xy_range(const k4a_calibration_t* calibration,
						  	 const k4a_calibration_type_t camera,
							 const int width,
							 const int height,
							 float& x_min,
							 float& x_max,
							 float& y_min,
							 float& y_max);

extern pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t* calibration, const k4a_calibration_type_t camera);

extern void remap(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type);




