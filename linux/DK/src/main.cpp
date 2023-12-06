#include"DK.h"

KinectDK DK;
Mat colorimg;
Mat depthimg;
k4a_imu_sample_t imu;

k4a_float2_t point2D_1, point2D_2;
k4a_float3_t point3D_1, point3D_2;

//#define FPS

int main()
{
	Rect red_max_rect;
	int thresh_red = 170;
	Mat image_edge_red;

	DK.colorInit(COLOR_BGRA_1280x720_30Hz);
	DK.depthInit(DEPTH_NFOV_640x576_30HZ);
	DK.Configuration();

	int i=3;
	while (1)
	{
		high_resolution_clock::time_point t1 = high_resolution_clock::now();
		DK.updateFrame();

		DK.get_colorImage(colorimg, NOUSE);	
		DK.get_depthImage(depthimg, DEPTH_ALIGN_TO_COLOR);

		imshow("colorimg", colorimg);
		imshow("depthimg", depthimg);

#ifdef FPS
		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
		printf("FPS:%.6f \n", 1 / time_span.count());
#endif // FPS

		DK.Release();

		if (waitKey(1) == 27)
		{
			break;
		}

	}

}

