#include"DK.h"

KinectDK DK;
Mat colorimg;
Mat depthimg;

//#define FPS

int main()
{

	DK.colorInit(COLOR_BGRA_1280x720_30Hz);
	DK.depthInit(DEPTH_NFOV_640x576_30HZ);
	DK.Configuration();

	while (1)
	{
		high_resolution_clock::time_point t1 = high_resolution_clock::now();
		DK.updateFrame();

		DK.get_colorImage(colorimg, NOUSE);
		DK.get_depthImage(depthimg, NOUSE);
		
		
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
