#include"DK.h"
#include "util.h"

KinectDK::KinectDK()
{
	Flg_InitColor = false;
	Flg_InitDepth = false;
	Flg_Color_to_Depth = false;
	Flg_Depth_to_Color = false;
	device = NULL;
	k4a_device_open(0, &device); // 打开设备
 
}

KinectDK::~KinectDK()
{
	k4a_device_stop_cameras(device); // 关闭相机
	k4a_device_close(device); // 关闭设备
}


void KinectDK::colorInit(DKColorType ColorType)
{
	// 获取控制格式
	ProduceColorCfg(ColorType, _ColorCfg);

	Flg_InitColor = true;
}
void KinectDK::depthInit(DKDepthType DepthType)
{
	ProduceDepthCfg(DepthType, _DepthCfg);

	Flg_InitDepth = true;
}

void KinectDK::Configuration()
{	
	/*仅打开彩色*/
	if (Flg_InitColor == true&& Flg_InitDepth == false)
	{
		cfg.camera_fps = _ColorCfg.fps;
		cfg.color_format = _ColorCfg.format;
		cfg.color_resolution = _ColorCfg.resolution;
		k4a_device_start_cameras(device, &cfg);
	}
	/*仅打开深度*/
	if (Flg_InitColor == false && Flg_InitDepth == true)
	{
		cfg.camera_fps = _DepthCfg.fps;
		cfg.depth_mode = _DepthCfg.mode;
		k4a_device_start_cameras(device, &cfg);

        k4a_device_get_calibration(device, cfg.depth_mode, cfg.color_resolution, &calibration);
        transformation = k4a_transformation_create(&calibration);

        pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_DEPTH);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
            pinhole.width,
            pinhole.height,
            pinhole.width * (int)sizeof(coordinate_t),
            &lut);
        create_undistortion_lut(&calibration, K4A_CALIBRATION_TYPE_DEPTH, &pinhole, lut, interpolation_type);

        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            pinhole.width,
            pinhole.height,
            pinhole.width * (int)sizeof(uint16_t),
            &undistorted);
	}
	/*全部打开*/
	if (Flg_InitColor == true && Flg_InitDepth == true)
	{
       
		cfg.camera_fps = _ColorCfg.fps;
		cfg.color_format = _ColorCfg.format;
		cfg.color_resolution = _ColorCfg.resolution;
		cfg.depth_mode = _DepthCfg.mode;
		cfg.synchronized_images_only = true;//确保深度和彩色图像都是可用的捕获
		k4a_device_start_cameras(device, &cfg);
       
		k4a_device_get_calibration(device, cfg.depth_mode, cfg.color_resolution, &calibration);
		transformation = k4a_transformation_create(&calibration);

        pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_DEPTH);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                         pinhole.width,
                         pinhole.height,
                         pinhole.width * (int)sizeof(coordinate_t),
                         &lut);
        create_undistortion_lut(&calibration, K4A_CALIBRATION_TYPE_DEPTH, &pinhole, lut, interpolation_type);

        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                         pinhole.width,
                         pinhole.height,
                         pinhole.width * (int)sizeof(uint16_t),
                         &undistorted);
       
	}	
}

void KinectDK::updateFrame()
{	
	get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE); // 读取传感器捕获，K4A_WAIT_INFINITE：一直等待捕获数据
}

void KinectDK::get_colorImage(Mat& colorImage, Align Align)
{
	if (Align == NOUSE)
	{		
		rbgimage = k4a_capture_get_color_image(sensor_capture); // 获取RGB图像
         /*用这种转换只可以读取BGRA*/
		//colorImage = Mat(k4a_image_get_height_pixels(rbgimage), k4a_image_get_width_pixels(rbgimage), CV_8UC4, k4a_image_get_buffer(rbgimage)); 
         /*这个是支持任何格式的*/
        colorImage = k4a_get_mat(rbgimage);

		k4a_image_release(rbgimage); // 释放图像句柄
	}
	if (Align == COLOR_ALIGN_TO_DEPTH)
	{
		Flg_Color_to_Depth = true;

		rbgimage = k4a_capture_get_color_image(sensor_capture); // 获取RGB图像

		k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
			k4a_image_get_width_pixels(depthimage),
			k4a_image_get_height_pixels(depthimage),
			k4a_image_get_width_pixels(depthimage) * 4 * (int)sizeof(uint8_t),
			&transformed_color_image);

		k4a_transformation_color_image_to_depth_camera(transformation,
            depthimage,
			rbgimage,
			transformed_color_image);

		colorImage = Mat(k4a_image_get_height_pixels(transformed_color_image),
			k4a_image_get_width_pixels(transformed_color_image),
			CV_8UC4,
			k4a_image_get_buffer(transformed_color_image));
		
		k4a_image_release(rbgimage); // 释放图像句柄
       
	}
}

void KinectDK::get_depthImage(Mat& depthImage, Align Align)
{
	if (Align == NOUSE)
	{
		depthimage = k4a_capture_get_depth_image(sensor_capture); // 获取depth图像
        //remap(depthimage, lut, undistorted, interpolation_type);  //单独获取深度图时候可以用来矫正

		depthImage = Mat(k4a_image_get_height_pixels(depthimage),
			             k4a_image_get_width_pixels(depthimage),
			             CV_16U, 
			             k4a_image_get_buffer(depthimage));

		k4a_image_release(depthimage);
	}	
	if (Align == DEPTH_ALIGN_TO_COLOR)
	{
		Flg_Depth_to_Color = true;

		depthimage = k4a_capture_get_depth_image(sensor_capture); // 获取depth图像
        //remap(depthimage, lut, undistorted, interpolation_type);

		k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
			k4a_image_get_width_pixels(rbgimage),
			k4a_image_get_height_pixels(rbgimage),
			k4a_image_get_width_pixels(rbgimage) * sizeof(uint16_t),
			&transformed_depth_image);

		k4a_transformation_depth_image_to_color_camera(transformation,
            depthimage,
			transformed_depth_image);

		depthImage = Mat(k4a_image_get_height_pixels(transformed_depth_image),
			k4a_image_get_width_pixels(transformed_depth_image),
			CV_16U,
			k4a_image_get_buffer(transformed_depth_image));


		k4a_image_release(depthimage);
	}	
}

void KinectDK::get_irImage(Mat& irImage)
{
    irimage = k4a_capture_get_ir_image(sensor_capture); // 获取depth图像
        
    irImage = Mat(k4a_image_get_height_pixels(irimage),
        k4a_image_get_width_pixels(irimage),
        CV_16U,
        k4a_image_get_buffer(irimage));

    k4a_image_release(irimage);
}

void KinectDK::point2D_to_Point3D(k4a_float2_t point2D,int distance, k4a_float3_t& point3D)
{
     
    k4a_calibration_2d_to_3d(&calibration,
        &point2D, 
        distance, 
        K4A_CALIBRATION_TYPE_COLOR,
        K4A_CALIBRATION_TYPE_COLOR,
        &point3D,
        &valid);
    
    if (valid == 0)
    {
        cout << "wan dan le" << endl;
    }

}

void KinectDK::imu_start()
{
    k4a_device_start_imu(device);
}
void KinectDK::get_imu(k4a_imu_sample_t& imu_sample)
{
   
    k4a_device_get_imu_sample(device, &imu_sample, 0);

}

void KinectDK::Release()
{
	k4a_capture_release(sensor_capture);
      
	if (Flg_Color_to_Depth == true)
	{
		k4a_image_release(transformed_color_image);
	}
	if (Flg_Depth_to_Color == true)
	{
		k4a_image_release(transformed_depth_image);
	}
}



void KinectDK::ProduceColorCfg(DKColorType Colortype, ColorCfg& _ColorCfg)
{
	switch (Colortype)
	{
	case COLOR_BGRA_1280x720_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_BGRA32, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_720P; break;
	case COLOR_BGRA_1920x1080_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_BGRA32, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_1080P; break;
	case COLOR_BGRA_2560x1440_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_BGRA32, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_1440P; break;
	case COLOR_BGRA_3840x2160_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_BGRA32, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_2160P; break;
	case COLOR_BGRA_2048x1536_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_BGRA32, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_1536P; break;
	case COLOR_BGRA_4096x3072_15Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_BGRA32, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_15, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_3072P; break;
	
/************************************************************************************/
	case COLOR_MJPG_1280x720_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_MJPG, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_720P; break;
	case COLOR_MJPG_1920x1080_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_MJPG, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_1080P; break;
	case COLOR_MJPG_2560x1440_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_MJPG, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_1440P; break;
	case COLOR_MJPG_3840x2160_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_MJPG, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_2160P; break;
	case COLOR_MJPG_2048x1536_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_MJPG, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_1536P; break;
	case COLOR_MJPG_4096x3072_15Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_MJPG, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_15, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_3072P; break;
	
/************************************************************************************/
	case COLOR_NV12_1280x720_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_NV12, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_720P; break;
	
/************************************************************************************/
	case COLOR_YUY2_1280x720_30Hz:_ColorCfg.format = K4A_IMAGE_FORMAT_COLOR_YUY2, _ColorCfg.fps = K4A_FRAMES_PER_SECOND_30, _ColorCfg.resolution = K4A_COLOR_RESOLUTION_2160P; break;
	
	}
}
void KinectDK::ProduceDepthCfg(DKDepthType DepthType, DepthCfg& _DepthCfg)
{
	switch (DepthType)
	{
	case DEPTH_NFOV_320x288_30HZ:_DepthCfg.mode = K4A_DEPTH_MODE_NFOV_2X2BINNED, _DepthCfg.fps = K4A_FRAMES_PER_SECOND_30; break;
	case DEPTH_NFOV_640x576_30HZ:_DepthCfg.mode = K4A_DEPTH_MODE_NFOV_UNBINNED, _DepthCfg.fps = K4A_FRAMES_PER_SECOND_30; break;
	case DEPTH_WFOV_512x512_30HZ:_DepthCfg.mode = K4A_DEPTH_MODE_WFOV_2X2BINNED, _DepthCfg.fps = K4A_FRAMES_PER_SECOND_30; break;
	case DEPTH_WFOV_1024x1024_15HZ:_DepthCfg.mode = K4A_DEPTH_MODE_WFOV_UNBINNED, _DepthCfg.fps = K4A_FRAMES_PER_SECOND_15; break;
	
	}
}




// 在所有点都有有效投影的单位平面上计算一个保守的边界框
extern void compute_xy_range(const k4a_calibration_t* calibration,
                             const k4a_calibration_type_t camera,
                             const int width,
                             const int height,
                             float& x_min,
                             float& x_max,
                             float& y_min,
                             float& y_max)
{
    // Step outward from the centre point until we find the bounds of valid projection
    const float step_u = 0.25f;
    const float step_v = 0.25f;
    const float min_u = 0;
    const float min_v = 0;
    const float max_u = (float)width - 1;
    const float max_v = (float)height - 1;
    const float center_u = 0.5f * width;
    const float center_v = 0.5f * height;

    int valid;
    k4a_float2_t p;
    k4a_float3_t ray;

    // search x_min
    for (float uv[2] = { center_u, center_v }; uv[0] >= min_u; uv[0] -= step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_min = ray.xyz.x;
    }

    // search x_max
    for (float uv[2] = { center_u, center_v }; uv[0] <= max_u; uv[0] += step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_max = ray.xyz.x;
    }

    // search y_min
    for (float uv[2] = { center_u, center_v }; uv[1] >= min_v; uv[1] -= step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_min = ray.xyz.y;
    }

    // search y_max
    for (float uv[2] = { center_u, center_v }; uv[1] <= max_v; uv[1] += step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_max = ray.xyz.y;
    }
}

extern pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t* calibration, const k4a_calibration_type_t camera)
{
    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        width = calibration->color_camera_calibration.resolution_width;
        height = calibration->color_camera_calibration.resolution_height;
    }

    float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
    compute_xy_range(calibration, camera, width, height, x_min, x_max, y_min, y_max);

    pinhole_t pinhole;

    float fx = 1.f / (x_max - x_min);
    float fy = 1.f / (y_max - y_min);
    float px = -x_min * fx;
    float py = -y_min * fy;

    pinhole.fx = fx * width;
    pinhole.fy = fy * height;
    pinhole.px = px * width;
    pinhole.py = py * height;
    pinhole.width = width;
    pinhole.height = height;

    return pinhole;
}

extern void create_undistortion_lut(const k4a_calibration_t* calibration,
                                    const k4a_calibration_type_t camera,
                                    const pinhole_t* pinhole,
                                    k4a_image_t lut,
                                    interpolation_t type)

{
    coordinate_t* lut_data = (coordinate_t*)(void*)k4a_image_get_buffer(lut);

    k4a_float3_t ray;
    ray.xyz.z = 1.f;

    int src_width = calibration->depth_camera_calibration.resolution_width;
    int src_height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        src_width = calibration->color_camera_calibration.resolution_width;
        src_height = calibration->color_camera_calibration.resolution_height;
    }

    for (int y = 0, idx = 0; y < pinhole->height; y++)
    {
        ray.xyz.y = ((float)y - pinhole->py) / pinhole->fy;

        for (int x = 0; x < pinhole->width; x++, idx++)
        {
            ray.xyz.x = ((float)x - pinhole->px) / pinhole->fx;

            k4a_float2_t distorted;
            int valid;
            k4a_calibration_3d_to_2d(calibration, &ray, camera, camera, &distorted, &valid);

            coordinate_t src;
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                // Remapping via nearest neighbor interpolation
                src.x = (int)floorf(distorted.xy.x + 0.5f);
                src.y = (int)floorf(distorted.xy.y + 0.5f);
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                // Remapping via bilinear interpolation
                src.x = (int)floorf(distorted.xy.x);
                src.y = (int)floorf(distorted.xy.y);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }

            if (valid && src.x >= 0 && src.x < src_width && src.y >= 0 && src.y < src_height)
            {
                lut_data[idx] = src;

                if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // Compute the floating point weights, using the distance from projected point src to the
                    // image coordinate of the upper left neighbor
                    float w_x = distorted.xy.x - src.x;
                    float w_y = distorted.xy.y - src.y;
                    float w0 = (1.f - w_x) * (1.f - w_y);
                    float w1 = w_x * (1.f - w_y);
                    float w2 = (1.f - w_x) * w_y;
                    float w3 = w_x * w_y;

                    // Fill into lut
                    lut_data[idx].weight[0] = w0;
                    lut_data[idx].weight[1] = w1;
                    lut_data[idx].weight[2] = w2;
                    lut_data[idx].weight[3] = w3;
                }
            }
            else
            {
                lut_data[idx].x = INVALID;
                lut_data[idx].y = INVALID;
            }
        }
    }
}

extern void remap(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type)
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint16_t* src_data = (uint16_t*)(void*)k4a_image_get_buffer(src);
    uint16_t* dst_data = (uint16_t*)(void*)k4a_image_get_buffer(dst);
    coordinate_t* lut_data = (coordinate_t*)(void*)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint16_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                const uint16_t neighbors[4]{ src_data[lut_data[i].y * src_width + lut_data[i].x],
                                             src_data[lut_data[i].y * src_width + lut_data[i].x + 1],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x + 1] };

                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR
                    if (neighbors[0] == 0 || neighbors[1] == 0 || neighbors[2] == 0 || neighbors[3] == 0)
                    {
                        continue;
                    }

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = std::min(std::min(neighbors[0], neighbors[1]),
                        std::min(neighbors[2], neighbors[3]));
                    float depth_max = std::max(std::max(neighbors[0], neighbors[1]),
                        std::max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }
                }

                dst_data[i] = (uint16_t)(neighbors[0] * lut_data[i].weight[0] + neighbors[1] * lut_data[i].weight[1] +
                    neighbors[2] * lut_data[i].weight[2] + neighbors[3] * lut_data[i].weight[3] +
                    0.5f);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}