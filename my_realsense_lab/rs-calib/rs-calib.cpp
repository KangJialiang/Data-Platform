#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

int imgWidth ;
int imgHeight ;
int cam_id ; // 0: infrared Left ; 1: infrared Right ; 2: color
int fps = 30;

int main()
{
    //read settings file
    char settings_buf[20];
    FILE *fp=fopen("/home/fwf/7788/camera_calibration_tool/settings.txt","r");
    if(fp==NULL)
    printf("can not real settings file: /home/fwf/7788/camera_calibration_tool/settings.txt!");

    fgets(settings_buf, 20, fp);
    sscanf(settings_buf,"%d %d", &imgWidth, &imgHeight);

    fgets(settings_buf, 20, fp);
    sscanf(settings_buf,"%d", &cam_id);

    // initialize realsense camera
    // contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    // create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, imgWidth, imgHeight, RS2_FORMAT_Y16, fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, imgWidth, imgHeight, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, imgWidth, imgHeight, RS2_FORMAT_Y8, fps);


    printf("%d %d %d\n",imgWidth,imgHeight,cam_id);

    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile selection = pipe.start(cfg);


    // disable emitter
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) 
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    rs2::frameset frames;
    rs2::frame frame;

    //select sensor
    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    rs2::sensor infrared_sensor = sensors[0];
    rs2::sensor color_sensor = sensors[1];
    rs2::sensor sensor_selected;

    if(cam_id == 2)
    {
        sensor_selected = color_sensor;
    }
    else if(cam_id == 0)
    {
        sensor_selected = infrared_sensor;
    }
    else if(cam_id == 1)
    {
        sensor_selected = infrared_sensor;
    }

    char ImgFile[50];
    int counter = 0;

    int reaction = -1;
    //adjust camera position to collect imgs.
    printf("Adjust you camera position and Press \" Enter \" on your keyboard to capture a image!\n");
    printf("Press \" ESC \" to exit after capture at least 10 images!\n");
    while(1)
    {
        sprintf(ImgFile,"/home/fwf/7788/camera_calibration_tool/chess/%05d.png",counter);
        frames = pipe.wait_for_frames();
        if(cam_id == 2)
        {
            frame = frames.get_color_frame();
            cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_16U, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
            //imshow
            cv::imshow ("image_viwer", frameMat);
            if(reaction == 10)
            {
                cv::imwrite(ImgFile,frameMat);
                counter += 1;
                printf("Image: %s has been written!\n",ImgFile);
            }        
        }
        else if(cam_id == 0)
        {
            frame = frames.get_infrared_frame(1);
            cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_8U, (void*)frame.get_data(),  cv::Mat::AUTO_STEP);
            //imshow
            cv::imshow ("image_viwer", frameMat);
            if(reaction == 10)
            {
                cv::imwrite(ImgFile,frameMat);
                counter += 1;
                printf("Image: %s has been written!\n",ImgFile);
            }      
        }
        else if(cam_id == 1)
        {
            frame = frames.get_infrared_frame(2);
            cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_8U, (void*)frame.get_data(),  cv::Mat::AUTO_STEP);
            //imshow
            cv::imshow ("image_viwer", frameMat);
            if(reaction == 10)
            {
                cv::imwrite(ImgFile,frameMat);
                counter += 1;
                printf("Image: %s has been written!\n",ImgFile);
            }       
        }
        fflush(stdout);
        reaction = cv::waitKey(20);
        if(reaction == 27) break;
    }
}