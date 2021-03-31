#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include<math.h>


int imgWidth ;
int imgHeight ;

//suitable exposure times for color anf infrared cameras
//for realsense D455:
//color: 1 ~ 10000 usec(int)
//infrared: 1 ~ 165000 usec(int)
double infrared_min, infrared_max, color_min, color_max;
int exposure_mun = 150, img_num=8;


void set_exposure_time(rs2::sensor sensor, int exposure_time)
{
    if (sensor.supports(RS2_OPTION_EXPOSURE))
    {
        auto range = sensor.get_option_range(RS2_OPTION_EXPOSURE); 
        if (exposure_time < range.min || exposure_time > range.max)
        {
            printf("ERROR: Invalid Exposure Time %d !(the range is %f ~ %f.)\nExposure Time stay unchanged.",exposure_time,range.min,range.max);
            return;
        }
        sensor.set_option(RS2_OPTION_EXPOSURE, exposure_time);
        return;        
    }
}




int main()
{
    //read settings file
    char settings_buf[20];
    FILE *fp=fopen("/home/fwf/my_dataset/realsense_gamma_calib/settings.txt","r");
    if(fp==NULL)
        printf("can not real settings file: /home/fwf/my_dataset/realsense_gamma_calib/settings.txt!");

    fgets(settings_buf, 20, fp);
    sscanf(settings_buf,"%d %d",&imgWidth, &imgHeight);

    fgets(settings_buf, 20, fp);
    sscanf(settings_buf,"%lf %lf",&infrared_min, &infrared_max);

    fgets(settings_buf, 20, fp);
    sscanf(settings_buf,"%lf %lf",&color_min, &color_max);

    // initialize realsense camera
    // contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    // create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, imgWidth, imgHeight, RS2_FORMAT_Y16, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, imgWidth, imgHeight, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, imgWidth, imgHeight, RS2_FORMAT_Y8, 30);

    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile selection = pipe.start(cfg);

    // disable emitter
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) 
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    rs2::frameset frames;
    rs2::frame frame;
    rs2::frame frameLeft;
    rs2::frame frameRight;

    //select sensor
    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    rs2::sensor infrared_sensor = sensors[0];
    rs2::sensor color_sensor = sensors[1];
    

    //image counter
    int counter = 0;

    //time log
    std::fstream color_time, left_time, right_time;
    color_time.open("/home/fwf/my_dataset/realsense_gamma_calib/color/times.txt",std::ios::out|std::ios::trunc);
    left_time.open("/home/fwf/my_dataset/realsense_gamma_calib/infrared_left/times.txt",std::ios::out|std::ios::trunc);
    right_time.open("/home/fwf/my_dataset/realsense_gamma_calib/infrared_right/times.txt",std::ios::out|std::ios::trunc);

    int color_exposure_time, infraredL_exposure_time, infraredR_exposure_time;
    int t_infraredL, t_infraredR, t_color;
    double gap_infrared = pow(infrared_max/infrared_min, 1.0/exposure_mun), gap_color = pow(color_max/color_min, 1.0/exposure_mun);

    //collect images for gamma calib
    for(double t_infrared_d = infrared_min, t_color_d = color_min; counter < exposure_mun*img_num; t_infrared_d*=gap_infrared,t_color_d*=gap_color)
    {
        t_infraredL = t_infrared_d;
        t_infraredR = t_infrared_d;
        t_color = t_color_d;
        set_exposure_time(infrared_sensor,t_infraredL);
        set_exposure_time(color_sensor,t_color);

        //get each frame
        frames = pipe.wait_for_frames();
        frame = frames.get_color_frame();
        frameLeft  = frames.get_infrared_frame(1);
        frameRight = frames.get_infrared_frame(2);
        
        //check exposure_time has been changed
        infraredL_exposure_time = frameLeft.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
        infraredR_exposure_time = frameLeft.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
        color_exposure_time = frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
        while(color_exposure_time != t_color || infraredL_exposure_time != t_infraredL || infraredR_exposure_time != t_infraredR)
        {
            //printf("color exposure time: %d != %d infrared exposure time: %d != %d\n",color_exposure_time,t_color,t_infrared,infrared_exposure_time,t);
            frames = pipe.wait_for_frames();
            frame = frames.get_color_frame();
            frameLeft  = frames.get_infrared_frame(1);
            frameRight = frames.get_infrared_frame(2);
            infraredR_exposure_time = frameLeft.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            infraredL_exposure_time = frameLeft.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            color_exposure_time = frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
        }
        //for each exposure time, we will save 10 images
        for(int i = 0; i < img_num; i++)
        {
            // Creating OpenCV Matrix from a color image
            cv::Mat color(cv::Size(imgWidth, imgHeight), CV_16U, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat matLeft (cv::Size(imgWidth, imgHeight), CV_8U, (void*)frameLeft.get_data(),  cv::Mat::AUTO_STEP);
            cv::Mat matRight(cv::Size(imgWidth, imgHeight), CV_8U, (void*)frameRight.get_data(), cv::Mat::AUTO_STEP);
            //save images
            char nameImg[20], nameColor[100], nameLeft[100], nameRight[100];
            sprintf(nameImg, "%05d%s", counter, ".png");
            sprintf(nameColor, "/home/fwf/my_dataset/realsense_gamma_calib/%s/images/%s", "color", nameImg);
            sprintf(nameLeft,  "/home/fwf/my_dataset/realsense_gamma_calib/%s/images/%s", "infrared_left", nameImg);
            sprintf(nameRight, "/home/fwf/my_dataset/realsense_gamma_calib/%s/images/%s", "infrared_right", nameImg);

            cv::imwrite(nameColor, color);
            cv::imwrite(nameLeft,  matLeft);
            cv::imwrite(nameRight, matRight);

            //write time log 
            char ImgId[20];
            sprintf(ImgId, "%05d ", counter);
            color_time << ImgId << frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) << " " << t_color/1000.0 <<"\n";
            left_time << ImgId << frameLeft.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) << " " << t_infraredL/1000.0 <<"\n";
            right_time << ImgId << frameRight.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) << " " << t_infraredR/1000.0 <<"\n";

            frames = pipe.wait_for_frames();
            frame = frames.get_color_frame();
            frameLeft  = frames.get_infrared_frame(1);
            frameRight = frames.get_infrared_frame(2);
            counter++;
            if(counter % 50 ==0)
                printf("%d\n",counter);
        }
    }
    color_time.close();
    left_time.close();
    right_time.close();
    return 0;
}