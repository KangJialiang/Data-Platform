#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

int imgWidth ;
int imgHeight ;
int cam_id ; // 0: infrared Left ; 1: infrared Right ; 2: color
int exposure_time; //infrared: 1 ~ 165000 usec(int)  color: 1 ~ 10000 usec(int) 
int fps; //Cannot be setted arbitrary. Must be sellected with certain number. See by realsense-viwer! 15, 30, 60, 90 are in commen.

void set_exposure_time(rs2::sensor sensor, int exposure_time)
{
    if (sensor.supports(RS2_OPTION_EXPOSURE))
    {
        auto range = sensor.get_option_range(RS2_OPTION_EXPOSURE); 
        if (exposure_time < range.min || exposure_time > range.max)
        {
            printf("ERROR: Invalid Exposure Time %d !(the range is %f ~ %f.)\n\
            Exposure Time stay unchanged.\n",exposure_time,range.min,range.max);
            set_exposure_time(sensor, exposure_time);
        }
        sensor.set_option(RS2_OPTION_EXPOSURE, exposure_time);        
    }
}

int main()
{
    //read settings file
    char settings_buf[20];
    FILE *fp=fopen("/home/fwf/my_dataset/realsense_vignette_calib/settings.txt","r");
    if(fp==NULL)
    printf("can not real settings file: /home/fwf/my_dataset/realsense_vignette_calib/settings.txt!");

    fgets(settings_buf, 20, fp);
    sscanf(settings_buf,"%d %d", &imgWidth, &imgHeight);

    fgets(settings_buf, 20, fp);
    sscanf(settings_buf,"%d", &cam_id);

    fgets(settings_buf, 20, fp);
    sscanf(settings_buf,"%d", &exposure_time);

    fgets(settings_buf, 20, fp);
    sscanf(settings_buf,"%d", &fps);

    char log_file[50];

    // initialize realsense camera
    // contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    // create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, imgWidth, imgHeight, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, imgWidth, imgHeight, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, imgWidth, imgHeight, RS2_FORMAT_Y8, fps);


    printf("%d %d %d %d %d",imgWidth,imgHeight,cam_id,exposure_time,fps);

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
        sprintf(log_file, "/home/fwf/my_dataset/realsense_vignette_calib/color/times.txt");
        sensor_selected = color_sensor;

    }
    else if(cam_id == 0)
    {
        sprintf(log_file, "/home/fwf/my_dataset/realsense_vignette_calib/infrared_left/times.txt");
        sensor_selected = infrared_sensor;
    }
    else if(cam_id == 1)
    {
        sprintf(log_file, "/home/fwf/my_dataset/realsense_vignette_calib/infrared_right/times.txt");
        sensor_selected = infrared_sensor;
    }
    

    //time log
    std::fstream time_log;
    time_log.open(log_file,std::ios::out|std::ios::trunc);

    rs2_time_t timestamp;
    int actual_time;

    //set and (check) exposure time
    set_exposure_time(sensor_selected, exposure_time);

    // frames = pipe.wait_for_frames();
    // if(cam_id = 2)
    //     frame = frames.get_color_frame();
    // else if (cam_id = 0)
    //     frame = frames.get_infrared_frame(1);
    // else if (cam_id = 1)
    //     frame = frames.get_infrared_frame(2);
    
    // actual_time = frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
    // while(exposure_time != actual_time)
    // {
    //     frames = pipe.wait_for_frames();
    //     if(cam_id = 2)
    //         frame = frames.get_color_frame();
    //     else if (cam_id = 0)
    //         frame = frames.get_infrared_frame(1);
    //     else if (cam_id = 1)
    //         frame = frames.get_infrared_frame(2);
        
    //     actual_time = frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
    // }

    int counter = 0;
    char ImgName[20], ImgFile[50], ImgId[20];

    //adjust camera position to collect imgs.
    printf("Adjust you camera position!\n\
            Press \"Space\" on your keyboard or just wait for 5s to start collect images!\n");
    for(int i = 0; i < fps*5; i++)
    {
        frames = pipe.wait_for_frames();
        if(cam_id == 2)
        {
            frame = frames.get_color_frame();
            cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
            //imshow
            cv::imshow ("image_viwer", frameMat);
            if(cv::waitKey(5) == 32)
                break;
        }
        else if(cam_id == 0)
        {
            frame = frames.get_infrared_frame(1);
            cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_8U, (void*)frame.get_data(),  cv::Mat::AUTO_STEP);
            //imshow
            cv::imshow ("image_viwer", frameMat);
            if(cv::waitKey(5) == 32)
                break;
        }
        else if(cam_id == 1)
        {
            frame = frames.get_infrared_frame(2);
            cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_8U, (void*)frame.get_data(),  cv::Mat::AUTO_STEP);
            //imshow
            cv::imshow ("image_viwer", frameMat);
            if(cv::waitKey(5) == 32)
                break;
        }


    }

    cv::destroyAllWindows();
    printf("Now start to collect images for vignette calibration!\n\
            You can stop manually by press \"Esc\" on you keyboard!\n");

    //collect images for vignette calib
    for(int i = 0; i < 800; i++)
    {
        sprintf(ImgName, "%05d%s", counter, ".png");
        //get each frame
        frames = pipe.wait_for_frames();
        if(cam_id == 2)
        {
            frame = frames.get_color_frame();
            cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
            sprintf(ImgFile, "/home/fwf/my_dataset/realsense_vignette_calib/%s/images/%s", "color", ImgName);
            cv::imwrite(ImgFile, frameMat);

            sprintf(ImgId, "%05d ", counter);
            time_log << ImgId << frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)  << " " << frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)/1000.0 <<"\n";

            //imshow
            cv::imshow ("image_collecter", frameMat);
            if(cv::waitKey(5) == 27)
                break;       
        }
        else if (cam_id == 0)
        {
            frame = frames.get_infrared_frame(1);
            cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_8U, (void*)frame.get_data(),  cv::Mat::AUTO_STEP);
            sprintf(ImgFile, "/home/fwf/my_dataset/realsense_vignette_calib/%s/images/%s", "infrared_left", ImgName);
            cv::imwrite(ImgFile, frameMat);

            sprintf(ImgId, "%05d ", counter);
            time_log << ImgId << frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)  << " " << frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)/1000.0 <<"\n";

            //imshow
            cv::imshow ("image_collecter", frameMat);
            if(cv::waitKey(5) == 27)
                break;       
        }
        else if (cam_id == 1)
        {
            frame = frames.get_infrared_frame(2);
            cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_8U, (void*)frame.get_data(),  cv::Mat::AUTO_STEP);
            sprintf(ImgFile, "/home/fwf/my_dataset/realsense_vignette_calib/%s/images/%s", "infrared_right", ImgName);
            cv::imwrite(ImgFile, frameMat);

            sprintf(ImgId, "%05d ", counter);
            time_log << ImgId << frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)  << " " << frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)/1000.0 <<"\n";

            //imshow
            cv::imshow ("image_collecter", frameMat);
            if(cv::waitKey(5) == 27)
                break;       
        } 
        counter++;
    }

    time_log.flush();
    time_log.close();

}
