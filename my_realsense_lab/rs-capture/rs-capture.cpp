#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

int imgWidth = 1280 ;
int imgHeight = 720 ;

int laser_option = 0;
int set_ae = 1500;

void parseArgument(char* arg)
{
    int option;
    if(sscanf(arg,"laser=%d",&option)==1)
    {
        if(option==1)
        {
            laser_option = 1;
            printf("I will trun on the laser!\n");
        }
        if(option==2)
        {
            laser_option = 2;
            printf("Laser auto controll!");
        }

        return;
    }

    if(sscanf(arg,"set_ae=%d",&option)==1)
    {
        set_ae = option;
        printf("I will set ae_setpoints to %d",set_ae);
        return;
    }

}



int main(int argc, char** argv)
{
	for(int i=1; i<argc;i++)
		parseArgument(argv[i]);

    // initialize realsense camera
    // contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    // create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;


    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_DEPTH, imgWidth, imgHeight, RS2_FORMAT_Z16, 15);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, imgWidth, imgHeight, RS2_FORMAT_Y8, 15);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, imgWidth, imgHeight, RS2_FORMAT_Y8, 15);

    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile selection = pipe.start(cfg);

    // disable emitter
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    if (laser_option==0) 
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
    else if(laser_option==1)
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
    else if(laser_option==2)
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 2);

    rs400::advanced_mode advnc_mode(selected_device);
    STAEControl ae_ctrl;
    ae_ctrl.meanIntensitySetPoint = set_ae;
    advnc_mode.set_ae_control(ae_ctrl);

    rs2::frameset frames;
    rs2::frame frameLeft;
    rs2::frame frameRight;

    //select sensor
    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    rs2::sensor infrared_sensor = sensors[0];

    infrared_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
    


    //time log
    std::fstream log_time;
    log_time.open("/home/fwf/my_dataset/gongzhong_5f/times.txt",std::ios::out|std::ios::trunc);

    //camera warm up
    for(int i=0; i<30; i++)
    {
        //get each frame
        frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        frameLeft  = frames.get_infrared_frame(1);
        frameRight = frames.get_infrared_frame(2);
    }
    int counter = 0;
    while(1)
    {
        //get each frame
        frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        frameLeft  = frames.get_infrared_frame(1);
        frameRight = frames.get_infrared_frame(2);

        // Creating OpenCV Matrix from a color image
        cv::Mat matLeft (cv::Size(imgWidth, imgHeight), CV_8U, (void*)frameLeft.get_data(),  cv::Mat::AUTO_STEP);
        cv::Mat matRight(cv::Size(imgWidth, imgHeight), CV_8U, (void*)frameRight.get_data(), cv::Mat::AUTO_STEP);
        //save images
        char nameDepth[100], nameLeft[100], nameRight[100];
        sprintf(nameLeft,  "/home/fwf/my_dataset/gongzhong_5f/image_0/%05d.png", counter);
        sprintf(nameRight, "/home/fwf/my_dataset/gongzhong_5f/image_1/%05d.png", counter);
        sprintf(nameDepth, "/home/fwf/my_dataset/gongzhong_5f/depth/%05d.raw",counter);


        cv::imwrite(nameLeft,  matLeft);
        cv::imwrite(nameRight, matRight);

        FILE* fp=fopen(nameDepth,"wb");
        fwrite(depth.get_data(),sizeof(short),imgWidth*imgHeight,fp);
        fflush(fp);
        fclose(fp);

        //write time log 
        char ImgId[20];
        sprintf(ImgId, "%05d ", counter);
        log_time << ImgId << " " <<frameLeft.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) << " " << frameLeft.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)*0.001 << " "<< frameLeft.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL) <<" "\
        << frameRight.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) << " " << frameRight.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)*0.001 << " " <<frameRight.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL)<<"\n";


        log_time.flush();
        counter += 1;
        printf("%d\n",counter);
        cv::imshow("imgLeft",matLeft);
        if(cv::waitKey(5)==27) break;
    }
    
    log_time.close();
}

