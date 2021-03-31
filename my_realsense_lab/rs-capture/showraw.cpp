#include<iostream> 
#include<opencv2/opencv.hpp>
using namespace std; 
using namespace cv; 

int main()
{ 
    int IMAGE_WIDTH = 1280, IMAGE_HEIGHT = 720; 
    FILE* fp; 
    char* imagedata; 
    int framesize = IMAGE_WIDTH*IMAGE_HEIGHT; 
    fp = fopen("/home/fwf/my_dataset/gongzhong_5f/depth/00000.raw", "rb"); 
    imagedata = (char*)malloc(sizeof(short)* framesize); 
    fread(imagedata, sizeof(short), framesize, fp); 
    cv::Mat depthMat(Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_16U, (void*)imagedata, cv::Mat::AUTO_STEP); 
    printf("Depth RAW Central Value: %d mm", depthMat.at<short>(IMAGE_HEIGHT / 2, IMAGE_WIDTH / 2)); 
    fclose(fp); 
    free(imagedata); 
    return 0; 
}