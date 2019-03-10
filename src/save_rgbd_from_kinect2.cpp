/**
 * 
 * 函数功能：采集iaikinect2输出的彩色图和深度图数据，并以文件的形式进行存储
 * 
 * 
 * 分隔符为　逗号'，'　　
 * 时间戳单位为秒(s)　精确到小数点后６位(us)
 * 
 * maker:crp
 * 2017-5-13
 */

#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>//将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include<sensor_msgs/image_encodings.h>//头文件sensor_msgs/Image是ROS下的图像的类型，这个头文件中包含对图像进行编码的函数

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include<iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;

Mat rgb, depth;
char successed_flag1 =0,successed_flag2=0;

string topic1_name = "/kinect2/qhd/image_color"; //topic 名称
string topic2_name = "/kinect2/qhd/image_depth_rect";

string filename_rgbdata="/home/crp/recordData/RGBD/rgbdata.txt";
string filename_depthdata="/home/crp/recordData/RGBD/depthdata.txt";
string save_imagedata = "/home/crp/recordData/RGBD/";

bool display_IMU5211( unsigned char buf[21] ,timeval time_stamp,string &out_result);
void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);
void  callback_function_color( const sensor_msgs::Image::ConstPtr  image_data);
void  callback_function_depth( const sensor_msgs::Image::ConstPtr  image_data);
int main(int argc,char** argv)
{
    string out_result;

    namedWindow("image color",CV_WINDOW_AUTOSIZE);
    namedWindow("image depth",CV_WINDOW_AUTOSIZE);
    ros::init(argc,argv,"kinect2_listen");
    if(!ros::ok())
             return 0;
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe(topic1_name,50,callback_function_color);
    ros::Subscriber sub2 = n.subscribe(topic2_name,50,callback_function_depth);
    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();

   string rgb_str, dep_str;

   struct timeval time_val;
   struct timezone tz;
   double time_stamp;

    ofstream fout_rgb(filename_rgbdata.c_str());
    if(!fout_rgb)
    {
            cerr<<filename_rgbdata<<" file not exist"<<endl;
    }

    ofstream fout_depth(filename_depthdata.c_str());
    if(!fout_depth)
    {
         cerr<<filename_depthdata<<" file not exist"<<endl;
    }

    while(ros::ok())
    {

             if( successed_flag1 )
            {
                 gettimeofday(&time_val,&tz);//us
				//  time_stamp =time_val.tv_sec+ time_val.tv_usec/1000000.0;
                 ostringstream os_rgb;
                 os_rgb<<time_val.tv_sec<<"."<<time_val.tv_usec;
                 rgb_str = save_imagedata+"rgb/"+os_rgb.str()+".png";
                  imwrite(rgb_str,rgb);
                  fout_rgb<<os_rgb.str()<<",rgb/"<<os_rgb.str()<<".png\n";
                  successed_flag1=0;
				  imshow("image color",rgb);
				  cout<<"rgb -- time:  " <<  time_val.tv_sec<<"."<<time_val.tv_usec<<endl;
				   waitKey(1);

            }
            
            if( successed_flag2 )
            {    
                gettimeofday(&time_val,&tz);//us
                ostringstream os_dep;
                os_dep<<time_val.tv_sec<<"."<<time_val.tv_usec;
                dep_str =save_imagedata+"depth/"+os_dep.str()+".png";// 输出图像目录
                imwrite(dep_str,depth);
                fout_depth<<os_dep.str()<<",depth/"<<os_dep.str()<<".png\n";
				successed_flag2=0;
				//   imshow("image depth",depth);
				  cout<<"depth -- time:" <<  time_val.tv_sec<<"."<<time_val.tv_usec<<endl;

             }
           
			
    }

    ros::waitForShutdown();
    ros::shutdown();

    return 0;

}
void  callback_function_color(const sensor_msgs::Image::ConstPtr  image_data)
{    
   cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
    //   cout<<"the frame_id:"<<image_data->frame_id.c_str()<<endl;
//    cout<<"the image heigh"<<image_data->height<<endl;
//    cout<<"the image width"<<image_data->width<<endl;
//    cout<<"the image step"<<image_data->step<<endl;
//    cout<<"listen ...."<<endl;

   pCvImage = cv_bridge::toCvShare(image_data, image_data->encoding);//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
   pCvImage->image.copyTo(rgb);
    successed_flag1 = 1;

}
void  callback_function_depth(const sensor_msgs::Image::ConstPtr  image_data)
{
    Mat temp;
   cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
   pCvImage = cv_bridge::toCvShare(image_data, image_data->encoding);//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
   pCvImage->image.copyTo(depth);


//   cout<<"the  depth heigh"<<image_data->height<<endl;
//   cout<<"the depth width"<<image_data->width<<endl;
//   cout<<"the depth step"<<image_data->step<<endl;
//   cout<<"listen ...."<<endl;

   //dispDepth(temp, depth, 12000.0f);
   successed_flag2=1;
   //imshow("Mat depth",depth/256);
   //cv::waitKey(1);



}
void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
{
          cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
          const uint32_t maxInt = 255;

          #pragma omp parallel for
          for(int r = 0; r < in.rows; ++r)
          {
                const uint16_t *itI = in.ptr<uint16_t>(r);
                uint8_t *itO = tmp.ptr<uint8_t>(r);

                for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
                {
                  *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
                }
          }

          cv::applyColorMap(tmp, out, COLORMAP_JET);
}
