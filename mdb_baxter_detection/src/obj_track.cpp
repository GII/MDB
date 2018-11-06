//| This file is a part of the CAFER framework developped within
//| the DREAM project (http://www.robotsthatdream.eu/).
//|
//| Copyright 2018, GII / Universidad de la Coruna (UDC)
//| Main contributor(s): 
//|   * Luis Calvo, luis.calvo@udc.es
//|
//| This file is also part of MDB.
//| 
//| * MDB is free software: you can redistribute it and/or modify it under the
//| * terms of the GNU Affero General Public License as published by the Free
//| * Software Foundation, either version 3 of the License, or (at your option) any
//| * later version.
//| *
//| * MDB is distributed in the hope that it will be useful, but WITHOUT ANY
//| * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
//| * A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
//| * details.
//| *
//| * You should have received a copy of the GNU Affero General Public License
//| * along with MDB. If not, see <http://www.gnu.org/licenses/>.

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/impl/duration.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Bool.h"
#include <cstdint>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h> 
#include <random>

#include "mdb_common/ObjDet.h"
#include "mdb_common/MObjDet.h"
//#include <mdb_common/TabSeg.h>

#include "std_msgs/Bool.h"

using namespace cv;
using namespace std;

class obj_track{
	private:
		cv::Mat blur;
		cv::Mat hsv;
		cv::Mat mask;

	public:
		ros::NodeHandle nh;
		ros::Subscriber color_sub;
		ros::Publisher track_pub;
		ros::Publisher coor_pub;
		ros::Publisher det_pub;

		cv::Scalar low_thres; 
		cv::Scalar high_thres;

		float table_per;

	    ~obj_track(){}

		void initPublishers(std::string obj){
			track_pub = nh.advertise<sensor_msgs::Image>("/tracking/"+obj+"_img", 1);
			coor_pub = nh.advertise<mdb_common::ObjDet>("/tracking/"+obj, 1);
			det_pub = nh.advertise<std_msgs::Bool>("/tracking/"+obj+"_flag", 1);
		}

		cv_bridge::CvImagePtr ros2cv (const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr cv_ptr;
			try {
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			} catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
			return cv_ptr;
		}

		void obtainMask (cv::Mat image, cv::Scalar rangelow, cv::Scalar rangehigh){
			GaussianBlur(image, blur, cv::Size(11,11), 0);
		    cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
			inRange(hsv, rangelow, rangehigh, mask);
			//ROS_INFO_STREAM("mask type: " << mask.type());	
			erode(mask, mask, Mat(), Point(-1, -1), 2, 1, 1);
			dilate(mask, mask, Mat(), Point(-1, -1), 2, 1, 1);
		}

		cv::Mat segment_table (cv_bridge::CvImagePtr cimg_ptr, int low_t, int high_t){
			Mat auxImg;
			cvtColor(cimg_ptr->image, auxImg, cv::COLOR_BGR2GRAY);
			threshold(auxImg, auxImg, low_t, high_t, cv::THRESH_BINARY);

			std::vector<std::vector<Point>> contours2;
			std::vector<Vec4i> hierarchy2;
			findContours(auxImg, contours2, hierarchy2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
			sort(contours2.begin(),contours2.end(),[](const vector<Point> &a, const vector<Point> &b){return contourArea(a) > contourArea(b);});

			Mat table_mask = Mat::zeros(auxImg.rows, auxImg.cols, CV_8UC1);
			drawContours(table_mask, contours2, 0, Scalar(255), CV_FILLED);
			cimg_ptr->image.copyTo(auxImg, table_mask);				

			return auxImg;
		}		

		void track_object (cv_bridge::CvImagePtr cimg_ptr){
			bool sense;

			if (nh.hasParam("/baxter_sense") and nh.getParam("/baxter_sense", sense)){
				ROS_INFO_STREAM(sense);
				if (sense) {
					obtainMask(segment_table(cimg_ptr, 150, 200), low_thres, high_thres);

					std::vector<std::vector<Point>> contours;
					std::vector<Vec4i> hierarchy;
					findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

					std_msgs::Bool flag;

					if (contours.size()>0){
						sort(contours.begin(),contours.end(),[](const vector<Point> &a, const vector<Point> &b){return contourArea(a) > contourArea(b);});

						Rect boundRect = boundingRect(contours[0]);
						Moments m = moments(contours[0]);
						int uu = m.m10/m.m00;
						int vv = m.m01/m.m00;	
						float radius = boundRect.size().width/2.0;

						drawContours(cimg_ptr->image, contours, 0, cv::Scalar(255,255,0), 1, 8, hierarchy, 0, Point(0,0));
						circle(cimg_ptr->image, Point2f(uu,vv), 1, cv::Scalar(0,0,255), -1, 8, 0);

						track_pub.publish(cimg_ptr->toImageMsg());
		
						mdb_common::ObjDet coor = mdb_common::ObjDet();
						coor.u.data = uu;
						coor.v.data = vv;
						coor.radius.data = radius;
						flag.data = true;
						det_pub.publish(flag);
						coor_pub.publish(coor);
					}else{
						flag.data = false;
						det_pub.publish(flag);
						track_pub.publish(cimg_ptr->toImageMsg());
					}
				}
			}
		}
};
