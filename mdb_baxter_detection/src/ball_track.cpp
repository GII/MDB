//| This file is a part of the CAFER framework developped within
//| the DREAM project (http://www.robotsthatdream.eu/).
//|
//| Copyright 2016, GII / Universidad de la Coruna (UDC)
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
#include <math.h> 
#include "opencv2/core/eigen.hpp"
#include <random>

#include "mdb_baxter_detection/ObjDet.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "obj_track.cpp"

using namespace cv;
using namespace std;

class ball_track: public obj_track{
	public:
		ball_track(){
			initPublishers("ball");
			nh.getParam("table_per", table_per); 
			//low_thres = cv::Scalar(47, 74, 18);
			//high_thres = cv::Scalar(83, 255, 255);
			//low_thres = cv::Scalar(53, 80, 87);
			//high_thres = cv::Scalar(79, 186, 183);
			//low_thres = cv::Scalar(41, 90, 21);
			//high_thres = cv::Scalar(118, 255, 179);
			low_thres = cv::Scalar(37, 69, 53);
			high_thres = cv::Scalar(100, 255, 255);
			initSubscriber();
		}

	    ~ball_track(){}

		void initSubscriber(void){
			color_sub = nh.subscribe("/image_color", 1, &ball_track::colorCb, this);
		}

		void colorCb(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr color_ptr = ros2cv(msg);
			track_object(color_ptr);
		}
};

int main(int argc, char **argv) { 
	ros::init(argc, argv, "ball_track");
	ball_track bt = ball_track();
	ros::spin();
    return 0;
}
