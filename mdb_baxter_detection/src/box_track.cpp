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
#include <math.h> 
#include "opencv2/core/eigen.hpp"
#include <random>

#include "mdb_common/ObjDet.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "obj_track.cpp"

#include <ros/console.h>

using namespace cv;
using namespace std;

class box_track: public obj_track {
	public:
		box_track(){
			initPublishers("box");
			initSubscriber();
			nh.getParam("table_per", table_per); 
			//low_thres = cv::Scalar(106, 70, 17);
			//high_thres = cv::Scalar(148, 255, 209);
			//low_thres = cv::Scalar(131, 183, 45);
			//high_thres = cv::Scalar(179, 255, 165);
			//low_thres = cv::Scalar(101, 124, 37);
			//high_thres = cv::Scalar(157, 255, 165);
			//low_thres = cv::Scalar(79, 70, 45);
			//high_thres = cv::Scalar(168, 255, 191);

			low_thres = cv::Scalar(74, 26, 64);
			high_thres = cv::Scalar(117, 195, 142);

			//low_thres = cv::Scalar(115, 0, 0);
			//high_thres = cv::Scalar(179, 255, 255);

		}

	    ~box_track(){}

		void initSubscriber(void){
			color_sub = nh.subscribe("/image_color", 1, &box_track::colorCb, this);
		}

		void colorCb(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr color_ptr = ros2cv(msg);
			track_object(color_ptr);
		}
};

int main(int argc, char **argv) { 
	ros::init(argc, argv, "box_track");
	box_track bxt =  box_track();
	ros::spin();
    return 0;
}
