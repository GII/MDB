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

#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

void colorCb(const sensor_msgs::ImageConstPtr& msg){
	ROS_INFO_STREAM("wipe");
	/*cv_bridge::CvImageConstPtr color_const_ptr = cros2cv(msg);
	cv_bridge::CvImagePtr color_ptr;
	color_ptr->encoding = color_const_ptr->encoding;
	color_ptr->header = color_const_ptr->header;
	color_ptr->image = color_const_ptr->image;
	track_object(color_ptr);*/
}

class cylinder_track: public obj_track{
	public:
		cylinder_track(){
			initPublishers("cylinder");
			low_thres = cv::Scalar(67, 40, 87);
			high_thres = cv::Scalar(109, 255, 255);
			initSubscriber();
		}

	    ~cylinder_track(){}

		void initSubscriber(void){
			image_transport::ImageTransport it(nh);
			//image_transport::Subscriber color_sub = it.subscribe("/image_color", 1, &cylinder_track::colorCb, this);
			//image_transport::Subscriber color_sub = it.subscribe("/camera/image/", 1, &cylinder_track::colorCb, this);
			it.subscribe("/camera/image", 1, colorCb);
			//nh.setParam("/cylinder_track/image_transport", "compressed");
		}

		cv_bridge::CvImageConstPtr cros2cv (const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImageConstPtr cv_ptr;
			try {
				cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
			} catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
			return cv_ptr;
		}
};

int main(int argc, char **argv) { 
	ros::init(argc, argv, "cylinder_track");
	cylinder_track bt = cylinder_track();
	ros::spin();
    return 0;
}
