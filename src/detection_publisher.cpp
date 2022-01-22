#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <csignal>

#define FOLLOW_DISTANCE 2000
#define ROTATION_TOLERANCE 20

class detection_publisher{
	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Publisher pub;
		image_transport::Subscriber rgb_sub, depth_sub;
		ros::Subscriber box_sub;
		ros::Publisher twist_pub;
		cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
		geometry_msgs::Twist twist;

		//Bounding box information
		cv::Rect2d bbox;
		
		//rotation parameters
		float turn_speed;

		int curr_depth;
		
	public:
		detection_publisher(): it(nh), turn_speed(3.0/320.0), curr_depth(0)
		{
			//Subscribe to topic
			rgb_sub = it.subscribe("/camera/color/image_raw", 1,&detection_publisher::imageCallback, this);
			depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1,&detection_publisher::depthCallback, this);
			//depth_sub = it.subscribe("/camera/extrinsics/depth_to_color", 1,&detection_publisher::depthCallback, this);
			box_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1,&detection_publisher::boxCallback, this);
			
			//New topic which is the result of image.
			pub = it.advertise("custom_detection_image", 1);
			
			//Comtrol command
			twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
			twist.linear.x = 0;
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;
		}
		~detection_publisher()
		{
			twist.linear.x = 0;
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;
			twist_pub.publish(twist);
		}
		void boxCallback(const darknet_ros_msgs::BoundingBoxes& msg2){
			//ROS_INFO("boxCallback");

			if(!rgb_ptr) return;
			
			//Initialize angular velocity is zero
			twist.linear.x = 0;
			twist.angular.z = 0;
			bbox.x = 0;
			bbox.y = 0;
			bbox.width = 0;
			bbox.height = 0;
			
			if(msg2.bounding_boxes.size()!=0) //Yolo detect items.
			{
				for(auto iter=msg2.bounding_boxes.begin(); iter!= msg2.bounding_boxes.end(); ++iter)
				{
					if(iter->id == 0) //human's id is 0
					{ 
						//Get bounding box information
						bbox.x = iter->xmin;
                        bbox.y = iter->ymin;
                        bbox.width = iter->xmax - iter->xmin;
                        bbox.height = iter->ymax - iter->ymin;
						
						//640 x 480 for each image
						//horizontal distance with center point of image
						float th = 320 - (iter->xmax + iter->xmin) / 2.0;
						if(th < ROTATION_TOLERANCE && th > -ROTATION_TOLERANCE) th = 0;
						
						//counting angular velocity
						twist.angular.z = th * turn_speed;
						
						break;
					}
				}
			}

			if(!depth_ptr)
			{
				twist_pub.publish(twist);
				return;
			}

			if(bbox.x == 0 && bbox.width == 0) return;

			curr_depth = depth_ptr->image.at<u_int16_t>(bbox.x + bbox.width/2, bbox.y + bbox.height);
			if(curr_depth < FOLLOW_DISTANCE) twist.linear.x = - 0.1;
			else if(curr_depth > FOLLOW_DISTANCE) twist.linear.x = 0.1;
			
			twist_pub.publish(twist);
			show();
		}
  	
		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			//ROS_INFO("imageCallback");
			try
			{
				rgb_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}

		void depthCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			try
			{
				depth_ptr = cv_bridge::toCvCopy(msg);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}

		void show()
		{
			system("clear");
			std::cout << "***\033[1mdarknet_ros_nodes detection_publisher***\n\033[0m";
			std::cout << ((rgb_ptr) ? "\033[0;32m[OK] RGB Image\033[0m\n" :  "\033[0;31m[ERROR] RGB Image\033[0m\n");
			std::cout << ((depth_ptr) ? "\033[0;32m[OK] Align Depth\033[0m\n" :  "\033[0;31m[ERROR] Align Depth\033[0m\n");
			std::cout << "bounding box center : ( " << bbox.x + bbox.width/2.0 << " , " << bbox.y + bbox.height/2.0 << " )\n";
			std::cout << "\033[0;33mcenter depth : " << curr_depth  << "\n\033[0m";
			std::cout << "twist :\n"
					  << "\tlinear  x : " << twist.linear.x << "\n"
					  << "\tlinear  y : " << twist.linear.y << "\n"
					  << "\tlinear  z : " << twist.linear.z << "\n"
					  << "\tangular x : " << twist.angular.x << "\n"
					  << "\tangular y : " << twist.angular.y << "\n"
					  << "\tangular z : " << twist.angular.z << "\n";
			std::cout << "rotation tolerance : " << ROTATION_TOLERANCE << "\n";
			std::cout << "Follow distance : " << FOLLOW_DISTANCE << "\n";
			if(rgb_ptr)
			{
				cv::imshow("rgb image", rgb_ptr->image);
			}
		}
};

void SIGINT_Handler(int signum)
{
	ros::shutdown();
}

int main(int argc, char** argv)
{
	//cv::namedWindow("depth image");
  	//cv::startWindowThread();
	cv::namedWindow("rgb image");
  	cv::startWindowThread();
	signal(SIGINT, SIGINT_Handler);  
	ros::init(argc, argv, "detection_publisher");
	detection_publisher ic;
  	ros::spin();
	std::cout << "\ndone\n";
 	return 0;
}
