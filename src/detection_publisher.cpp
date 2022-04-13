#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_nodes/Tracker_on_off.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <csignal>
#include <pthread.h>

#define FOLLOW_DISTANCE 1000
#define FOLLOW_TOLERANCE 300
#define FOLLOW_MAX_DISTANCE 2000
#define FOLLOW_SPEED 5.0/4000;

#define ROTATION_TOLERANCE 20
#define ROTATION_SPEED 3.0/320

class detection_publisher{
	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber depth_sub;
		image_transport::Subscriber image_sub;
		image_transport::Publisher image_pub;
		ros::Subscriber box_sub;
		ros::Publisher twist_pub;
		cv_bridge::CvImagePtr depth_ptr;
		cv_bridge::CvImagePtr image_ptr;
		geometry_msgs::Twist twist;

		pthread_mutex_t mutex;
		ros::ServiceServer service;
		bool is_on;
	public:
		detection_publisher(): it(nh), is_on(false)
		{
			//Subscribe to topic
			depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1,&detection_publisher::depthCallback, this);
			image_sub = it.subscribe("/camera/color/image_raw", 1,&detection_publisher::imageCallback, this);
			box_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1,&detection_publisher::boxCallback, this);
			image_pub = it.advertise("/camera/on_off/image_raw", 1);
			//Comtrol command
			twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
			twist.linear.x = 0;
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;

			// sevice
			service = nh.advertiseService("detection_publisher/Tracker_on_off", &detection_publisher::add, this);

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
		bool add(darknet_ros_nodes::Tracker_on_off::Request  &req,
         darknet_ros_nodes::Tracker_on_off::Response &res)
		{
			if(is_on){
					twist.linear.x = 0;
					twist.linear.y = 0;
					twist.linear.z = 0;
					twist.angular.x = 0;
					twist.angular.y = 0;
					twist.angular.z = 0;
					twist_pub.publish(twist);
				}
			is_on = !is_on;
			res.sum = req.a + req.b;
			ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
			ROS_INFO("sending back response: [%ld]", (long int)res.sum);
			ROS_INFO("is_on: %d", is_on);
			return true;
		}

		void boxCallback(const darknet_ros_msgs::BoundingBoxes& msg2){
			if(!is_on)
				return;
			system("clear");
			std::cout << "***\033[1mdarknet_ros_nodes detection_publisher***\n\033[0m";

			// 0 for x, for y
			int center[2] = {0,0};

			// pthread_mutex_lock(&mutex);
			twist.linear.x = 0;
			twist.angular.z = 0;
			// pthread_mutex_unlock(&mutex);

			if(msg2.bounding_boxes.size()!=0) //Yolo detect items.
			{
				int i = 0;
				for(auto iter=msg2.bounding_boxes.begin(); iter!= msg2.bounding_boxes.end(); ++iter, ++i)
				{
					if(iter->id == 0) //human's id is 0
					{ 
						//Get bounding box information
						center[0] = (iter->xmax + iter->xmin) / 2;
						center[1] = (iter->ymax + iter->ymin) / 2;

						//640 x 480 for each image
						//horizontal distance with center point of image
						float th = 320 - center[0];
						if(th < ROTATION_TOLERANCE && th > -ROTATION_TOLERANCE) th = 0;
							
						//counting angular velocity
						// pthread_mutex_lock(&mutex);
						twist.angular.z = th * ROTATION_SPEED;
						// pthread_mutex_unlock(&mutex);

						break;
					}
				}
			}

			std::cout << "bounding box center : ( " << center[0] << " , " << center[1] << " )\n";

			// pthread_mutex_lock(&mutex);
			if(depth_ptr && center[0] != 0 && center[1] != 0)
			{
				int curr_depth = depth_ptr->image.at<uint16_t>(center[1], center[0]);
				std::cout << "\033[0;33mcenter depth : " << curr_depth  << "\n\033[0m";
				curr_depth -= FOLLOW_DISTANCE;
				if(curr_depth < 0) curr_depth = 0;
				else if(curr_depth < FOLLOW_TOLERANCE && curr_depth > -FOLLOW_TOLERANCE) curr_depth = 0;
				else if(curr_depth > FOLLOW_MAX_DISTANCE) curr_depth = 0;
				twist.linear.x = curr_depth * FOLLOW_SPEED;
			}
			else
			{
				std::cout << "\033[0;31m[ERROR] Align Depth\033[0m\n";;
			}
				
			twist_pub.publish(twist);
			std::cout << "twist :\n"
					  << "\tlinear  x : " << twist.linear.x << "\n"
					  << "\tlinear  y : " << twist.linear.y << "\n"
					  << "\tlinear  z : " << twist.linear.z << "\n"
					  << "\tangular x : " << twist.angular.x << "\n"
					  << "\tangular y : " << twist.angular.y << "\n"
					  << "\tangular z : " << twist.angular.z << "\n";
			// pthread_mutex_unlock(&mutex);
		}

		void depthCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			if(!is_on)
				return;
			try
			{
				// pthread_mutex_lock(&mutex);
				depth_ptr = cv_bridge::toCvCopy(msg);
				// pthread_mutex_unlock(&mutex);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}
		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			try
			{
				if(is_on){
					image_pub.publish(msg);
				}
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}

		// void show()
		// {
		// 	pthread_mutex_lock(&mutex);
		// 	system("clear");
		// 	std::cout << "***\033[1mdarknet_ros_nodes detection_publisher***\n\033[0m";
		// 	std::cout << ((depth_ptr) ? "\033[0;32m[OK] Align Depth\033[0m\n" :  "\033[0;31m[ERROR] Align Depth\033[0m\n");
		// 	std::cout << "bounding box center : ( " << bbox.x + bbox.width/2 << " , " << bbox.y + bbox.height/2 << " )\n";
		// 	std::cout << "\033[0;33mcenter depth : " << curr_depth  << "\n\033[0m";

		// 	std::cout << "rotation tolerance : " << ROTATION_TOLERANCE << "\n";
		// 	std::cout << "Follow distance : " << FOLLOW_DISTANCE << "\n";
		// 	pthread_mutex_unlock(&mutex);
		// }
};

void SIGINT_Handler(int signum)
{
	ros::shutdown();
}

int main(int argc, char** argv)
{
	signal(SIGINT, SIGINT_Handler);  
	ros::init(argc, argv, "detection_publisher");
	detection_publisher ic;
  	//ros::spin();
	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    	spinner.spin();
	std::cout << "\ndone\n";
 	return 0;
}
