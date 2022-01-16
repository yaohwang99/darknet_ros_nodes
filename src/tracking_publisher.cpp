#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

using namespace cv;

class tracking_publisher{
	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Publisher pub;
		image_transport::Subscriber sub;
		ros::Subscriber box_sub;
		ros::Publisher twist_pub;
		cv_bridge::CvImagePtr cv_ptr;
		geometry_msgs::Twist twist;
		//Center point of bounding box
		int targetx;
		int targety;

        int tolerance;
        bool tracking;
        Ptr<Tracker> tracker;
        Rect2d bbox;
		
		//rotation parameters
		float th;
		float turn_speed;
		
	public:
		tracking_publisher(): it(nh),th(0),turn_speed(3.0/320.0),targetx(320),targety(240),tolerance(20), tracking(false)
		{
			//Subscribe to topic
			sub = it.subscribe("/camera/color/image_raw", 1,&tracking_publisher::imageCallback, this);
			box_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1,&tracking_publisher::boxCallback, this);
			
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

            tracker = TrackerCSRT::create();
		}
		~tracking_publisher()
		{
			twist.linear.x = 0;
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;
			twist_pub.publish(twist);
			ROS_INFO("Detection publisher dtor");
		}
		void boxCallback(const darknet_ros_msgs::BoundingBoxes& msg2){
			//ROS_INFO("boxCallback");
			
            if(tracking) return;
			
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
                        tracker->init(cv_ptr->image,bbox);
                        tracking = true;
						break;
					}
				}
			}
		}
  	
		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			//ROS_INFO("imageCallback");
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
            if(tracking)
            {
                bool ok = tracker->update(cv_ptr->image, bbox);
                if (ok)
	            {
                    // Tracking success : Draw the tracked object
                    rectangle(cv_ptr->image, bbox, Scalar( 255, 0, 0 ), 2, 1 );
	            }
	            else
	            {
	                // Tracking failure detected.
	                putText(cv_ptr->image, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
	            }      
            }
            pub.publish(cv_ptr->toImageMsg());
            
		}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detection_publisher");
  tracking_publisher ic;
  ros::spin();
 return 0;
}
