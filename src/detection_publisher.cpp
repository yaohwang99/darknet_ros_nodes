#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
class detection_publisher{
	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Publisher pub;
		image_transport::Subscriber sub;
		ros::Subscriber box_sub;
		ros::Publisher twist_pub;
		cv_bridge::CvImagePtr cv_ptr;
		geometry_msgs::Twist twist;
		int xmin;
		int ymin;
		int xmax;
		int ymax;
		int id;
		int cnt;
		int th;
		float turn;
		int targetx;
		int targety;
	public:
		detection_publisher(): it(nh),xmin(0),ymin(0),xmax(0),ymax(0),id(1),cnt(0),th(0),turn(1.0),targetx(320),targety(240)
  {
    sub = it.subscribe("/usb_cam/image_raw", 1,&detection_publisher::imageCallback, this);
    box_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1,&detection_publisher::boxCallback, this);
    pub = it.advertise("custom_detection_image", 1);
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    
  }
  	void boxCallback(const darknet_ros_msgs::BoundingBoxes& msg2){
  		//ROS_INFO("boxCallback");
  		if(msg2.bounding_boxes.size()!=0){
	  		for(auto iter=msg2.bounding_boxes.begin(); iter!= msg2.bounding_boxes.end();iter++){
	  		
		  		if(iter->id==0){
		  				
			  			xmin=iter->xmin;
				  		ymin=iter->ymin;
				  		xmax=iter->xmax;
				  		ymax=iter->ymax;
				  		targetx=int((xmin+xmax)/2);
		  				targety=int((ymin+ymax)/2);
		  				if(targetx>=340) th=-1;
		  				else if(targetx<=300) th=1;
		  				else th=0;
		  				twist.angular.z = th * turn;
				  		publishImage();
				  		break;
		  			}else {
		  			th=0;
		  			}
	  		}
  		}
  		//publishImage();
  		
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
		  
	}
	void publishImage(){
	//ROS_INFO("publishImage");
		cv::rectangle(cv_ptr->image, cv::Point(xmin, ymin),cv::Point(xmax,ymax), cv::Scalar(0, 255, 0));
		cv::line(cv_ptr->image, cv::Point(300, 0),cv::Point(300,480), cv::Scalar(255, 0, 0));
		cv::line(cv_ptr->image, cv::Point(340, 0),cv::Point(340,480), cv::Scalar(255, 0, 0));
		cv::circle(cv_ptr->image, cv::Point(targetx, targety),10, cv::Scalar(0, 0, 255));
		twist_pub.publish(twist);
		pub.publish(cv_ptr->toImageMsg());
	}

};




int main(int argc, char** argv)
{

  ros::init(argc, argv, "detection_publisher");
  detection_publisher ic;
  ros::spin();

 	return 0;
 
}
