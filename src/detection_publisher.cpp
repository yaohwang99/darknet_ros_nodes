#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <ros/console.h>
class detection_publisher{
	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Publisher pub;
		image_transport::Subscriber sub;
		ros::Subscriber box_sub;
		cv_bridge::CvImagePtr cv_ptr;
		int xmin;
		int ymin;
		int xmax;
		int ymax;
		int id;
		int cnt;
	public:
		detection_publisher(): it(nh),xmin(0),ymin(0),xmax(0),ymax(0),id(1),cnt(0)
  {
    sub = it.subscribe("/usb_cam/image_raw", 1,&detection_publisher::imageCallback, this);
    box_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1,&detection_publisher::boxCallback, this);
    pub = it.advertise("custom_detection_image", 1);
    
  }
  	void boxCallback(const darknet_ros_msgs::BoundingBoxes& msg2){
  		ROS_INFO("boxCallback");
  		if(msg2.bounding_boxes.size()!=0){
  		for(auto iter=msg2.bounding_boxes.begin(); iter!= msg2.bounding_boxes.end();iter++)
  			if(iter->id==0){
  				
	  			xmin=iter->xmin;
		  		ymin=iter->ymin;
		  		xmax=iter->xmax;
		  		ymax=iter->ymax;
		  		int targetx=int((xmin+xmax)/2);
  				int targety=int((ymin+ymax)/2);
		  		publishImage();
		  		break;
  			}else continue;
	  		
  		}
  		//publishImage();
  		
  	}
	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
			ROS_INFO("imageCallback");
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
	ROS_INFO("publishImage");
		cv::rectangle(cv_ptr->image, cv::Point(xmin, ymin),cv::Point(xmax,ymax), cv::Scalar(0, 255, 0));
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
