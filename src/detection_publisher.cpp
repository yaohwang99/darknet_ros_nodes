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
#include <csignal>



class detection_publisher{
	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Publisher pub;
		image_transport::Subscriber img_sub;
		image_transport::Subscriber depth_sub;
		ros::Subscriber box_sub;
		ros::Publisher twist_pub;
		cv_bridge::CvImagePtr cv_ptr;
		cv_bridge::CvImagePtr depth_cv_ptr;
		geometry_msgs::Twist twist;
		sensor_msgs::Image depth_img;
		//bounding box information
		int xmin;
		int ymin;
		int xmax;
		int ymax;
		//Center point of bounding box
		int targetx;
		int targety;
		int depth_targetx;
		int depth_targety;
		//rotation parameters
		float th;
		float turn_speed;
		int distance;
		
		
	public:
		detection_publisher(): it(nh),xmin(0),ymin(0),xmax(0),ymax(0),th(0),turn_speed(3.0/320.0),targetx(320),targety(240),distance(0)
  {
  	//Subscribe to topic
    img_sub = it.subscribe("/camera/color/image_raw", 1,&detection_publisher::imageCallback, this);
    depth_sub = it.subscribe("/camera/depth/image_rect_raw", 1,&detection_publisher::depthCallback, this);
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
  	void boxCallback(const darknet_ros_msgs::BoundingBoxes& msg2){
  		//ROS_INFO("boxCallback");
  		
  		//Initialize angular velocity is zero
  		twist.angular.z = 0;
  		
  		if(msg2.bounding_boxes.size()!=0) //Yolo detect items.
  		{
	  		for(auto iter=msg2.bounding_boxes.begin(); iter!= msg2.bounding_boxes.end(); ++iter){
		  		if(iter->id == 0) //human's id is 0
		  		{ 
		  			//Get bounding box information
			  		xmin = iter->xmin;
				  	ymin = iter->ymin;
				  	xmax = iter->xmax;
				  	ymax = iter->ymax;
				  	
				  	//Center point of bounding box
				  	targetx = int((xmin+xmax)/2);
		  			targety = int((ymin+ymax)/2);
		  			depth_targetx=int(targetx+102);
		  			depth_targety=targety;
		  			//640 x 480 for each image
		  			//horizontal distance with center point of image
		  			th = 320 - targetx;
		  			//tolerance is 20 pixel
		  			if(th < 20 && th >-20) th = 0;
					
					//counting angular velocity
		  			twist.angular.z = th * turn_speed;
		  			//distance in mm
		  			if(depth_cv_ptr)
					distance = depth_cv_ptr->image.at<u_int16_t>(depth_targetx, depth_targety);
		  			//ROS_INFO("th: %f", th );
		  			//ROS_INFO("turn: %f", turn_speed );
		  			//ROS_INFO("twist.angular.z: %f", twist.angular.z );
		  			ROS_INFO("distance: %d",distance);
		  			if(depth_cv_ptr)
				  	ROS_INFO("center distance: %d",depth_cv_ptr->image.at<u_int16_t>(424, 320));
				  	//Draw bounding box
				  	cv::rectangle(cv_ptr->image, cv::Point(xmin, ymin),cv::Point(xmax,ymax), cv::Scalar(0, 255, 0));
				  	cv::circle(cv_ptr->image, cv::Point(targetx, targety),10, cv::Scalar(0, 0, 255));
				  	cv::circle(depth_cv_ptr->image, cv::Point(targetx+102, targety),10, cv::Scalar(255));
				  	break;
		  		}
	  		}
  		}
  		
		
		
		//Tolerance range
  		cv::line(cv_ptr->image, cv::Point(300, 0),cv::Point(300,480), cv::Scalar(255, 0, 0));
		cv::line(cv_ptr->image, cv::Point(340, 0),cv::Point(340,480), cv::Scalar(255, 0, 0));
		
		//topic pulish
		twist_pub.publish(twist);
		pub.publish(cv_ptr->toImageMsg());
  	}
  	
	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
			//ROS_INFO("imageCallback");
			//ROS_INFO("height:%d",msg->height);
		//ROS_INFO("width:%d",msg->width);
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
	void depthCallback(const sensor_msgs::ImageConstPtr& msg){
		//ROS_INFO("dheight:%d",msg->height);
		//ROS_INFO("raw_distance:%d",msg->data[depth_targetx + depth_targety*848]);
		try
		    {
		      depth_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
		      if(depth_cv_ptr){
		      	ROS_INFO("depth image width: %d",depth_cv_ptr->image.cols);
		      ROS_INFO("depth image height %d",depth_cv_ptr->image.rows);
		      cv::imshow("depth",depth_cv_ptr->image);
		      }
		      
		    }
	    catch (cv_bridge::Exception& e)
		    {
		      ROS_ERROR("cv_bridge exception: %s", e.what());
		      return;
		    }
		
	}
	
	void SIGINT_Handler(int signum)
	{
		twist.angular.z = 0;
		twist_pub.publish(twist);
	}
};




int main(int argc, char** argv)
{
	
  ros::init(argc, argv, "detection_publisher");
  cv::namedWindow("depth");
  cv::startWindowThread();
  detection_publisher ic;
  //signal(SIGINT, ic.SIGINT_Handler);
  ros::spin();

 return 0;
}
