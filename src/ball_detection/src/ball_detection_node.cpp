#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& rgb_msg, const ImageConstPtr& depth_msg)
{

    cv_bridge::CvImagePtr cv_ptr_bgr;
    cv_bridge::CvImagePtr cv_ptr_depth;
    try
    {
        cv_ptr_bgr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    //Define upper and lower bounds

    //Blur the rgb image

    //Convert to HSV

    //Mask based on range

    //Erode mask

    //Dilate Mask

    // Blob detector

    //Get largest blob

    // Get Moments of Blob

    // Detect  Centroid of mask


    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> rgb_sub(nh, "/rgb/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/depth_to_rgb/image_raw", 1);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}