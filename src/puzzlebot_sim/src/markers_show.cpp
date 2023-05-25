#include <aruco/aruco.h>

#include <ros/ros.h>
#include "aruco_ros/aruco_ros_utils.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
class MarkerRvizShow
{
private:
	ros::NodeHandle nh;
	ros::Publisher marker_rviz_pub;
	ros::Subscriber sub;
	visualization_msgs::MarkerArray marker_array;
	double marker_size;
	std::string marker_frame_;
	std::string camera_frame_;
	std::string reference_frame_;
	tf::TransformListener listener;
public:
	MarkerRvizShow() : nh("~")
	{
		std::string advertise, subscribe;
		nh.param<std::string>("advertise", advertise, "");
		nh.param<std::string>("subscribe", subscribe, "");

		sub = nh.subscribe( subscribe + "/markers", 10, &MarkerRvizShow::markers_callback, this);
		marker_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>((advertise + "/markers_show"), 1);
		nh.param<double>("marker_size", marker_size, 0.05);
		nh.param<std::string>("reference_frame", reference_frame_, "");
		nh.param<std::string>("camera_frame", camera_frame_, "");
		nh.param<std::string>("marker_frame", marker_frame_, "");

		marker_array = visualization_msgs::MarkerArray();
	}

	void markers_callback(const aruco_msgs::MarkerArray &markers)
	{
		// for each marker in markers
		for (size_t i = 0; i < markers.markers.size(); ++i)
		{
			aruco_msgs::Marker msg_marker = markers.markers[i];
			// publish rviz marker representing the ArUco marker patch
			visualization_msgs::Marker visMarker;
			visMarker.header = msg_marker.header;
			visMarker.ns = "aruco_markers";
			visMarker.id = msg_marker.id;
			visMarker.type = visualization_msgs::Marker::CUBE;
			visMarker.action = visualization_msgs::Marker::ADD;
			visMarker.pose = msg_marker.pose.pose;
			visMarker.scale.x = marker_size;
			visMarker.scale.y = marker_size;
			visMarker.scale.z = 0.001;
			visMarker.color.r = 1.0;
			visMarker.color.g = 0;
			visMarker.color.b = 0;
			visMarker.color.a = 1.0;


			bool marker_exists = false;
			for (size_t i = 0; i < marker_array.markers.size(); ++i)
			{
				visualization_msgs::Marker marker = marker_array.markers[i];
				if (marker.id == visMarker.id)
				{
					marker_exists = true;
					marker_array.markers[i] = visMarker;
				}
			}
			if (!marker_exists)
			{
				marker_array.markers.push_back(visMarker);
			}
		}
		marker_rviz_pub.publish(marker_array);
	}
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "markers_show");
	MarkerRvizShow node;

	ros::spin();
}