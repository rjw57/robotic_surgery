#include "k2_client.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int pointBufferSize = 2605056;
int streamSize = pointBufferSize + sizeof(double);
std::string cameraName = "depth";
std::string imageTopicSubName = "image_raw";
std::string cameraInfoSubName = "camera_info";

int main(int argC,char **argV)
{
	ros::init(argC,argV,"startPointCloud");
	ros::NodeHandle n;
	
	std::string serverAddress;
	n.getParam("/serverNameOrIP",serverAddress);
	Socket mySocket(serverAddress.c_str(),"9005",streamSize);
	
	ros::Publisher pub = n.advertise<PointCloud>("pointCloud",1);
    
    PointCloud:Ptr pc (new PointCloud);
    pc->resize(512,424);
     
	while(ros::ok())
	{
	    cout << "Reading data..." << endl;
		mySocket.readData();
		
		cout << "Copying data..." << endl;
		memcpy(&pc->points,&mySocket.mBuffer, pointBufferSize);
		
		double utcTime;
		memcpy(&utcTime,&mySocket.mBuffer[pointBufferSize],sizeof(double));
		pc.header.stamp = ros::Time(utcTime);

        cout << "Publishing point cloud" << endl;
		pub.publish(pc);
		ros::spinOnce();
	}
	return 0;
}
