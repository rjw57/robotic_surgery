#include "k2_client.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int pointBufferSize = 2605056;
int streamSize = pointBufferSize + sizeof(double);

int main(int argC,char **argV)
{
	ros::init(argC,argV,"startPointCloud");
	ros::NodeHandle n;

	std::string serverAddress;
	n.getParam("/serverNameOrIP",serverAddress);
	Socket mySocket(serverAddress.c_str(),"9005",streamSize);

	ros::Publisher pub = n.advertise<PointCloud>("point_cloud",1);

    PointCloud::Ptr pc (new PointCloud);

    pc->header.frame_id =  ros::this_node::getNamespace().substr(1,std::string::npos) + "/kinect_pcl";
	while(ros::ok())
	{
		// TODO(Somhtr): change to ROS' logging API
		cout << "Reading data..." << endl;
		mySocket.readData();

		// TODO(Somhtr): change to ROS' logging API
		cout << "Copying data..." << endl;
		uint64_t ptr = 0;
		float x,y,z;
		while(ptr<pointBufferSize)
		{
			memcpy(&x, &mySocket.mBuffer[ptr  ],4);
			memcpy(&y, &mySocket.mBuffer[ptr+4],4);
			memcpy(&z, &mySocket.mBuffer[ptr+8],4);

			pc->push_back(pcl::PointXYZ(x,y,z));
			ptr += 12;
		}

		double utcTime;
		memcpy(&utcTime,&mySocket.mBuffer[pointBufferSize],sizeof(double));
		pc->header.stamp = ros::Time(utcTime).toSec();

		pub.publish(pc);
		pc->clear();

		ros::spinOnce();
	}
	return 0;
}
