#include "ros_api.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h> 
#include <sensor_msgs/LaserScan.h>
#include "ldlidar_driver.h"

struct LaserPose {
  double x, y, z;
  double roll, pitch, yaw;
  std::string base_frame_id;
};

// Passiamo il timestamp generato nel loop principale per una sincronizzazione assoluta
void PublishLidarData(ldlidar::Points2D& src, LaserScanSetting& setting, 
                      LaserPose& pose, ros::Publisher& scan_pub, 
                      ros::Publisher& pc_pub, const ros::Time& current_timestamp);

uint64_t GetSystemTimeStamp(void);

int main(int argc, char **argv) {
  ros::init(argc, argv, "sdpo_driver_laser_2d");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  std::string port_name;
  int serial_port_baudrate;
  LaserScanSetting setting;
  LaserPose pose; 

  // Nomi frame standard 5dpo: usiamo parametri passati dal launch per modularità
  nh_private.param<std::string>("base_frame_id", pose.base_frame_id, "base_footprint");
  nh_private.param<std::string>("laser_frame_id", setting.frame_id, "laser");
  
  nh_private.getParam("port_name", port_name);
  nh_private.param("port_baudrate", serial_port_baudrate, 230400);

  // Lettura pose in gradi e conversione in radianti
  nh_private.param("laser_pose_x", pose.x, 0.0);
  nh_private.param("laser_pose_y", pose.y, 0.0);
  nh_private.param("laser_pose_z", pose.z, 0.0);
  nh_private.param("laser_pose_roll", pose.roll, 0.0);
  nh_private.param("laser_pose_pitch", pose.pitch, 0.0);
  nh_private.param("laser_pose_yaw", pose.yaw, 0.0);

  pose.roll  *= M_PI / 180.0;
  pose.pitch *= M_PI / 180.0;
  pose.yaw   *= M_PI / 180.0;

  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();
  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp));
  ldlidarnode->EnableFilterAlgorithnmProcess(true);

  if (!ldlidarnode->Start(ldlidar::LDType::LD_19, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
    ROS_ERROR("Lidar start failed");
    return -1;
  }

  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud>("laser_scan_point_cloud", 10);
  
  ros::Rate r(10);
  ldlidar::Points2D laser_scan_points;
  
  while (ros::ok()) {
    if (ldlidarnode->GetLaserScanData(laser_scan_points, 1500) == ldlidar::LidarStatus::NORMAL) {
        // Generiamo il timestamp UNICO qui e lo passiamo alla funzione
        ros::Time current_timestamp = ros::Time::now(); 
        PublishLidarData(laser_scan_points, setting, pose, scan_pub, pc_pub, current_timestamp);
    }
    r.sleep();
  }
  
  ldlidarnode->Stop();
  delete ldlidarnode;
  return 0;
}

void PublishLidarData(ldlidar::Points2D& src, LaserScanSetting& setting, 
                      LaserPose& pose, ros::Publisher& scan_pub, 
                      ros::Publisher& pc_pub, const ros::Time& current_timestamp) {
  
  static tf::TransformBroadcaster tf_broadcaster;
  
  // 1. Pubblicazione TF (Sincronizzata al microsecondo con i dati)
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.x, pose.y, pose.z));
  tf::Quaternion q;
  q.setRPY(pose.roll, pose.pitch, pose.yaw);
  transform.setRotation(q);
  
  // Usiamo il timestamp passato per la TF
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_timestamp, pose.base_frame_id, setting.frame_id));

  // 2. Preparazione PointCloud (Stamp identico alla TF)
  sensor_msgs::PointCloud pc;
  pc.header.stamp = current_timestamp;
  pc.header.frame_id = setting.frame_id;

  // 3. Preparazione LaserScan (Stamp identico alla TF)
  sensor_msgs::LaserScan scan;
  scan.header.stamp = current_timestamp;
  scan.header.frame_id = setting.frame_id;
  scan.angle_min = 0;
  scan.angle_max = 2 * M_PI;
  scan.range_min = 0.02;
  scan.range_max = 12.0;
  scan.angle_increment = (2 * M_PI) / (src.size() - 1);
  scan.ranges.assign(src.size(), std::numeric_limits<float>::quiet_NaN());

  for (size_t i = 0; i < src.size(); ++i) {
    float range = src[i].distance / 1000.f;
    // Inversione segno angolo per CCW standard ROS (stile RPLIDARS2.cpp)
    float angle = -ANGLE_TO_RADIAN(src[i].angle); 

    geometry_msgs::Point32 p;
    p.x = range * cos(angle);
    p.y = range * sin(angle);
    p.z = 0;
    pc.points.push_back(p);

    if (i < scan.ranges.size()) scan.ranges[i] = range;
  }

  pc_pub.publish(pc);
  scan_pub.publish(scan);
}

uint64_t GetSystemTimeStamp(void) {
  auto tp = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}
