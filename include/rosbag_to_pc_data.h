#pragma once

#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

class rosbagToPcFiles{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*publisher*/
		ros::Publisher pub_original_;
		ros::Publisher pub_filtered_;
        /*buffer*/
        size_t num_msg_ = 0;
        size_t num_save_ = 0;
		/*parameter*/
		std::string rosbag_path_;
		std::string save_dir_;
        bool sleep_for_debug_;
        bool save_merged_pcd_;
        double debug_hz_;
        double rot_r_deg_;
        double rot_p_deg_;
        double rot_y_deg_;
        bool apply_reverse_rotation_;
        bool remove_ground_;
        double m_per_cell_;
		int grid_dim_;
		double height_diff_threshold_;
        bool filter_x_;
        double x_min_;
        double x_max_;
        bool filter_y_;
        double y_min_;
        double y_max_;
        bool filter_z_;
        double z_min_;
        double z_max_;

	public:
		rosbagToPcFiles();
        std::string getDefaultSaveDir();
        void convert();
        void rotation(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc, float r_deg, float p_deg, float y_deg, bool inverse);
        float degToRad(double deg);
        void removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_pc);
        void filterXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc);
		void debugPublication(sensor_msgs::PointCloud2 ros_pc, const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc);
        virtual void save(std::string save_name, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc) = 0;
        void outputInfo();
};