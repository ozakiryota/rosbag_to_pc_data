#include "rosbag_to_pc_data.h"

class rosbagToPcd : public rosbagToPcFiles
{
    void save(std::string save_name, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc) override
    {
        std::string save_path = save_name + ".pcd";
        pcl::io::savePCDFileASCII(save_path, *pcl_pc);
        std::cout << "Save: " << save_path << std::endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_to_pcd");
	
	rosbagToPcd rosbag_to_pcd;
    rosbag_to_pcd.convert();
    rosbag_to_pcd.outputInfo();
}