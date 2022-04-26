#include <pcl/io/ply_io.h>

#include "load_rosbag.h"

class rosbagToPly : public rosbagToPcFiles
{
    void save(std::string save_name, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc) override
    {
        std::string save_path = save_name + ".ply";
        pcl::io::savePLYFileASCII(save_path, *pcl_pc);
        std::cout << "Save: " << save_path << std::endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_to_ply");
	
	rosbagToPly rosbag_to_ply;
    rosbag_to_ply.convert();
    rosbag_to_ply.outputInfo();
}