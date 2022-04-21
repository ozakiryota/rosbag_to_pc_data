#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class rosbagToPcd{
	private:
		/*node handle*/
		// ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*buffer*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ {new pcl::PointCloud<pcl::PointXYZI>};
		/*parameter*/
		std::string rosbag_path_;
		std::string save_dir_;

	public:
		rosbagToPcd();
        std::string getDefaultSaveDir();
        void convert();

		// void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        // void filter(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
        // void useHeightmap(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, size_t &obstacle_counter, size_t &ground_counter);
		// void publication(std_msgs::Header header);
};

rosbagToPcd::rosbagToPcd()
	: nh_private_("~")
{
	std::cout << "----- rosbag_to_pcd -----" << std::endl;
	/*parameter*/
    if(!nh_private_.getParam("rosbag_path", rosbag_path_)){
        std::cerr << "Set rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "rosbag_path_ = " << rosbag_path_ << std::endl;

    nh_private_.param("save_dir", save_dir_, getDefaultSaveDir());
	std::cout << "save_dir_ = " << save_dir_ << std::endl;
}

std::string rosbagToPcd::getDefaultSaveDir()
{
    const char *tmp = getenv("ROS_WORKSPACE");
    std::string save_path(tmp ? tmp : "");
    if(save_path.empty()){
        std::cerr << "Cannot get $ROS_WORKSPACE." << std::endl;
        exit(true);
    }
    save_dir_ += std::string("/src/rosbag_to_pcd/save/") + rosbag_path_;
    return save_path;
}

void rosbagToPcd::convert()
{
    std::filesystem::create_directory(save_dir_);

    rosbag::Bag bag;
    rosbag::View view;
    rosbag::View::iterator view_it;

    try{
        bag.open(rosbag_path_, rosbag::bagmode::Read);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << rosbag_path_ << std::endl;
        exit(true);
    }

    view.addQuery(bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
    view_it = view.begin();

    while(view_it != view.end()){
        sensor_msgs::PointCloud2ConstPtr ros_pc = view_it->instantiate<sensor_msgs::PointCloud2>();
        std::stringstream save_path;
        save_path << save_dir_ << "/" << ros_pc->header.stamp;
        pcl::fromROSMsg(*ros_pc, *pcl_pc_);
        pcl::io::savePCDFileASCII(save_path.str(), *pcl_pc_);
        std::cout << "save_path.str() = " << save_path.str() << std::endl;
        ++view_it;
    }
}

// void rosbagToPcd::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
// {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pc (new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*msg, *pc);
//     filter(pc);
// 	publication(msg->header);
// }

// void rosbagToPcd::filter(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
// {
//     obstacle_pc_->points.resize(pc->points.size());
//     ground_pc_->points.resize(pc->points.size());

//     size_t obstacle_counter = 0;
//     size_t ground_counter = 0;
//     useHeightmap(pc, obstacle_counter, ground_counter);

//     obstacle_pc_->points.resize(obstacle_counter);
//     ground_pc_->points.resize(ground_counter);
// }

// void rosbagToPcd::useHeightmap(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, size_t &obstacle_counter, size_t &ground_counter)
// {
//     float min[grid_dim_][grid_dim_];
//     float max[grid_dim_][grid_dim_];
//     bool init[grid_dim_][grid_dim_];
//     memset(&init, 0, grid_dim_*grid_dim_);

//     /*build height map*/
//     for(size_t i = 0; i < pc->points.size(); ++i){
//         int x = ((grid_dim_ / 2) + pc->points[i].x / m_per_cell_);
//         int y = ((grid_dim_ / 2) + pc->points[i].y / m_per_cell_);
//         if(x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_){
//             if(!init[x][y]){
//                 min[x][y] = pc->points[i].z;
//                 max[x][y] = pc->points[i].z;
//                 init[x][y] = true;
//             }
//             else{
//                 min[x][y] = (min[x][y] < pc->points[i].z ? min[x][y] : pc->points[i].z);
//                 max[x][y] = (max[x][y] > pc->points[i].z ? max[x][y] : pc->points[i].z);
//             }
//         }
//     }

//     /*display points where map has height-difference > threshold*/
//     for(size_t i = 0; i < pc->points.size(); ++i){
//         int x = (grid_dim_ / 2) + pc->points[i].x / m_per_cell_;
//         int y = (grid_dim_ / 2) + pc->points[i].y / m_per_cell_;
//         if(x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]){
//             if(max[x][y] - min[x][y] > height_diff_threshold_){
//                 obstacle_pc_->points[obstacle_counter] = pc->points[i];
//                 obstacle_counter++;
//             }
//             else{
//                 ground_pc_->points[ground_counter] = pc->points[i];
//                 ground_counter++;
//             }
//         }
//     }
// }

// void rosbagToPcd::publication(std_msgs::Header header)
// {
//     if(!obstacle_pc_->points.empty()){
//         sensor_msgs::PointCloud2 obstacle_ros_pc;
//         pcl::toROSMsg(*obstacle_pc_, obstacle_ros_pc);
//         obstacle_ros_pc.header = header;
//         pub_obstacle_.publish(obstacle_ros_pc);
//     }

//     if(!ground_pc_->points.empty()){
//         sensor_msgs::PointCloud2 ground_ros_pc;
//         pcl::toROSMsg(*ground_pc_, ground_ros_pc);
//         ground_ros_pc.header = header;
//         pub_ground_.publish(ground_ros_pc);
//     }
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_to_pcd");
	
	rosbagToPcd rosbag_to_pcd;
    rosbag_to_pcd.convert();
}