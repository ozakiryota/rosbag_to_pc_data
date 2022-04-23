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
        bool remove_ground_;
        double m_per_cell_;
		int grid_dim_;
		double height_diff_threshold_;

	public:
		rosbagToPcd();
        std::string getDefaultSaveDir();
        void convert();
        void removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_pc);
		void publication(sensor_msgs::PointCloud2 ros_pc, const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc);
        void outputInfo();
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

    nh_private_.param("remove_ground", remove_ground_, false);
	std::cout << "remove_ground_ = " << (bool)remove_ground_ << std::endl;
    nh_private_.param("m_per_cell", m_per_cell_, 1.0);
	std::cout << "m_per_cell_ = " << m_per_cell_ << std::endl;
	nh_private_.param("grid_dim", grid_dim_, 100);
	std::cout << "grid_dim_ = " << grid_dim_ << std::endl;
	nh_private_.param("height_diff_threshold", height_diff_threshold_, 0.1);
	std::cout << "height_diff_threshold_ = " << height_diff_threshold_ << std::endl;

    /*publisher*/
	pub_original_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/original", 1);
	pub_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/filtered", 1);
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
    std::filesystem::remove_all(save_dir_);
    std::filesystem::create_directory(save_dir_);

    rosbag::Bag bag;
    rosbag::View view;
    rosbag::View::iterator view_itr;

    try{
        bag.open(rosbag_path_, rosbag::bagmode::Read);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << rosbag_path_ << std::endl;
        exit(true);
    }

    view.addQuery(bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
    view_itr = view.begin();

    while(view_itr != view.end()){
        sensor_msgs::PointCloud2ConstPtr ros_pc = view_itr->instantiate<sensor_msgs::PointCloud2>();
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*ros_pc, *pcl_pc);
        if(remove_ground_)  removeGround(pcl_pc);
        if(!pcl_pc->points.empty()){
            std::stringstream save_path;
            save_path << save_dir_ << "/" << ros_pc->header.stamp;
            pcl::io::savePCDFileASCII(save_path.str(), *pcl_pc);
            std::cout << "Save: " << save_path.str() << std::endl;
            std::cout << "ros_pc->header.stamp.sec = " << ros_pc->header.stamp.sec << std::endl;
            std::cout << "ros_pc->header.stamp.nsec = " << ros_pc->header.stamp.nsec << std::endl;
            std::cout << "pcl_pc->points.size() = " << pcl_pc->points.size() << std::endl;
            ++num_save_;
        }
        ++view_itr;
        ++num_msg_;
        publication(*ros_pc, pcl_pc);
    }
}

void rosbagToPcd::removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_pc)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_obstacle_pc (new pcl::PointCloud<pcl::PointXYZI>);
    tmp_obstacle_pc->points.resize(pcl_pc->points.size());
    size_t obstacle_counter = 0;

    float min[grid_dim_][grid_dim_];
    float max[grid_dim_][grid_dim_];
    bool init[grid_dim_][grid_dim_];
    memset(&init, 0, grid_dim_*grid_dim_);

    /*build height map*/
    for(size_t i = 0; i < pcl_pc->points.size(); ++i){
        int x = ((grid_dim_ / 2) + pcl_pc->points[i].x / m_per_cell_);
        int y = ((grid_dim_ / 2) + pcl_pc->points[i].y / m_per_cell_);
        if(x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_){
            if(!init[x][y]){
                min[x][y] = pcl_pc->points[i].z;
                max[x][y] = pcl_pc->points[i].z;
                init[x][y] = true;
            }
            else{
                min[x][y] = (min[x][y] < pcl_pc->points[i].z ? min[x][y] : pcl_pc->points[i].z);
                max[x][y] = (max[x][y] > pcl_pc->points[i].z ? max[x][y] : pcl_pc->points[i].z);
            }
        }
    }

    /*display points where map has height-difference > threshold*/
    for(size_t i = 0; i < pcl_pc->points.size(); ++i){
        int x = (grid_dim_ / 2) + pcl_pc->points[i].x / m_per_cell_;
        int y = (grid_dim_ / 2) + pcl_pc->points[i].y / m_per_cell_;
        if(x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]){
            if(max[x][y] - min[x][y] < height_diff_threshold_)  continue;
        }
        tmp_obstacle_pc->points[obstacle_counter] = pcl_pc->points[i];
        obstacle_counter++;
    }

    tmp_obstacle_pc->points.resize(obstacle_counter);
    tmp_obstacle_pc->height = 1;
    tmp_obstacle_pc->width = obstacle_counter;

    pcl_pc = tmp_obstacle_pc;
}

void rosbagToPcd::publication(sensor_msgs::PointCloud2 original_ros_pc, const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pcl_pc)
{
    const std::string debug_frame = "debug";

    original_ros_pc.header.frame_id = debug_frame;
    pub_original_.publish(original_ros_pc);

    if(!filtered_pcl_pc->points.empty() && remove_ground_){
        sensor_msgs::PointCloud2 filtered_ros_pc;
        pcl::toROSMsg(*filtered_pcl_pc, filtered_ros_pc);
        filtered_ros_pc.header.frame_id = debug_frame;
        pub_filtered_.publish(filtered_ros_pc);
    }
}

void rosbagToPcd::outputInfo()
{
    std::string save_path = save_dir_ + "/info.txt";
    std::ofstream ofs(save_path);
    ofs << "num_save / num_msg = " << num_save_ / (double)num_msg_ * 100.0 << " %" << std::endl;
    ofs.close();
    std::cout << "Save: " << save_path << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_to_pcd");
	
	rosbagToPcd rosbag_to_pcd;
    rosbag_to_pcd.convert();
    rosbag_to_pcd.outputInfo();
}