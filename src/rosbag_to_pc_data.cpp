#include "rosbag_to_pc_data.h"

rosbagToPcFiles::rosbagToPcFiles()
	: nh_private_("~")
{
	std::cout << "----- rosbag_to_pc_data -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("rosbag_path", rosbag_path_)){
        std::cerr << "Set rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "rosbag_path_ = " << rosbag_path_ << std::endl;

    nh_private_.param("save_dir", save_dir_, getDefaultSaveDir());
	std::cout << "save_dir_ = " << save_dir_ << std::endl;

    sleep_for_debug_ = nh_private_.getParam("debug_hz", debug_hz_);
	std::cout << "debug_hz_ = " << debug_hz_ << std::endl;

    nh_private_.param("rot_r_deg", rot_r_deg_, 0.0);
	std::cout << "rot_r_deg_ = " << rot_r_deg_ << std::endl;
    nh_private_.param("rot_p_deg", rot_p_deg_, 0.0);
	std::cout << "rot_p_deg_ = " << rot_p_deg_ << std::endl;
    nh_private_.param("rot_y_deg", rot_y_deg_, 0.0);
	std::cout << "rot_y_deg_ = " << rot_y_deg_ << std::endl;
    nh_private_.param("apply_reverse_rotation", apply_reverse_rotation_, false);
	std::cout << "apply_reverse_rotation_ = " << (bool)apply_reverse_rotation_ << std::endl;

    nh_private_.param("save_merged_pcd", save_merged_pcd_, false);
	std::cout << "save_merged_pcd_ = " << (bool)save_merged_pcd_ << std::endl;

    nh_private_.param("remove_ground", remove_ground_, false);
	std::cout << "remove_ground_ = " << (bool)remove_ground_ << std::endl;
    nh_private_.param("m_per_cell", m_per_cell_, 1.0);
	std::cout << "m_per_cell_ = " << m_per_cell_ << std::endl;
	nh_private_.param("grid_dim", grid_dim_, 100);
	std::cout << "grid_dim_ = " << grid_dim_ << std::endl;
	nh_private_.param("height_diff_threshold", height_diff_threshold_, 0.1);
	std::cout << "height_diff_threshold_ = " << height_diff_threshold_ << std::endl;

    filter_x_ = nh_private_.getParam("x_min", x_min_);
    if(filter_x_){
    	std::cout << "x_min_ = " << x_min_ << std::endl;
        filter_x_ = nh_private_.getParam("x_max", x_max_);
	    std::cout << "x_max_ = " << x_max_ << std::endl;
    }
    filter_y_ = nh_private_.getParam("y_min", y_min_);
    if(filter_y_){
	    std::cout << "y_min_ = " << y_min_ << std::endl;
        filter_y_ = nh_private_.getParam("y_max", y_max_);
	    std::cout << "y_max_ = " << y_max_ << std::endl;
    }
    filter_z_ = nh_private_.getParam("z_min", z_min_);
    if(filter_z_){
    	std::cout << "z_min_ = " << z_min_ << std::endl;
        filter_z_ = nh_private_.getParam("z_max", z_max_);
    	std::cout << "z_max_ = " << z_max_ << std::endl;
    }

    /*publisher*/
	pub_original_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/original", 1);
	pub_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/filtered", 1);
}

std::string rosbagToPcFiles::getDefaultSaveDir()
{
    const char *tmp = getenv("ROS_WORKSPACE");
    std::string save_path(tmp ? tmp : "");
    if(save_path.empty()){
        std::cerr << "Cannot get $ROS_WORKSPACE." << std::endl;
        exit(true);
    }
    save_dir_ += std::string("/src/rosbag_to_pc_data/save/") + rosbag_path_;
    return save_path;
}

void rosbagToPcFiles::convert()
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

    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_pc (new pcl::PointCloud<pcl::PointXYZI>);

    ros::Rate loop_rate(debug_hz_);
    while(view_itr != view.end()){
        sensor_msgs::PointCloud2ConstPtr ros_pc = view_itr->instantiate<sensor_msgs::PointCloud2>();
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*ros_pc, *pcl_pc);

        if(rot_r_deg_ || rot_p_deg_ || rot_y_deg_)  rotation(pcl_pc, rot_r_deg_, rot_p_deg_, rot_y_deg_, false);
        if(filter_x_ || filter_y_ || filter_z_) filterXYZ(pcl_pc);
        if(remove_ground_)  removeGround(pcl_pc);
        if(apply_reverse_rotation_)  rotation(pcl_pc, rot_r_deg_, rot_p_deg_, rot_y_deg_, true);

        if(!pcl_pc->points.empty()){
            std::stringstream save_name_ss;
            save_name_ss << save_dir_ << "/" << ros_pc->header.stamp;
            save(save_name_ss.str(), pcl_pc);
            ++num_save_;
            if(save_merged_pcd_)    *merged_pc += *pcl_pc;
        }
        ++view_itr;
        ++num_msg_;

        debugPublication(*ros_pc, pcl_pc);
        if(sleep_for_debug_)    loop_rate.sleep();
    }

    if(save_merged_pcd_ && !merged_pc->points.empty()){
        std::string save_merged_pcd_name = save_dir_ + "/merged";
        save(save_merged_pcd_name, merged_pc);
    }
}

void rosbagToPcFiles::rotation(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc, float r_deg, float p_deg, float y_deg, bool inverse)
{
    Eigen::Affine3f transformatoin = pcl::getTransformation(0.0, 0.0, 0.0, degToRad(r_deg), degToRad(p_deg), degToRad(y_deg));
    if(inverse) transformatoin = transformatoin.inverse();
	pcl::transformPointCloud(*pcl_pc, *pcl_pc, transformatoin);
}

float rosbagToPcFiles::degToRad(double deg)
{
	double rad = deg / 180.0 * M_PI;
	rad = atan2(sin(rad), cos(rad));
	return rad;
}

void rosbagToPcFiles::filterXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc)
{
    pcl::PassThrough<pcl::PointXYZI> pt;
    if(filter_x_){
        pt.setInputCloud(pcl_pc);
        pt.setFilterFieldName("x");
        pt.setFilterLimits(x_min_, x_max_);
        pt.filter(*pcl_pc);
    }
    if(filter_y_){
        pt.setInputCloud(pcl_pc);
        pt.setFilterFieldName("y");
        pt.setFilterLimits(y_min_, y_max_);
        pt.filter(*pcl_pc);
    }
    if(filter_z_){
        pt.setInputCloud(pcl_pc);
        pt.setFilterFieldName("z");
        pt.setFilterLimits(z_min_, z_max_);
        pt.filter(*pcl_pc);
    }
}

void rosbagToPcFiles::removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_pc)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_obstacle_pc (new pcl::PointCloud<pcl::PointXYZI>);
    tmp_obstacle_pc->points.resize(pcl_pc->points.size());
    size_t obstacle_counter = 0;

    float min[grid_dim_][grid_dim_];
    float max[grid_dim_][grid_dim_];
    bool init[grid_dim_][grid_dim_];
    memset(&init, 0, grid_dim_*grid_dim_);

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

void rosbagToPcFiles::debugPublication(sensor_msgs::PointCloud2 original_ros_pc, const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pcl_pc)
{
    const std::string debug_frame = "debug";

    original_ros_pc.header.frame_id = debug_frame;
    pub_original_.publish(original_ros_pc);

    if(!filtered_pcl_pc->points.empty()){
        sensor_msgs::PointCloud2 filtered_ros_pc;
        pcl::toROSMsg(*filtered_pcl_pc, filtered_ros_pc);
        filtered_ros_pc.header.frame_id = debug_frame;
        pub_filtered_.publish(filtered_ros_pc);
    }
}

void rosbagToPcFiles::outputInfo()
{
    std::string save_path = save_dir_ + "/info.txt";
    std::ofstream ofs(save_path);
    ofs << "num_save / num_msg = " << num_save_ / (double)num_msg_ * 100.0 << " %" << std::endl;
    ofs.close();
    std::cout << "Save: " << save_path << std::endl;
}