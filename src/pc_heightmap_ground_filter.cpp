#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PcHeightmapGroundFilter{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber sub_;
		/*publisher*/
		ros::Publisher pub_ground_;
		ros::Publisher pub_obstacle_;
        /*buffer*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_pc_ {new pcl::PointCloud<pcl::PointXYZI>};
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc_ {new pcl::PointCloud<pcl::PointXYZI>};
		/*parameter*/
		double m_per_cell_;
		int grid_dim_;
		double height_diff_threshold_;

	public:
		PcHeightmapGroundFilter();
		void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void filter(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
        void useHeightmap(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, size_t &obstacle_counter, size_t &ground_counter);
		void publication(std_msgs::Header header);
};

PcHeightmapGroundFilter::PcHeightmapGroundFilter()
	: nh_private_("~")
{
	std::cout << "----- pc_heightmap_ground_filter -----" << std::endl;
	/*parameter*/
	nh_private_.param("m_per_cell", m_per_cell_, 0.5);
	std::cout << "m_per_cell_ = " << m_per_cell_ << std::endl;
	nh_private_.param("grid_dim", grid_dim_, 320);
	std::cout << "grid_dim_ = " << grid_dim_ << std::endl;
	nh_private_.param("height_diff_threshold", height_diff_threshold_, 0.01);
	std::cout << "height_diff_threshold_ = " << height_diff_threshold_ << std::endl;
	/*subscriber*/
	sub_ = nh_.subscribe("/point_cloud", 1, &PcHeightmapGroundFilter::callback, this);
	/*publisher*/
	pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_heightmap_ground_filter/ground", 1);
	pub_obstacle_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_heightmap_ground_filter/obstacle", 1);
}

void PcHeightmapGroundFilter::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pc);
    filter(pc);
	publication(msg->header);
}

void PcHeightmapGroundFilter::filter(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
    obstacle_pc_->points.resize(pc->points.size());
    ground_pc_->points.resize(pc->points.size());

    size_t obstacle_counter = 0;
    size_t ground_counter = 0;
    useHeightmap(pc, obstacle_counter, ground_counter);

    obstacle_pc_->points.resize(obstacle_counter);
    ground_pc_->points.resize(ground_counter);
}

void PcHeightmapGroundFilter::useHeightmap(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, size_t &obstacle_counter, size_t &ground_counter)
{
    float min[grid_dim_][grid_dim_];
    float max[grid_dim_][grid_dim_];
    bool init[grid_dim_][grid_dim_];
    memset(&init, 0, grid_dim_*grid_dim_);

    /*build height map*/
    for(size_t i = 0; i < pc->points.size(); ++i){
        int x = ((grid_dim_ / 2) + pc->points[i].x / m_per_cell_);
        int y = ((grid_dim_ / 2) + pc->points[i].y / m_per_cell_);
        if(x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_){
            if(!init[x][y]){
                min[x][y] = pc->points[i].z;
                max[x][y] = pc->points[i].z;
                init[x][y] = true;
            }
            else{
                min[x][y] = (min[x][y] < pc->points[i].z ? min[x][y] : pc->points[i].z);
                max[x][y] = (max[x][y] > pc->points[i].z ? max[x][y] : pc->points[i].z);
            }
        }
    }

    /*display points where map has height-difference > threshold*/
    for(size_t i = 0; i < pc->points.size(); ++i){
        int x = (grid_dim_ / 2) + pc->points[i].x / m_per_cell_;
        int y = (grid_dim_ / 2) + pc->points[i].y / m_per_cell_;
        if(x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]){
            if(max[x][y] - min[x][y] > height_diff_threshold_){
                obstacle_pc_->points[obstacle_counter] = pc->points[i];
                obstacle_counter++;
            }
            else{
                ground_pc_->points[ground_counter] = pc->points[i];
                ground_counter++;
            }
        }
    }
}

void PcHeightmapGroundFilter::publication(std_msgs::Header header)
{
    if(!obstacle_pc_->points.empty()){
        sensor_msgs::PointCloud2 obstacle_ros_pc;
        pcl::toROSMsg(*obstacle_pc_, obstacle_ros_pc);
        obstacle_ros_pc.header = header;
        pub_obstacle_.publish(obstacle_ros_pc);
    }

    if(!ground_pc_->points.empty()){
        sensor_msgs::PointCloud2 ground_ros_pc;
        pcl::toROSMsg(*ground_pc_, ground_ros_pc);
        ground_ros_pc.header = header;
        pub_ground_.publish(ground_ros_pc);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_heightmap_ground_filter");
	
	PcHeightmapGroundFilter pc_heightmap_ground_filter;

	ros::spin();
}