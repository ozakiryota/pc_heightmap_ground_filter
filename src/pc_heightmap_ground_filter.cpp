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
		double meter_per_cell_;
		int cell_per_axis_;
		double min_obstacle_height_;

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
	nh_private_.param("meter_per_cell", meter_per_cell_, 0.5);
	std::cout << "meter_per_cell_ = " << meter_per_cell_ << std::endl;
	nh_private_.param("cell_per_axis", cell_per_axis_, 320);
	std::cout << "cell_per_axis_ = " << cell_per_axis_ << std::endl;
	nh_private_.param("min_obstacle_height", min_obstacle_height_, 0.01);
	std::cout << "min_obstacle_height_ = " << min_obstacle_height_ << std::endl;
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
    float min[cell_per_axis_][cell_per_axis_];
    float max[cell_per_axis_][cell_per_axis_];
    bool init[cell_per_axis_][cell_per_axis_];
    memset(&init, 0, cell_per_axis_*cell_per_axis_);

    /*build height map*/
    for(size_t i = 0; i < pc->points.size(); ++i){
        int x = ((cell_per_axis_ / 2) + pc->points[i].x / meter_per_cell_);
        int y = ((cell_per_axis_ / 2) + pc->points[i].y / meter_per_cell_);
        if(x >= 0 && x < cell_per_axis_ && y >= 0 && y < cell_per_axis_){
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
        int x = (cell_per_axis_ / 2) + pc->points[i].x / meter_per_cell_;
        int y = (cell_per_axis_ / 2) + pc->points[i].y / meter_per_cell_;
        if(x >= 0 && x < cell_per_axis_ && y >= 0 && y < cell_per_axis_ && init[x][y]){
            if(max[x][y] - min[x][y] > min_obstacle_height_){
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