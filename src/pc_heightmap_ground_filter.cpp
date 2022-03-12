#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PcHeightmapGroundFilter{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nhPrivate_;
		/*subscriber*/
		ros::Subscriber sub_;
		/*publisher*/
		ros::Publisher pub_ground_;
		ros::Publisher pub_obstacle_;
        /*buffer*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc {new pcl::PointCloud<pcl::PointXYZI>};
		/*parameter*/
		double m_per_cell_;
		int grid_dim_;
		double height_diff_threshold_;

	public:
		PcHeightmapGroundFilter();
		void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		// void pcToRings(const sensor_msgs::PointCloud2& pc_msg);
		// void ringsToImage(void);
		// void publication(std_msgs::Header header);
};

PcHeightmapGroundFilter::PcHeightmapGroundFilter()
	: nhPrivate_("~")
{
	std::cout << "--- pc_heightmap_ground_filter ---" << std::endl;
	/*parameter*/
	nhPrivate_.param("m_per_cell", m_per_cell_, 0.5);
	std::cout << "m_per_cell_ = " << m_per_cell_ << std::endl;
	nhPrivate_.param("grid_dim", grid_dim_, 320);
	std::cout << "grid_dim_ = " << grid_dim_ << std::endl;
	nhPrivate_.param("height_diff_threshold", height_diff_threshold_, 0.01);
	std::cout << "height_diff_threshold_ = " << height_diff_threshold_ << std::endl;
	/*subscriber*/
	sub_ = nh_.subscribe("/point_cloud", 1, &PcHeightmapGroundFilter::callback, this);
	/*publisher*/
	pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_heightmap_ground_filter/ground", 1);
	pub_obstacle_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_heightmap_ground_filter/obstacle", 1);
}

void PcHeightmapGroundFilter::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *pc);
	// for(size_t i=0 ; i<_rings.size() ; ++i){
	// 	_rings[i]->points.clear();
	// }
	// pcToRings(*msg);
	// ringsToImage();
	// publication(msg->header);
}

// void PcHeightmapGroundFilter::pcToRings(const sensor_msgs::PointCloud2& pc_msg)
// {
// 	sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(pc_msg,"ring");
// 	sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc_msg,"x");
// 	sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc_msg,"y");
// 	sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc_msg,"z");
// 	sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(pc_msg,"intensity");

// 	for( ; iter_ring!=iter_ring.end() ; ++iter_ring, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity){
// 		pcl::PointXYZI tmp;
// 		tmp.x = *iter_x;
// 		tmp.y = *iter_y;
// 		tmp.z = *iter_z;
// 		tmp.intensity = *iter_intensity;
// 		_rings[*iter_ring]->points.push_back(tmp);
// 	}
// }

// void PcHeightmapGroundFilter::ringsToImage(void)
// {
// 	/*reset*/
// 	// _img_cv_64f = cv::Mat::zeros(_num_ring, _points_per_ring, CV_64FC1);
// 	_img_cv_64f = cv::Mat(_num_ring, _points_per_ring, CV_64FC1, cv::Scalar(-1));
// 	/*input*/
// 	double angle_resolution = 2*M_PI/(double)_points_per_ring;
// 	for(size_t i=0 ; i<_rings.size() ; ++i){
// 		int row = _rings.size() - i - 1;
// 		for(size_t j=0 ; j<_rings[i]->points.size() ; ++j){
// 			double angle = atan2(_rings[i]->points[j].y, _rings[i]->points[j].x);
// 			int col = _points_per_ring - (int)((angle + M_PI)/angle_resolution) - 1;
// 			if(std::isnan(_rings[i]->points[j].x) || std::isnan(_rings[i]->points[j].y))    continue;
// 			_img_cv_64f.at<double>(row, col) = sqrt(_rings[i]->points[j].x*_rings[i]->points[j].x + _rings[i]->points[j].y*_rings[i]->points[j].y);
// 		}
// 	}
// 	/*convert*/
// 	_img_cv_64f.convertTo(_img_cv_16u, CV_16UC1, 1/_depth_resolution, 0);
// 	_img_cv_64f.convertTo(_img_cv_8u, CV_8UC1, 255/_max_range, 0);
// 	/*save*/
// 	if(_save_limit > 0 && _save_counter < _save_limit){
// 		/*CV_64FC1*/
// 		std::string save_mat64f_path = _save_root_path + "/" + _save_img_name + std::to_string(_save_counter) + "_64f.xml";
// 		cv::FileStorage fs(save_mat64f_path, cv::FileStorage::WRITE);
// 		if(!fs.isOpened()){
// 			std::cout << save_mat64f_path << " cannot be opened" << std::endl;
// 			exit(1);
// 		}
// 		fs << "mat" << _img_cv_64f;
// 		fs.release();
// 		/*CV_16UC1*/
// 		std::string save_img16u_path = _save_root_path + "/"  + _save_img_name + std::to_string(_save_counter) + "_16u.jpg";
// 		cv::imwrite(save_img16u_path, _img_cv_16u);
// 		/*CV_8UC1*/
// 		std::string save_img8u_path = _save_root_path + "/"  + _save_img_name + std::to_string(_save_counter) + "_8u.jpg";
// 		cv::imwrite(save_img8u_path, _img_cv_8u);
// 		/*count*/
// 		++_save_counter;
// 		/*print*/
// 		std::cout << "----- " << _save_counter << " -----" << std::endl;
// 		std::cout << "_img_cv_64f: " << _img_cv_64f.size().height << " x " << _img_cv_64f.size().width << std::endl;
// 		std::cout << "_img_cv_16u: " << _img_cv_16u.size().height << " x " << _img_cv_16u.size().width << std::endl;
// 		std::cout << "_img_cv_8u: " << _img_cv_8u.size().height << " x " << _img_cv_8u.size().width << std::endl;
// 		//for(int row=0 ; row<_img_cv_64f.size().height ; row+=_img_cv_64f.size().height/3){
// 		//	for(int col=0 ; col<_img_cv_64f.size().width ; col+=_img_cv_64f.size().width/3){
// 		//		std::cout << "_img_cv_64f.at<double>(" << row << ", " << col << ") = " << _img_cv_64f.at<double>(row, col) << std::endl;
// 		//		std::cout << "_img_cv_16u.at<unsigned short>(" << row << ", " << col << ") = " << _img_cv_16u.at<unsigned short>(row, col) << std::endl;
// 		//		std::cout << "_img_cv_16u.at<unsigned long int>(" << row << ", " << col << ") = " << _img_cv_16u.at<unsigned long int>(row, col) << std::endl;
// 		//		std::cout << "(int)_img_cv_8u.at<unsigned char>(" << row << ", " << col << ") = " << (int)_img_cv_8u.at<unsigned char>(row, col) << std::endl;
// 		//	}
// 		//}
// 	}
// }

// void PcHeightmapGroundFilter::publication(std_msgs::Header header)
// {
// 	sensor_msgs::ImagePtr img_ros_64f = cv_bridge::CvImage(header, "64FC1", _img_cv_64f).toImageMsg();
// 	sensor_msgs::ImagePtr img_ros_16u = cv_bridge::CvImage(header, "mono16", _img_cv_16u).toImageMsg();
// 	sensor_msgs::ImagePtr img_ros_8u = cv_bridge::CvImage(header, "mono8", _img_cv_8u).toImageMsg();
// 	_pub_img_64f.publish(img_ros_64f);
// 	_pub_img_16u.publish(img_ros_16u);
// 	_pub_img_8u.publish(img_ros_8u);

// 	/*check*/
// 	//cv_bridge::CvImagePtr cv_ptr_64f = cv_bridge::toCvCopy(img_ros_64f, img_ros_64f->encoding);
// 	//cv_bridge::CvImagePtr cv_ptr_16u = cv_bridge::toCvCopy(img_ros_16u, img_ros_16u->encoding);
// 	//cv_bridge::CvImagePtr cv_ptr_8u = cv_bridge::toCvCopy(img_ros_8u, img_ros_8u->encoding);
// 	//for(int row=0 ; row<cv_ptr_64f->image.size().height ; row+=cv_ptr_64f->image.size().height/3){
// 	//	for(int col=0 ; col<cv_ptr_64f->image.size().width ; col+=cv_ptr_64f->image.size().width/3){
// 	//		std::cout << "_img_cv_64f.at<double>(" << row << ", " << col << ") = " << _img_cv_64f.at<double>(row, col) << std::endl;
// 	//		std::cout << "cv_ptr_64f->image.at<double>(" << row << ", " << col << ") = " << cv_ptr_64f->image.at<double>(row, col) << std::endl;
// 	//		std::cout << "cv_ptr_16u->image.at<unsigned short>(" << row << ", " << col << ") = " << cv_ptr_16u->image.at<unsigned short>(row, col) << std::endl;
// 	//		std::cout << "(int)cv_ptr_8u->image.at<unsigned char>(" << row << ", " << col << ") = " << (int)cv_ptr_8u->image.at<unsigned char>(row, col) << std::endl;
// 	//	}
// 	//}
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_heightmap_ground_filter");
	
	PcHeightmapGroundFilter pc_heightmap_ground_filter;

	ros::spin();
}