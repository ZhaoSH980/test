/*
1. The AwsSubscriber code/header file has been modified from what Wei Liang has developed to cater for the needs (different QoS of different async waitiset participants ) of Sensor Fusion
2. Care has already been taken that if old data samples of either vision of lidar come to this module, then systems waits for new data and error message would be sent to health monitoring
3. This old data can come when vision or lidar sensor dies or because of DDS lag
4.

Important - th_dist_ -> it defines what all points belongs to ground plane

sensor_height_ is not used as a paramter in any calculation - care needs to be taken in setting the passthrough filters. These vary with respect to sensor height and they have to be changed manually.
1). try decreasing th_seeds_
2). ESTIMATED MEAN is going to get affected by below commands - hence need to give less or accurate th_seeds_
	d_ = -(normal_.transpose() * seeds_mean)(0, 0);
	th_dist_d_ = th_dist_ - d_;  //th_dist_ is a parameter we set in the beginning
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
//#include "convert.h"
//#include "lidarseg_ros/json.hpp"

/*
//#include "pcl_to_dds.h"
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>
#include <rti/util/util.hpp> // for sleep()
#include <dds/domain/ddsdomain.hpp>
#include <dds/core/ddscore.hpp>
#include <rti/core/cond/AsyncWaitSet.hpp>
#include "AwsSubscriber.hpp"
*/
#include <string>
#include <iostream>
#include <sys/time.h>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <signal.h>

#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h> // convert between ROS & PCL

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/crop_box.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// using eigen lib
#include <Eigen/Dense>

//#include <msgRsLidar.hpp>
//#include <SensorFusion.hpp> //IDL
//#include "health_monitoring.hpp"


/*************************kalman filter tracking****************************/
#include "../include/Hungarian.h"
#include "../include/KalmanTracker.h"
#include "assert.h"
std::vector<TrackingObject> currObjects;
std::vector<TrackingObject> prevTrackingObjects;
int frame_id = 1;
/***************************************************************************/
#define VPoint pcl::PointXYZI
#define THREAD_POOL_SIZE 5
#define MAX_OBJECTS_PER_THREAD 2048

// SIZE limits for bounding box to be published
// boxes/hulls of size greater than TH_UP or less than TH_DOWN are not published
#define TH_UP   13// because the size of bus in SG is upto 12m.
#define TH_DOWN 0.2

#define DEBUG false

using namespace std;
using namespace cv;
//using json = nlohmann::json;

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;



volatile sig_atomic_t flag = 1;
int lidar_number = 0;
int NO_OBJECT = 0;
float TH_DIST_D_ = 0;

// number of point cloud frames processed
int pcl_count = 0 ;
float times = 0 ;
float dimen1Max, dimen1Min, dimen2Min, dimen2Max, dimen3Min, dimen3Max;

// scaling_factor is inversely proportional to undersegmentation
float scaling_factor = 2; //3.5; // scaling factor for binary image that effects clustering

float dist_threshold = 13; // threshold to remove points that occur by obstruction of the self driving car
float average_z_20 = 0;  //average of 20 points with lowest Z

// count the bounding box in each frame
int box_count_check;
int box_count_check2;

std::string  cluster_topic, polygon_box_topic;

//msgFusedObjects vec_of_objects_to_publish;
//rslidar_pointcloud::Convert *decoder_l, *decoder_c, *decoder_r;

//ground seed points
pcl::PointCloud<VPoint>::Ptr g_seeds_pc(new pcl::PointCloud<VPoint>());
//ground points
pcl::PointCloud<VPoint>::Ptr g_ground_pc(new pcl::PointCloud<VPoint>());
//duplicate of above
pcl::PointCloud<VPoint>::Ptr g_ground_pc_dup(new pcl::PointCloud<VPoint>());
//non-ground points
pcl::PointCloud<VPoint>::Ptr g_not_ground_pc(new pcl::PointCloud<VPoint>());
//duplicate of above
pcl::PointCloud<VPoint>::Ptr g_not_ground_pc_dup(new pcl::PointCloud<VPoint>());

/////////// ros sub & pub

ros::Publisher _pub;
ros::Subscriber sub_c, sub_l, sub_r;
//pcl::PointCloud<pcl::PointXYZI> cloud_c, cloud_l, cloud_r;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_c;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_l;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_r;
ros::Publisher polygon_box_pub_;
ros::Publisher pub_marker1, pub_marker2, pub_marker3, pub_marker4, pub_marker5, pub_marker6, pub_marker7;

/////////////////////////


/*
//functions for lidar driver
rslidar_pointcloud::Convert* populate_decoder(std::string filename)
{
	json j;
	std::fstream f(filename);
	f >> j;

	int domain;
	string data_dir, qos_path, profile_name, curves_path, angle_path, channel_path, curves_rate_path, model_name, pub_topic, sub_topic;

	domain  = j["rs_pointcloud"]["domain_id"];
	data_dir  = j["rs_pointcloud"]["data_base_dir"];
	qos_path = j["rs_pointcloud"]["qos_path"];
	profile_name = j["rs_pointcloud"]["profile_name"];
	curves_path = j["rs_pointcloud"]["curves_path"];
	angle_path = j["rs_pointcloud"]["angle_path"];
	channel_path = j["rs_pointcloud"]["channel_path"];
	curves_rate_path = j["rs_pointcloud"]["curves_rate_path"];
	pub_topic = j["rs_pointcloud"]["pub_topic"];
	model_name = j["rs_driver"]["model_name"];
	sub_topic = j["rs_driver"]["scan_pub"];


	std::cout << "====================== configuration ==============================" << std::endl;
	std::cout << "domain: " << domain << std::endl;
	std::cout << "qos path: " << qos_path << std::endl;
	std::cout << "profile name: " << profile_name << std::endl;
	std::cout << "profile: " << profile_name << std::endl;

	curves_path = data_dir + curves_path;
	angle_path = data_dir + angle_path;
	channel_path = data_dir + channel_path;
	if (curves_rate_path != "")
		curves_rate_path = data_dir + curves_rate_path;

	std::cout << "\ndata dir: " << data_dir << std::endl;
	std::cout << "curves file: " << curves_path << std::endl;
	std::cout << "angle file: " << angle_path << std::endl;
	std::cout << "channel file: " << channel_path << std::endl;
	std::cout << "curves_rate_path: " << curves_rate_path << std::endl;
	std::cout << "model: " << model_name << std::endl;
	std::cout << "subscribe packets topic: " << sub_topic << std::endl;
	std::cout << "publish points topic: " << pub_topic << std::endl;

	std::cout << "====================================================================" << std::endl;

	rslidar_pointcloud::Convert* conv = new rslidar_pointcloud::Convert(sub_topic, pub_topic, domain, qos_path, profile_name, curves_path, angle_path,
		channel_path, curves_rate_path, model_name);

	return conv;
}
*/


/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool inline point_cmp(VPoint& a, VPoint& b)
{
    return a.z < b.z;
}


/*
    @brief point_dist_cmp -> distance function to remove points that are closer to autonomous vehicle.
    @return true if squared distance is less than 9 units
*/
//have set the paramters based on the car dimensions
bool inline point_dist_cmp(VPoint& a)
{
    if (a.x > -2 and a.x < 4.2 )
    {
        //if (a.y < 1.9 and a.y > -1.9)
        if (a.y < 1.4 and a.y > -1.4)
        {
            return true;
        }
    }
    else
        return false;

    //return (a.z * a.z + a.y * a.y + a.x * a.x) < dist_threshold;


    ///TO-DO: can replace with path-through filter


}

// curvature represent integer label for object. Method to compare label of two objects
bool inline point_cmp_label(pcl::PointXYZRGBNormal& a, pcl::PointXYZRGBNormal& b)
{
    return a.curvature < b.curvature;
}

bool inline point_cmp_for_box_x_y(pcl::PointXYZRGBNormal& a, pcl::PointXYZRGBNormal& b)
{
    return (a.x < b.x || (a.x == b.x && a.y < b.y));
}

bool inline point_cmp_for_box_x_z(pcl::PointXYZRGBNormal& a, pcl::PointXYZRGBNormal& b)
{
    return (a.x < b.x || (a.x == b.x && a.z < b.z));
}

bool inline point_cmp_for_box_y_z(pcl::PointXYZRGBNormal& a, pcl::PointXYZRGBNormal &b)
{
    return (a.y < b.y || (a.y == b.y && a.z < b.z));
}

bool inline comp(float& a, float& b)
{
    return (a < b);
}

// functor to compare distance of two points with threshold distance (_dist2)
struct EuclideanDistanceFunctor
{
    int _dist2;
    EuclideanDistanceFunctor(int dist) : _dist2(dist * dist) {}

    bool operator()(const Point& lhs, const Point& rhs) const
    {
        return ((lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y)) < _dist2;
    }
};

// Cross product of two vectors OA and OB
// returns positive for counter clockwise turn, and negative for clockwise turn
float inline cross_product_x_y(pcl::PointXYZRGBNormal& O, pcl::PointXYZRGBNormal& A, pcl::PointXYZRGBNormal& B)
{
    return (A.x - O.x) * (B.y - O.y)
           - (A.y - O.y) * (B.x - O.x);
}

float inline cross_product_x_z(pcl::PointXYZRGBNormal& O, pcl::PointXYZRGBNormal& A, pcl::PointXYZRGBNormal& B)
{
    return (A.x - O.x) * (B.z - O.z)
           - (A.z - O.z) * (B.x - O.x);
}

float inline cross_product_y_z(pcl::PointXYZRGBNormal& O, pcl::PointXYZRGBNormal& A, pcl::PointXYZRGBNormal& B)
{
    return (A.y - O.y) * (B.z - O.z)
           - (A.z - O.z) * (B.y - O.y);
}

// Returns a list of points on the convex hull in counter-clockwise order
vector<pcl::PointXYZRGBNormal> convex_hull(vector<pcl::PointXYZRGBNormal>& A, int type)
{
    int n = A.size(), k = 0;

    if (n <= 3)
        return A;

    vector<pcl::PointXYZRGBNormal> ans(2 * n);

    // Sort points lexicographically
    if (type == 0)
        sort(A.begin(), A.end(), point_cmp_for_box_x_y);
    else
    {
        if (type == 1)
        {
            sort(A.begin(), A.end(), point_cmp_for_box_x_z);
        }
        else
        {
            sort(A.begin(), A.end(), point_cmp_for_box_y_z);
        }
    }

    // Build lower hull
    for (int i = 0; i < n; ++i)
    {

        // If the point at K-1 position is not a part
        // of hull as vector from ans[k-2] to ans[k-1]
        // and ans[k-2] to A[i] has a clockwise turn
        if (type == 0)
        {
            while (k >= 2 && cross_product_x_y(ans[k - 2], ans[k - 1], A[i]) <= 0)
                k--;
        }
        else
        {
            if (type == 1)
            {
                while (k >= 2 && cross_product_x_z(ans[k - 2], ans[k - 1], A[i]) <= 0)
                    k--;
            }
            else
            {
                while (k >= 2 && cross_product_y_z(ans[k - 2], ans[k - 1], A[i]) <= 0)
                    k--;
            }
        }
        ans[k++] = A[i];
    }

    // Build upper hull
    for (size_t i = n - 1, t = k + 1; i > 0; --i)
    {
        // If the point at K-1 position is not a part
        // of hull as vector from ans[k-2] to ans[k-1]
        // and ans[k-2] to A[i] has a clockwise turn
        if (type == 0)
        {
            while (k >= t && cross_product_x_y(ans[k - 2], ans[k - 1], A[i - 1]) <= 0)
                k--;
        }
        else
        {
            if (type == 1)
            {
                while (k >= t && cross_product_x_z(ans[k - 2], ans[k - 1], A[i - 1]) <= 0)
                    k--;
            }
            else
            {
                while (k >= t && cross_product_y_z(ans[k - 2], ans[k - 1], A[i - 1]) <= 0)
                    k--;
            }
        }

        ans[k++] = A[i - 1];
    }

    // Resize the array to desired size
    ans.resize(k - 1);

    return ans;
}


// (X[i], Y[i]) are coordinates of i'th point.
float polygonArea(vector<pcl::PointXYZRGBNormal>& X, int n, int type)
{
    // Initialze area
    float area = 0.0;

    // Calculate value of shoelace formula
    int j = n - 1;
    for (int i = 0; i < n; i++)
    {
        if (type == 0)
            area += (X[j].x + X[i].x) * (X[j].y - X[i].y);
        else
        {
            if (type == 1)
            {
                area += (X[j].x + X[i].x) * (X[j].z - X[i].z);
            }
            else
            {
                area += (X[j].y + X[i].y) * (X[j].z - X[i].z);
            }
        }
        j = i;  // j is previous vertex to i
    }

    // Return absolute value
    return abs(area / 2.0);
}


/*
    makeImageFromPointCloud method to convert 3D pointcloud into 2D binary image

*/
cv::Mat makeImageFromPointCloud(pcl::PointCloud<VPoint>& cloud, std::string dimensionToRemove)
{
    VPoint cloudMin, cloudMax;

    //cout << cloud.points.size() <<  endl;

    if (cloud.points.size() == 0)
    {
        NO_OBJECT = 1;
        cv::Mat img;
        return img;
    }

    pcl::getMinMax3D(cloud, cloudMin, cloudMax);

    std::string dimen1, dimen2;

    dimen1 = "x";
    dimen2 = "y";
    dimen1Min = cloudMin.x;
    dimen1Max = cloudMax.x;
    dimen2Min = cloudMin.y;
    dimen2Max = cloudMax.y;
    dimen3Min = cloudMin.z;
    dimen3Max = cloudMax.z;

    // creating image matrix of size factor* pointcloud size with initial value zero.
    // zero represent black pixel and 255 represent white pixel
    // bigger value of factor removes chance of undersegmentation
    cv::Mat mat(static_cast<int>(scaling_factor * (-dimen1Min + dimen1Max + 1)), static_cast<int>(scaling_factor * (-dimen2Min + dimen2Max + 1)), CV_8UC1);
    mat = cv::Scalar(0);
    int intensity = 255;
    int k = 0 ;
    int m = 0;

    // projecting cloud point into an integer coordinates then assign white pixel to it.
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {

        k = static_cast<int>(scaling_factor * (cloud.points[i].x - dimen1Min));
        m = static_cast<int>(scaling_factor * (cloud.points[i].y - dimen2Min));
        mat.at<uchar>(k, m) = 255;
    }

    return mat;
}


// isvalid method to check whether a bounding box has size of interest.
// too small or too big bounding boxes are usually ignored for better view.
bool inline isvalid(float& current_hull_dim_x, float& current_hull_dim_y, float &current_hull_dim_z)
{
    if (current_hull_dim_x > TH_UP || current_hull_dim_y > TH_UP || current_hull_dim_z > TH_UP )
    {
        box_count_check++;
        return false;
    }
    else
    {
        if (current_hull_dim_x < TH_DOWN && current_hull_dim_y < TH_DOWN && current_hull_dim_z < TH_DOWN)
        {
            box_count_check++;
            box_count_check2++;
            return false;
        }
        else
            return true;
    }
}

//////////////////////////////////////////////////////////////////////////////////////

/*
    @brief Ground Plane fitting ROS Node.
    @param Velodyne Pointcloud topic.
    @param Sensor Model.
    @param Sensor height for filtering error mirror points.
    @param Num of segment, iteration, LPR
    @param Threshold of seeds distance, and ground plane distance

    @subscirbe:/velodyne_points
    @publish:/points_no_ground, /points_ground
*/
class GroundPlaneFit
{
public:
    GroundPlaneFit();

//private:
    std::string point_topic_;

    int sensor_model_;
    double sensor_height_;
    int num_iter_;
    int num_lpr_;
    double th_seeds_;
    double th_dist_;

    void velodyne_callback_(pcl::PointCloud<pcl::PointXYZI>& pcl_pc_combined);
    void estimate_plane_(void);
    void extract_initial_seeds_(pcl::PointCloud<VPoint>& p_sorted);
    void ClusteringCallback(pcl::PointCloud<VPoint>& in_cloud_msg);

    float d_;
    MatrixXf normal_;
    float th_dist_d_;
};


/*
    @brief Constructor of GPF Node.
    @return void
*/
GroundPlaneFit::GroundPlaneFit()
{
    // Init ROS related
    //cout << "Inititalizing Ground Plane Fitter...\n sensor_height_\n" <<  endl;

    sensor_height_ =  0.36;//because the PCD is transformed to
    //cout << "Set Sensor Height: " << sensor_height_ <<  endl;

    num_iter_ =  25;//15;
    //cout << "Set Num of Iteration: " << num_iter_ <<  endl;

    num_lpr_ = 20;
    //cout << "Num of LPR(Lowest Point Representatives): " << num_lpr_ << endl;

    th_seeds_ = 0.5;//1.2;//1,2 - this is used to get the seeds from
    //cout << "Seeds Threshold (th_seeds_): " << th_seeds_ << endl;

    th_dist_ = 0.40;//0.45;//0.6;- From estimated ground plane, all the points that lie upto th_dist height will be considered as plane
    //cout << "Distance Threshold: " << th_dist_ << endl;

}



void GroundPlaneFit::estimate_plane_(void)
{
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc_dup, cov, pc_mean);

    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));

    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);

    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;//th_dist_ is a parameter we set in the beginning

    // return the equation parameters
}


//find initial seeds for groundplane extraction
void GroundPlaneFit::extract_initial_seeds_(pcl::PointCloud<VPoint>& p_sorted)
{
    float sum = 0;

    // make a copy of p_sorted points.
    pcl::PointCloud<VPoint> p_sorted1 = p_sorted ;

    // make_heap method creates a heap of pointcloud wrt to z axis.
    make_heap (p_sorted1.points.begin(), p_sorted1.points.end(), point_cmp);

    // find 'num_lpr_' minimum points and take their average.
    for (int i = 0; i < num_lpr_ ; i++)
    {
        pop_heap(p_sorted1.points.begin(), p_sorted1.points.end(), point_cmp);
        p_sorted1.points.pop_back();

        sum += p_sorted1.points.front().z;

    }

    average_z_20 += p_sorted1.points.front().z ;
    double lpr_height = sum / num_lpr_ ; // in case divide by 0

    // clear ground seed points
    g_seeds_pc->clear();

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seeds_)
        {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}


//////////////////////////////////////////////////////////////////////////////////////


void GroundPlaneFit::ClusteringCallback(pcl::PointCloud<VPoint>& laserCloudIn)
{
//	vec_of_objects_to_publish.Objects().clear();
//	vec_of_objects_to_publish.number_of_objects(0.0);

    std::cout << "Non-Ground Point cloud size: " << laserCloudIn.points.size() << std::endl;

    //we define this way to store various properties in PointXYZRGBNormal
    pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr cloudrgb (new pcl::PointCloud< pcl::PointXYZRGBNormal >);
    pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr polygonboxrgb (new pcl::PointCloud< pcl::PointXYZRGBNormal >);

    // call makeImageFromPointCloud method on input point cloud with projection on x y plane
    cv::Mat img = makeImageFromPointCloud(laserCloudIn, "z");

    //if no object is detected in the pcd
    if (NO_OBJECT == 1)
    {
//        vec_of_objects_to_publish.number_of_objects() = (int)0.0;
        cout << "*** --- NO OBJECT --- ***" << endl;
        return;
    }

    // Get all white pixel points
    vector<cv::Point> pts;
    findNonZero(img, pts);

    // Define the distance between clusters -> threshold distance to group a point in that cluster
    int euclidean_distance = 2.0;

    vector<int> labels;

    // partition method to cluster different white object in an image
    // based on Disjoint set union data structure
    //segmentation is performed here
    int n_labels = partition(pts, labels, EuclideanDistanceFunctor(euclidean_distance));


    // colors vector to store different colors for each object
    vector<Vec4i> colors;
    for (int i = 0; i < n_labels; i++)
    {
        colors.push_back(Vec4i(rand() & 255, rand() & 255, rand() & 255, i));
    }

    // Assign color to each pixels
    // pixels in same cluster are assigned same color
    // res matrix is now colored image
    Mat4i res(img.rows, img.cols, Vec4i(0, 0, 0, 0));
    for (int i = 0; i < pts.size(); i++)
    {
        res(pts[i]) = colors[labels[i]];
    }

    // copy pointcloud of point type PointXYZRGB to pointcloud of point type PointXYZRGBNormal 
    // curvature in normal represent label of object. Same object have same label.
    pcl::copyPointCloud(laserCloudIn, *cloudrgb);


    int k;
    int m;
    uint8_t r, g, b;
    uint32_t rgb;

    // transforming image back into point cloud.
    // project cloud points into x ,y coordinates.
    // assign color at this x,y coordinates to the cloud point.
    // assign label at this x,y coordinates to the curvature of cloud point
    for (size_t i = 0; i < cloudrgb->points.size (); i++)
    {
        k = static_cast<int>(scaling_factor * (laserCloudIn.points[i].x - dimen1Min));
        m = static_cast<int>(scaling_factor * (laserCloudIn.points[i].y - dimen2Min));


        //r = (uint8_t)(res.at<cv::Vec4i>(k, m)[2]);
        //g = (uint8_t)(res.at<cv::Vec4i>(k, m)[1]);
        //b = (uint8_t)(res.at<cv::Vec4i>(k, m)[0]);
        //rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        //cloudrgb->points[i].rgb = *reinterpret_cast<float*>(&rgb);
        cloudrgb->points[i].curvature = (int)(res.at<cv::Vec4i>(k, m)[3]);
    }

    /*
    * Method for drawing Convex Hull
    */

    // sort the point cloud using the label (curvature type in pointcloud)
    sort(cloudrgb->begin(), cloudrgb->end(), point_cmp_label);

    pcl::PointXYZRGBNormal current_point;
    vector<pcl::PointXYZRGBNormal> vector_of_current_boxpoints;


    visualization_msgs::MarkerArray marker_array_msg1;
    marker_array_msg1.markers.resize(500);
    visualization_msgs::MarkerArray marker_array_msg2;
    marker_array_msg2.markers.resize(500);
    visualization_msgs::MarkerArray marker_array_msg3;
    marker_array_msg3.markers.resize(500);

    //text visualization
    visualization_msgs::MarkerArray marker_array_msg4;
    marker_array_msg4.markers.resize(500);

    //centre visualization
    visualization_msgs::MarkerArray marker_array_msg5;
    marker_array_msg5.markers.resize(500);

    //centre visualization
    visualization_msgs::MarkerArray marker_array_msg6;
    marker_array_msg6.markers.resize(500);

    visualization_msgs::MarkerArray marker_array_msg7;
    marker_array_msg7.markers.resize(500);

//    int box_count = 0;
    int box_count_total = 0;
    int current_hull_value = 0;
    int box_no = 0;
    float maxx = -500, maxy = -100, maxz = -30, minx  = +500, miny = +100, minz = +30;
    float current_hull_dim_x, current_hull_dim_y, current_hull_dim_z;

    vector<Vertexes> Groups;
    Groups.clear();
//	msgFusedObjects vec_of_objects;
//	vec_of_objects.Objects().clear();
    
    //centre of the 3D bounding box
    vector<pcl::PointXYZ> vector_of_boxcentre;
    //size of box
    vector<pcl::PointXYZ> vector_of_boxsize;

    std::cout << "cloudrgb cluster size: " << cloudrgb->size() << std::endl;
    std::cout << "cloudrgb point size:   " << cloudrgb->points.size() << std::endl;

    //cloudrgb at this point is sorted based on the label of objects present in it
    for (int i = 0 ; i < cloudrgb->size(); i++)
    {
        current_point = cloudrgb->points[i];
        vector_of_current_boxpoints.push_back(current_point);

        //this will be entered only if all points belonging to one object gets pushed into vector_of_current_boxpoints
        if ((current_hull_value != current_point.curvature) || (i == (cloudrgb->size() - 1)))
        {

            //because when this loop is entered, the last push_back would be of the next object - hence need to be popped out
            vector_of_current_boxpoints.pop_back();

            //get the max and min in x y z dimensions of the current object/segment
            current_hull_dim_x = maxx - minx;
            current_hull_dim_y = maxy - miny;
            current_hull_dim_z = maxz - minz;

            //if the object dimensions are less than 13 and greater than 0.2
            if (isvalid(current_hull_dim_x, current_hull_dim_y, current_hull_dim_z) )
            {
                pcl::PointXYZ boxcentre_point;

                boxcentre_point.x = (maxx + minx) / 2;
                boxcentre_point.y = (maxy + miny) / 2;
                boxcentre_point.z = (maxz + minz) / 2;

                //box centre for the current object calculated and pushed nack
                vector_of_boxcentre.push_back(boxcentre_point);

                pcl::PointXYZ boxsize_point;

                boxsize_point.x = maxx - minx;
                boxsize_point.y = maxy - miny;
                boxsize_point.z = maxz - minz;
                //box size calculated and pushed back
                vector_of_boxsize.push_back(boxsize_point);

                box_count_total++;

//				FusedObject one_object;
                                std::vector<Vector3> points;
//				ConvexHullPoints convex_hull_points;

                // Find the convex hull
                //vector<pcl::PointXYZRGBNormal> bounding_box_points;
                vector<pcl::PointXYZRGBNormal> ans1 = convex_hull(vector_of_current_boxpoints, 0);
                float surfacearea1 = polygonArea(ans1, ans1.size(), 0);
                float volume1 = surfacearea1 * (maxz - minz);

                //convex hull bounding box is set to have the same height which is maxz-minz
//				convex_hull_points.height() = (float) maxz - minz;
//				convex_hull_points.points().clear();
                                points.clear();


                marker_array_msg1.markers[box_no].header.frame_id = "vehicle"; //"centre_lidar";
                marker_array_msg1.markers[box_no].id = box_no;
                marker_array_msg1.markers[box_no].action = visualization_msgs::Marker::ADD;
                marker_array_msg1.markers[box_no].scale.x = 0.1;
                marker_array_msg1.markers[box_no].scale.y = 0.1;
                marker_array_msg1.markers[box_no].scale.z = 0.1;                
                
                marker_array_msg2.markers[box_no] = marker_array_msg1.markers[box_no];
                marker_array_msg3.markers[box_no] = marker_array_msg1.markers[box_no];
                
                marker_array_msg1.markers[box_no].type = visualization_msgs::Marker::LINE_STRIP;
                marker_array_msg1.markers[box_no].color.b = 1.0;
                marker_array_msg1.markers[box_no].color.a = 1.0;
                marker_array_msg1.markers[box_no].lifetime = ros::Duration(0.1);
                
                marker_array_msg2.markers[box_no].type = visualization_msgs::Marker::LINE_STRIP;
                marker_array_msg2.markers[box_no].color.r = 1.0;
                marker_array_msg2.markers[box_no].color.a = 1.0;
                marker_array_msg2.markers[box_no].lifetime = ros::Duration(0.1);
                
                marker_array_msg3.markers[box_no].type = visualization_msgs::Marker::LINE_LIST;
                marker_array_msg3.markers[box_no].color.g = 1.0;
                marker_array_msg3.markers[box_no].color.a = 1.0;
                marker_array_msg3.markers[box_no].lifetime = ros::Duration(0.1);

                // text visualization
                marker_array_msg4.markers[box_no] = marker_array_msg1.markers[box_no];
                marker_array_msg4.markers[box_no].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker_array_msg4.markers[box_no].color.g = 1.0;
                marker_array_msg4.markers[box_no].color.r = 1.0;
                marker_array_msg4.markers[box_no].color.b = 0.0;
                marker_array_msg4.markers[box_no].color.a = 1.0;
                marker_array_msg4.markers[box_no].lifetime = ros::Duration(0.1);

                //centre visualization
                marker_array_msg5.markers[box_no] = marker_array_msg1.markers[box_no];
                marker_array_msg5.markers[box_no].type = visualization_msgs::Marker::SPHERE;
                marker_array_msg5.markers[box_no].color.g = 1.0;
                marker_array_msg5.markers[box_no].color.r = 1.0;
                marker_array_msg5.markers[box_no].color.b = 0.0;
                marker_array_msg5.markers[box_no].color.a = 1.0;
                marker_array_msg5.markers[box_no].lifetime = ros::Duration(0.1);

                int size_counter = 0;
                //pushing the convex hull points into boundary_box_points in this loop
                for (int i = 0; i < ans1.size(); i++)
                {
                    pcl::PointXYZRGBNormal boxpoint1 = ans1[i];
                    pcl::PointXYZRGBNormal boxpoint2 = ans1[i];
                    //cout << "boxpoint1: " << boxpoint1 << endl;
                    boxpoint1.z = minz;
                    boxpoint2.z = maxz;
                    //cout << "boxpoint1withz: " << boxpoint1 << endl;
                    //bounding_box_points.push_back(boxpoint1);
                    //bounding_box_points.push_back(boxpoint2);

                    //polygonboxrgb->points.push_back(boxpoint1);                    
                    
                    //polygonboxrgb->points.push_back(boxpoint2);    //commenting this line will not send the top layer of convex hull points into the polygonboxrgb
                    //cout<<"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxpolygonboxrgbxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"<<polygonboxrgb->points[1]<<endl;
                    geometry_msgs::Point p1;
                    p1.x = boxpoint1.x;
                    p1.y = boxpoint1.y;
                    p1.z = boxpoint1.z;
                    
                    geometry_msgs::Point p2;
                    p2.x = boxpoint2.x;
                    p2.y = boxpoint2.y;
                    p2.z = boxpoint2.z;
                    
                    marker_array_msg1.markers[box_no].points.push_back(p1);
                    marker_array_msg2.markers[box_no].points.push_back(p2);
                    
                    marker_array_msg3.markers[box_no].points.push_back(p1);
                    marker_array_msg3.markers[box_no].points.push_back(p2);

                    size_counter++;
					Vector3 pt;
					pt.x = boxpoint2.x;
					pt.y = boxpoint2.y;
					pt.z = boxpoint2.z;
					points.push_back(pt);
					//convex_hull_points.size(size_counter);
                }
                if (DEBUG)
                    cout << "size_counter: " << size_counter << endl;

                //add again the 1st vertex to close the polygon shape
                pcl::PointXYZRGBNormal boxpoint1 = ans1[0];
                pcl::PointXYZRGBNormal boxpoint2 = ans1[0];

                geometry_msgs::Point p1;
                p1.x = boxpoint1.x;
                p1.y = boxpoint1.y;
                p1.z = minz;
            
                geometry_msgs::Point p2;
                p2.x = boxpoint2.x;
                p2.y = boxpoint2.y;
                p2.z = maxz;

                marker_array_msg1.markers[box_no].points.push_back(p1);
                marker_array_msg2.markers[box_no].points.push_back(p2); 

                marker_array_msg4.markers[box_no].pose.position.x = p2.x;
                marker_array_msg4.markers[box_no].pose.position.y = p2.y;
                marker_array_msg4.markers[box_no].pose.position.z = p2.z;
                marker_array_msg4.markers[box_no].scale.x = 1;
                marker_array_msg4.markers[box_no].scale.y = 1;
                marker_array_msg4.markers[box_no].scale.z = 1;               
                std::string text = std::to_string(boxcentre_point.x) + ","
                                    + std::to_string(boxcentre_point.y) + ","
                                    + std::to_string(boxcentre_point.z);  
                marker_array_msg4.markers[box_no].text = text;


                marker_array_msg5.markers[box_no].pose.position.x = boxcentre_point.x;
                marker_array_msg5.markers[box_no].pose.position.y = boxcentre_point.y;
                marker_array_msg5.markers[box_no].pose.position.z = boxcentre_point.z;
                marker_array_msg5.markers[box_no].scale.x = 1;
                marker_array_msg5.markers[box_no].scale.y = 1;
                marker_array_msg5.markers[box_no].scale.z = 1;
				//one_object.boundary() = convex_hull_points;
				//vec_of_objects.Objects().push_back(Vertexes);
                Groups.push_back(points);
//                box_count++;
                box_no += 1;
            }

            //set current_hull_value to the object label
            current_hull_value = current_point.curvature;

            //setting maxx, minx and so on
            maxx = current_point.x;
            minx = current_point.x;
            maxy = current_point.y;
            miny = current_point.y;
            maxz = current_point.z;
            minz = current_point.z;

			vector_of_current_boxpoints.erase(vector_of_current_boxpoints.begin(), vector_of_current_boxpoints.end());
			vector_of_current_boxpoints.push_back(current_point);
        }

        if (current_point.x > maxx) maxx = current_point.x;
        if (current_point.y > maxy) maxy = current_point.y;
        if (current_point.z > maxz) maxz = current_point.z;
        if (current_point.x < minx) minx = current_point.x;
        if (current_point.y < miny) miny = current_point.y;
        if (current_point.z < minz) minz = current_point.z;
    }
    /*
    	//if number of detected object segments are greater than zero
    	//publish in dds format
    	if ( (int)vec_of_objects.Objects().size() > 0)
    	{

    		vec_of_objects.number_of_objects() = (int)vec_of_objects.Objects().size();

    		for (int k = 0; k < (int)vec_of_objects.Objects().size(); k++)
    		{

    			//vec_of_objects.Objects()[k].track_id((int)1 * (int)k);
    			vec_of_objects.Objects()[k].track_id((int)0);// Set to Zero - means no tracking

    			vec_of_objects.Objects()[k].lidar_on((int)(1.0));
    			vec_of_objects.Objects()[k].vision_on((int)(0.0));
    			vec_of_objects.Objects()[k].radar_on((int)(0.0));

    			vec_of_objects.Objects()[k].obstacle_age() = 0;
    			vec_of_objects.Objects()[k].track_id() = 0;
    			vec_of_objects.Objects()[k].obstacle_type() = OBSTACLE_TYPE::UNKNOWN;

    			//the location_x, location_y and location_z are estimated as the centre of the bounding box
    			float location_x, location_y, location_z;
    			float x_length, y_length, z_length;
    			location_x = vector_of_boxcentre[k].x;
    			location_y = vector_of_boxcentre[k].y;
    			location_z = vector_of_boxcentre[k].z;

    			x_length = vector_of_boxsize[k].x;
    			y_length = vector_of_boxsize[k].y;
    			z_length = vector_of_boxsize[k].z;


    			vec_of_objects.Objects()[k].location_x() = location_x;
    			vec_of_objects.Objects()[k].location_y() = location_y;
    			vec_of_objects.Objects()[k].location_z() = location_z;

    			vec_of_objects.Objects()[k].direction_x() = 1.0;
    			vec_of_objects.Objects()[k].direction_y() = 0.0;
    			vec_of_objects.Objects()[k].direction_z() = 0.0;

    			vec_of_objects.Objects()[k].size_x_length() = x_length;
    			vec_of_objects.Objects()[k].size_y_width() = y_length;
    			vec_of_objects.Objects()[k].size_z_height() = z_length + 2*TH_DIST_D_ + sensor_height_;
    		}

            if (DEBUG) {
    		    cout << "\n\n**********\nDetected LIDAR Objects: " << (int)vec_of_objects.Objects().size() << "\n**********\n" << endl;

        		for (int k = 0; k < (int) vec_of_objects.Objects().size(); k++ )
        		{
        			cout  << "Object Location (X, Y, Z): " << k << ": " << (float)vec_of_objects.Objects()[k].location_x() << ", " << (float)vec_of_objects.Objects()[k].location_y() << ", " << (float)vec_of_objects.Objects()[k].location_z() << "[Height: " << (float)vec_of_objects.Objects()[k].boundary().height() << "]" << endl;

        			for (int l = 0; l < (int) vec_of_objects.Objects()[k].boundary().points().size(); l++)
        			{
        				cout <<  vec_of_objects.Objects()[k].boundary().points()[l].x() << " " << vec_of_objects.Objects()[k].boundary().points()[l].y() << " " << vec_of_objects.Objects()[k].boundary().points()[l].z() << endl;

        			}

        			cout << "Object Size (X, Y, Z):    "<< vec_of_objects.Objects()[k].size_x_length() << " " << vec_of_objects.Objects()[k].size_y_width() << " " << vec_of_objects.Objects()[k].size_z_height() << endl;
        		}
        		//cout << "vec_of_objects.number_of_objects(): " << (int)vec_of_objects.number_of_objects() << endl;
        		//cout << " (int)vec_of_objects.Objects().size(): " << (int)vec_of_objects.Objects().size() << endl;
        		cout <<  vec_of_objects.Objects().size() << " objects detected in ";
            }

    		vec_of_objects_to_publish = vec_of_objects;
    	}
    	//if number of detected objects is zero, then clear the datastructure
    	else if ( (int)vec_of_objects.Objects().size() == 0)
    	{
    		//msgFusedObjects temp;
    		//vec_of_objects_to_publish = temp;

    		vec_of_objects_to_publish.Objects().clear();
    		vec_of_objects_to_publish.number_of_objects() = (int)0.0;

    		cout << "*** --- NO OBJECT --- ***" << endl;
    	}
    */
    std::cout<< "box no: "<< box_no<<std::endl;
/*******************tracking start********************************/
    assert(vector_of_boxcentre.size() == vector_of_boxsize.size());
    currObjects.resize(vector_of_boxcentre.size());
    //cout<<"Groups[i].size():"<<Groups.size()<<endl;
    for (int i= 0; i < vector_of_boxcentre.size(); i++)
    {
        currObjects[i].id = i;
        currObjects[i].centrePoint.x = vector_of_boxcentre[i].x;
        currObjects[i].centrePoint.y = vector_of_boxcentre[i].y;
        currObjects[i].centrePoint.z = vector_of_boxcentre[i].z; 
        currObjects[i].objectSize.x = vector_of_boxsize[i].x;
        currObjects[i].objectSize.y = vector_of_boxsize[i].y;
        currObjects[i].objectSize.z = vector_of_boxsize[i].z;
        currObjects[i].boundary= Groups[i];
        //std::cout <<"Groups[i].size()"<<Groups[i].size()<<std::endl;
        //for (int j=0; j < Groups[i].size(); j++){
        //    std::cout <<"x:"<<Groups[i][j].x<<"y:"<<Groups[i][j].y<<"z:"<<Groups[i][j].z<<std::endl;
        //}
    }
    
    std::cout << "frame_id: " << frame_id << std::endl;
    frame_id++;
    auto updatedTrackingObjects = tracking_ID_assignment(currObjects, prevTrackingObjects);
    prevTrackingObjects = updatedTrackingObjects;

    // ros visiualization

    for (unsigned int i = 0; i < updatedTrackingObjects.size(); i++)
    {
        marker_array_msg6.markers[i].header.frame_id = "velodyne"; //"centre_lidar";
        marker_array_msg6.markers[i].id = i;
        marker_array_msg6.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array_msg6.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        marker_array_msg6.markers[i].color.g = 0.0;
        marker_array_msg6.markers[i].color.r = 0.0;
        marker_array_msg6.markers[i].color.b = 1.0;
        marker_array_msg6.markers[i].color.a = 1.0;
        marker_array_msg6.markers[i].lifetime = ros::Duration(0.1);
        geometry_msgs::Point p2;
        pcl::PointXYZRGBNormal boxpoint1;
        for (int j=0; j< updatedTrackingObjects[i].boundary.size();j++){
            p2.x = updatedTrackingObjects[i].boundary[j].x;
            p2.y = updatedTrackingObjects[i].boundary[j].y;
            p2.z = updatedTrackingObjects[i].boundary[j].z;
            //cout<<"updatedTrackingObjects[i]:"<<updatedTrackingObjects[i].boundary[j].x<<","<<updatedTrackingObjects[i].boundary[j].y<<","<<updatedTrackingObjects[i].boundary[j].z<<endl;
            marker_array_msg6.markers[i].points.push_back(p2);
            boxpoint1.x = p2.x;
            boxpoint1.y = p2.y;
            boxpoint1.z = p2.z;
            polygonboxrgb->points.push_back(boxpoint1);
        }
        marker_array_msg6.markers[i].scale.x = 0.1;
        marker_array_msg6.markers[i].scale.y = 0.1;
        marker_array_msg6.markers[i].scale.z = 0.1;


    }



/*******************tracking end**********************************/
    sensor_msgs::PointCloud2 polygonbox_msg;
    pcl::toROSMsg(*polygonboxrgb, polygonbox_msg);
    //no timestamp as of now
    //polygonbox_msg.header.stamp = in_cloud_msg->header.stamp;
    polygonbox_msg.header.frame_id = "velodyne"; //"/centre_lidar";
    polygon_box_pub_.publish(polygonbox_msg);

    pub_marker1.publish(marker_array_msg1);
    pub_marker2.publish(marker_array_msg2);
    pub_marker3.publish(marker_array_msg3);
    pub_marker4.publish(marker_array_msg4);
    pub_marker5.publish(marker_array_msg5);
    pub_marker6.publish(marker_array_msg6);
    pub_marker7.publish(marker_array_msg7);
    return; 
}



//HERE we perform single ground plane estimation. If we are planning to use for long range sensing,
// it is better to divide the point cloud to three parts and then estimate ground plane in each part.
//important to vary the passthrough filter parameters if sensor_height changes
void GroundPlaneFit::velodyne_callback_(pcl::PointCloud<pcl::PointXYZI>& PCD)
{
    //important to vary the passthrough filter if sensor_height changes
/*
//passthrough filter in X direction applied on PCD
    pcl::PassThrough<VPoint> passx;
    passx.setInputCloud(PCD.makeShared());
    passx.setFilterFieldName ("x");
    //passx.setFilterLimits (0, 30);
    //passx.setFilterLimits (0, 55);
    //passx.setFilterLimits (-12, 40);
    //passx.setFilterLimits (-12, 80);
    passx.setFilterLimits (0, 40);

    passx.filter (PCD);

//passthrough filter in Y direction applied on PCD
    // filter in y direction i.e. perpendicular to direction of travel (left +ve)
    pcl::PassThrough<VPoint> passy;
    passy.setInputCloud(PCD.makeShared());
    passy.setFilterFieldName ("y");
    //passy.setFilterLimits (-10, 10);
    //passy.setFilterLimits (-10, 10);
    //passy.setFilterLimits (-40, 40);
    passy.setFilterLimits (-15, 15);

    passy.filter (PCD);

//passthrough filter in Z direction applied on PCD
    // filter in z direction (up +ve and down -ve)
    pcl::PassThrough<VPoint> passz;
    passz.setInputCloud(PCD.makeShared());
    passz.setFilterFieldName ("z");
    passz.setFilterLimits (-2.0, 2.2);//ROI in Z direction - for sensor_height_ 0.35
    passz.filter (PCD);

*/
    //more clean passthrough filtering
    pcl::CropBox<VPoint> boxFilter;
    pcl::CropBox<VPoint> crop;
    crop.setMin(Eigen::Vector4f(0.0,-0.8,0.0,1.0)); 
    crop.setMax(Eigen::Vector4f(1.4, 0.8, 4.0,1.0)); 
    crop.setInputCloud(PCD.makeShared());
    crop.setNegative(true);
    crop.filter(PCD);
    _pub.publish(PCD); 
    boxFilter.setMin(Eigen::Vector4f( -20.0,-30.0,-0.8, 1.0)); //-80.0,-50.0,-0.0, 1.0
    boxFilter.setMax(Eigen::Vector4f(70.0, 30.0, 3.5, 1.0)); //80.0, 50.0, 3.0, 1.0
    boxFilter.setInputCloud(PCD.makeShared());
    boxFilter.filter(PCD);
//downsampling
// Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (PCD.makeShared());
    //sor.setLeafSize (0.2f, 0.2f, 0.2f);
    //sor.setLeafSize (0.15f, 0.15f, 0.15f);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);    
    sor.filter(PCD);

    VPoint cloudMin, cloudMax;
    pcl::getMinMax3D(PCD, cloudMin, cloudMax);

    /*
    @brief point_dist_cmp -> distance function to remove points that are closer to autonomous vehicle.
    @return true if the point is near or in the car dimensions

    filter points using pointIndices and ExtractIndices method of PCL
    then remove the points in inliers from the laserCloudIn
    */
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    int cou = 0;
    for (int i = 0; i < PCD.points.size(); i++)
    {
        if (point_dist_cmp(PCD.points[i]))
        {
            inliers->indices.push_back(i);
            cou++;
        }
    }

    extract.setInputCloud(PCD.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);//gives out all other points other than the passed inliers
    extract.filter(PCD);

/*
    /// publish combined pointcloud for visualization
    PCD.header.frame_id = "vehicle";
//    PCD.header.stamp = centre_curr_timestamp;
    _pub.publish(PCD);       // direct pcl::Pointcloud<T> as it is compatible with sensor_msgs::PointCloud2
*/

//again filter based on the sensor_height to find the interest region for performing segmentation

//this small block to only send a part of pcd for ground plane extraction
    pcl::PointCloud<pcl::PointXYZI> pcd_for_ground_seeds;
    pcl::PassThrough<VPoint> pass;
    pass.setInputCloud(PCD.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.7, 0.5);//for sensor_height_ 0.35 - it tries to find ground seeds within these limits
    pass.filter (pcd_for_ground_seeds);

    // call function extract_initial_seeds_ to extract initial seeds to estimate initial plane
    extract_initial_seeds_(pcd_for_ground_seeds);//g_seeds_pc will be set in this function

    // assign seeds points as initial ground
    g_ground_pc_dup = g_seeds_pc;//g_seeds_pc contains all the points whose z values is less than lpr_height_ + th_seeds_

    //converting pointcloud to matrix
    MatrixXf points(PCD.points.size(), 3);
    int j = 0;
    for (auto p : PCD.points)
    {
        points.row(j++) << p.x, p.y, p.z;
    }

    TH_DIST_D_ = 0;
    // 5. Ground plane fitter mainloop
    //iteratively estimate plane and refine it so that the ground plane can be estimated accurately
    for (int i = 0; i < num_iter_; i++)
    {
        estimate_plane_();
        g_ground_pc_dup->clear();
        g_not_ground_pc_dup->clear();

        if (DEBUG)
            cout << "value of FINAL th_dist_d_: " << th_dist_d_ << endl;

        if (i == num_iter_ - 1)
            TH_DIST_D_ = th_dist_d_;

        // ground plane model
        VectorXf result = points * normal_;

        //if a point lies within a distance of th_dist_ from the ground plane we consider them as ground point
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (result[r] < th_dist_d_)
            {
                g_ground_pc_dup->points.push_back(PCD[r]);
            }
            else
            {
                g_not_ground_pc_dup->points.push_back(PCD[r]);
            }
        }
    }

    //pushing the non-ground points into this structure.
    for (int i = 0; i < g_not_ground_pc_dup->points.size(); i++)
    {
        g_not_ground_pc->points.push_back(g_not_ground_pc_dup->points[i]);
    }

/*
    //send the points that do not belong to the ground for segmentation
    //segmentation happends in this function
    ClusteringCallback(*g_not_ground_pc);

    g_ground_pc->clear();
    g_not_ground_pc->clear();
*/
    return;
}


bool do_exit = false;
void vi_sigint_handler(int sig)
{
    do_exit = true;
}

//
void call_voxelgrid(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_in_ptr_)
{
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (pcl_pointcloud_in_ptr_->makeShared());
    sor.setLeafSize (0.15f, 0.15f, 0.15f);
    sor.filter (*pcl_pointcloud_in_ptr_);

    return;
}

///TO-DO: use ros time synchronizer for 3 lidar subscription




void cloud_cb_c (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    if (DEBUG)
        std::cout<<"centre lidar callback..."<<std::endl;

    // Convert to PCL data type
    cloud_c.reset(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl_conversions::toPCL(*cloud_msg, cloud_c);
    pcl::fromROSMsg (*cloud_msg, *cloud_c);

//    call_voxelgrid(cloud_c);
    if (DEBUG)
        std::cout<<"cloud_c size: "<<cloud_c->points.size()<<std::endl;
}

void cloud_cb_l (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert to PCL data type
    //pcl_conversions::toPCL(*cloud_msg, cloud_l);
    cloud_l.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*cloud_msg, *cloud_l);

//    call_voxelgrid(cloud_l);
    if (DEBUG)
        std::cout<<"cloud_l size: "<<cloud_l->points.size()<<std::endl;
}

void cloud_cb_r (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert to PCL data type
    //pcl_conversions::toPCL(*cloud_msg, cloud_r);
    cloud_r.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*cloud_msg, *cloud_r);

//    call_voxelgrid(cloud_r);
    if (DEBUG)
        std::cout<<"cloud_r size: "<<cloud_r->points.size()<<std::endl;
}



int main(int argc, char** argv)
{

    //this function is handle Ctrl+c command and to stop the program
//	signal(SIGINT, vi_sigint_handler);


    /*
    	//////////////////////////////
    	//added based on Wei Liang's recommendation
    	// Get the default factory QoS
    	dds::domain::qos::DomainParticipantFactoryQos factory_qos;

    	// Apply the max_objects_per_thread setting to the SystemResourceLimits policy
    	//int32_t max_objects_per_thread = 2048;
    	rti::core::policy::SystemResourceLimits resource_limits(MAX_OBJECTS_PER_THREAD);
    	factory_qos << resource_limits;

    	// Apply the factory_qos so that DomainParticipants will take it as default
    	dds::domain::DomainParticipant::participant_factory_qos(factory_qos);
    	//////////////////////////////

    	//loading config files for lidars
    	decoder_r = populate_decoder("config/right_lidar.json");
    	decoder_c = populate_decoder("config/centre_lidar.json");
    	decoder_l = populate_decoder("config/left_lidar.json");

    	//qos for receiving lidar data packets
    	string topic_name = "rslidar_points";
    	string qos_path = "qos/pointcloud_pubsub.xml";
    	string profile_name = "RsPointCloud_Library::RsPointCloud_Profile";

    	int domain_id = 2;//for reading lidar packets
    	int domain_id_lidar = 0;// for publishing lidar objects

    	//I am using the same qos as vision module for publishing the lidar objects
    	//this is because if both vision and lidar objects have same QOS, then its easy for sensor fusion because can use same qos to subscribe.
    	dds::core::status::StatusMask mask = dds::core::status::StatusMask::all();
    	dds::core::QosProvider qos_provider_lidar("./qos/qos_lidar.xml", "Lidar_Library::Lidar_Profile");
    	dds::domain::qos::DomainParticipantQos dpqos_lidar = qos_provider_lidar.participant_qos();
    	dds::domain::DomainParticipant participant_lidar(domain_id_lidar, dpqos_lidar, NULL, mask);// 0 to publish Sensor Fusion Output
    	dds::topic::Topic<msgFusedObjects> topic (participant_lidar, "msgLidarObjects");
    	dds::pub::DataWriter<msgFusedObjects> writer_lidar(dds::pub::Publisher(participant_lidar), topic, qos_provider_lidar.datawriter_qos());

    	// Health Monitoring
    	dds::topic::Topic<msgSME> topic_sme(participant_lidar, "msgSME");
    	dds::core::QosProvider qos_provider2("./qos/qos_healthMonitoring_Lidar.xml", "HMSME_Library::HMSME_Profile");
    	dds::pub::qos::DataWriterQos qos2 = qos_provider2.datawriter_qos("::HMSME_Profile::HMSMEDataWriter");
    	dds::pub::DataWriter<msgSME> sme_writer(dds::pub::Publisher(participant_lidar), topic_sme, qos2);
    	msgSME mSME;
    */

    // Initialize ROS
    ros::init(argc,argv,"lidarsegmentation");
    ros::NodeHandle node_handle_;

    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber
    sub_c = node_handle_.subscribe <sensor_msgs::PointCloud2>("/centre_lidar/rslidar_points", 10, cloud_cb_c);
    sub_l = node_handle_.subscribe <sensor_msgs::PointCloud2>("/l_lidar/rslidar_points", 10, cloud_cb_l);
    sub_r = node_handle_.subscribe <sensor_msgs::PointCloud2>("/r_lidar/rslidar_points", 10, cloud_cb_r);

    cloud_c.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_l.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_r.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // Create ROS publisher for the output point cloud
    _pub = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI>> ("pcloud_comb", 1);

    node_handle_.param< std::string >("polygon_box_topic", polygon_box_topic, "polygon_box");
    ROS_INFO("Only polygon box topic Output Point Cloud: %s", polygon_box_topic.c_str());
    polygon_box_pub_ =  node_handle_.advertise<sensor_msgs::PointCloud2>(polygon_box_topic, 1);

    pub_marker1 = node_handle_.advertise<visualization_msgs::MarkerArray>("normals_marker_array1", 200);
    pub_marker2 = node_handle_.advertise<visualization_msgs::MarkerArray>("normals_marker_array2", 200);
    pub_marker3 = node_handle_.advertise<visualization_msgs::MarkerArray>("normals_marker_array3", 200);
    pub_marker4 = node_handle_.advertise<visualization_msgs::MarkerArray>("normals_marker_array4", 200);
    pub_marker5 = node_handle_.advertise<visualization_msgs::MarkerArray>("normals_marker_array5", 200);
    pub_marker6 = node_handle_.advertise<visualization_msgs::MarkerArray>("normals_marker_array6", 200);
    pub_marker7 = node_handle_.advertise<visualization_msgs::MarkerArray>("normals_marker_array7", 200);
    if (DEBUG)
        cout << "\n\n/////////// \n Lidar Segmentation \n/////////////////\n" << endl;

    if (DEBUG)
        cout << "\n\n/////////// \n Reading from THREE lidars \n//////////////\n" << endl;

    int count = 0;

    /*
        // An AsyncWaitSet (AWS) for smulti-threaded events .
    	// The AWS will provide the infrastructure to receive samples using
    	// multiple threads.
    	//Here - latest_sample function gives the last data even if there is no more data from sensor
    	rti::core::cond::AsyncWaitSet async_waitset(
    		rti::core::cond::AsyncWaitSetProperty()
    		.thread_pool_size(THREAD_POOL_SIZE));

    	async_waitset.start();

    	AwsSubscriber<rslidarScan> reader1(
    		domain_id,
    		"c_lidar/rslidar_packets",
    		async_waitset);

    	AwsSubscriber<rslidarScan> reader2(
    		domain_id,
    		"l_lidar/rslidar_packets",
    		async_waitset);

    	AwsSubscriber<rslidarScan> reader3(
    		domain_id,
    		"r_lidar/rslidar_packets",
    		async_waitset);

    	rslidarScan centre_lidar_data;
    	rslidarScan left_lidar_data;
    	rslidarScan right_lidar_data;
    */

    float start_s = clock(); //Take start timestamp
    float comp_time_prev = 0;

    //used to find if we received old data - happens when lidar sensors fails for some reason
    long long centre_last_timestamp, left_last_timestamp, right_last_timestamp;
    long long centre_curr_timestamp, left_curr_timestamp, right_curr_timestamp;

    bool first_iter = true;
    int curr_last_same = 0;

    ros::Rate loop_rate(10);

    START_AGAIN:

    //while (!do_exit)
    while (ros::ok())
    {
        //if its first frame - just wait for 0.5 sec
        //if (pcl_count == 0)
        //usleep(500000); //sleep

        //restart the counter
        if (pcl_count > 10000000)
            pcl_count = 1;


        //to ensure that this module runs only at 10 Hz, we calculate the time taken for current iteration and then sleep for some time to meet 10 Hz
/*
        if (pcl_count != 0)
        {
            float time_taken = ( (comp_time_prev - start_s) / double(CLOCKS_PER_SEC) * 1000);
            //cout << time_taken << endl;
            if ( time_taken > 0 and time_taken < 100 )   //0.1s
            {
                cout << "Sleeping for " << (100 - time_taken) << " ms." << endl;
                usleep((100 - time_taken) * 1000);
            }
        }
*/
        start_s = clock();
        pcl::PointCloud<pcl::PointXYZI> PCD,PCD3lidar;
        pcl_count++;

        if (argc == 1 or argc > 2)
        {
            cout << "\n ********** \n WRONG Command Line Argument \n **********" << endl;
            cout << "Restart the program and give the correct command line arguments " << endl;
            cout << "\n\n Options: " << endl;
            cout << "./lidarSegmentation_main 3   -> uses three lidars and front camera for sensor fusion\n\n " << endl;
            abort();
        }
        else if (argc == 2 and atoi(argv[1]) == 3 )
        {
            /*
            centre_lidar_data = reader1.latest_sample();
            left_lidar_data = reader2.latest_sample();
            right_lidar_data = reader3.latest_sample();

            decoder_c->processScanLocal(centre_lidar_data);
            PCD += *(decoder_c->outPointsLocal);
            */
            int size_c = cloud_c->points.size();
            if (size_c == 0)
            {
                cout << "\n**********\nNot getting Data from CENTRE Lidar\n Wait for 50ms if you have just started the program\n then CHECK if its ON\n**********\n" << endl;
                //mSME.EventType(4); // 4 Error
                //sme_writer.write(mSME);
                usleep(50000);

                ros::spinOnce ();

                //goto START_AGAIN;
                continue;
            }
            Eigen::Matrix4f mat_centre;

/*	    mat_centre <<	0.9995,  0.0,  0.0296,   1.15,
                        0.0,     1.0,  0.0000000, 0.0, 
                        -0.0297,  0.0000000,  0.9995, 2.1,
                          0,          0,          0,           1;
*/
	    mat_centre <<	0.9985486759,  0.0435975903,  0.0296,   1.15,
                            -0.0436194,     0.9990482,  0.0000000, 0.0, 
                            -0.02968190676, -0.00129549618,  0.9995, 2.1,
                          0,          0,          0,           1;

            pcl::PointCloud<pcl::PointXYZI> tx_centre_pcd;
            //pcl::transformPointCloud(*(decoder_l->outPointsLocal), tx_left_pcd, mat_left);
            pcl::transformPointCloud(*cloud_c, tx_centre_pcd, mat_centre);
            PCD = tx_centre_pcd;

            if (DEBUG)
                cout << "size_c: " << size_c << endl;
            //decoder_l->processScanLocal(left_lidar_data);
            int size_l = cloud_l->points.size();
/*
            if (size_l == 0)
            {
                cout << "\n*********\n Not getting Data from LEFT Lidar \n Wait for a minute if you have just started the program\n then CHECK if its ON\n********\n" << endl;
                //mSME.EventType(4); // 4 Error
                //sme_writer.write(mSME);

                usleep(50000);

                ros::spinOnce ();

                //goto START_AGAIN;
                continue;
            }
*/            
            //if (differencel < 30000){
            Eigen::Matrix4f mat_left;

//	    mat_left <<	0.9872099,   -0.1545410, -0.03685418,  1.15,
//                        0.1452098,    0.7849727,  0.6021523,   0.58,
//                        -0.06414841, -0.5998680,  0.79745,     1.895,
//                          0,          0,          0,           1;

	    mat_left <<	0.986416354041198,   -0.154541, -0.054077749167146,  1.15,
                        0.15569668734798,    0.7849727,  0.59952633269119,   0.58,
                        -0.050221223817157, -0.5998680,  0.798448092075684,     1.895,
                          0,          0,          0,           1;

            pcl::PointCloud<pcl::PointXYZI> tx_left_pcd;
            //pcl::transformPointCloud(*(decoder_l->outPointsLocal), tx_left_pcd, mat_left);
            //pcl::transformPointCloud(*cloud_l, tx_left_pcd, mat_left);

            //PCD3lidar = PCD + tx_left_pcd;
            //int size_l = PCD.points.size() - size_c;
            if (DEBUG)
                cout << "size_c+l: " << PCD.points.size() << endl; 
            //}  
          

            //decoder_r->processScanLocal(right_lidar_data);
            int size_r = cloud_r->points.size();
/*
            if (size_r == 0)
            {
                cout << "\n*******\nNot getting Data from RIGHT Lidar \nWait for a minute if you have just started the program\nthen CHECK if its ON\n*******\n" << endl;
                //mSME.EventType(4); // 4 Error
                //sme_writer.write(mSME);

                usleep(50000);

                ros::spinOnce ();

                //goto START_AGAIN;
                continue;
            }
*/            
            //if (differencer < 30000){
            Eigen::Matrix4f mat_right;
//            mat_right <<    0.9984747,  -0.01144485,  0.05109005,   1.15,
//                            0.03939581,  0.8064116,  -0.58899009,  -0.58,
//                            -0.03459052, 0.5910845,  0.8057811,     1.895,
//                            0,           0,          0,             1;


            mat_right <<    0.99922601602359,  -0.01144485,  0.0333077098115,   1.15,
                            0.02890472151557,  0.8064116,  -0.589598094541878,  -0.58,
                            -0.020241009188092, 0.5910845,  0.806269223956488,     1.89,
                            0,           0,          0,             1;

//            mat_right <<    0.99922601602359,  -0.01138670088679098905,  0.03332763454664028275,   1.15,
//                            0.02890472151557,  0.80538136482819606032,  -0.59100464031021618718,  -0.58,
//                            -0.020241009188092, 0.5924907950498212585,  0.80523639477480206526,     1.89, //1.895
//                            0,           0,          0,             1;

            pcl::PointCloud<pcl::PointXYZI> tx_right_pcd;
            //pcl::transformPointCloud(*(decoder_r->outPointsLocal), tx_right_pcd, mat_right);
            //pcl::transformPointCloud(*cloud_r, tx_right_pcd, mat_right);
            //PCD3lidar += tx_right_pcd;
            if (DEBUG)
                cout << "size_c+l+r: " << PCD.points.size() << endl; 
            //}  


        }
        else
        {
            cout << "\n *********************** \n WRONG Command Line Argument \n ***********************" << endl;
            cout << "Restart the program and give the correct command line arguments " << endl;
            cout << "\n\n Options: " << endl;
            cout << "./lidarSegmentation 3   -> uses three lidars and front camera for sensor fusion\n\n " << endl;
            abort();
        }


        if (PCD.points.size() == 0)
        {
            cout << " \n **************** \n\n DID NOT GET DATA - WAITING  \n\n **************** " << endl;

            ros::spinOnce ();

            //goto START_AGAIN;
            continue;
        }

        if (first_iter)
        {
            //centre_last_timestamp = centre_lidar_data.packets()[0].stamp();
            //left_last_timestamp = left_lidar_data.packets()[0].stamp();
            //right_last_timestamp = right_lidar_data.packets()[0].stamp();
            centre_last_timestamp = cloud_c->header.stamp;

            first_iter = false;
        }

        //centre_curr_timestamp = centre_lidar_data.packets()[0].stamp();
        //left_curr_timestamp = left_lidar_data.packets()[0].stamp();
        //right_curr_timestamp = right_lidar_data.packets()[0].stamp();
        centre_curr_timestamp = cloud_c->header.stamp;


        //centre lidar to centre of rear axis

        //Checking if we receive an old data - can happen due to latest_sample function of AwsSubscriber
        if (centre_last_timestamp == centre_curr_timestamp)
        {
            //if old data, then increment
            curr_last_same++;
        }
        else
        {
            curr_last_same = 0;
        }

        //if we get more than 5 samples of old data then we report to health monitoring
        //this can happen when sensor dies
        if (curr_last_same > 5)
        {
            //mSME.EventType(4); // 4 Error
            //sme_writer.write(mSME);
            cout << "\nGOT OLD LIDAR data\nCHECK IF ANY LIDAR IS POWERED OFF\n--- WAITING FOR LATEST DATA ---\n" << endl;

            ros::spinOnce ();

            //goto START_AGAIN;
            //do_exit = true;
            continue;
        }
        else
        {
            if (pcl_count % 10 == 0)
            {
                //report that I am working well
                //mSME.EventType(2); // 2 Working
                //sme_writer.write(mSME);
            }
        }

        /// publish combined pointcloud for visualization
        PCD.header.frame_id = "vehicle";//vehicle
        //PCD.height = PCD.width = 1;
        PCD.header.stamp = centre_curr_timestamp;
        //_pub.publish(PCD);       // direct pcl::Pointcloud<T> as it is compatible with sensor_msgs::PointCloud2


        GroundPlaneFit node;
        //ACTUAL FUNCTION THAT DOES LIDAR OBJECT SEGMENTATION AND WRITES THE OBJECTS TO A GLOBAL VARIABLE
        node.velodyne_callback_(PCD);
       
        //send the points that do not belong to the ground for segmentation
        //segmentation happends in this function
        node.ClusteringCallback(*g_not_ground_pc);

        g_ground_pc->clear();
        g_not_ground_pc->clear();


        //get the time and write to dds msg header
        struct timeval inferStart, inferEnd;
        gettimeofday(&inferStart, NULL);
//		vec_of_objects_to_publish.header().time_stamp().sec_()  = inferStart.tv_sec;
//		vec_of_objects_to_publish.header().time_stamp().nsec_() = inferStart.tv_usec / 1000;
//		writer_lidar.write(vec_of_objects_to_publish);

        //centre_last_timestamp = centre_lidar_data.packets()[0].stamp();
        //left_last_timestamp = left_lidar_data.packets()[0].stamp();
        //right_last_timestamp = right_lidar_data.packets()[0].stamp();
        centre_last_timestamp = cloud_c->header.stamp;

        //THIS IS TO HANDLE CASE WHEN NO OBJECTS ARE DETECTED BY LIDAR - WHEN PLANAR GROUND ALL AROUND
        NO_OBJECT = 0;
        float stop_s = clock();

        std::cout << "LidarSeg Time:" <<(stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 << " ms." << endl;

        if (do_exit == true)
        {
            //mSME.EventType(4); // 4 Error
            //sme_writer.write(mSME);
            cout <<  "\n\nStopping the Program\n" << endl;

            exit(1);
        }

        comp_time_prev = clock();

        //ROS
        ros::spinOnce ();
        loop_rate.sleep ();
    }

//    if (do_exit == true)
//    {
        //mSME.EventType(4); // 4 Error
        //sme_writer.write(mSME);
//        cout <<  "\n\nStopping the Program ...\n" << endl;

        //exit(1);
//    }

    return 0;
}

