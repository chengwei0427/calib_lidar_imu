
#ifndef _CALIB_LIDAR_IMU_H_
#define _CALIB_LIDAR_IMU_H_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
//  ndt
#include <pcl/registration/registration.h>
// #include <pclomp/ndt_omp.h>
// #include <pclomp/gicp_omp.h>

#ifndef USE_GTSAM_OPT
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/navigation/ImuFactor.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#endif

using namespace std;
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct LidarData
{
    double stamp;
    CloudT::Ptr cloud;
    LidarData() : stamp(0), cloud(new CloudT()) {}
};

struct LidarFrame
{
    double stamp;
    Eigen::Matrix4d T;
    Eigen::Matrix4d gT;
    CloudT::Ptr cloud{nullptr};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImuData
{
    double stamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
    Eigen::Quaterniond rot;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#ifndef USE_GTSAM_OPT
/// GTSAM Factor
using gtsam::symbol_shorthand::R; // Rotation

class HECFactor : public gtsam::NoiseModelFactor1<gtsam::Rot3>
{
private:
    gtsam::Point3 m_axis_I_;
    gtsam::Point3 m_axis_L_;

public:
    HECFactor(gtsam::Key i, gtsam::Point3 axis_I, gtsam::Point3 axis_L, const gtsam::SharedNoiseModel &model) : gtsam::NoiseModelFactor1<gtsam::Rot3>(model, i), m_axis_I_(axis_I), m_axis_L_(axis_L) {}

    gtsam::Vector evaluateError(const gtsam::Rot3 &I_R_L, boost::optional<gtsam::Matrix &> H = boost::none) const
    {
        gtsam::Matrix H_Rp_R, H_Rp_p;
        gtsam::Point3 error = m_axis_I_ - I_R_L.rotate(m_axis_L_, H_Rp_R, H_Rp_p);
        if (H)
            (*H) = (gtsam::Matrix(3, 3) << -H_Rp_R).finished();
        return (gtsam::Vector(3) << error.x(), error.y(), error.z()).finished();
    }
};
#endif

class CalibLidarIMU
{
public:
    CalibLidarIMU();
    ~CalibLidarIMU();

    //@brief: add lidar data and calculate lidar odometry
    void addLidarData(const LidarData &data);

    //@brief: add imu data and cache
    void addImuData(const ImuData &data);

    //@brief: integration imu data, align lidar odom and imu
    Eigen::Vector3d calib(bool integration = false);

    //@brief: save temp local_map_
    void saveNDTmap(std::string &mapfile);

    Eigen::Vector3d solve(int count = 0, const Eigen::Matrix3d &ric = Eigen::Matrix3d::Identity());
    Eigen::Vector3d soveIter();

    Eigen::Vector3d solve_gtsam();

    //@brief: optimize again
    Eigen::Vector3d optimizeTwice();

    void setRegistration(pcl::Registration<PointT, PointT>::Ptr reg)
    {
        registration_ = reg;
    }

    void saveQuParam(std::string &file);
    void saveEulerParam(std::string &file);
    void loadQuParam(std::string &file);

private:
    // pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndtInit(double ndt_resolution);

    void downsampleCloud(CloudT::Ptr in_cloud, CloudT::Ptr out_cloud, float leaf_size);

    bool checkKeyScan(const Eigen::Matrix4d &pose);
    void updateKeyScan(const LidarFrame &frame);

    static inline double normalize_angle(double ang_degree)
    {
        if (ang_degree > 180.0)
            ang_degree -= 180.0;
        if (ang_degree < -180.0)
            ang_degree += 360.0;
        return ang_degree;
    }

    //void display_pc(const CloudT::Ptr &cloud, std::string name = std::string("viewr"));
    //void display_pc2(const CloudT::Ptr &cloud1, const CloudT::Ptr &cloud2, std::string name = std::string("viewr"));

    CloudT::Ptr last_cloud;
    vector<LidarFrame> lidar_buffer_;                                       // record relative transform between neighbor lidar frame
    vector<ImuData> imu_buffer_;                                            // record raw imu datas
    vector<pair<LidarFrame, Eigen::Quaterniond>> aligned_lidar_imu_buffer_; // aligned lidar frame and interpolated imu attitude at lidar stamp
    Eigen::Quaterniond q_ItoSensor;

    CloudT::Ptr map_cloud_; // local map
    //pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_omp_{nullptr}; // register object
    pcl::Registration<PointT, PointT>::Ptr registration_;

    std::vector<std::pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#ifndef USE_GTSAM_OPT
    /// GTSAM stuff
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_values;
    gtsam::noiseModel::Diagonal::shared_ptr rotationNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector3(1, 1, 1)));
#endif
};

#endif //_CALIB_LIDAR_IMU_H_