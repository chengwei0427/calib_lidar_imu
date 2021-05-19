#include "calib_lidar_imu.h"
#include "math_utils.h"

#include <omp.h>
#include <utility>
// #include <tf/tf.h>
//  pcl
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#define USE_SCAN_2_MAP true

CalibLidarIMU::CalibLidarIMU()
    : map_cloud_(new CloudT), last_cloud(new CloudT)
{
    imu_buffer_.clear();
    ndt_omp_ = ndtInit(0.5);
}

pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr CalibLidarIMU::ndtInit(double ndt_resolution)
{
    auto ndt = pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr(
        new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt->setResolution(ndt_resolution);
    int avalib_cpus = omp_get_max_threads();
    ndt->setNumThreads(avalib_cpus);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setTransformationEpsilon(1e-3);
    ndt->setStepSize(0.01);
    ndt->setMaximumIterations(60);
    return ndt;
}

CalibLidarIMU::~CalibLidarIMU()
{
}

void CalibLidarIMU::addImuData(const ImuData &data)
{
    imu_buffer_.push_back(data);
}

void CalibLidarIMU::addLidarData(const LidarData &data)
{
    if (!data.cloud || data.cloud->size() == 0)
    {
        cout << "no cloud in lidar data !!!" << endl;
        return;
    }

    if (!ndt_omp_)
    {
        cout << "register no initialize !!!" << endl;
        return;
    }
#if USE_SCAN_2_MAP //  scan-to-submap

    CloudT::Ptr downed_cloud(new CloudT);
    downsampleCloud(data.cloud, downed_cloud, 0.1);

    LidarFrame frame;
    frame.stamp = data.stamp;
    frame.T = Eigen::Matrix4d::Identity();
    frame.gT = Eigen::Matrix4d::Identity();
    frame.cloud = downed_cloud;

    CloudT::Ptr scan_in_target(new CloudT);
    if (map_cloud_->empty())
    {
        *map_cloud_ += *data.cloud;
        ndt_omp_->setInputTarget(map_cloud_);
    }
    else
    {
        ndt_omp_->setInputSource(downed_cloud);
        CloudT::Ptr aligned(new CloudT);
        ndt_omp_->align(*aligned, lidar_buffer_.back().gT.cast<float>());
        if (!ndt_omp_->hasConverged())
        {
            cout << "register cant converge, please check initial value !!!" << endl;
            return;
        }
        Eigen::Matrix4d pose_out = (ndt_omp_->getFinalTransformation()).cast<double>();

        frame.gT = pose_out;
        Eigen::Matrix4d last_T_l_m = lidar_buffer_.back().gT;
        frame.T = last_T_l_m.inverse() * pose_out;
        // double dx = frame.T.block<3, 1>(0, 3).norm();
        // double da = std::acos(Eigen::Quaterniond(frame.T.block<3, 3>(0, 0)).w());
        // std::cout << "dx: " << dx << "  da: " << da * 180.0 / M_PI << std::endl;
        //Eigen::Matrix3d RL_delta = frame.T.topLeftCorner<3, 3>();
        // Eigen::Matrix3d RL_delta = frame.T.block<3, 3>(0, 0);
        // std::cout << RL_delta << std::endl;
        // Eigen::Vector3d euler_delta = RL_delta.eulerAngles(2, 1, 0);
        // std::cout << "adj euler degree: " << (euler_delta * 180.0 / M_PI).transpose() << std::endl;

        // Eigen::Vector3d euler_d = mathutils::GetRPYFromRotationMatrix(RL_delta);
        // std::cout << "adj2 euler degree: " << (euler_d * 180.0 / M_PI).transpose() << std::endl;
    }
    lidar_buffer_.push_back(move(frame));
    // static int i = 0;
    // std::cout << i++ << " ------------------------- " << std::endl;
    //std::cout << lidar_buffer_.back().T << std::endl;
    // getchar();

    if (true)
    {
        updateKeyScan(lidar_buffer_.back());
    }
#else //  scan-to-scan
    CloudT::Ptr downed_cloud(new CloudT);
    downsampleCloud(data.cloud, downed_cloud, 0.1);

    LidarFrame frame;
    frame.stamp = data.stamp;
    // init first lidar frame and set it as zero pose
    if (last_cloud->empty())
    {
        last_cloud.reset(new CloudT);
        *last_cloud += *data.cloud;

        frame.T = Eigen::Matrix4d::Identity();
        frame.gT = Eigen::Matrix4d::Identity();
        frame.cloud = downed_cloud;
    }
    else
    {
        // get transform between neighbor frames
        ndt_omp_->setInputSource(downed_cloud);
        ndt_omp_->setInputTarget(last_cloud);
        CloudT::Ptr aligned(new CloudT);
        ndt_omp_->align(*aligned, Eigen::Matrix4f::Identity());

        if (!ndt_omp_->hasConverged())
        {
            cout << "register cant converge, please check initial value !!!" << endl;
            return;
        }
        Eigen::Matrix4d result_T = (ndt_omp_->getFinalTransformation()).cast<double>();

        last_cloud = downed_cloud;

        frame.T = result_T;
        frame.cloud = downed_cloud;
        Eigen::Matrix4d temp1 = lidar_buffer_.back().gT;
        Eigen::Matrix4d temp2 = result_T;
        frame.gT = temp1 * temp2;
    }

    lidar_buffer_.push_back(move(frame));
#endif
}

bool CalibLidarIMU::checkKeyScan(const Eigen::Matrix4d &pose)
{
    static Eigen::Vector3d pos_last(0, 0, 0);
    static Eigen::Vector3d ypr_last(0, 0, 0);

    Eigen::Vector3d pos_now = pose.block<3, 1>(0, 3);
    double dist = (pos_now - pos_last).norm();

    const Eigen::Matrix3d rotation(pose.block<3, 3>(0, 0));
    Eigen::Vector3d ypr = mathutils::R2ypr(rotation);
    Eigen::Vector3d delta_angle = ypr - ypr_last;
    for (size_t i = 0; i < 3; i++)
        delta_angle(i) = normalize_angle(delta_angle(i));
    delta_angle = delta_angle.cwiseAbs();

    if (lidar_buffer_.size() == 0 || dist > 0.2 || delta_angle(0) > 5.0 || delta_angle(1) > 5.0 || delta_angle(2) > 5.0)
    {
        pos_last = pos_now;
        ypr_last = ypr;
        return true;
    }
    return false;
}

void CalibLidarIMU::updateKeyScan(const LidarFrame &frame)
{
    // if (checkKeyScan(frame.gT))
    {
        CloudT::Ptr scan_target(new CloudT);
        pcl::transformPointCloud(*frame.cloud, *scan_target, frame.gT);
        *map_cloud_ += *scan_target;

        downsampleCloud(map_cloud_, scan_target, 0.2);
        map_cloud_ = scan_target;
        ndt_omp_->setInputTarget(map_cloud_);
    }
}

Eigen::Vector3d CalibLidarIMU::optimizeTwice()
{
    if (!ndt_omp_ || aligned_lidar_imu_buffer_.size() < 10)
    {
        cout << "register no initialize OR no align data buffer !" << endl;
        return Eigen::Vector3d{0.0, 0.0, 0.0};
    }
    if (map_cloud_)
    {
        map_cloud_.reset(new CloudT);
        *map_cloud_ += *aligned_lidar_imu_buffer_[0].first.cloud;
        ndt_omp_->setInputTarget(map_cloud_);
    }

    Eigen::Matrix3d R_i_sensor = aligned_lidar_imu_buffer_[0].first.gT.block<3, 3>(0, 0);
    Eigen::Matrix3d ric = q_ItoSensor.toRotationMatrix();
    for (int j = 1; j < aligned_lidar_imu_buffer_.size(); j++)
    {
        Eigen::Quaterniond q_ij_imu = aligned_lidar_imu_buffer_[j - 1].second.conjugate() * aligned_lidar_imu_buffer_[j].second;
        Eigen::Matrix3d R_ij_sensor_est = ric.inverse() * q_ij_imu * ric;
        Eigen::Matrix3d R_j_sensor_est = R_i_sensor * R_ij_sensor_est;
        // Eigen::Matrix4d R_j_sensor_predict = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d R_j_sensor_predict = aligned_lidar_imu_buffer_[j].first.gT;
        R_j_sensor_predict.block<3, 3>(0, 0) = R_j_sensor_est;

        ndt_omp_->setInputSource(aligned_lidar_imu_buffer_[j].first.cloud);
        CloudT::Ptr aligned(new CloudT);
        ndt_omp_->align(*aligned, R_j_sensor_predict.cast<float>());
        if (!ndt_omp_->hasConverged())
        {
            cout << "register cant converge, please check initial value !!!" << endl;
            return Eigen::Vector3d{0.0, 0.0, 0.0};
        }

        Eigen::Matrix4d pose_out = (ndt_omp_->getFinalTransformation()).cast<double>();
        //  update aligned_lidar_imu_buffer_
        aligned_lidar_imu_buffer_[j].first.gT = pose_out;
        aligned_lidar_imu_buffer_[j].first.T = aligned_lidar_imu_buffer_[j - 1].first.gT.inverse() * pose_out;

        R_i_sensor = pose_out.block<3, 3>(0, 0);
        //  update map
        CloudT::Ptr scan_target(new CloudT);
        pcl::transformPointCloud(*aligned_lidar_imu_buffer_[j].first.cloud, *scan_target, pose_out);
        *map_cloud_ += *scan_target;

        downsampleCloud(map_cloud_, scan_target, 0.2);
        map_cloud_ = scan_target;
        ndt_omp_->setInputTarget(map_cloud_);
    }
    //  update corres
    if (corres.size() > 0)
        corres.clear();
    for (size_t j = 1; j < aligned_lidar_imu_buffer_.size(); ++j)
    {
        Eigen::Quaterniond delta_qij_imu = aligned_lidar_imu_buffer_[j - 1].second.conjugate() * aligned_lidar_imu_buffer_[j].second;
        delta_qij_imu.normalize();

        Eigen::Matrix3d R_si_toS0 = aligned_lidar_imu_buffer_[j - 1].first.gT.topLeftCorner<3, 3>();
        Eigen::Matrix3d R_sj_toS0 = aligned_lidar_imu_buffer_[j].first.gT.topLeftCorner<3, 3>();
        Eigen::Matrix3d delta_ij_sensor = R_si_toS0.transpose() * R_sj_toS0;
        Eigen::Quaterniond delta_qij_sensor(delta_ij_sensor);
        delta_qij_sensor.normalize();

        corres.push_back(std::move(std::pair<Eigen::Quaterniond, Eigen::Quaterniond>(delta_qij_sensor, delta_qij_imu)));
    }
    char *home_env_var = std::getenv("HOME");
    std::string file = std::string(home_env_var) + "/temp2.txt";
    saveQuParam(file);

    Eigen::Vector3d eular = solve();
    return eular;
}

Eigen::Vector3d CalibLidarIMU::calib(bool integration)
{
    if (lidar_buffer_.size() == 0 || imu_buffer_.size() == 0)
    {
        cout << "no lidar data or imu data !!!" << endl;
        return Eigen::Vector3d{0.0, 0.0, 0.0};
    }
    std::cout << "total lidar buffer size " << lidar_buffer_.size() << ", imu buffer size " << imu_buffer_.size() << std::endl;

    //  remove invalid lidarframe which get before first imu
    auto invalid_lidar_iter = lidar_buffer_.begin();
    for (; invalid_lidar_iter != lidar_buffer_.end(); invalid_lidar_iter++)
    {
        if (invalid_lidar_iter->stamp >= imu_buffer_[0].stamp)
            break;
    }
    if (invalid_lidar_iter != lidar_buffer_.begin())
        lidar_buffer_.erase(lidar_buffer_.begin(), invalid_lidar_iter);
    std::cout << "after time sync, lidar frame size: " << lidar_buffer_.size() << std::endl;
    if (lidar_buffer_.size() < 5)
    {
        cout << "valid lidar frame is " << lidar_buffer_.size() << " may not right" << endl;
        return move(Eigen::Vector3d(0.0, 0.0, 0.0));
    }

    //  sync lidarFrame and Imu
    auto last_imu_iter = imu_buffer_.begin();
    for (int i = 0; i < lidar_buffer_.size(); i++)
    {
        // get lidar frame
        const auto &lidar_frame = lidar_buffer_[i];

        // get last imu frame which before current lidar frame
        for (; last_imu_iter != imu_buffer_.end(); last_imu_iter++)
        {
            if (last_imu_iter->stamp >= lidar_frame.stamp)
                break;
        }
        if (last_imu_iter != imu_buffer_.begin())
            last_imu_iter--;

        // get interpolated imu attitude at lidar stamp
        auto imu_it1 = last_imu_iter;
        auto imu_it2 = last_imu_iter + 1;
        if (imu_buffer_.end() == imu_it2)
        {
            std::cout << "lidarframe last not have imu" << std::endl;
            break;
        }
        assert(imu_it2->stamp >= lidar_frame.stamp || imu_it1->stamp < imu_it2->stamp); // this shouldnt happen
        Eigen::Quaterniond q_b1_start = imu_it1->rot;
        Eigen::Quaterniond q_b2_end = imu_it2->rot;
        double scale = (lidar_frame.stamp - imu_it1->stamp) / (imu_it2->stamp - imu_it1->stamp);
        Eigen::Quaterniond q_inter = mathutils::getInterpolatedAttitude(q_b1_start, q_b2_end, scale);

        // buffer aligned information
        aligned_lidar_imu_buffer_.push_back(move(pair<LidarFrame, Eigen::Quaterniond>(lidar_frame, q_inter)));
    }

    size_t valid_size = aligned_lidar_imu_buffer_.size();
    if (valid_size < 15)
    {
        std::cout << "valid rotation pairs is " << valid_size << ", too less." << std::endl;
        return Eigen::Vector3d{0.0, 0.0, 0.0};
    }

    if (corres.size() > 0)
        corres.clear();
    for (size_t j = 1; j < aligned_lidar_imu_buffer_.size(); ++j)
    {
        Eigen::Quaterniond delta_qij_imu = aligned_lidar_imu_buffer_[j - 1].second.conjugate() * aligned_lidar_imu_buffer_[j].second;
        delta_qij_imu.normalize();

        Eigen::Matrix3d R_si_toS0 = aligned_lidar_imu_buffer_[j - 1].first.gT.topLeftCorner<3, 3>();
        Eigen::Matrix3d R_sj_toS0 = aligned_lidar_imu_buffer_[j].first.gT.topLeftCorner<3, 3>();
        Eigen::Matrix3d delta_ij_sensor = R_si_toS0.transpose() * R_sj_toS0;
        Eigen::Quaterniond delta_qij_sensor(delta_ij_sensor);
        delta_qij_sensor.normalize();

        corres.push_back(std::move(std::pair<Eigen::Quaterniond, Eigen::Quaterniond>(delta_qij_sensor, delta_qij_imu)));
    }

    std::cout << "corres size: " << corres.size() << std::endl;

    char *home_env_var = std::getenv("HOME");
    std::string file = std::string(home_env_var) + "/temp.txt";
    saveQuParam(file);
    std::string euler_file = std::string(home_env_var) + "/euler.txt";
    // saveEulerParam(euler_file);
    Eigen::Vector3d eular = solve();
    return eular;
}

Eigen::Vector3d CalibLidarIMU::soveIter()
{
    Eigen::Matrix3d ric = Eigen::Matrix3d::Identity();
    int frame_count_half = corres.size() / 2;
    Eigen::Vector3d euler = solve(frame_count_half);
    ric = Eigen::Matrix3d(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
    euler = solve(0, ric);
}

Eigen::Vector3d CalibLidarIMU::solve(int count, const Eigen::Matrix3d &ric)
{
    if (count == 0)
        count = corres.size();
    //std::cout << "ric: " << ric << std::endl;
    Eigen::MatrixXd A(count * 4, 4);
    A.setZero();
#ifdef CALIB_ZJU
    for (size_t j = 0; j < count; ++j)
    {
        Eigen::Quaterniond delta_qij_imu = corres[j].second;
        Eigen::Quaterniond delta_qij_sensor = corres[j].first;

        Eigen::AngleAxisd R_vector1(delta_qij_sensor.toRotationMatrix());
        Eigen::AngleAxisd R_vector2(delta_qij_imu.toRotationMatrix());
        double delta_angle = 180.0 / M_PI * std::fabs(R_vector1.angle() - R_vector2.angle());
        double huber = delta_angle > 2.0 ? 2.0 / delta_angle : 1.0;
        Eigen::Matrix4d lq_mat = mathutils::LeftQuatMatrix(delta_qij_sensor);
        Eigen::Matrix4d rq_mat = mathutils::RightQuatMatrix(delta_qij_imu);

        A.block<4, 4>(j * 4, 0) = huber * (lq_mat - rq_mat);
    }
#else
    for (size_t j = 0; j < count; ++j)
    {
        Eigen::Quaterniond delta_qij_imu = corres[j].second;
        Eigen::Quaterniond delta_qij_sensor = corres[j].first;

        Eigen::Matrix3d r = ric.inverse() * delta_qij_imu * ric;
        Eigen::Quaterniond r2(r);

        double delta_angle = 180.0 / M_PI * delta_qij_sensor.angularDistance(r2);
        //std::cout << delta_angle << std::endl;
        double huber = delta_angle > 1.0 ? 1.0 / delta_angle : 1.0;

        Eigen::Matrix4d L, R;
        double w = delta_qij_sensor.w();
        Eigen::Vector3d q = delta_qij_sensor.vec();
        L.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        w = delta_qij_imu.w();
        q = delta_qij_imu.vec();
        R.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        A.block<4, 4>(j * 4, 0) = huber * (L - R);
    }
#endif
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
    Eigen::Quaterniond q_ItoS(x);
    std::cout << "quaternion: " << q_ItoS.w() << " " << q_ItoS.x() << " " << q_ItoS.y() << " " << q_ItoS.z() << std::endl;
    q_ItoSensor = q_ItoS;
    Eigen::Vector4d cov = svd.singularValues();

    std::cout << "cov(2): " << cov(2) << std::endl;
    if (cov(2) > 0.25)
        std::cout << "get a good calib result" << std::endl;

    std::cout << "quaternion: " << q_ItoSensor.w() << " " << q_ItoSensor.x() << " " << q_ItoSensor.y() << " " << q_ItoSensor.z() << std::endl;
    Eigen::Matrix3d rx = q_ItoSensor.toRotationMatrix();

    Eigen::Vector3d euler_d = mathutils::GetRPYFromRotationMatrix(rx);
    std::cout << "euler from matrix: " << euler_d[0] << " " << euler_d[1] << " " << euler_d[2] << std::endl;

    Eigen::Vector3d eular = rx.eulerAngles(2, 1, 0);
    // std::cout << "euler from eulerAngles(2,1,0): " << eular[2] << "  " << eular[1] << " " << eular[0] << std::endl;

    return euler_d;
}

void CalibLidarIMU::saveQuParam(std::string &file)
{
    std::ofstream fout(file, std::ios::out);
    Eigen::Quaterniond q_sensor, q_imu;
    for (int i = 0; i < corres.size(); i++)
    {
        q_sensor = corres[i].first;
        q_imu = corres[i].second;
        fout << std::fixed << std::setprecision(9)
             << q_sensor.w() << " " << q_sensor.x() << " " << q_sensor.y() << " " << q_sensor.z()
             << " " << q_imu.w() << " " << q_imu.x() << " " << q_imu.y() << " " << q_imu.z() << std::endl;
    }
    fout.close();
    std::cout << "save " << file << std::endl;
}
void CalibLidarIMU::saveEulerParam(std::string &file)
{
    std::ofstream fout(file, std::ios::out);
    Eigen::Vector3d e_sensor, e_imu;
    for (int i = 0; i < corres.size(); i++)
    {
        e_sensor = corres[i].first.toRotationMatrix().eulerAngles(2, 1, 0);
        e_imu = corres[i].second.toRotationMatrix().eulerAngles(2, 1, 0);
        fout << std::fixed << std::setprecision(9)
             << e_sensor[2] * 180.0 / M_PI << " " << e_sensor[1] * 180.0 / M_PI << " " << e_sensor[0] * 180.0 / M_PI
             << " " << e_imu[2] * 180.0 / M_PI << " " << e_imu[1] * 180.0 / M_PI << " " << e_imu[0] * 180.0 / M_PI << std::endl;
    }
    fout.close();
    std::cout << "save " << file << std::endl;
}

inline void strim(std::string &str)
{
    if (str.empty())
        return;
    str.erase(0, str.find_first_not_of(' '));
    str.erase(str.find_last_not_of("\r") + 1);
}
inline std::vector<double> str2vec(const std::string &s)
{
    std::string a = s;
    strim(a);
    std::vector<double> data;
    data.reserve(100);
    std::stringstream ss(a.c_str());
    std::string t;
    while (getline(ss, t, ' '))
    {
        if (t.length() == 0)
            continue;
        data.push_back(std::atof(t.c_str()));
    }
    return data;
}

void CalibLidarIMU::loadQuParam(std::string &file)
{
    if (corres.size() > 0)
        corres.clear();
    std::ifstream fin(file, std::ios::in);
    if (!fin)
    {
        std::cout << "error opening file: " << file << std::endl;
        return;
    }
    Eigen::Quaterniond q_sensor, q_imu;
    std::string temp_line;
    while (true)
    {
        if (!getline(fin, temp_line))
            break;
        std::vector<double> value = str2vec(temp_line);
        if (value.size() != 8)
        {
            std::cout << "load param error,param:" << temp_line << std::endl;
            break;
        }
        Eigen::Quaterniond q_sensor(value[0], value[1], value[2], value[3]);
        Eigen::Quaterniond q_imu(value[4], value[5], value[6], value[7]);
        corres.push_back(std::move(std::pair<Eigen::Quaterniond, Eigen::Quaterniond>(q_sensor, q_imu)));
    }
    fin.close();
}

void CalibLidarIMU::downsampleCloud(CloudT::Ptr in_cloud, CloudT::Ptr out_cloud, float leaf_size)
{
    pcl::VoxelGrid<PointT> ds;
    ds.setInputCloud(in_cloud);
    ds.setLeafSize((float)leaf_size, (float)leaf_size, (float)leaf_size);
    ds.filter(*out_cloud);
}

void CalibLidarIMU::saveNDTmap(std::string &mapfile)
{
    std::cout << "save pcd at:" << mapfile << ", cloud size: " << map_cloud_->size() << std::endl;
    if (map_cloud_->empty())
    {
        //TODO: compute map
        return;
    }
    pcl::io::savePCDFile<PointT>(mapfile, *map_cloud_);
}