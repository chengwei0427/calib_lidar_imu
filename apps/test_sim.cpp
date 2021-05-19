//C++
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "math_utils.h"

using namespace std;
std::vector<std::pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres;

void saveQuParam(std::string &file)
{
    std::ofstream fout(file, std::ios::app);
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

void saveQu(std::string &file, Eigen::Quaterniond &q_sensor, Eigen::Quaterniond &q_imu)
{
    std::ofstream fout(file, ios::out | std::ios::app);
    fout << std::fixed << std::setprecision(9)
         << q_sensor.w() << " " << q_sensor.x() << " " << q_sensor.y() << " " << q_sensor.z()
         << " " << q_imu.w() << " " << q_imu.x() << " " << q_imu.y() << " " << q_imu.z() << std::endl;
    fout.close();
}

double generate(int t)
{
    srand(t);
    // std::cout << t << std::endl;
    int x = rand() % 60 - 30; // -30~30
    return x / 10.0;
}

Eigen::Quaterniond getInterpolatedAttitude(const Eigen::Quaterniond &q_start, const Eigen::Quaterniond &q_end, double scale)
{
    return q_start.slerp(scale, q_end);
}

Eigen::Quaterniond getInterpolatedAttitude2(const Eigen::Quaterniond &q_s_w, const Eigen::Quaterniond &q_e_w, double scale)
{
    if (0 == scale || scale > 1)
        return move(Eigen::Quaterniond().Identity());

    // calculate angleaxis difference
    Eigen::Quaterniond q_e_s = q_s_w.inverse() * q_e_w;
    q_e_s.normalize();
    Eigen::AngleAxisd diff_angle_axis(q_e_s);

    // interpolated attitude by scale
    double interpolated_angle = diff_angle_axis.angle() * scale;
    Eigen::Quaterniond q_ie_s(Eigen::AngleAxisd(interpolated_angle, diff_angle_axis.axis()).toRotationMatrix());
    Eigen::Quaterniond q_ie_w = q_s_w * q_ie_s;
    q_ie_w.normalize();

    return move(q_ie_w);
}

int main()
{
    char *home_env_var = std::getenv("HOME");
    std::string sim_file = std::string(home_env_var) + "/sim_qua.txt";
    Eigen::Vector3d sim_R(0.0024, -1.27, 0.298); //  roll-pitch-yaw
    Eigen::Matrix3d true_R = Eigen::Matrix3d(Eigen::AngleAxisd(sim_R[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(sim_R[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(sim_R[0], Eigen::Vector3d::UnitX()));
    // Eigen::Matrix3d true_R = Eigen::Matrix3d(Eigen::AngleAxisd(sim_R[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(sim_R[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(sim_R[2], Eigen::Vector3d::UnitZ()));
    std::cout << "R:\n"
              << true_R << std::endl;
    std::cout << "R_inv:\n"
              << true_R.inverse() << std::endl;
    std::cout << "R_trans:\n"
              << true_R.transpose() << std::endl;

    Eigen::Vector3d sim_euler = true_R.eulerAngles(2, 1, 0);
    std::cout << "euler: " << sim_euler[2] << " " << sim_euler[1] << " " << sim_euler[0] << std::endl;
    // Eigen::Vector3d sim_euler = true_R.eulerAngles(0, 1, 2);
    // std::cout << "euler: " << sim_euler[0] << " " << sim_euler[1] << " " << sim_euler[2] << std::endl;
    Eigen::Quaterniond sim_q(true_R);
    std::cout << "quaterniond: " << sim_q.w() << " " << sim_q.x() << " " << sim_q.y() << " " << sim_q.z() << std::endl;
    Eigen::Matrix3d sim_q2m = sim_q.toRotationMatrix();
    std::cout << "q to matrix:\n"
              << sim_q2m << std::endl;

    { //  测试球面线性插值
        std::cout << " ----- slerp ------" << std::endl;
        Eigen::Vector3d e(0.1, 0.9, -0.2);
        Eigen::Matrix3d R = Eigen::Matrix3d(Eigen::AngleAxisd(e[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(e[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(e[2], Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond qa(R);
        double t = 0.3;
        Eigen::Quaterniond qq = getInterpolatedAttitude(sim_q, qa, t);
        Eigen::Quaterniond qq2 = getInterpolatedAttitude2(sim_q, qa, t);
        std::cout << qq.w() << " " << qq.x() << " " << qq.y() << " " << qq.z() << std::endl;
        std::cout << qq2.w() << " " << qq2.x() << " " << qq2.y() << " " << qq2.z() << std::endl;
    }
    {
        std::cout << " ---------- M2euler ---------" << std::endl;
        std::cout << true_R << std::endl;
        Eigen::Vector3d eu = true_R.eulerAngles(2, 1, 0);
        std::cout << "eu: " << eu[2] << " " << eu[1] << " " << eu[0] << std::endl;
        Eigen::Vector3d euler_d = mathutils::GetRPYFromRotationMatrix(true_R);
        std::cout << "euler_d: " << euler_d[0] << " " << euler_d[1] << " " << euler_d[2] << std::endl;
        Eigen::Matrix3d R = Eigen::Matrix3d(Eigen::AngleAxisd(euler_d[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler_d[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler_d[0], Eigen::Vector3d::UnitX()));
        std::cout << R << std::endl;
    }

    for (int i = 0; i < 300; i++)
    {
        Eigen::Vector3d sim(generate(i * 6 + 1), generate(i * 6 - 50), generate(i * 6 + 5));
        // std::cout << sim[0] << " " << sim[1] << " " << sim[2] << std::endl;
        Eigen::Matrix3d ric = Eigen::Matrix3d(Eigen::AngleAxisd(sim[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(sim[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(sim[2], Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond q(ric);
        Eigen::Matrix3d simM = true_R.inverse() * q * true_R;
        Eigen::Quaterniond q_imu(simM);
        saveQu(sim_file, q, q_imu);
    }

    // saveQuParam(sim_file);
    return 0;
}