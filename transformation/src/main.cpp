#include <iostream>
#include <cmath>
#include <array>
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

struct Quaternion {
    double x, y, z, w;// 四元数 q = w + xi + yj + zk

    Quaternion() : x(0), y(0), z(0), w(1) {}

    Quaternion(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}

    // 欧拉角转四元数 (roll, pitch, yaw in radians)
    static Quaternion fromEuler(double roll, double pitch, double yaw) {
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    // 四元数转欧拉角 (返回单位为弧度)
    std::array<double, 3> toEuler() const { 
        std::array<double, 3> euler;

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            euler[1] = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        euler[2] = std::atan2(siny_cosp, cosy_cosp);

        return euler;
    }

    // 四元数乘法：组合旋转
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w,
            w * q.w - x * q.x - y * q.y - z * q.z
        );
    }

    // 四元数旋转向量（旋转位置）
    std::array<double, 3> rotateVector(const std::array<double, 3>& v) const {
        Quaternion qv(v[0], v[1], v[2], 0);
        Quaternion q_conj(-x, -y, -z, w);
        Quaternion q_res = (*this) * qv * q_conj;
        return {q_res.x, q_res.y, q_res.z};
    }
};

struct Pose {
    std::array<double, 3> position;   // x, y, z
    std::array<double, 3> rpy;        // 欧拉角 roll, pitch, yaw (单位：弧度)

    Pose() : position{0, 0, 0}, rpy{0, 0, 0} {}

    Pose(std::array<double, 3> pos, std::array<double, 3> euler) : position(pos), rpy(euler) {}
};

// 将一个位姿从局部坐标系转换到全局坐标系
Pose transformPose(const Pose& local_pose, const Pose& frame_pose) {
    Quaternion q_local = Quaternion::fromEuler(local_pose.rpy[0], local_pose.rpy[1], local_pose.rpy[2]);
    Quaternion q_frame = Quaternion::fromEuler(frame_pose.rpy[0], frame_pose.rpy[1], frame_pose.rpy[2]);

    Quaternion q_global = q_frame * q_local;
    std::array<double, 3> global_rpy = q_global.toEuler();

    // 旋转局部位置到全局
    std::array<double, 3> rotated_position = q_frame.rotateVector(local_pose.position);
    std::array<double, 3> global_position = {
        frame_pose.position[0] + rotated_position[0],
        frame_pose.position[1] + rotated_position[1],
        frame_pose.position[2] + rotated_position[2]
    };

    return Pose(global_position, global_rpy);
}
int main() {
    // 本地位姿 (x, y, z, roll, pitch, yaw) - 单位为弧度
    Pose local({1, 0, 0}, {0, 0, M_PI / 2});  // 向x轴移动1米，绕z轴旋转90度

    // 坐标系A相对于世界坐标系的位姿
    Pose frame({0, 0, 0}, {0, 0, M_PI / 2});  // 同样绕z轴旋转90度

    Pose global = transformPose(local, frame);

    std::cout << "Global Position: ";
    for (auto p : global.position) std::cout << p << " ";
    std::cout << "\nGlobal RPY (deg): ";
    for (auto r : global.rpy) std::cout << RAD2DEG(r) << " ";
    std::cout << std::endl;

    return 0;
}
