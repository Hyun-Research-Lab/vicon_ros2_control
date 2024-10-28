#include "vicon_hardware_interface/vicon_tracking_object.hpp"
#include <stdexcept> // std::runtime_error
#include <typeinfo>

namespace vicon_hardware_interface
{

    // utility methods
    void ProjectHalfStateToFullState(const HalfState &hs, FullState &out_fs)
    {
        out_fs.time = hs.time;
        out_fs.position_x = hs.position_x;
        out_fs.position_y = hs.position_y;
        out_fs.position_z = hs.position_z;
        out_fs.velocity_x = 0;
        out_fs.velocity_y = 0;
        out_fs.velocity_z = 0;
        out_fs.orientation_qw = hs.orientation_qw;
        out_fs.orientation_qx = hs.orientation_qx;
        out_fs.orientation_qy = hs.orientation_qy;
        out_fs.orientation_qz = hs.orientation_qz;
        out_fs.omegab_1 = 0;
        out_fs.omegab_2 = 0;
        out_fs.omegab_3 = 0;
    }

    void FullStateToPositionVector(const FullState &fs, Vector<3> &out_vec)
    {
        out_vec(0) = fs.position_x;
        out_vec(1) = fs.position_y;
        out_vec(2) = fs.position_z;
    }

    void FullStateToQuaternionVector(const FullState &fs, Vector<4> &out_vec)
    {
        out_vec(0) = fs.orientation_qw;
        out_vec(1) = fs.orientation_qx;
        out_vec(2) = fs.orientation_qy;
        out_vec(3) = fs.orientation_qz;
    }

    ViconTrackingObjectPrivate::ViconTrackingObjectPrivate()
    {   
        // butter(50, 5/(200/2))
        // A << 0.0000, 0, 0, 0, 0,
        //     0.2764, -0.4472, -0.5528, 0, 0,
        //     0.2764, 0.5528, 0.4472, 0, 0,
        //     0.1056, 0.2111, 0.5528, -0.2361, -0.7639,
        //     0.1056, 0.2111, 0.5528, 0.7639, 0.2361;
        // B << 1.4142, 0.3909, 0.3909, 0.1493, 0.1493;
        // C << 0.0373, 0.0747, 0.1954, 0.2701, 0.4370;
        // D = 0.0528;

        // butter(5, 5/(200/2))
        A << 0.8541, 0, 0, 0, 0,
            0.1287, 0.7644, -0.1389, 0, 0,
            0.0101, 0.1389, 0.9891, 0, 0,
            0.0008, 0.0104, 0.1484, 0.8960, -0.1492,
            0.0001, 0.0008, 0.0117, 0.1492, 0.9883;
        B << 0.2064,
            0.0143,
            0.0011,
            0.0001,
            0.0000;
        C << 0.0000, 0.0003, 0.0041, 0.0528, 0.7030;
        D = 2.3410e-06;

        X = Eigen::Matrix<double, 5, 6>::Random();
    }

    // u is the raw data, u = {p_x, p_y, p_z, wtheta_x, wtheta_y, wtheta_z}
    Vector<6> ViconTrackingObjectPrivate::_DoButterworthFilterUpdate(Vector<6> &u)
    {
        Vector<6> Y;
        for (size_t i = 0; i < 6; i++)
        {
            Y(i) = C * X.col(i) + D * u(i);
            X.col(i) = A * X.col(i) + B * u(i);
        }
        return Y; // filtered data, Y = {p_x, p_y, p_z, wtheta_x, wtheta_y, wtheta_z}
    }

    // the ViconHardwareInterface gives us some data, we need to use it
    // to update the position/velocity estimates, and save its full state
    void ViconTrackingObjectPrivate::PushData(const HalfState &hs)
    {

        // the first time we are given an object, we will assume that its velocities are zero
        // and we will completely fill the buffer with its position data
        if (!haveReceivedData)
        {
            FullState fs;
            ProjectHalfStateToFullState(hs, fs);
            filteredDataPrevious = fs;
            haveReceivedData = true;
            return;
        }

        // obtain wtheta from the quaternion
        Eigen::Quaternion<double> q_raw(hs.orientation_qw, hs.orientation_qx, hs.orientation_qy, hs.orientation_qz);
        Eigen::AngleAxis<double> angle_axis_raw(q_raw);
        Vector<3> omega_theta_raw = angle_axis_raw.axis() * angle_axis_raw.angle();

        // filter the position and exponential coordiante (w_theta) using butterworth filter
        Vector<6> u;
        u << hs.position_x, hs.position_y, hs.position_z, omega_theta_raw(0), omega_theta_raw(1), omega_theta_raw(2);
        Vector<6> Y = _DoButterworthFilterUpdate(u);

        // bypass butterworth filter
        // Y = u;

        FullState fs_filtered;
        fs_filtered.time = hs.time;
        fs_filtered.position_x = Y(0);
        fs_filtered.position_y = Y(1);
        fs_filtered.position_z = Y(2);

        // convert Y(3), Y(4), Y(5) back to quaternion
        Vector<3> omega_theta_filtered = Y.tail<3>();
        Eigen::AngleAxis<double> angle_axis_filtered(omega_theta_filtered.norm(), omega_theta_filtered.normalized());
        Eigen::Quaternion<double> q_filtered(angle_axis_filtered);

        fs_filtered.orientation_qw = q_filtered.w();
        fs_filtered.orientation_qx = q_filtered.x();
        fs_filtered.orientation_qy = q_filtered.y();
        fs_filtered.orientation_qz = q_filtered.z();

        // calculate the linear velocity using a simple finite difference
        double dt = fs_filtered.time - filteredDataPrevious.time;

        if (dt > 1e-4)
        {
            fs_filtered.velocity_x = (fs_filtered.position_x - filteredDataPrevious.position_x) / dt;
            fs_filtered.velocity_y = (fs_filtered.position_y - filteredDataPrevious.position_y) / dt;
            fs_filtered.velocity_z = (fs_filtered.position_z - filteredDataPrevious.position_z) / dt;
        }
        else
        {
            fs_filtered.velocity_x = 0;
            fs_filtered.velocity_y = 0;
            fs_filtered.velocity_z = 0;
        }

        // calculate the angular velocity from the filtered quaternions
        // rotation matrix from quaternion
        Eigen::Quaternion<double> q_prev(filteredDataPrevious.orientation_qw, filteredDataPrevious.orientation_qx, filteredDataPrevious.orientation_qy, filteredDataPrevious.orientation_qz);
        Matrix<3> R2 = q_filtered.toRotationMatrix();
        Matrix<3> R1 = q_prev.toRotationMatrix();

        Matrix<3> R = R1.transpose() * R2;
        Eigen::AngleAxis<double> omega_theta(R);
        Vector<3> omega = omega_theta.axis();
        double theta = omega_theta.angle();

        double theta_dot = 0;
        if (dt > 1e-4)
        {
            theta_dot = theta / dt;
        }

        Vector<3> omegab_filtered = omega * theta_dot;

        fs_filtered.omegab_1 = omegab_filtered(0);
        fs_filtered.omegab_2 = omegab_filtered(1);
        fs_filtered.omegab_3 = omegab_filtered(2);

        // set the latest filtered data
        filteredDataPrevious = fs_filtered;
    }

    bool ViconTrackingObject::AddSensor(const std::string &name)
    {
        if (HasSensorWithName(name))
        {
            // sensor already exists
            return false;
        }

        viconObjects[name] = ViconTrackingObjectPrivate();
        viconObjectNames.insert(name);
        return true;
    }

    void ViconTrackingObject::PushData(const std::string &name, const HalfState &hs)
    {
        if (!HasSensorWithName(name))
        {
            throw std::runtime_error("Sensor not found: " + name);
        }

        viconObjects[name].PushData(hs);
    }

    const FullState &ViconTrackingObject::GetLatestState(const std::string &name) const
    {
        if (!HasSensorWithName(name))
        {
            throw std::runtime_error("Sensor not found: " + name);
        }
        return viconObjects.at(name).GetLatestState();
    }

}