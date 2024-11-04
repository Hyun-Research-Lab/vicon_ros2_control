#include "vicon_hardware_interface/vicon_tracking_object.hpp"
#include <stdexcept> // std::runtime_error
#include <typeinfo>
#include "modern_robotics.h"

namespace vicon_hardware_interface
{

    // utility methods
    void ProjectHalfStateToFullState(const HalfState &hs, FullState &out_fs)
    {
        out_fs.time = hs.time;
        out_fs.px = hs.px;
        out_fs.py = hs.py;
        out_fs.pz = hs.pz;
        out_fs.vx = 0;
        out_fs.vy = 0;
        out_fs.vz = 0;
        out_fs.qw = hs.qw;
        out_fs.qx = hs.qx;
        out_fs.qy = hs.qy;
        out_fs.qz = hs.qz;
        out_fs.wb1 = 0;
        out_fs.wb2 = 0;
        out_fs.wb3 = 0;
    }

    void FullStateToPositionVector(const FullState &fs, Vector<3> &out_vec)
    {
        out_vec(0) = fs.px;
        out_vec(1) = fs.py;
        out_vec(2) = fs.pz;
    }

    void FullStateToQuaternionVector(const FullState &fs, Vector<4> &out_vec)
    {
        out_vec(0) = fs.qw;
        out_vec(1) = fs.qx;
        out_vec(2) = fs.qy;
        out_vec(3) = fs.qz;
    }

    ViconTrackingObjectPrivate::ViconTrackingObjectPrivate()
    {
        // butter(5, 50/(200/2)) (50 Hz)
        // A << 0.0000, 0, 0, 0, 0,
        //     0.2764, -0.4472, -0.5528, 0, 0,
        //     0.2764, 0.5528, 0.4472, 0, 0,
        //     0.1056, 0.2111, 0.5528, -0.2361, -0.7639,
        //     0.1056, 0.2111, 0.5528, 0.7639, 0.2361;
        // B << 1.4142, 0.3909, 0.3909, 0.1493, 0.1493;
        // C << 0.0373, 0.0747, 0.1954, 0.2701, 0.4370;
        // D = 0.0528;

        // butter(5, 5/(200/2)) (5 Hz)
        // A_butter << 0.8541, 0, 0, 0, 0,
        //     0.1287, 0.7644, -0.1389, 0, 0,
        //     0.0101, 0.1389, 0.9891, 0, 0,
        //     0.0008, 0.0104, 0.1484, 0.8960, -0.1492,
        //     0.0001, 0.0008, 0.0117, 0.1492, 0.9883;
        // B_butter << 0.2064,
        //     0.0143,
        //     0.0011,
        //     0.0001,
        //     0.0000;
        // C_butter << 0.0000, 0.0003, 0.0041, 0.0528, 0.7030;
        // D_butter = 2.3410e-06;

        // 20 Hz Butterworth filter with 5 states
        // [A, B, C, D] = butter(5, 20/(200/2))
        // A << 0.5095, 0, 0, 0, 0,
        //     0.3007, 0.2260, -0.3984, 0, 0,
        //     0.0977, 0.3984, 0.8706, 0, 0,
        //     0.0243, 0.0991, 0.4652, 0.5309, -0.4974,
        //     0.0079, 0.0322, 0.1512, 0.4974, 0.8384;
        // B << 0.6936,
        //     0.1382,
        //     0.0449,
        //     0.0112,
        //     0.0036;
        // C << 0.0028, 0.0114, 0.0534, 0.1759, 0.6500;
        // D = 0.0013;

        // 20 Hz Butterworth filter with 10 states
        // [A, B, C, D] = butter(10, 20/(200/2))

        A_butter << 0.1446, -0.3719, 0, 0, 0, 0, 0, 0, 0, 0,
            0.3719, 0.8792, 0, 0, 0, 0, 0, 0, 0, 0,
            0.0717, 0.3625, 0.1872, -0.3858, 0, 0, 0, 0, 0, 0,
            0.0233, 0.1178, 0.3858, 0.8747, 0, 0, 0, 0, 0, 0,
            0.0048, 0.0244, 0.0801, 0.3892, 0.2779, -0.4152, 0, 0, 0, 0,
            0.0016, 0.0079, 0.0260, 0.1265, 0.4152, 0.8651, 0, 0, 0, 0,
            0.0004, 0.0018, 0.0060, 0.0293, 0.0963, 0.4327, 0.4280, -0.4640, 0, 0,
            0.0001, 0.0006, 0.0020, 0.0095, 0.0313, 0.1406, 0.4640, 0.8492, 0, 0,
            0.0000, 0.0002, 0.0005, 0.0026, 0.0084, 0.0378, 0.1249, 0.4977, 0.6567, -0.5383,
            0.0000, 0.0001, 0.0002, 0.0008, 0.0027, 0.0123, 0.0406, 0.1617, 0.5383, 0.8251;

        B_butter << 0.5259,
            0.1709,
            0.0330,
            0.0107,
            0.0022,
            0.0007,
            0.0002,
            0.0001,
            0.0000,
            0.0000;

        C_butter << 0.0000, 0.0000, 0.0001, 0.0003, 0.0010, 0.0043, 0.0143, 0.0572, 0.1903, 0.6453;
        D_butter = 1.6836e-06;

        X_butter = Eigen::Matrix<double, BUTTER_STATES, 3>::Zero();

        /* Savitzky-Golay Filter */
        Eigen::Matrix<double, 3 * (SGOLAY_WINDOW_SIZE + 1), 9> Ak;

        // set the first block column to identity
        for (size_t i = 0; i < SGOLAY_WINDOW_SIZE + 1; i++)
        {
            Ak.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity();
        }

        // set the second block column to (t_{k-n} - t_k) * I
        for (size_t i = 0; i < SGOLAY_WINDOW_SIZE + 1; i++)
        {
            double a = (SGOLAY_WINDOW_SIZE - i) * SGOLAY_DT;
            Ak.block<3, 3>(3 * i, 3) = a * Eigen::Matrix3d::Identity();
        }

        // set the third block column to 1/2(t_{k-n} - t_k)^2 * I
        for (size_t i = 0; i < SGOLAY_WINDOW_SIZE + 1; i++)
        {
            double a = (SGOLAY_WINDOW_SIZE - i) * SGOLAY_DT;
            Ak.block<3, 3>(3 * i, 6) = 0.5 * a * a * Eigen::Matrix3d::Identity();
        }

        // now, calculate A_sgolay = (Ak^T Ak)^-1 Ak^T
        // note that we can simply write X\Y as X.colPivHouseholderQr().solve(Y).eval()
        // see https://stackoverflow.com/questions/35189394/eigen-equivalent-to-octave-matlab-mldivide-for-rectangular-matrices
        // const auto X = Ak.transpose() * Ak;
        // const auto Y = Ak.transpose();
        // A_sgolay = X.colPivHouseholderQr().solve(Y).eval();
        A_sgolay = (Ak.transpose() * Ak).inverse() * Ak.transpose();

        std::cout << "A_sgolay: " << A_sgolay << std::endl;
    }

    // u is the raw data, u = {p_x, p_y, p_z}
    Vector<3> ViconTrackingObjectPrivate::_DoButterworthFilterUpdate(Vector<3> &u)
    {
        Vector<3> Y;
        for (size_t i = 0; i < 3; i++)
        {
            Y(i) = C_butter * X_butter.col(i) + D_butter * u(i);
            X_butter.col(i) = A_butter * X_butter.col(i) + B_butter * u(i);
        }
        return Y; // filtered data, Y = {p_x, p_y, p_z}
    }

    // compute the right trivialized tangent d exp on SO(3)
    // INPUTS:    a       : 3 vector, isomorphism to element of so(3)
    // OUTPUTS:   res     : diff exponential of a
    // Closed form expression of right trivialized tangent d exp on SO(3)
    // https://gitlab.tue.nl/robotics-lab-public/savitzky-golay-filtering-on-so3/-/blob/main/functions/dexpSO3.m?ref_type=heads
    Eigen::Matrix3d ViconTrackingObjectPrivate::_dexpSO3(const Eigen::Vector3d &a) const
    {
        Eigen::Matrix3d res;

        double phi = a.norm();
        Eigen::Matrix3d a_hat = mr::VecToso3(a);

        if (phi < 1e-6)
        {
            res = Eigen::Matrix3d::Identity() + 0.5 * a_hat;
        }
        else
        {
            double beta = std::sin(phi * 0.5) * std::sin(phi * 0.5) / (phi * phi * 0.25);
            double alpha = std::sin(phi) / phi;
            res = Eigen::Matrix3d::Identity() + 0.5 * beta * a_hat + (1.0 / (phi * phi)) * (1 - alpha) * a_hat * a_hat;
        }

        return res;
    }

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d> ViconTrackingObjectPrivate::_filterSO3(const Eigen::Matrix3d &R)
    {
        // populate the vector b_sgolay
        for (size_t i = 0; i < SGOLAY_WINDOW_SIZE; i++)
        {
            b_sgolay(Eigen::seq(3 * i, 3 * i + 2)) = mr::so3ToVec(mr::MatrixLog3(_GetR(i) * R.transpose()));
        }
        b_sgolay.tail(3) = Eigen::Vector3d::Zero();

        // solve the least squares problem
        // but since (Ak^T Ak)^-1 Ak^T is already calculated, we simply have
        const Eigen::Vector<double, 9> rho = A_sgolay * b_sgolay;
        const Eigen::Vector3d rho0 = rho.head(3);
        const Eigen::Vector3d rho1 = rho(Eigen::seq(3, 5));

        // calculate rotation matrix
        Eigen::Matrix3d R_est = mr::MatrixExp3(mr::VecToso3(rho0)) * R;

        // estimate the angular velocity (in spatial frame)
        Eigen::Vector3d w_est = _dexpSO3(rho0) * rho1;

        return std::make_tuple(R_est, w_est);
    }

    // the ViconHardwareInterface gives us some data, we need to use it
    // to update the position/velocity estimates, and save its full state
    bool ViconTrackingObjectPrivate::PushData(const HalfState &hs)
    {

        // check the that data is valid
        // the quaternion should be normalized, but will be zero if the object is not being tracked
        Vector<4> check(hs.qw, hs.qx, hs.qy, hs.qz);
        if (check.norm() < 1e-6)
        {
            lastDataWasInvalid = true;
            // do not update the last (valid) data
            return false;
        }

        // this is where we will store the filtered data
        FullState fs;

        // the first time we are given an object, we will assume that its velocities are zero
        // and we will completely fill the buffer with its position data
        if (haveNeverReceivedValidData)
        {
            // save the rest of the data
            ProjectHalfStateToFullState(hs, fs);
            fsPrev = fs;
            haveNeverReceivedValidData = false;
            return true;
        }

        // filter the position/velocity using butterworth filter
        Vector<3> u;
        u << hs.px, hs.py, hs.pz;
        Vector<3> Y = _DoButterworthFilterUpdate(u);

        // record the filtered position data
        fs.time = hs.time;
        fs.px = Y(0);
        fs.py = Y(1);
        fs.pz = Y(2);

        // calculate the velocity
        double dt = hs.time - fsPrev.time;
        if (dt > 0)
        {
            fs.vx = (fs.px - fsPrev.px) / dt;
            fs.vy = (fs.py - fsPrev.py) / dt;
            fs.vz = (fs.pz - fsPrev.pz) / dt;
        }
        else
        {
            fs.vx = fsPrev.vx;
            fs.vy = fsPrev.vy;
            fs.vz = fsPrev.vz;
        }

        // now, smooth out the velocity
        // this is done by taking the average of the current and previous velocity
        fs.vx = 0.5 * (fs.vx + fsPrev.vx);
        fs.vy = 0.5 * (fs.vy + fsPrev.vy);
        fs.vz = 0.5 * (fs.vz + fsPrev.vz);

        // get quaternion
        Eigen::Quaterniond q_raw(hs.qw, hs.qx, hs.qy, hs.qz);

        // convert to rotation matrix
        Eigen::Matrix3d R_raw = q_raw.toRotationMatrix();

        if (hasFullRBuffer)
        {
            // this will run once we have a full buffer
            auto [R_est, w_est] = _filterSO3(R_raw);

            auto wb = w_est;
            // auto wb = R_est.transpose() * w_est; // w_est is in the {s} frame, R_est is {s -> b} frame
            // convert R to quaternion
            Eigen::Quaterniond q_est(R_est);

            // verify that the quaternion is pointing in the same direction as the previous quaternion
            // this is to prevent the quaternion from flipping
            Eigen::Quaterniond q_prev(fsPrev.qw, fsPrev.qx, fsPrev.qy, fsPrev.qz);

            // for now, use the same quaternion as the raw data
            if (q_est.vec().dot(q_raw.vec()) < -0.5)
            {
                q_est = Eigen::Quaterniond(-q_est.w(), -q_est.x(), -q_est.y(), -q_est.z());
            }

            // save the estimates
            fs.qw = q_est.w();
            fs.qx = q_est.x();
            fs.qy = q_est.y();
            fs.qz = q_est.z();

            fs.wb1 = wb(0);
            fs.wb2 = wb(1);
            fs.wb3 = wb(2);
        }

        // set the latest filtered data
        fsPrev = fs;
        _PushCircularBuffer(R_raw);
        lastDataWasInvalid = false;

        return true;
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

    bool ViconTrackingObject::PushData(const std::string &name, const HalfState &hs)
    {
        if (!HasSensorWithName(name))
        {
            throw std::runtime_error("Sensor not found: " + name);
        }

        return viconObjects[name].PushData(hs);
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