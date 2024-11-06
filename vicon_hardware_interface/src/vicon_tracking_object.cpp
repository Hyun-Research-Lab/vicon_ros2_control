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
        /* Butterworth Filters */
        // copy sand paste from CreateButterworth.m
        Apos_butter << 8.710534e-01, -8.823664e-02,
            8.823664e-02, 9.958389e-01;
        Bpos_butter << 1.247855e-01,
            5.884733e-03;
        Cpos_butter << 3.119636e-02, 7.056356e-01;
        Dpos_butter = 2.080567e-03;

        Avel_butter << 9.099300e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
            8.351194e-02, 8.543784e-01, -8.745027e-02, 0.000000e+00, 0.000000e+00,
            3.938323e-03, 8.745027e-02, 9.958760e-01, 0.000000e+00, 0.000000e+00,
            1.800776e-04, 3.998615e-03, 9.126031e-02, 9.391689e-01, -9.144888e-02,
            8.492246e-06, 1.885699e-04, 4.303727e-03, 9.144888e-02, 9.956874e-01;
        Bvel_butter << 1.273782e-01,
            5.569630e-03,
            2.626571e-04,
            1.200985e-05,
            5.663701e-07;
        Cvel_butter << 3.002462e-06, 6.666952e-05, 1.521597e-03, 3.233206e-02, 7.055820e-01;
        Dvel_butter = 2.002421e-07;

        Aw_butter << 9.099300e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
            8.351194e-02, 8.543784e-01, -8.745027e-02, 0.000000e+00, 0.000000e+00,
            3.938323e-03, 8.745027e-02, 9.958760e-01, 0.000000e+00, 0.000000e+00,
            1.800776e-04, 3.998615e-03, 9.126031e-02, 9.391689e-01, -9.144888e-02,
            8.492246e-06, 1.885699e-04, 4.303727e-03, 9.144888e-02, 9.956874e-01;
        Bw_butter << 1.273782e-01,
            5.569630e-03,
            2.626571e-04,
            1.200985e-05,
            5.663701e-07;
        Cw_butter << 3.002462e-06, 6.666952e-05, 1.521597e-03, 3.233206e-02, 7.055820e-01;
        Dw_butter = 2.002421e-07;

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
    Vector<3> ViconTrackingObjectPrivate::_DoButterworthFilterPos(Vector<3> &u)
    {
        Vector<3> Y;
        for (size_t i = 0; i < 3; i++)
        {
            Y(i) = Cpos_butter * Xpos_butter.col(i) + Dpos_butter * u(i);
            Xpos_butter.col(i) = Apos_butter * Xpos_butter.col(i) + Bpos_butter * u(i);
        }
        return Y; // filtered data, Y = {p_x, p_y, p_z}
    }

    Vector<3> ViconTrackingObjectPrivate::_DoButterworthFilterVel(Vector<3> &u)
    {
        Vector<3> Y;
        for (size_t i = 0; i < 3; i++)
        {
            Y(i) = Cvel_butter * Xvel_butter.col(i) + Dvel_butter * u(i);
            Xvel_butter.col(i) = Avel_butter * Xvel_butter.col(i) + Bvel_butter * u(i);
        }
        return Y;
    }

    Vector<3> ViconTrackingObjectPrivate::_DoButterworthFilterW(Vector<3> &u)
    {
        Vector<3> Y;
        for (size_t i = 0; i < 3; i++)
        {
            Y(i) = Cw_butter * Xw_butter.col(i) + Dw_butter * u(i);
            Xw_butter.col(i) = Aw_butter * Xw_butter.col(i) + Bw_butter * u(i);
        }
        return Y;
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
            // i don't think this case should ever happen
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
        const Eigen::Vector3d rho0 = rho(Eigen::seq(0, 2));
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
        Vector<3> Y = _DoButterworthFilterPos(u);

        // record the filtered position data
        fs.time = hs.time;

        fs.px = Y(0);
        fs.py = Y(1);
        fs.pz = Y(2);

        // uncomment to use raw position data
        // fs.px = u(0);
        // fs.py = u(1);
        // fs.pz = u(2);

        // do butterworth filter for velocity, too
        double dt = hs.time - fsPrev.time;
        if (dt > 0)
        {
            u << (fs.px - fsPrev.px) / dt, (fs.py - fsPrev.py) / dt, (fs.pz - fsPrev.pz) / dt;
        }
        else
        {
            u << fsPrev.vx, fsPrev.vy, fsPrev.vz;
        }
        Y = _DoButterworthFilterVel(u);
        fs.vx = Y(0);
        fs.vy = Y(1);
        fs.vz = Y(2);

        // uncomment to use raw velocity data
        // fs.vx = u(0);
        // fs.vy = u(1);
        // fs.vz = u(2);


        /* filter quaternion */
        // get quaternion
        Eigen::Quaterniond q_raw(hs.qw, hs.qx, hs.qy, hs.qz);

        // convert to rotation matrix

        Eigen::Matrix3d R_raw = q_raw.toRotationMatrix();

        if (hasFullRBuffer)
        {
            // this will run once we have a full buffer
            auto [R_est, w_est] = _filterSO3(R_raw);
            Eigen::Quaterniond q_est(R_est);

            // // auto wb = w_est;
            // auto wb = R_est.transpose() * w_est; // w_est is in the {s} frame, R_est is {s -> b} frame
            // // convert R to quaternion

            // // verify that the quaternion is pointing in the same direction as the previous quaternion
            // // this is to prevent the quaternion from flipping
            Eigen::Quaterniond q_prev(fsPrev.qw, fsPrev.qx, fsPrev.qy, fsPrev.qz);

            // // for now, use the same quaternion as the raw data
            if (q_est.vec().dot(q_prev.vec()) < -0.5)
            {
                q_est = Eigen::Quaterniond(-q_est.w(), -q_est.x(), -q_est.y(), -q_est.z());
            }

            // save the estimates
            fs.qw = q_est.w();
            fs.qx = q_est.x();
            fs.qy = q_est.y();
            fs.qz = q_est.z();

            // uncomment to use the Savitzky-Golay filter for angular velocity (not recommended)
            // fs.wb1 = wb(0);
            // fs.wb2 = wb(1);
            // fs.wb3 = wb(2);
        }

        // uncomment to use raw orientation
        // fs.qw = q_raw.w();
        // fs.qx = q_raw.x();
        // fs.qy = q_raw.y();
        // fs.qz = q_raw.z();
        
        /* filter angular velocity */

        // use first order difference to get omega
        Eigen::Quaterniond q_prev(fsPrev.qw, fsPrev.qx, fsPrev.qy, fsPrev.qz);
        Eigen::Quaterniond q_now(fs.qw, fs.qx, fs.qy, fs.qz);
        Eigen::Matrix3d R1 = q_prev.toRotationMatrix();
        Eigen::Matrix3d R2 = q_now.toRotationMatrix();

        Eigen::Vector3d w;
        if (dt > 1e-6)
        {
            w = mr::so3ToVec(mr::MatrixLog3(R1.transpose() * R2) / dt);
        }
        else
        {
            w = Eigen::Vector3d::Zero();
        }

        Y = _DoButterworthFilterW(w);
        fs.wb1 = Y(0);
        fs.wb2 = Y(1);
        fs.wb3 = Y(2);

        // uncomment to use raw angular velocity
        // fs.wb1 = w(0);
        // fs.wb2 = w(1);
        // fs.wb3 = w(2);

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