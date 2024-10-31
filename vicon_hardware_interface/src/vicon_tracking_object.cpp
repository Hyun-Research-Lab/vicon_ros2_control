#include "vicon_hardware_interface/vicon_tracking_object.hpp"
#include <stdexcept> // std::runtime_error
#include <typeinfo>

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

        // A << 0.1446,   -0.3719,         0,         0,         0,         0,         0,         0,         0,         0,
        //      0.3719,    0.8792,         0,         0,         0,         0,         0,         0,         0,         0,
        //      0.0717,    0.3625,    0.1872,   -0.3858,         0,         0,         0,         0,         0,         0,
        //      0.0233,    0.1178,    0.3858,    0.8747,         0,         0,         0,         0,         0,         0,
        //      0.0048,    0.0244,    0.0801,    0.3892,    0.2779,   -0.4152,         0,         0,         0,         0,
        //      0.0016,    0.0079,    0.0260,    0.1265,    0.4152,    0.8651,         0,         0,         0,         0,
        //      0.0004,    0.0018,    0.0060,    0.0293,    0.0963,    0.4327,    0.4280,   -0.4640,         0,         0,
        //      0.0001,    0.0006,    0.0020,    0.0095,    0.0313,    0.1406,    0.4640,    0.8492,         0,         0,
        //      0.0000,    0.0002,    0.0005,    0.0026,    0.0084,    0.0378,    0.1249,    0.4977,    0.6567,   -0.5383,
        //      0.0000,    0.0001,    0.0002,    0.0008,    0.0027,    0.0123,    0.0406,    0.1617,    0.5383,    0.8251;

        // B << 0.5259,
        //     0.1709,
        //     0.0330,
        //     0.0107,
        //     0.0022,
        //     0.0007,
        //     0.0002,
        //     0.0001,
        //     0.0000,
        //     0.0000;

        // C << 0.0000,    0.0000,    0.0001,    0.0003,    0.0010,    0.0043,    0.0143,    0.0572,    0.1903,    0.6453;
        // D = 1.6836e-06;

        X = Eigen::Matrix<double, BUTTERWORTH_FILTER_STATES, 6>::Zero();
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
    bool ViconTrackingObjectPrivate::PushData(const HalfState &hs)
    {

        // check the that data is valid
        // the quaternion should be normalized, but will be zero if the object is not being tracked
        Vector<4> q_raw(hs.qw, hs.qx, hs.qy, hs.qz);
        if (q_raw.norm() < 1e-6)
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
            // grab the theta from the quaternion
            Eigen::Quaternion<double> q(hs.qw, hs.qx, hs.qy, hs.qz);
            Eigen::AngleAxis<double> angleAxis(q);
            thetaPrev = angleAxis.angle();

            // save the rest of the data
            ProjectHalfStateToFullState(hs, fs);
            fsPrev = fs;
            haveNeverReceivedValidData = false;
            return true;
        }
        
        // first, make the quaternion continuous
        Eigen::Quaterniond qContinuous; // store its value here

        Eigen::Quaterniond q(hs.qw, hs.qx, hs.qy, hs.qz);
        Eigen::Quaterniond qPrev(hsPrev.qw, hsPrev.qx, hsPrev.qy, hsPrev.qz);

        double imNorm = q.vec().norm();
        double imNormPrev = qPrev.vec().norm();

        if (imNorm < 1e-4 || imNormPrev < 1e-4) {
            // theta = 0 or 2 pi
            if (qPrev.w() > 0.95) {
                qContinuous = Eigen::Quaterniond(std::abs(hs.qw), hs.qx, hs.qy, hs.qz);
            }
            else if (qPrev.w() < -0.95) {
                qContinuous = Eigen::Quaterniond(-std::abs(hs.qw), hs.qx, hs.qy, hs.qz);
            }
        }
        else {
            // the quaternion has non-zero imaginary part, so there is a chance 
            // that its components are discontinuous
            Vector<3> omega = q.vec() / imNorm;
            Vector<3> omegaPrev = qPrev.vec() / imNormPrev;
            
            // the quaternion flipped by 180 degrees
            if (omega.dot(omegaPrev) < -0.99) {
                qContinuous = Eigen::Quaterniond(-hs.qw, -hs.qx, -hs.qy, -hs.qz);
            }
            // we are currently in the zero order hold state, 
            // and the quaternion has flipped by more than 90 degrees
            else if (lastDataWasInvalid && omega.dot(omegaPrev) < 0) {
                qContinuous = Eigen::Quaterniond(-hs.qw, -hs.qx, -hs.qy, -hs.qz);
            }
            else {
                qContinuous = Eigen::Quaterniond(hs.qw, hs.qx, hs.qy, hs.qz);
            }
        }

        // now we need a smooth theta (remove wrapping at 0, pi)
        
        // first, we need to determine whether the previous theta is closer to an even 
        // or an odd multiple of pi
        // double tmp = std::abs(thetaPrev) / (2*M_PI) + 0.25f;
        // double decimal = tmp - (int)tmp;
        // int isCloserToOddMultipleOfPi = (decimal > 0.5) ? 1 : 0;

        // // depending on the answer, we know whether to use 
        // // the acos or asin function to get the angle
        // // Recall that q = (cos(theta/2), sin(theta/2) * n)
        // // where n is a unit axis of rotation
        // double theta;
        // if (isCloserToOddMultipleOfPi) {
        //     theta = 2.0 * std::acos(qContinuous.w());
        // }
        // else {
        //     theta = 2.0 * std::asin(qContinuous.vec().norm());
        // }

        Eigen::AngleAxis<double> aaContinuous(qContinuous);
        double theta = aaContinuous.angle(); // TODO: add the offset
        Vector<3> axis = aaContinuous.axis();

        fs.qContinuous_w = qContinuous.w();
        fs.qContinuous_x = qContinuous.x();
        fs.qContinuous_y = qContinuous.y();
        fs.qContinuous_z = qContinuous.z();

        // get the exponential coordinate of the qContinuous
        Vector<3> expc;
        if (qContinuous.vec().norm() < 1e-6) {
            expc << 0, 0, 0;
        }
        else {
            // expc3 = theta * omega_axis
            expc = theta * axis;
        }

        fs.eigen1 = axis(0);
        fs.eigen2 = axis(1);
        fs.eigen3 = axis(2);

        fs.qaxis1 = qContinuous.vec()(0) / qContinuous.vec().norm();
        fs.qaxis2 = qContinuous.vec()(1) / qContinuous.vec().norm();
        fs.qaxis3 = qContinuous.vec()(2) / qContinuous.vec().norm();

        // if inner product of eigen_axis and qContinuous_axis are negative
        // then we need to "unwrap" theta
        if (axis.dot(qContinuous.vec()) < 0) {
            theta = 2*M_PI - theta;
        }

        fs.theta = theta;

        // filter the position and exponential coordiante (w_theta) using butterworth filter
        Vector<6> u;
        u << hs.px, hs.py, hs.pz, expc(0), expc(1), expc(2);
        Vector<6> Y = _DoButterworthFilterUpdate(u);

        // save the previous (smooth) theta
        thetaPrev = theta;

        // bypass butterworth filter
        // Y = u;
        
        // record the filtered position data
        fs.time = hs.time;
        fs.px = Y(0);
        fs.py = Y(1);
        fs.pz = Y(2);

        // convert the exponential coordinate back to a quaternion
        // and then record this value
        double thetaFiltered = Y.tail(3).norm();
        double sinThetaFiltered = std::sin(thetaFiltered / 2.0);
        
        if (std::abs(thetaFiltered) < 1e-6) {
            fs.qw = 1;
            fs.qx = 0;
            fs.qy = 0;
            fs.qz = 0;
        }
        else {
            fs.qw = std::cos(thetaFiltered / 2.0);
            fs.qx = sinThetaFiltered * Y(3) / thetaFiltered;
            fs.qy = sinThetaFiltered * Y(3) / thetaFiltered;
            fs.qz = sinThetaFiltered * Y(3) / thetaFiltered;
        }
        
        
        // calculate the linear velocity using a simple finite difference
        double dt = fs.time - fsPrev.time;

        if (dt > 1e-6)
        {
            fs.vx = (fs.px - fsPrev.px) / dt;
            fs.vy = (fs.py - fsPrev.py) / dt;
            fs.vz = (fs.pz - fsPrev.pz) / dt;
        }
        else
        {
            fs.vx = 0;
            fs.vy = 0;
            fs.vz = 0;
        }

        // calculate the angular velocity from the filtered quaternions
        // rotation matrix from quaternion
        Eigen::Quaterniond qFiltered;
        Matrix<3> R2 = qFiltered.toRotationMatrix();
        Matrix<3> R1 = qPrev.toRotationMatrix();

        Matrix<3> R = R1.transpose() * R2;
        Eigen::AngleAxisd omega_theta(R);
        Vector<3> omega = omega_theta.axis();
        theta = omega_theta.angle();

        double theta_dot;
        if (dt < 1e-6)
        {
            theta_dot = 0;
        }
        else {
            theta_dot = theta / dt;
        }

        fs.wb1 = theta_dot * omega(0);
        fs.wb2 = theta_dot * omega(1);
        fs.wb3 = theta_dot * omega(2);

        // set the latest filtered data
        fsPrev = fs;
        lastDataWasInvalid = false;
        hsPrev.px = hs.px;
        hsPrev.py = hs.py;
        hsPrev.pz = hs.pz;
        hsPrev.qw = qContinuous.w();
        hsPrev.qx = qContinuous.x();
        hsPrev.qy = qContinuous.y();
        hsPrev.qz = qContinuous.z();


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