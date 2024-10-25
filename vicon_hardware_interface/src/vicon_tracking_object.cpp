#include "vicon_hardware_interface/vicon_tracking_object.hpp"
#include <stdexcept>   // std::runtime_error

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
        out_fs.orientation_qx = hs.orientation_qx;
        out_fs.orientation_qy = hs.orientation_qy;
        out_fs.orientation_qz = hs.orientation_qz;
        out_fs.orientation_qw = hs.orientation_qw;
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
        out_vec(0) = fs.orientation_qx;
        out_vec(1) = fs.orientation_qy;
        out_vec(2) = fs.orientation_qz;
        out_vec(3) = fs.orientation_qw;
    }

    // the ViconHardwareInterface gives us some data, we need to use it
    // to update the position/velocity estimates, and save its full state
    void ViconTrackingObject::PushData(const HalfState &hs)
    {

        // the first time we are given an object, we will assume that its velocities are zero
        // and we will completely fill the buffer with its position data
        if (!buffersInitialized)
        {
            FullState fs;
            ProjectHalfStateToFullState(hs, fs);
            for (size_t i = 0; i < VICON_BUFFER_LENGTH; i++)
            {
                // push to raw and filtered data
                _PushRaw(fs);
                _PushFiltered(fs);
            }
            buffersInitialized = true;
            return;
        }

        // use an IIR filter to filter the position data
        FullState fs_raw;
        ProjectHalfStateToFullState(hs, fs_raw);

        // the filtered position vector
        Vector<3> x0, x1, x2, x3; // raw
        FullStateToPositionVector(fs_raw, x0);
        FullStateToPositionVector(_GetRawData(0), x1);
        FullStateToPositionVector(_GetRawData(1), x2);
        FullStateToPositionVector(_GetRawData(2), x3);

        Vector<3> y1, y2; // filtered
        FullStateToPositionVector(_GetFilteredData(0), y1);
        FullStateToPositionVector(_GetFilteredData(1), y2);
        
        Vector<3> p;
        double a0 = 1 / 4.0;
        double a1 = 1 / 4.0;
        double a2 = 1 / 4.0;
        double a3 = 1 / 4.0;
        double b1 = 0.0;
        double b2 = 0.0;
        p = a0 * x0 + a1 * x1 + a2 * x2 + a3 * x3 + b1 * y1 + b2 * y2;

        // filtered quaternion vector
        Vector<4> q;
        FullStateToQuaternionVector(fs_raw, q);
        Vector<4> q1, q2;
        FullStateToQuaternionVector(_GetFilteredData(0), q1);
        FullStateToQuaternionVector(_GetFilteredData(1), q2);

        
        
        // normalize the quaternion when we are done
        q *= 1.0/q.norm();

        // calculate the linear velocity using a simple finite difference
        double dt = fs_raw.time - _GetRawData(0).time;
        Vector<3> v;
        v[0] = (p[0] - y1[0]) / dt;
        v[1] = (p[1] - y1[1]) / dt;
        v[2] = (p[2] - y1[2]) / dt;

        // do quaternion derivative (this approximation is not the best, but works for small time steps)
        Vector<3> omegab;
        // q_dot[0] = (p[3] - y1[3]) / dt;
        // q_dot[1] = (p[4] - y1[4]) / dt;
        // q_dot[2] = (p[5] - y1[5]) / dt;
        // q_dot[3] = (p[6] - y1[6]) / dt;


        // push to raw and filtered data arrays
        FullState fs_filtered;
        fs_filtered.time = fs_raw.time;
        fs_filtered.position_x = p[0];
        fs_filtered.position_y = p[1];
        fs_filtered.position_z = p[2];
        fs_filtered.velocity_x = v[0];
        fs_filtered.velocity_y = v[1];
        fs_filtered.velocity_z = v[2];
        fs_filtered.orientation_qx = p[3];
        fs_filtered.orientation_qy = p[4];
        fs_filtered.orientation_qz = p[5];
        fs_filtered.orientation_qw = p[6];
        fs_filtered.omegab_1 = omegab[0];
        fs_filtered.omegab_2 = omegab[1];
        fs_filtered.omegab_3 = omegab[2];

        // push to the buffers
        _PushRaw(fs_raw);
        _PushFiltered(fs_filtered);

        // update the output state
        outputState = fs_filtered;
    }

    void ViconTrackingObject::_PushRaw(const FullState &fs)
    {
        statesRaw[latestRawIndex] = fs;
        latestRawIndex = (latestRawIndex + 1) % VICON_BUFFER_LENGTH;
    }

    void ViconTrackingObject::_PushFiltered(const FullState &fs)
    {
        statesFiltered[latestFilteredIndex] = fs;
        latestFilteredIndex = (latestFilteredIndex + 1) % VICON_BUFFER_LENGTH;
    }

    const FullState &ViconTrackingObject::_GetRawData(const size_t previousIndex) const
    {
        if (previousIndex >= VICON_BUFFER_LENGTH)
        {
            throw std::runtime_error("Index out of bounds. Expected index to be less than buffer length.");
        }

        size_t index;
        if (latestRawIndex >= previousIndex)
        {
            index = latestRawIndex - previousIndex;
        }
        else
        {
            index = VICON_BUFFER_LENGTH + latestRawIndex - previousIndex;
        }

        return statesRaw[index];
    }

    const FullState &ViconTrackingObject::_GetFilteredData(const size_t previousIndex) const
    {
        if (previousIndex >= VICON_BUFFER_LENGTH)
        {
            throw std::runtime_error("Index out of bounds. Expected index to be less than buffer length.");
        }

        size_t index;
        if (latestFilteredIndex >= previousIndex)
        {
            index = latestFilteredIndex - previousIndex;
        }
        else
        {
            index = VICON_BUFFER_LENGTH + latestFilteredIndex - previousIndex;
        }

        return statesFiltered[index];
    }


}