#ifndef VICON_TRACKING_OBJECT_HPP_
#define VICON_TRACKING_OBJECT_HPP_

#include <array>
#include <Eigen/Dense> // eigen library

namespace vicon_hardware_interface
{
    struct FullState {
        double time;
        double position_x;
        double position_y;
        double position_z;
        double velocity_x;
        double velocity_y;
        double velocity_z;
        double orientation_qx;
        double orientation_qy;
        double orientation_qz;
        double orientation_qw;
        double angular_velocity_qx_dot;
        double angular_velocity_qy_dot;
        double angular_velocity_qz_dot;
        double angular_velocity_qw_dot;
    };

    struct HalfState {
        double time;
        double position_x;
        double position_y;
        double position_z;
        double orientation_qx;
        double orientation_qy;
        double orientation_qz;
        double orientation_qw;
    };

    template <size_t N> 
    using Vector = Eigen::Matrix<double, N, 1>;

    void ProjectHalfStateToFullState(const HalfState &hs, FullState &out_fs);
    void FullStateToPositionVector(const FullState &fs, Vector<3> &out_vec);
    void FullStateToQuaternionVector(const FullState &fs, Vector<4> &out_vec);

    class ViconTrackingObject
    {
    public:
        ViconTrackingObject() {}
        ~ViconTrackingObject() {}

    private:
        static constexpr size_t VICON_BUFFER_LENGTH = 5;

        size_t latestRawIndex;
        size_t latestFilteredIndex;
        bool buffersInitialized = false;
        std::array<FullState, VICON_BUFFER_LENGTH> statesRaw;
        std::array<FullState, VICON_BUFFER_LENGTH> statesFiltered;
        FullState outputState;

        void _PushRaw(const FullState &fs);
        void _PushFiltered(const FullState &fs);

        const FullState& _GetRawData(const size_t previousIndex) const;
        const FullState& _GetFilteredData(const size_t previousIndex) const;

    public:
        const FullState & GetOutputState() const { return outputState; }
        void PushData(const HalfState &hs);

    };
} // namespace vicon_hardware_interface

#endif // VICON_TRACKING_OBJECT_HPP_
