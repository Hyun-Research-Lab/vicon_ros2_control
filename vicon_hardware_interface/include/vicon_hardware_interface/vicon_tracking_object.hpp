#ifndef VICON_TRACKING_OBJECT_HPP_
#define VICON_TRACKING_OBJECT_HPP_

#include <array>
#include <Eigen/Dense> // eigen library
#include <iostream>
#include <set>
#include <map>
#include <tuple>

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
        double omegab_1;
        double omegab_2;
        double omegab_3;
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
    
    template <size_t N>
    using Matrix = Eigen::Matrix<double, N, N>;

    void ProjectHalfStateToFullState(const HalfState &hs, FullState &out_fs);
    void FullStateToPositionVector(const FullState &fs, Vector<3> &out_vec);
    void FullStateToQuaternionVector(const FullState &fs, Vector<4> &out_vec);

    class ViconTrackingObjectPrivate
    {
    public:
        ViconTrackingObjectPrivate();
        ~ViconTrackingObjectPrivate() {}

    private:
        
        bool haveReceivedData = false;
        FullState filteredDataPrevious;

        // butterworth filter
        static constexpr size_t VICON_BUFFER_LENGTH = 5;
        Matrix<VICON_BUFFER_LENGTH> A;
        Vector<VICON_BUFFER_LENGTH> B;
        Eigen::Matrix<double, 1, VICON_BUFFER_LENGTH> C;
        double D;
        Eigen::Matrix<double, VICON_BUFFER_LENGTH, 6> X; // state vector

        Vector<6> _DoButterworthFilterUpdate(Vector<6> &u);

    public:
        void PushData(const HalfState &hs);
        const FullState& GetLatestState() const { return filteredDataPrevious; };
    };

    class ViconTrackingObject
    {
        public:
            ViconTrackingObject() {}
            ~ViconTrackingObject() {}

        private:
            std::map<std::string, ViconTrackingObjectPrivate> viconObjects;
            std::set<std::string> viconObjectNames;

        public:
            bool AddSensor(const std::string &name);
            void PushData(const std::string &name, const HalfState &hs);
            const FullState& GetLatestState(const std::string &name) const;

            bool HasSensorWithName(const std::string &name) const {
                return viconObjectNames.find(name) != viconObjectNames.end();
            }

            const std::set<std::string>& GetSensorNames() const {
                return viconObjectNames;
            }
    };

} // namespace vicon_hardware_interface

#endif // VICON_TRACKING_OBJECT_HPP_
