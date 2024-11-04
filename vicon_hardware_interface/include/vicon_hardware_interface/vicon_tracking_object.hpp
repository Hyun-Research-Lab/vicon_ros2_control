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
        // position p = (px,py,pz)
        double px;
        double py;
        double pz;

        // velocity v = (vx,vy,vz)
        double vx;
        double vy;
        double vz;

        // quaternion q= w + xi + yj + zk
        double qx;
        double qy;
        double qz;
        double qw;

        // angular velocity in body frame
        // wb = (wb1, wb2, wb3)
        double wb1;
        double wb2;
        double wb3;
    };

    struct HalfState {
        double time;
        double px;
        double py;
        double pz;
        double qx;
        double qy;
        double qz;
        double qw;
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
        
        bool haveNeverReceivedValidData = true;
        FullState fsPrev;

        bool lastDataWasInvalid = false;

        // butterworth filter
        static constexpr size_t BUTTER_STATES = 10;
        // static constexpr size_t BUTTER_STATES = 5;
        Matrix<BUTTER_STATES> A_butter;
        Vector<BUTTER_STATES> B_butter;
        Eigen::Matrix<double, 1, BUTTER_STATES> C_butter;
        double D_butter;
        Eigen::Matrix<double, BUTTER_STATES, 3> X_butter; // state vector

        Vector<3> _DoButterworthFilterUpdate(Vector<3> &u);

        // Savitzky-Golay filter
        static constexpr size_t SGOLAY_WINDOW_SIZE = 20;
        // static constexpr size_t SGOLAY_POLYNOMIAL_ORDER = 2; // currently 2 is implemented
        static constexpr double SGOLAY_DT = 1.0 / 200.0;
        Eigen::Matrix<double, 9, 3*(SGOLAY_WINDOW_SIZE+1)> A_sgolay;
        Eigen::Vector<double, 3*(SGOLAY_WINDOW_SIZE+1)> b_sgolay;
        std::array<Eigen::Matrix3d, SGOLAY_WINDOW_SIZE> R_history;

        // create circular buffer
        bool hasFullRBuffer = false;
        size_t sgolay_index = 0;
        void _PushCircularBuffer(const Eigen::Matrix3d &R) {
            R_history[sgolay_index] = R;
            sgolay_index = (sgolay_index + 1) % SGOLAY_WINDOW_SIZE;
            
            // if we have filled the buffer, then the index will wrap around to zero
            if (sgolay_index == 0) {
                hasFullRBuffer = true;
            }
        }
        // index = 0 is the oldest, index = 1 is second oldest, etc.
        const Eigen::Matrix3d & _GetR(const size_t index) const {
            return R_history[(sgolay_index + index) % SGOLAY_WINDOW_SIZE];
        }
        
        // filter methods
        Eigen::Matrix3d _dexpSO3(const Eigen::Vector3d &a) const;
        std::tuple<Eigen::Matrix3d, Eigen::Vector3d> _filterSO3(const Eigen::Matrix3d &R);

    public:
        bool PushData(const HalfState &hs);
        const FullState& GetLatestState() const { return fsPrev; };
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
            bool PushData(const std::string &name, const HalfState &hs);
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
