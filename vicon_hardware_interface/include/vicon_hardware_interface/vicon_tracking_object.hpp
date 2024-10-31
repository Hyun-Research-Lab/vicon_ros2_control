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

        // temp for logging
        

        double qContinuous_w;
        double qContinuous_x;
        double qContinuous_y;
        double qContinuous_z;

        double theta;
        double eigen1;
        double eigen2;
        double eigen3;

        double qaxis1;
        double qaxis2;
        double qaxis3;

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
        HalfState hsPrev;
        double thetaPrev;

        // butterworth filter
        // static constexpr size_t BUTTERWORTH_FILTER_STATES = 10;
        static constexpr size_t BUTTERWORTH_FILTER_STATES = 5;
        Matrix<BUTTERWORTH_FILTER_STATES> A;
        Vector<BUTTERWORTH_FILTER_STATES> B;
        Eigen::Matrix<double, 1, BUTTERWORTH_FILTER_STATES> C;
        double D;
        Eigen::Matrix<double, BUTTERWORTH_FILTER_STATES, 6> X; // state vector

        Vector<6> _DoButterworthFilterUpdate(Vector<6> &u);

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
