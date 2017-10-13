from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp cimport bool

cdef extern from "types.h" namespace "imua":
    cdef cppclass Accelerometer:
        float  samplingRate
        int    size
        float* t
        float* x
        float* y
        float* z

    cdef cppclass Gyroscope:
        float  samplingRate
        int    size
        float* t
        float* x
        float* y
        float* z

    cdef cppclass IMU:
        Gyroscope     gyro
        Accelerometer accl

    cdef cppclass Detection:
        float start
        float end
        float value
        string description

    cdef cppclass Euler:
        float  sampling_rate
        int    num_samples
        float* t
        float* roll
        float* pitch
        float* yaw

        float  roll_mean
        float  pitch_mean
        float  yaw_mean

        int    do_init

ctypedef vector[Detection]& Dets

cdef extern from "generic.h" namespace "imua::generic":
    cdef bool detectJumps(const IMU& imu,
        Dets& dets,
        const float, const float)
    cdef bool detectShakiness(const IMU& imu,
        Dets& dets,
        float threshold_low,
        float threshold_high)

    cdef bool detectPans(const IMU& imu, Dets& dets)

    cdef void detectFlips(const IMU& imu,
        const Euler& euler,
        Dets& dets)
    cdef void detectSpins(const IMU& imu,
        const Euler& euler,
        Dets& dets,
        int secant_length,
        float threshold_spin_degrees,
        int threshold_samples)
    cdef void detectCorners(const IMU& imu,
        const Euler& euler,
        Dets& dets,
        int secant_length,
        float threshold_spin_degrees,
        int threshold_samples)

cdef extern from "gpmf/gpmf_parser.h":
    ctypedef struct IMU_t:
        pass

    void DeAllocateImu(IMU_t* imu)

cdef extern from "example/example.cpp":
    cdef bool readImuData(string& path, IMU_t& imu)
    cdef void copyImuData(IMU_t& inp, IMU& out)
