from libcpp.string cimport string
from libcpp cimport bool
from libcpp.vector cimport vector

class Event(object):
    def __init__(self, float start, float end, float value, str type):
        self.start = start
        self.end = end
        self.value = value

        self.type = type

    def __repr__(self):
        return '{type}({start:.2f} -> {end:.2f}, {value:.0f})'.format(
            type=self.type,
            start=self.start,
            end=self.end,
            value=self.value
        )

cdef void _add_dets(vector[Detection]& dets, list py_dets, str type):
    for det in dets:
        py_dets.append(Event(det.start, det.end, det.value, type=type))

def run_detection(str path):
    cdef IMU_t imu
    cdef string path_cpp = path
    result = readImuData(path_cpp, imu)

    if not result:
        return None

    cdef IMU imu2
    copyImuData(imu, imu2)

    cdef list py_dets = []
    cdef vector[Detection] dets

    detectJumps(imu2, dets, 4.0, 0.25)
    _add_dets(dets, py_dets, 'jump')

    dets.clear()
    detectShakiness(imu2, dets, 0.5, 1.)
    _add_dets(dets, py_dets, 'shaky')

    dets.clear()
    detectPans(imu2, dets)
    _add_dets(dets, py_dets, 'pan')

    DeAllocateImu(&imu)

    return py_dets
