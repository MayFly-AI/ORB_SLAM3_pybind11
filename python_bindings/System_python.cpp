#include "../pybind11/include/pybind11/pybind11.h"
#include "../pybind11/include/pybind11/stl.h" 
#include "../pybind11/include/pybind11/numpy.h"

#include "cvnp/cvnp.h"

#include <System.h>

namespace py = pybind11;

PYBIND11_MODULE(pyorbslam3, m)
{
    py::class_<Sophus::SE3f> PySophus(m, "Sophus");
    py::class_<ORB_SLAM3::System> PySystem(m, "System");

    PySystem.def(py::init<const string &, const string &, const ORB_SLAM3::System::eSensor,
            const bool, const int, const string &>(),
            py::arg("strVocFile"), py::arg("strSettingsFile"), py::arg("sensor"),
            py::arg("bUseViewer") = true, py::arg("initFr") = 0, py::arg("strSequence") = std::string());

    PySystem.def("GetImageScale", &ORB_SLAM3::System::GetImageScale);
    PySystem.def("TrackMonocular", &ORB_SLAM3::System::TrackMonocular);
    PySystem.def("SaveTrajectoryEuRoC", py::overload_cast<const string&>(&ORB_SLAM3::System::SaveTrajectoryEuRoC));
    PySystem.def("SaveKeyFrameTrajectoryEuRoC", py::overload_cast<const string&>(&ORB_SLAM3::System::SaveKeyFrameTrajectoryEuRoC));
    PySystem.def("ChangeDataset", &ORB_SLAM3::System::ChangeDataset);
    PySystem.def("Shutdown", &ORB_SLAM3::System::Shutdown);

    py::enum_<ORB_SLAM3::System::eSensor>(PySystem, "eSensor")
        .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
        .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
        .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO)
        .value("IMU_RGBD", ORB_SLAM3::System::eSensor::IMU_RGBD)
        .export_values();
}
