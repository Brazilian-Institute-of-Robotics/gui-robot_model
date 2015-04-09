rock_vizkit_plugin(robot_model-viz
    RobotVisualization.cpp RobotModel.cpp OSGHelpers.hpp
    SDFConversion.cpp URDFConversion.cpp
    DEPS_PKGCONFIG kdl_parser base-types base-viz sdformat
    MOC RobotVisualization.hpp
    HEADERS RobotVisualization.hpp RobotModel.h)

