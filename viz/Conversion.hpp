#ifndef ROBOT_MODEL_CONVERSION_HPP
#define ROBOT_MODEL_CONVERSION_HPP

#include <vizkit3d/RobotModel.h>

#include <boost/shared_ptr.hpp>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <urdf_model/link.h>
#include <QtCore/QDir>
#include <sdf/sdf.hh>

namespace robot_model
{
    typedef boost::shared_ptr<urdf::Visual> URDFVisualPtr;
    Visual urdf_to_visual(URDFVisualPtr visual, QDir baseDir);
    Visual sdf_to_visual(sdf::ElementPtr visual, QDir baseDir);
};

#endif

