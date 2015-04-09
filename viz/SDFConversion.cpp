#include "Conversion.hpp"
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <base/Logging.hpp>

using namespace osg;
using namespace std;

namespace robot_model
{

inline void sdf_pose_to_osg(sdf::ElementPtr pose, osg::Vec3& pos, osg::Quat& quat)
{
    double x, y, z;
    double roll, pitch, yaw;
    sscanf(pose->Get<std::string>().c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &roll, &pitch, &yaw);
    pos.set(x, y, z);
    osg::Quat q = osg::Quat(roll, osg::Vec3d(1, 0, 0), pitch, osg::Vec3d(0, 1, 0), yaw, osg::Vec3d(0, 0, 1));
    quat.set(q.x(), q.y(), q.z(), q.w());
}


inline void sdf_pose_to_osg(sdf::ElementPtr pose, osg::PositionAttitudeTransform& out)
{
    double x, y, z;
    double roll, pitch, yaw;
    sscanf(pose->Get<std::string>().c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &roll, &pitch, &yaw);
    out.setPosition(osg::Vec3(x, y, z));
    out.setAttitude(osg::Quat(roll, osg::Vec3d(1, 0, 0), pitch, osg::Vec3d(0, 1, 0), yaw, osg::Vec3d(0, 0, 1)));
}

inline void sdf_size_to_osg(sdf::ElementPtr size, osg::Vec3& out)
{
    double x, y, z;
    sscanf(size->Get<std::string>().c_str(), "%lf %lf %lf", &x, &y, &z);
    out.set(x, y, z);
}

inline void sdf_scale_to_osg(sdf::ElementPtr scale, osg::Vec3& out)
{
    sdf_size_to_osg(scale, out);
}

inline void sdf_color_to_osg(sdf::ElementPtr color, osg::Vec4& out)
{
    double r, g, b, a;
    sscanf(color->Get<std::string>().c_str(), "%lf %lf %lf %lf", &r, &g, &b, &a);
    out.set(r, g, b, a);
}


ref_ptr<Geode> sdf_to_osg_visual(sdf::ElementPtr sdf_geom_elem, osg::PositionAttitudeTransform& to_visual, QDir baseDir)
{
    ref_ptr<Geode> osg_visual;

    if (sdf_geom_elem->GetName() == "box"){
        Vec3f size;
        sdf_size_to_osg(sdf_geom_elem->GetElement("size"), size);
        ShapeDrawable* drawable = new ShapeDrawable(new Box(Vec3(0,0,0), size.x(), size.y(), size.z()));
        osg_visual = new Geode;
        osg_visual->asGeode()->addDrawable(drawable);
    }
    else if (sdf_geom_elem->GetName() == "cylinder"){
        double radius = sdf_geom_elem->GetElement("radius")->Get<double>();
        double length = sdf_geom_elem->GetElement("length")->Get<double>();
        ShapeDrawable* drawable = new ShapeDrawable(new Cylinder(Vec3d(0,0,0), radius, length));
        osg_visual = new Geode;
        osg_visual->asGeode()->addDrawable(drawable);
    }
    else if (sdf_geom_elem->GetName() == "sphere"){
        double radius = sdf_geom_elem->GetElement("radius")->Get<double>();
        ShapeDrawable* drawable = new ShapeDrawable(new Sphere(Vec3d(0,0,0), radius));
        osg_visual = new Geode;
        osg_visual->asGeode()->addDrawable(drawable);
    }
    else if  (sdf_geom_elem->GetName() == "mesh"){
        Vec3 scale;
        sdf_scale_to_osg(sdf_geom_elem->GetElement("scale"), scale);

        to_visual.setScale(scale);

        std::string uri = sdf_geom_elem->GetElement("uri")->Get<std::string>();

        std::string model_prefix = "model://";

        std::string filename = "";

        if(uri.compare(0, model_prefix.length(), model_prefix) == 0){
            filename = uri.substr(model_prefix.length());
        }
        else {
            filename = uri;
        }

        QString qfilename = QString::fromStdString(filename);
        if (QFileInfo(qfilename).isRelative()){
            filename = baseDir.absoluteFilePath(QFileInfo(qfilename).fileName()).toStdString();
//                filename = baseDir.absoluteFilePath(QFileInfo(qfilename).baseName()).toStdString();
//                filename += ".osg";
        }

        osg_visual = osgDB::readNodeFile(filename);

        if (!osg_visual) {
            LOG_ERROR("OpenSceneGraph did not succees loading the mesh file %s.", filename.c_str());
            throw std::runtime_error("Error loading mesh file.");
        }

    }
    else {
        osg_visual = new Geode;
    }
    return osg_visual;
}

static void sdf_initialize_osg_material(sdf::ElementPtr sdf_material, osg::StateSet& nodess){
    ref_ptr<Material> nodematerial = new Material;

    nodematerial->setSpecular(Material::FRONT,Vec4(0.2,
                                                             0.2,
                                                             0.2,
                                                             1));

    if (sdf_material->HasElement("ambient")){
        Vec4 ambient;
        sdf_color_to_osg(sdf_material->GetElement("ambient"), ambient);
        nodematerial->setAmbient(Material::FRONT,ambient);
    }

    if (sdf_material->HasElement("diffuse")){
        Vec4 diffuse;
        sdf_color_to_osg(sdf_material->GetElement("diffuse"), diffuse);
        nodematerial->setDiffuse(Material::FRONT,diffuse);
    }

    if (sdf_material->HasElement("specular")){
        Vec4 specular;
        sdf_color_to_osg(sdf_material->GetElement("specular"), specular);
        nodematerial->setSpecular(Material::FRONT, specular);

    }

    if (sdf_material->HasElement("emissive")){
        Vec4 emissive;
        sdf_color_to_osg(sdf_material->GetElement("emissive"), emissive);
        nodematerial->setEmission(Material::FRONT, emissive);
    }

    ref_ptr<StateSet> nodess = osg_visual->getOrCreateStateSet();
    nodess->setMode(GL_NORMALIZE, StateAttribute::ON);
    nodess->setAttribute(nodematerial.get());
}

Visual sdf_to_visual(sdf::ElementPtr sdf_visual, QDir baseDir){
    ref_ptr<PositionAttitudeTransform> to_visual = new PositionAttitudeTransform();
    sdf_pose_to_osg(sdf_visual->GetElement("pose"), *to_visual);

    ref_ptr<Geode> osg_visual = new Geode;
    if (sdf_visual->HasElement("geometry")){
        sdf::ElementPtr sdf_geometry  = sdf_visual->GetElement("geometry");
        sdf::ElementPtr sdf_geom_elem = sdf_geometry->GetFirstElement();
        osg_visual = sdf_to_osg_visual(sdf_geom_elem, *to_visual, baseDir);
    }

    if (sdf_visual->HasElement("material")){
        sdf_initialize_osg_material(sdf_visual->GetElement("material"), *(osg_visual->getOrCreateStateSet()));
    }

    //The smooting visitor calculates surface normals for correct shading
    osgUtil::SmoothingVisitor sv;
    osg_visual->accept(sv);

    to_visual->addChild(osg_visual);
    
    Visual result = { to_visual, osg_visual };
}

