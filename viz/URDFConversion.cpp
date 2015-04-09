#include "Conversion.hpp"

namespace robot_model
{

static osg::Vec3 urdf_to_osg(const urdf::Vector3 &in){
    return osg::Vec3(in.x, in.y, in.z);
}

static void urdf_to_osg(urdf::Pose& in, osg::PositionAttitudeTransform& out){
    out.setPosition(urdf_to_osg(in.position));
    //std::cout << in.position.x << ","<< in.position.y<<","<< in.position.z<<std::endl;
    out.setAttitude(osg::Quat(in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));
}

static ref_ptr<Geode> urdf_to_osg_geode(URDFVisualPtr visual, QDir baseDir)
{
    if(!visual){
        return new osg::Geode;
    }
    else if(visual->geometry->type == urdf::Geometry::MESH){
        urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(visual->geometry.get());
        to_visual->setScale(urdf_to_osg(mesh->scale));

        std::string prefix = "file://";
        std::string filename = "";
        if(mesh->filename.compare(0, prefix.length(), prefix) == 0){
            filename = mesh->filename.substr(prefix.length());
        }
        else
            filename = mesh->filename;
        LOG_DEBUG("Trying to load mesh file %s", filename.c_str());

        QString qfilename = QString::fromStdString(filename);
        if (QFileInfo(qfilename).isRelative())
            filename = baseDir.absoluteFilePath(qfilename).toStdString();

        osg::ref_ptr<Node> osg_visual = osgDB::readNodeFile(filename);
        if(!osg_visual){
            LOG_ERROR("OpenSceneGraph did not succees loading the mesh file %s.", filename.c_str());
            throw std::runtime_error("Error loading mesh file.");
        }
        if(!osg_visual){
            LOG_ERROR("Unecpected error loading mesh file %s", mesh->filename.c_str());
            throw(std::runtime_error("Couldn't load mesh file."));
        }
        return osg_visual;
    }
    else if(visual->geometry->type == urdf::Geometry::BOX){
        urdf::Box* box = dynamic_cast<urdf::Box*>(visual->geometry.get());
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(0,0,0), box->dim.x, box->dim.y, box->dim.z));
        osg::ref_ptr<Geode> osg_visual = new osg::Geode;
        osg_visual->addDrawable(drawable);
        return osg_visual;
    }
    else if(visual->geometry->type == urdf::Geometry::CYLINDER){
        urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(visual->geometry.get());
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0,0,0), cylinder->radius, cylinder->length));

        osg::ref_ptr<Geode> osg_visual = new osg::Geode;
        osg_visual->asGeode()->addDrawable(drawable);
        return osg_visual;
    }
    else if(visual->geometry->type == urdf::Geometry::SPHERE){
        urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(visual->geometry.get());
         osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0,0,0), sphere->radius));

        osg::ref_ptr<Geode> osg_visual = new osg::Geode;
        osg_visual->asGeode()->addDrawable(drawable);
        return osg_visual;
    }
    else
        throw std::argument_error("cannot handle URDF visual type " + boost::lexical_cast<string>(visual->geometry->type));
}

static void urdf_initialize_osg_material(URDFMaterialPtr material, osg::StateSet& nodess)
{
    nodess.setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    osg::ref_ptr<osg::Material> nodematerial = new osg::Material;

    std::string filename = material->texture_filename;
    if(filename != ""){
        QString qfilename = QString::fromStdString(material->texture_filename);
        if (QFileInfo(qfilename).isRelative())
            filename = baseDir.absoluteFilePath(qfilename).toStdString();
        osg::ref_ptr<osg::Image> texture_img = osgDB::readImageFile(filename);
        if(!texture_img){
            std::stringstream ss;
            ss << "Could not load texture from file '"<<material->texture_filename<<"'.";
            throw(std::runtime_error(ss.str()));
        }
        osg::ref_ptr<osg::TextureRectangle> texture_rect = osg::ref_ptr<osg::TextureRectangle>(new osg::TextureRectangle(texture_img));
        osg::ref_ptr<osg::TexMat> texmat = new osg::TexMat;
        texmat->setScaleByTextureRectangleSize(true);
        nodess.setTextureAttributeAndModes(0, texture_rect, osg::StateAttribute::ON);
        nodess.setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
    }
    //Specifying the colour of the object
    nodematerial->setDiffuse(osg::Material::FRONT,osg::Vec4(material->color.r,
                                                            material->color.g,
                                                            material->color.b,
                                                            material->color.a));
    nodematerial->setSpecular(osg::Material::FRONT,osg::Vec4(0.2,
                                                             0.2,
                                                             0.2,
                                                             1));

    //Attaching the newly defined state set object to the node state set
    nodess.setAttribute(nodematerial.get());
}

Visual urdf_to_visual(VisualPtr visual, QDir baseDir)
{
    ref_ptr<PositionAttitudeTransform> to_visual = new osg::PositionAttitudeTransform();
    ref_ptr<Geode> osg_visual = urdf_to_osg_visual(visual, baseDir);
    to_visual->addChild(osg_visual);
    
    if (visual)
        urdf_to_osg(visual->origin, *to_visual);
    if (visual && visual->material)
        urdf_to_osg_material(visual->material, *(osg_visual->getOrCreateStateSet()));

    //The smooting visitor calculates surface normals for correct shading
    osgUtil::SmoothingVisitor sv;
    osg_visual->accept(sv);

    Visual result;
    result.to_visual = to_visual;
    result.visual = osg_visual;
    return result;
}
}

