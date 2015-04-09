#include "RobotModel.h"
#include "Conversion.hpp"

#include <urdf_parser/urdf_parser.h>
#include "fstream"
#include "sstream"
#include "osg/Texture2D"
#include "osg/BlendFunc"
#include "osg/AlphaFunc"
#include "osg/Billboard"
#include "osg/PointSprite"
#include "osg/Point"
#include "osg/Geometry"
#include "osg/Image"
#include "osg/Material"
#include "osg/ShapeDrawable"
#include "osg/TextureRectangle"
#include "osg/TexMat"
#include "OSGHelpers.hpp"
#include <base/Logging.hpp>
#include <QFileInfo>
#include <osg/ShadeModel>
#include <osgUtil/SmoothingVisitor>

#include <fstream>
#include <streambuf>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/PluginQuery>

#define CALL_MEMBER_POINTER_FUNCTION(obj, fnc) ((obj).*(fnc))

using namespace std;
using namespace osg;
using namespace robot_model;

OSGSegment::OSGSegment(KDL::Segment seg)
{
    isSelected_=false;
    seg_ = seg;
    jointPos_ = 0;
    label_ = 0;
    visual_ = 0;
    post_transform_ = new Group();
    toTipOsg_ = new PositionAttitudeTransform();
    toTipOsg_->addChild(post_transform_);
    toTipOsg_->setName(seg_.getJoint().getName());
    post_transform_->setUserData( this );
    post_transform_->setUpdateCallback(new OSGSegmentCallback);

    toTipOsg_->setUserData(this);

    setupTextLabel();
    updateJoint();
}

ref_ptr<Group> OSGSegment::getToTipOsg() const{
    return toTipOsg_;
}

ref_ptr<Group> OSGSegment::getGroup() const{
    return post_transform_;
}

void OSGSegment::updateJoint(){
    toTipKdl_ = seg_.pose(jointPos_);
    kdl_to_osg(toTipKdl_, *toTipOsg_);
}

void OSGSegment::attachVisuals(std::vector<Visual> const &visual_array, QDir prefix)
{
    vector<Visual>::const_iterator itr = visual_array.begin();
    vector<Visual>::const_iterator itr_end = visual_array.end();

    for(itr = visual_array.begin(); itr != itr_end; ++itr)
        attachVisual(*itr, prefix);
}

void OSGSegment::attachVisual(Visual const& visual, QDir prefix)
{
    post_transform_->addChild(visual.to_visual);
    visual_ = visual.visual;
    visual_->setUserData(this);
    visual_->setName(seg_.getName());
}

void OSGSegment::removeLabel(){
    if(label_)
        post_transform_->removeChild(label_);
    label_ = 0;
}

void OSGSegment::attachLabel(std::string name, std::string filepath){
    if(label_)
        removeLabel();

    ref_ptr<Geode> geode = ref_ptr<Geode>(new Geode());
    ref_ptr<Geometry> geometry = ref_ptr<Geometry>(new Geometry());

    ref_ptr<Vec3Array> vertices = ref_ptr<Vec3Array>(new Vec3Array);
    vertices->push_back (Vec3 (0, 0, 0.0));

    geometry->setVertexArray (vertices);

    geometry->addPrimitiveSet(new DrawArrays(PrimitiveSet::POINTS,0,vertices->size()));

    geode->addDrawable(geometry);
    ref_ptr<StateSet> set = geode->getOrCreateStateSet();

    /// Setup the point sprites
    ref_ptr<PointSprite> sprite = ref_ptr<PointSprite>(new PointSprite());
    set->setTextureAttributeAndModes(0, sprite, StateAttribute::ON);

    /// Give some size to the points to be able to see the sprite
    ref_ptr<Point> point = ref_ptr<Point>(new Point());
    point->setSize(50);
    set->setAttribute(point);

    /// Disable depth test to avoid sort problems and Lighting
    set->setMode(GL_DEPTH_TEST, StateAttribute::OFF);
    set->setMode(GL_LIGHTING, StateAttribute::OFF);

    ref_ptr<BlendFunc> texture_blending_function = new BlendFunc();
    set->setAttributeAndModes(texture_blending_function.get(), StateAttribute::ON);

    ref_ptr<AlphaFunc> alpha_transparency_function = new AlphaFunc();
    alpha_transparency_function->setFunction(AlphaFunc::GEQUAL, 0.05);
    set->setAttributeAndModes(alpha_transparency_function.get(), StateAttribute::ON );

    /// The texture for the sprites
    ref_ptr<Texture2D> tex = ref_ptr<Texture2D>(new Texture2D());
    ref_ptr<Image> image = osgDB::readImageFile(filepath);
    image->flipVertical();
    tex->setImage(image);

    set->setTextureAttributeAndModes(0, tex, StateAttribute::ON);

    post_transform_->addChild(geode);

    geode->setName(name);
    geode->setUserData(this);

    label_ = geode;
}

void OSGSegment::setupTextLabel(){
    text_label_ = ref_ptr<osgText::Text>(new osgText::Text());
    text_label_geode_ = ref_ptr<Geode>(new Geode());
    ref_ptr<StateSet> set = text_label_geode_->getOrCreateStateSet();
    /// Disable depth test and Lighting
    set->setMode(GL_DEPTH_TEST, StateAttribute::OFF);
    set->setMode(GL_LIGHTING, StateAttribute::OFF);
    text_label_geode_->addDrawable(text_label_);

    //Text should be rather small and be always readable on the screen
    text_label_->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
    text_label_->setCharacterSize(20);

    osgText::Font* font = osgText::Font::getDefaultFont();
    font->setMinFilterHint(Texture::NEAREST); // aliasing when zoom out, this doesnt look so ugly because text is small
    font->setMagFilterHint(Texture::NEAREST); // aliasing when zoom in
    text_label_->setFont(font);

    text_label_->setAxisAlignment(osgText::Text::SCREEN);

    // Set the text to render with alignment anchor and bounding box around it:
    text_label_->setDrawMode(osgText::Text::TEXT |
                           osgText::Text::ALIGNMENT);
    text_label_->setAlignment(osgText::Text::CENTER_TOP);
    text_label_->setPosition( Vec3(0,0,0) );
    text_label_->setColor( Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

    text_label_->setBackdropType(osgText::Text::OUTLINE);
    text_label_->setBackdropColor(Vec4(0, 0, 0, 1.0f));
}

void OSGSegment::attachTextLabel(std::string text){
    if(text == ""){
        text = seg_.getName();
    }
    text_label_->setText(text);
    post_transform_->addChild(text_label_geode_);
}

void OSGSegment::removeTextLabel()
{
    post_transform_->removeChild(text_label_geode_);
}

bool OSGSegment::toggleSelected(){
    isSelected_ = !isSelected_;

    if(!visual_){
        std::clog << "Tried to highlight " << seg_.getName() << ", but it has no visual." << std::endl;
        return false;
    }
    ref_ptr<Group> parent = visual_->getParent(0);

    if(isSelected_){
        ref_ptr<osgFX::Outline> scribe = ref_ptr<osgFX::Outline>(new osgFX::Outline());
        scribe->setWidth(1);
        scribe->setColor(Vec4(1,0,0,1));
        scribe->addChild(visual_);
        parent->replaceChild(visual_, scribe);
    }
    else{
        //node already picked so we want to remove marker to unpick it.
        Node::ParentList parentList = parent->getParents();
        for(Node::ParentList::iterator itr=parentList.begin();
            itr!=parentList.end();
            ++itr)
        {
            (*itr)->replaceChild(parent, visual_);
        }
    }

    //update_visual();
    return isSelected_;
}


RobotModel::RobotModel(){
    //Root is the entry point to the scene graph
    root_ = ref_ptr<Group>(new Group());
    original_root_ = ref_ptr<Group>(new Group());
    loadFunctions["urdf"] = &RobotModel::loadURDF;
    loadFunctions["sdf"] = &RobotModel::loadSDF;
    loadEmptyScene();
}

ref_ptr<Node> RobotModel::loadEmptyScene(){
    original_root_->removeChildren(0, original_root_->getNumChildren());
    root_->removeChildren(0, root_->getNumChildren());
    jointNames_.clear();
    segmentNames_.clear();
    return root_;
}

ref_ptr<OSGSegment> RobotModel::makeOsgSegment(KDL::Segment const& kdl_seg, std::vector<Visual> const& visuals){
    ref_ptr<OSGSegment> seg = ref_ptr<OSGSegment>(new OSGSegment(kdl_seg));

    //Attach one visual to joint
    if (visuals.empty())
    {
        seg->attachVisual(Visual(), rootPrefix);
    }
    //Attach several visuals to joint
    else
    {
        seg->attachVisuals(visuals, rootPrefix);
    }

    return seg;
}

ref_ptr<Node> RobotModel::makeOsg( boost::shared_ptr<urdf::ModelInterface> urdf_model ){
    // Convert to KDL
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(*urdf_model, tree);
    
    // Store link-to-visual mappings. That's the only thing we can't store in
    // KDL
    map<string, Visual> visuals;
    typedef boost::shared_ptr<urdf::Link> URDFLinkPtr;
    vector<URDFLinkPtr> links;
    urdf_model->getLinks(links);
    for (vector<URDFLinkPtr>::const_iterator it = links.begin(); it != links.end(); ++it)
        visuals[(*it)->name] = urdf_to_visual((*it)->visual, rootPrefix);

    return makeOsg(tree, visuals);
}

ref_ptr<Node> RobotModel::makeOsg(KDL::Tree const& tree, map<string, Visual>& visuals)
{
    KDL::SegmentMap const& segment =
        tree.getSegments();

    map< string, ref_ptr<OSGSegment> > segments;

    // First pass, convert all segments to OSG
    for (KDL::SegmentMap::const_iterator it = segment.begin(); it != segment.end(); ++it)
    {
        KDL::Segment const& kdl_segment = it->second.segment;
        segments[it->first] = makeOsgSegment(kdl_segment, visuals[it->first]);

        segmentNames_.push_back(kdl_segment.getName());
        if(kdl_segment.getJoint().getType() != KDL::Joint::None)
            jointNames_.push_back(kdl_segment.getJoint().getName());
    }

    // Second pass, link segments
    for (KDL::SegmentMap::const_iterator parent_it = segment.begin(); parent_it != segment.end(); ++parent_it)
    {
        vector<KDL::SegmentMap::const_iterator> const& children =
            it->second.children;
        ref_ptr<OSGSegment> parent = segments[parent_it->first];
        for (vector<KDL::SegmentMap::const_iterator>::const_iterator child_it = children.begin();
                child_it != children.end(); ++it)
        {
            ref_ptr<OSGSegment> child = segments[child_it->first];
            parent->getGroup()->addChild(child->getToTipOsg());
        }
    }

    relocateRoot(tree.getRoot()->first);
    return root_;
}

Node* RobotModel::makeOsg( sdf::ElementPtr sdf_model )
{
    KDL::Tree tree;
    kdl_parser::treeFromSdfModel(sdf_model, tree);

    // Store link-to-visual mappings. That's the only thing we can't store in
    // KDL
    map<string, Visual> visuals;
    typedef boost::shared_ptr<urdf::Link> URDFLinkPtr;
    vector<URDFLinkPtr> links;
    urdf_model->getLinks(links);
    for (vector<URDFLinkPtr>::const_iterator it = links.begin(); it != links.end(); ++it)
        visuals.push_back(urdf_to_visual(it->visual, baseDir));

    root_->addChild(original_root_);
    return root_;
}

ref_ptr<Node> RobotModel::load(QString path){

    loadPlugins();
    loadEmptyScene();

    QString suffix = QFileInfo(path).suffix().toLower();

    /*
     * call the function based in the file suffix
     */
    if (loadFunctions.contains(suffix)){
        Node *node= CALL_MEMBER_POINTER_FUNCTION(*this, loadFunctions[suffix])(path);
        return node;
    }
    else {
        LOG_ERROR("the %s type of file is not supported .", suffix.toStdString().c_str());
    }

    return new Group();
}

Node* RobotModel::loadURDF(QString path)
{
    std::ifstream t( path.toStdString().c_str() );
    std::string xml_str((std::istreambuf_iterator<char>(t)),
                       std::istreambuf_iterator<char>());

    rootPrefix = QDir(QFileInfo(path).absoluteDir());

    //Parse urdf
    boost::shared_ptr<urdf::ModelInterface> model = urdf::parseURDF( xml_str );
    if (!model)
        return NULL;

    return makeOsg(model);
}

Node* RobotModel::loadSDF(QString path)
{
    rootPrefix = QDir(QFileInfo(path).absoluteDir());

    sdf::SDFPtr sdf(new sdf::SDF);

    if (!sdf::init(sdf)){
        LOG_ERROR("unable to initialize sdf.");
        return NULL;
    }

    if (!sdf::readFile(path.toStdString(), sdf)){
        LOG_ERROR("unabled to read sdf file %s.", path.toStdString().c_str());
        return NULL;
    }

    if (!sdf->root->HasElement("model")){
        LOG_ERROR("the <model> tag not exists");
        return NULL;
    }

    return makeOsg(sdf->root->GetElement("model"));
}

void RobotModel::loadPlugins()
{
    osgDB::FileNameList plugins = osgDB::listAllAvailablePlugins();
    for(osgDB::FileNameList::iterator itr = plugins.begin();
        itr != plugins.end();
        ++itr)
    {
        osgDB::ReaderWriterInfoList infoList;
        osgDB::queryPlugin(*itr, infoList);
    }
}

SdfElementPtrMap RobotModel::loadSdfModelLinks(sdf::ElementPtr sdf_model)
{
    SdfElementPtrMap links;

    sdf::ElementPtr linkElem = sdf_model->GetElement("link");
    while (linkElem){
        std::string link_name = linkElem->Get<std::string>("name");
        links.insert(std::make_pair<std::string, sdf::ElementPtr>(link_name, linkElem));
        linkElem = linkElem->GetNextElement("link");
    }

    return links;
}


ref_ptr<OSGSegment> RobotModel::getSegment(std::string name)
{
    ref_ptr<Node> node = findNamedNode(name, original_root_);
    if(!node){
        std::cerr << "Could not find segment with name: " << name << std::endl;
        return 0;
    }

    ref_ptr<OSGSegment> jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    if(!jnt){
        std::cerr << "Could not retrieve user data from node "<<name<<std::endl;
    }
    assert(jnt);
    return jnt;
}

bool RobotModel::relocateRoot(std::string name){
    ref_ptr<OSGSegment> seg = getSegment(name);
    if(seg){
        root_->removeChildren(0, root_->getNumChildren());
        root_->addChild(seg->post_transform_);
    }
    else{
        std::cerr << "Segment " << name << " is unknown! Could not relocate root!" << std::endl;
        std::cerr << "Known segments:\n";
        std::vector<std::string> names = getSegmentNames();
        for(uint i=0; i<names.size(); i++)
            std::cerr << "  - "<<names[i]<<"\n";
        std::cerr<<std::endl;
        return false;
    }
    current_root_name_ = name;

    return true;
}

bool RobotModel::setJointState(std::string jointName, double jointVal)
{
    ref_ptr<Node> node = findNamedNode(jointName, original_root_);
    if(!node)
        return false;

    ref_ptr<OSGSegment> jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    jnt->setJointPos(jointVal);
    return true;
}

bool RobotModel::setJointState(const std::map<std::string, double>& jointVals)
{
    for (std::map<std::string, double>::const_iterator it=jointVals.begin();
         it!=jointVals.end(); ++it){
        ref_ptr<Node> node = findNamedNode( it->first, original_root_);
        if(!node)
            return false;

        ref_ptr<OSGSegment> jnt = dynamic_cast<OSGSegment*>(node->getUserData());
        jnt->setJointPos(it->second);
    }
    return true;
}

bool RobotModel::toggleHighlight(std::string name)
{
    ref_ptr<OSGSegment> seg = getSegment(name);
    assert(seg);

    seg->toggleSelected();
    return seg->isSelected_;
}

Matrixd RobotModel::getRelativeTransform(std::string source_segment, std::string target_segment)
{
    return getTransformBetweenNodes(getSegment(source_segment)->post_transform_, getSegment(target_segment)->post_transform_);
}
