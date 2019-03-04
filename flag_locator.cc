#include "flag_locator.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FlagLocator)

void FlagLocator::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
	this->model = _parent;
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init();
	this->locationPub = this->node->Advertise<msgs::Pose_V>("~/model/position");
	
	printf("Flagplugin activated\n"); 
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&FlagLocator::OnUpdate, this));
}

void FlagLocator::OnUpdate()
{
}


