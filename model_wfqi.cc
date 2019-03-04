#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/util/util.hh>
#include <chrono>
#include <ignition/math/Vector3.hh>
#include "model_wfqi.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ModelWFQI)

ModelWFQI::ModelWFQI():
		learner{rl::FidoControlSystem(3, {-1, -1}, {1, 1}, 6)},
		flagLocation{ignition::math::Vector3d(0, 0, 0)},
 		lastDistance{4},
		minFlagDistance{4}		
{
}
		
ModelWFQI::~ModelWFQI() 
{
	
}

void ModelWFQI::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
	// Store the pointer in the model
	this->model = _parent;

	// Store the pointer to the main link
	this->mainLink = this->model->GetLinks()[0];

	// Listen to the world update event
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelWFQI::OnUpdate, this));

	//  Initialize transport
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init();
	 
	this->keyboardSub = this->node->Subscribe("~/keyboard/keypress", &ModelWFQI::OnKeyPress, this, true);
	this->flagSub = this->node->Subscribe("~/boat/position", &ModelWFQI::OnBoatPosition, this, true);
	printf("Boatplugin activated\n");

	rl::Action action = this->learner.chooseBoltzmanActionDynamic(this->GetState());

	// Apply action to actuators
	std::cout << "Propeller: "  << action[0] << std::endl;
	this->SetPropeller(action[0]);
	std::cout << "Bow Thruster: "  << action[1] << std::endl;
	this->SetBowThruster(action[1]);
}

void ModelWFQI::OnUpdate()
{
	if(this->mainLink->WorldForce().Length() < 0.005)
	{
		// Apply resulting state change to learning system
		clock_t begin = clock();
		this->learner.applyReinforcementToLastAction(this->CalcReward(), this->GetState());
		this->learningTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);
	
		// Choose Action and take time
		begin = clock();
		rl::Action action = this->learner.chooseBoltzmanActionDynamic(this->GetState());
		this->choosingTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);

		// Apply action to actuators
		std::cout << "Propeller: "  << action[0] << std::endl;
		this->SetPropeller(action[0]);
		std::cout << "Bow Thruster: "  << action[1] << std::endl;
		this->SetBowThruster(action[1]);

		this->PrintStatistics();
	}
}

rl::State ModelWFQI::GetState()
{
	double x, y, yaw;
	x = this->model->WorldPose().Pos().X();
	y = this->model->WorldPose().Pos().Y();
	yaw = this->model->WorldPose().Rot().Yaw();

	//Normalise state
	double sum = abs(x) + abs(y) + abs(yaw);
	x /= sum;
	y /= sum;
	yaw /= sum;

	return {x, y, yaw};
}

double ModelWFQI::CalcReward()
{
	double distance = this->FlagDistance(); 
	double difference = this->lastDistance - distance;	
	this->lastDistance = distance;

	double reward = difference / 10; // Maximum Distance

	if(distance < this->minFlagDistance) {
		this->minFlagDistance = distance;
	}

	return reward;
}

void ModelWFQI::SetBowThruster(double strength)
{
	this->mainLink->SetTorque(ignition::math::Vector3d(0, 0, strength * BOW_THRUSTER_FACTOR));
}
		
void ModelWFQI::SetPropeller(double strength)
{
	this->mainLink->AddRelativeForce(ignition::math::Vector3d(0, strength * PROPELLER_FACTOR, 0));
}

double ModelWFQI::FlagDistance()
{
	return this->model->WorldPose().Pos().Distance(this->flagLocation);
}

void ModelWFQI::PrintStatistics()
{
	double choosingTimeSum = 0;
	for(auto choosingTime : this->choosingTimes) {
		choosingTimeSum += choosingTime;
	}
	double choosingTimesAvg = choosingTimeSum / this->choosingTimes.size();
	std::cout << "Average action choosing time: " << choosingTimesAvg << " Sec" << std::endl;

	double learningTimeSum  = 0;
	for(auto learningTime : this->learningTimes) {
		learningTimeSum += learningTime;
	}
	double learningTimesAvg = learningTimeSum / this->learningTimes.size();
	std::cout << "Average learning time: " << learningTimesAvg << " Sec" << std::endl;

	std::cout << "Min Flag distance: " << this->minFlagDistance << " m" << std::endl;
	std::cout << "Current Flag Distance: " << this->FlagDistance() << " m " << std::endl;
}

void ModelWFQI::OnKeyPress(ConstAnyPtr &msg)
{
	// w key -> propeller forward
	if(msg->int_value() == 119) {
		SetPropeller(PROPELLER_FACTOR);
	}	

	// s key -> propeller backward
	if(msg->int_value() == 115) {
		SetPropeller(-PROPELLER_FACTOR);
	}	

	// a key -> exhaust thruster right
	if(msg->int_value() == 97) {
		SetBowThruster(BOW_THRUSTER_FACTOR);
	}	

	// d key -> exhaust thurster left
	if(msg->int_value() == 100) {
		SetBowThruster(-BOW_THRUSTER_FACTOR);
	}	
}

void ModelWFQI::OnBoatPosition(ConstPosePtr &msg)
{	
	//this->model->SetRelativePose(*msg);
}

