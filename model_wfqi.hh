#define BOW_THRUSTER_FACTOR 1000
#define PROPELLER_FACTOR 5000

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <Fido/Fido.h>
#include <ignition/transport/Node.hh>
#include <vector>

namespace gazebo
{
	class ModelWFQI : public ModelPlugin
	{
		public: ModelWFQI();

		public: ~ModelWFQI();

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr);

		public: void OnUpdate();

		private: double CalcReward();

		private: void SetBowThruster(double strength);
		
		private: void SetPropeller(double strength);

		private: void OnKeyPress(ConstAnyPtr &msg);

		private: void OnBoatPosition(ConstPosePtr &msg);

		private: void PrintStatistics();

		private: double FlagDistance();

		private: rl::State GetState();
		
		private: rl::FidoControlSystem learner;

		// |brief Pointer to controlling model		
		private: physics::ModelPtr model;
		
		// |brief Pointer to main link
		private: physics::LinkPtr mainLink;

		// |brief Flag location
		private: ignition::math::Vector3d flagLocation;

		// |brief Last distance from Flag
		private: double lastDistance;

		private: std::vector<double> choosingTimes;

		private: std::vector<double> learningTimes;

		private: double minFlagDistance;

		// |brief Pointer to propeller link
		private: physics::LinkPtr propeller;

		// |brief Pointer to bow thruster link
		private: physics::LinkPtr bowThruster;

		// |brief Pointer to connection
		private: event::ConnectionPtr updateConnection;

		// |brief Subscriber for keyboard topic
		private: transport::SubscriberPtr keyboardSub;

		// |brief Subscriber for flag locationSub
		private: transport::SubscriberPtr flagSub;

		// |brief node for communication
		private: transport::NodePtr node;
	};
}
