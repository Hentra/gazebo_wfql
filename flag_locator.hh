#include <gazebo/gazebo.hh>
#include <functional>
#include <gazebo/transport/Node.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
	// \brief 
	class FlagLocator : public ModelPlugin 
	{
		// |brief |see ModelPlugin
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr);

		// |brief Called every simulation step
		private: void OnUpdate();
	
		// |brief Pointer to controlling model		
		private: physics::ModelPtr model;

		// |brief Last location
		private: ignition::math::Vector3d lastLocation; 

		// |brief Pointer to a node for communication.
		private: transport::NodePtr node;

		// |brief Pointer to connection
		private: event::ConnectionPtr updateConnection;

		// |brief Pointer to a publisher which publishes flag location 
		private: transport::PublisherPtr locationPub;
	};
}
