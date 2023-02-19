#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class GripperMove : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GripperMove::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the left gripper joint
      this->model->GetJoint("gripper_to_left_finger_joint")->SetVelocity(0, 0.01);


      // stop when the left finger joint reaches 0.05 m
        if (this->model->GetJoint("gripper_to_left_finger_joint")->Position(0) > 0.05)
        {
            this->model->GetJoint("gripper_to_left_finger_joint")->SetVelocity(0, 0);
        }

        std::cout << "left finger joint velocity: " << this->model->GetJoint("gripper_to_left_finger_joint")->GetVelocity(0) << std::endl;

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GripperMove)
}