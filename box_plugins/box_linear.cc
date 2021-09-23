#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
// #include <ignition/math6/ignition/math.hh>

namespace gazebo
{
  class box_linear : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&box_linear::OnUpdate, this));
        this->vel = ignition::math::Vector3d(0, .5, 0);
      std::cout << "Start!\n";
    }

    public: void OnUpdate()
    {
      this->pose = this->model->WorldPose();
      if(this->pose.Y() >= 3)
      {

        this->vel = ignition::math::Vector3d(0, -1, 0);
        this->model->SetLinearVel(this->vel);
      }
      else if (this->pose.Y() < -3)
      {
        this->vel = ignition::math::Vector3d(0, 1, 0);
        this->model->SetLinearVel(this->vel);
      }
      else
      {
          this->model->SetLinearVel(this->vel);
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: ignition::math::Pose3d pose; //= this->model->WorldPose();
    private: ignition::math::Vector3d vel;// = this->model->WorldLinearVel();
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(box_linear)
}