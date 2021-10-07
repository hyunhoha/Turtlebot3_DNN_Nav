#ifndef _OBSTACLE_PLUGIN_HH_
#define _OBSTACLE_PLUGIN_HH_

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <cstdlib>
#include <ctime>
#include <random>
// #include <thread>
// #include <chrono>

int FLOAT_MIN = -1;
int FLOAT_MAX = 1;
// int TIME_TO_SLEEP = 

// #include <ignition/math6/ignition/math.hh>



// std::srand(100);

namespace gazebo
{
  class Obstacle_Plugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&Obstacle_Plugin::OnUpdate, this));
        this->vel = ignition::math::Vector3d(0, 0, 0);
      std::cout << "Start!\n";
      this->self_count = 0;
    }

    public: void OnUpdate()
    {
    //   this->self_count ++;
      
      if(this->self_count >= 6000)
      {
        std::srand(std::time(NULL));
        this->self_count = 0;
        this->pose = this->model->WorldPose();
        if(abs(this->pose.Y()) >= 10 || abs(this->pose.X()) >= 10)
        {
            // this->pose.Y() = 5;
            // this->pose.X() = 0;

            this->model->SetWorldPose(ignition::math::Pose3d(rand()%10-5,rand()%10-5,0,0,0,0));

            // this->vel = ignition::math::Vector3d(0, -1, 0);
            // this->model->SetLinearVel(this->vel);
        }

        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_real_distribution<float> distr(FLOAT_MIN, FLOAT_MAX);

        // auto velx = rand()%2 - 1;
        auto velx = distr(eng);
        auto vely = distr(eng);
        // auto vely = rand()%2 - 1;
        // this->vel = ignition::math::Vector3d(0.5*this->vel.X() + 0.3* velx, 0.5*this->vel.Y() + 0.5 * vely, 0.01);
        // this->model->SetLinearVel(this->vel);
      }
      else
      {
          this->self_count ++;
          if(this->self_count%2)
          {
            // this->model->SetLinearVel(this->vel);
          }
          
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: ignition::math::Pose3d pose; //= this->model->WorldPose();
    private: ignition::math::Vector3d vel;// = this->model->WorldLinearVel();
    private: int self_count;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Obstacle_Plugin)
}

#endif