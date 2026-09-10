#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <iostream>
#include <cmath>
#include <mutex>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

namespace civiclens
{
  struct LoadedWaste {
    gz::sim::Entity entity;
    gz::math::Vector3d local_offset;
    std::string waste_type;
    int bin_id;
  };

  class TruckGazeboPlugin : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
  public:
    TruckGazeboPlugin() : 
      state("IDLE"), 
      arm_seq_step(0), 
      arm_timer(0),
      current_bin_id(0),
      current_waste_type("BIODEGRADABLE"),
      target_x(0.0),
      target_y(0.0),
      target_yaw_cmd(0.0),
      bio_count(0),
      nonbio_count(0),
      plastic_count(0),
      telemetry_timer(0)
    {}

    ~TruckGazeboPlugin() override = default;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &) override
    {
      this->model = gz::sim::Model(_entity);
      
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint1"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint2"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint3"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint4"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint5"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint6"));

      this->node.Subscribe("/civiclens/truck_command", &TruckGazeboPlugin::OnCommand, this);
      this->state_pub = this->node.Advertise<gz::msgs::StringMsg>("/civiclens/truck_state");
      this->telemetry_pub = this->node.Advertise<gz::msgs::StringMsg>("/civiclens/truck_telemetry");

      std::cout << "[CivicLens Truck Plugin] Configured successfully with 6-DOF Arm & Multi-Compartment sorting." << std::endl;
    }

    void OnCommand(const gz::msgs::StringMsg &_msg)
    {
      std::lock_guard<std::mutex> lock(this->msg_mutex);
      std::string cmd = _msg.data();
      
      std::istringstream iss(cmd);
      std::string action;
      iss >> action;

      if (action == "GOTO") {
        iss >> this->target_x >> this->target_y >> this->target_yaw_cmd;
        this->state = "MOVING";
      } else if (action == "PICKUP") {
        iss >> this->current_bin_id;
        std::string wtype;
        if (iss >> wtype) {
          this->current_waste_type = wtype;
        } else {
          // Default mapping for the 5 bins
          if (this->current_bin_id == 1 || this->current_bin_id == 4) {
            this->current_waste_type = "BIODEGRADABLE";
          } else if (this->current_bin_id == 2) {
            this->current_waste_type = "NON_BIODEGRADABLE";
          } else {
            this->current_waste_type = "PLASTIC";
          }
        }
        this->state = "PICKING";
        this->arm_seq_step = 0;
        this->arm_timer = 0;
        std::cout << "[CivicLens Truck Plugin] Starting PICKUP for Bin " << this->current_bin_id 
                  << " (Waste: " << this->current_waste_type << ")" << std::endl;
      } else if (action == "UNLOAD") {
        this->state = "UNLOADING";
        this->arm_timer = 0;
        std::cout << "[CivicLens Truck Plugin] Starting UNLOAD at Base Station." << std::endl;
      }
    }

    void PublishState(const std::string &s) {
      gz::msgs::StringMsg msg;
      msg.set_data(s);
      this->state_pub.Publish(msg);
    }

    void PublishTelemetry(double truck_x, double truck_y, double truck_yaw) {
      std::ostringstream ss;
      ss << "{"
         << "\"x\":" << std::fixed << std::setprecision(2) << truck_x << ","
         << "\"y\":" << truck_y << ","
         << "\"yaw\":" << truck_yaw << ","
         << "\"state\":\"" << this->state << "\","
         << "\"bin_id\":" << this->current_bin_id << ","
         << "\"waste_type\":\"" << this->current_waste_type << "\","
         << "\"arm_step\":" << this->arm_seq_step << ","
         << "\"bio_count\":" << this->bio_count << ","
         << "\"nonbio_count\":" << this->nonbio_count << ","
         << "\"plastic_count\":" << this->plastic_count
         << "}";
      gz::msgs::StringMsg msg;
      msg.set_data(ss.str());
      this->telemetry_pub.Publish(msg);
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override
    {
      if (_info.paused) return;

      std::lock_guard<std::mutex> lock(this->msg_mutex);

      auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
      if (!poseComp) return;

      gz::math::Pose3d pose = poseComp->Data();
      double yaw = pose.Rot().Yaw();

      // Periodically publish telemetry (every 100 ticks = 10Hz at 1000Hz rate)
      this->telemetry_timer++;
      if (this->telemetry_timer >= 100) {
        this->telemetry_timer = 0;
        this->PublishTelemetry(pose.Pos().X(), pose.Pos().Y(), yaw);
      }

      // Update positions of loaded waste entities to stay secured in truck bed compartments
      for (const auto &item : this->loaded_wastes) {
        if (item.entity != gz::sim::kNullEntity) {
          gz::math::Vector3d world_p = pose.Pos() + pose.Rot().RotateVector(item.local_offset);
          gz::math::Pose3d item_pose(world_p, pose.Rot());
          if (!_ecm.EntityHasComponentType(item.entity, gz::sim::components::WorldPoseCmd::typeId)) {
            _ecm.CreateComponent(item.entity, gz::sim::components::WorldPoseCmd(item_pose));
          } else {
            _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(item.entity, item_pose);
          }
        }
      }

      if (this->state == "MOVING") {
        double dx = this->target_x - pose.Pos().X();
        double dy = this->target_y - pose.Pos().Y();
        double dist = std::sqrt(dx*dx + dy*dy);

        double target_yaw = std::atan2(dy, dx);
        if (dist < 0.25) {
          target_yaw = this->target_yaw_cmd;
        }

        double angle_diff = target_yaw - yaw;
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        if (dist < 0.25 && std::abs(angle_diff) < 0.08) {
          this->state = "IDLE";
          this->PublishState("DONE");
          return;
        }

        double v = 4.0; // Cruise speed (m/s)
        if (dist < 2.0) v = 1.2;
        
        double w = 3.2 * angle_diff;
        if (w > 2.2) w = 2.2;
        if (w < -2.2) w = -2.2;

        if (std::abs(angle_diff) > 0.15) {
          v = 0.0;
          w = (angle_diff > 0) ? 2.0 : -2.0;
        }
        
        if (dist < 0.25) v = 0.0; // Rotate in place to align with final orientation

        double dt = 0.001; 
        double new_x = pose.Pos().X() + v * std::cos(yaw) * dt;
        double new_y = pose.Pos().Y() + v * std::sin(yaw) * dt;
        double new_yaw = yaw + w * dt;

        gz::math::Pose3d new_pose(new_x, new_y, pose.Pos().Z(), 0, 0, new_yaw);
        
        if (!_ecm.EntityHasComponentType(this->model.Entity(), gz::sim::components::WorldPoseCmd::typeId)) {
          _ecm.CreateComponent(this->model.Entity(), gz::sim::components::WorldPoseCmd(new_pose));
        } else {
          _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(this->model.Entity(), new_pose);
        }
      } 
      else if (this->state == "PICKING") {
        this->arm_timer++;

        std::vector<double> target_angles = {0, 0, 0, 0, 0, 0};
        
        if (this->arm_seq_step == 0) {
          // Reach forward towards dustbin
          target_angles = {0.0, 0.6, -1.0, 0.0, 0.9, 0.0};
          if (this->arm_timer > 900) { 
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 1) {
          // Grasp waste bag with end-effector
          target_angles = {0.0, 0.85, -1.25, 0.0, 1.2, 0.0};
          if (this->arm_timer > 900) { 
            this->LiftWasteFromBin(_ecm, pose);
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 2) {
          // Lift waste high above dustbin
          target_angles = {0.0, -0.6, -0.6, 0.0, 0.2, 0.0};
          if (this->arm_timer > 1200) { 
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 3) {
          // Swing backward towards the designated segregation compartment
          if (this->current_waste_type == "BIODEGRADABLE") {
            // Compartment 1 (Bio, Green) at X = +0.40
            target_angles = {3.14159, 0.65, -1.35, 0.0, 0.65, 0.0};
          } else if (this->current_waste_type == "NON_BIODEGRADABLE") {
            // Compartment 2 (Non-Bio, Blue) at X = 0.0
            target_angles = {3.14159, 0.30, -0.95, 0.0, 0.65, 0.0};
          } else {
            // Compartment 3 (Plastic + Crusher) at X = -0.40 / Crusher at X = -0.80
            // First stage: Hover above crusher hopper
            target_angles = {3.14159, -0.45, -0.30, 0.0, 0.75, 0.0};
          }

          if (this->arm_timer > 1500) { 
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 4) {
          // Drop / compact waste into the targeted compartment
          if (this->current_waste_type == "BIODEGRADABLE") {
            target_angles = {3.14159, 0.75, -1.45, 0.0, 0.7, 0.0};
          } else if (this->current_waste_type == "NON_BIODEGRADABLE") {
            target_angles = {3.14159, 0.40, -1.05, 0.0, 0.7, 0.0};
          } else {
            // Plastic crusher feed and deposit into Compartment 3
            target_angles = {3.14159, -0.20, -0.65, 0.0, 0.75, 0.0};
          }
          
          if (this->arm_timer == 500) {
            this->DepositWasteIntoCompartment(_ecm, pose);
          }

          if (this->arm_timer > 1200) { 
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 5) {
          // Return arm to stowed transit position
          target_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          if (this->arm_timer > 1200) { 
            this->state = "IDLE"; 
            this->PublishState("DONE");
            std::cout << "[CivicLens Truck Plugin] Finished PICKUP sequence for Bin " 
                      << this->current_bin_id << std::endl;
          }
        }

        // Apply smooth kinematic joint interpolation
        for (size_t i = 0; i < 6; i++) {
          if (this->arm_joints[i] != gz::sim::kNullEntity) {
            auto posComp = _ecm.Component<gz::sim::components::JointPosition>(this->arm_joints[i]);
            double cur = (posComp && !posComp->Data().empty()) ? posComp->Data()[0] : 0.0;
            double step = (target_angles[i] - cur) * 0.015;
            _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
                this->arm_joints[i], {cur + step});
          }
        }
      }
      else if (this->state == "UNLOADING") {
        this->arm_timer++;

        // Arm gestures during unloading
        std::vector<double> unload_angles = {0.0, -0.4, 0.8, 0.0, -0.4, 0.0};
        for (size_t i = 0; i < 6; i++) {
          if (this->arm_joints[i] != gz::sim::kNullEntity) {
            auto posComp = _ecm.Component<gz::sim::components::JointPosition>(this->arm_joints[i]);
            double cur = (posComp && !posComp->Data().empty()) ? posComp->Data()[0] : 0.0;
            double step = (unload_angles[i] - cur) * 0.02;
            _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
                this->arm_joints[i], {cur + step});
          }
        }

        // Mid-point: Discharge all segregated payloads to base recycling facility
        if (this->arm_timer == 1000) {
          this->UnloadAllWasteAtBase(_ecm);
        }

        if (this->arm_timer > 2200) {
          // Reset arm
          for (size_t i = 0; i < 6; i++) {
            if (this->arm_joints[i] != gz::sim::kNullEntity) {
              _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
                  this->arm_joints[i], {0.0});
            }
          }
          this->state = "IDLE";
          this->PublishState("DONE");
          std::cout << "[CivicLens Truck Plugin] Finished UNLOAD sequence at Base Station." << std::endl;
        }
      }
    }

    void LiftWasteFromBin(gz::sim::EntityComponentManager &_ecm, const gz::math::Pose3d &truck_pose) {
      std::string waste_name = "waste_" + std::to_string(this->current_bin_id);
      gz::sim::Entity waste_model = _ecm.EntityByComponents(gz::sim::components::Name(waste_name));
      if (waste_model != gz::sim::kNullEntity) {
        // Move waste slightly above dustbin to simulate grasp
        gz::math::Vector3d lift_pos = truck_pose.Pos() + truck_pose.Rot().RotateVector(gz::math::Vector3d(1.4, 0.0, 1.2));
        gz::math::Pose3d lift_pose(lift_pos, truck_pose.Rot());
        if (!_ecm.EntityHasComponentType(waste_model, gz::sim::components::WorldPoseCmd::typeId)) {
          _ecm.CreateComponent(waste_model, gz::sim::components::WorldPoseCmd(lift_pose));
        } else {
          _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(waste_model, lift_pose);
        }
      }
    }

    void DepositWasteIntoCompartment(gz::sim::EntityComponentManager &_ecm, const gz::math::Pose3d &truck_pose) {
      std::string waste_name = "waste_" + std::to_string(this->current_bin_id);
      gz::sim::Entity waste_model = _ecm.EntityByComponents(gz::sim::components::Name(waste_name));
      
      gz::math::Vector3d comp_offset;
      if (this->current_waste_type == "BIODEGRADABLE") {
        comp_offset = gz::math::Vector3d(0.40, 0.0, 0.42);
        this->bio_count++;
      } else if (this->current_waste_type == "NON_BIODEGRADABLE") {
        comp_offset = gz::math::Vector3d(0.00, 0.0, 0.42);
        this->nonbio_count++;
      } else {
        comp_offset = gz::math::Vector3d(-0.40, 0.0, 0.42);
        this->plastic_count++;
      }

      if (waste_model != gz::sim::kNullEntity) {
        gz::math::Vector3d world_p = truck_pose.Pos() + truck_pose.Rot().RotateVector(comp_offset);
        gz::math::Pose3d final_pose(world_p, truck_pose.Rot());
        if (!_ecm.EntityHasComponentType(waste_model, gz::sim::components::WorldPoseCmd::typeId)) {
          _ecm.CreateComponent(waste_model, gz::sim::components::WorldPoseCmd(final_pose));
        } else {
          _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(waste_model, final_pose);
        }

        // Register loaded waste item to be tracked while driving
        LoadedWaste item;
        item.entity = waste_model;
        item.local_offset = comp_offset;
        item.waste_type = this->current_waste_type;
        item.bin_id = this->current_bin_id;
        this->loaded_wastes.push_back(item);
      }
    }

    void UnloadAllWasteAtBase(gz::sim::EntityComponentManager &_ecm) {
      int idx = 0;
      for (const auto &item : this->loaded_wastes) {
        if (item.entity != gz::sim::kNullEntity) {
          // Deposit on Base Station Unloading Pad / Shredder Facility
          // Base station is near (0, -10), unloading pad is at (3.5, -10.0, 0.25)
          double ux = 3.5 + (idx * 0.4);
          double uy = -10.0 + ((idx % 2 == 0) ? 0.3 : -0.3);
          double uz = 0.25;

          gz::math::Pose3d pad_pose(ux, uy, uz, 0, 0, 0);
          if (!_ecm.EntityHasComponentType(item.entity, gz::sim::components::WorldPoseCmd::typeId)) {
            _ecm.CreateComponent(item.entity, gz::sim::components::WorldPoseCmd(pad_pose));
          } else {
            _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(item.entity, pad_pose);
          }
        }
        idx++;
      }
      this->loaded_wastes.clear();
      // Reset compartment counts
      this->bio_count = 0;
      this->nonbio_count = 0;
      this->plastic_count = 0;
    }

  private:
    gz::sim::Model model{gz::sim::kNullEntity};
    gz::transport::Node node;
    gz::transport::Node::Publisher state_pub;
    gz::transport::Node::Publisher telemetry_pub;

    std::mutex msg_mutex;
    std::string state;
    int arm_seq_step;
    int arm_timer;

    int current_bin_id;
    std::string current_waste_type;

    double target_x;
    double target_y;
    double target_yaw_cmd;

    int bio_count;
    int nonbio_count;
    int plastic_count;

    int telemetry_timer;

    std::vector<gz::sim::Entity> arm_joints;
    std::vector<LoadedWaste> loaded_wastes;
  };
}

GZ_ADD_PLUGIN(civiclens::TruckGazeboPlugin,
              gz::sim::System,
              civiclens::TruckGazeboPlugin::ISystemConfigure,
              civiclens::TruckGazeboPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(civiclens::TruckGazeboPlugin, "civiclens::TruckGazeboPlugin")

