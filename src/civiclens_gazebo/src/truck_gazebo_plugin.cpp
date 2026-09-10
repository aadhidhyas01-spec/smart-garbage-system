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
    gz::sim::Entity model_entity;
    gz::sim::Entity link_entity;
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
      telemetry_timer(0),
      wheel_rot_angle(0.0),
      current_velocity(0.0),
      battery_soc(98.6),
      motor_rpm(0.0),
      motor_speed_kmh(0.0),
      ultrasonic_dist(5.0),
      crusher_status("IDLE"),
      crusher_torque(12.0),
      crusher_rpm(0.0),
      active_waste_entity(gz::sim::kNullEntity),
      active_waste_link_entity(gz::sim::kNullEntity)
    {}

    ~TruckGazeboPlugin() override = default;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &) override
    {
      this->model = gz::sim::Model(_entity);
      
      // 6-DOF Robotic Arm joints
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint1"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint2"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint3"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint4"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint5"));
      this->arm_joints.push_back(this->model.JointByName(_ecm, "arm_joint6"));

      // Wheel revolute joints for active driving rotation
      this->wheel_joints.push_back(this->model.JointByName(_ecm, "fl_joint"));
      this->wheel_joints.push_back(this->model.JointByName(_ecm, "fr_joint"));
      this->wheel_joints.push_back(this->model.JointByName(_ecm, "rl_joint"));
      this->wheel_joints.push_back(this->model.JointByName(_ecm, "rr_joint"));

      this->node.Subscribe("/civiclens/truck_command", &TruckGazeboPlugin::OnCommand, this);
      this->state_pub = this->node.Advertise<gz::msgs::StringMsg>("/civiclens/truck_state");
      this->telemetry_pub = this->node.Advertise<gz::msgs::StringMsg>("/civiclens/truck_telemetry");

      std::cout << "[CivicLens Truck Plugin] Configured: High-Fidelity Vehicle, Active Kinematic Wheels, 6-DOF Arm & Multi-Compartment sorting." << std::endl;
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
        if (this->state == "PICKING") {
          return; // Already actively executing picking sequence
        }
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
        this->active_waste_entity = gz::sim::kNullEntity;
        this->active_waste_link_entity = gz::sim::kNullEntity;
        std::cout << "[CivicLens Truck Plugin] Starting PICKUP for Bin " << this->current_bin_id 
                  << " (Waste: " << this->current_waste_type << ")" << std::endl;
      } else if (action == "UNLOAD") {
        this->state = "UNLOADING";
        this->arm_timer = 0;
        std::cout << "[CivicLens Truck Plugin] Starting UNLOAD at Base Station." << std::endl;
      }
    }

    void SetEntityWorldPose(gz::sim::EntityComponentManager &_ecm, gz::sim::Entity ent, const gz::math::Pose3d &p) {
      if (ent == gz::sim::kNullEntity) return;
      if (!_ecm.EntityHasComponentType(ent, gz::sim::components::WorldPoseCmd::typeId)) {
        _ecm.CreateComponent(ent, gz::sim::components::WorldPoseCmd(p));
      } else {
        _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(ent, p);
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
         << "\"plastic_count\":" << this->plastic_count << ","
         << "\"ultrasonic_dist_m\":" << std::setprecision(2) << this->ultrasonic_dist << ","
         << "\"motor_speed_kmh\":" << this->motor_speed_kmh << ","
         << "\"motor_rpm\":" << std::setprecision(0) << this->motor_rpm << ","
         << "\"battery_soc_pct\":" << std::setprecision(1) << this->battery_soc << ","
         << "\"battery_voltage\":" << (70.0 + (this->battery_soc * 0.04)) << ","
         << "\"crusher_status\":\"" << this->crusher_status << "\","
         << "\"crusher_rpm\":" << this->crusher_rpm << ","
         << "\"crusher_torque_nm\":" << std::setprecision(1) << this->crusher_torque << ","
         << "\"lidar_range_m\":" << (this->state == "MOVING" ? 18.5 : 2.8) << ","
         << "\"imu_pitch_deg\":0.15"
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

      // Periodically publish telemetry (every 100 ticks = 10Hz)
      this->telemetry_timer++;
      if (this->telemetry_timer >= 100) {
        this->telemetry_timer = 0;
        this->PublishTelemetry(pose.Pos().X(), pose.Pos().Y(), yaw);
      }

      // Update positions of loaded waste entities to stay secured in truck bed compartments
      for (const auto &item : this->loaded_wastes) {
        gz::math::Vector3d world_p = pose.Pos() + pose.Rot().RotateVector(item.local_offset);
        gz::math::Pose3d item_pose(world_p, pose.Rot());
        this->SetEntityWorldPose(_ecm, item.model_entity, item_pose);
        this->SetEntityWorldPose(_ecm, item.link_entity, item_pose);
      }

      if (this->state == "MOVING") {
        double dx = this->target_x - pose.Pos().X();
        double dy = this->target_y - pose.Pos().Y();
        double dist = std::sqrt(dx*dx + dy*dy);

        // Simulated Ultrasonic sensor reading to target
        this->ultrasonic_dist = std::max(0.65, dist);

        double target_yaw = std::atan2(dy, dx);
        if (dist < 0.25) {
          target_yaw = this->target_yaw_cmd;
        }

        double angle_diff = target_yaw - yaw;
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        if (dist < 0.25 && std::abs(angle_diff) < 0.08) {
          this->state = "IDLE";
          this->current_velocity = 0.0;
          this->motor_speed_kmh = 0.0;
          this->motor_rpm = 0.0;
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
        
        if (dist < 0.25) v = 0.0; // Align with final orientation

        this->current_velocity = v;
        this->motor_speed_kmh = v * 3.6;
        this->motor_rpm = (v / (2 * M_PI * 0.385)) * 60.0 * 8.5; // Gear ratio 8.5:1
        if (v > 0.01) {
          this->battery_soc = std::max(10.0, this->battery_soc - 0.00015);
        }

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

        // Rotate wheels kinematically with vehicle motion (scaled 0.385m radius)
        this->wheel_rot_angle += (v / 0.385) * dt;
        for (auto wj : this->wheel_joints) {
          if (wj != gz::sim::kNullEntity) {
            _ecm.SetComponentData<gz::sim::components::JointPositionReset>(wj, {this->wheel_rot_angle});
          }
        }
      } 
      else if (this->state == "PICKING") {
        this->arm_timer++;
        this->current_velocity = 0.0;
        this->motor_speed_kmh = 0.0;
        this->motor_rpm = 0.0;
        this->ultrasonic_dist = 0.85; // Close docking reading at bin
        this->battery_soc = std::max(10.0, this->battery_soc - 0.00025);

        // Find active waste entity and child link if not cached
        if (this->active_waste_entity == gz::sim::kNullEntity) {
          std::string waste_name = "waste_" + std::to_string(this->current_bin_id);
          this->active_waste_entity = _ecm.EntityByComponents(gz::sim::components::Name(waste_name));
          if (this->active_waste_entity != gz::sim::kNullEntity) {
            gz::sim::Model wm(this->active_waste_entity);
            this->active_waste_link_entity = wm.LinkByName(_ecm, "link");
          }
        }

        std::vector<double> target_angles = {0, 0, 0, 0, 0, 0};
        gz::math::Vector3d waste_local_pos(2.50, 0.0, 0.78);
        bool update_waste = false;
        
        if (this->arm_seq_step == 0) {
          // STEP 0: Reach forward directly towards top rim of dustbin (2.50m forward)
          target_angles = {0.0, 0.65, 0.35, 0.0, 0.55, 0.0};
          if (this->arm_timer > 800) { 
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 1) {
          // STEP 1: Grasp waste inside roadside dustbin cavity
          target_angles = {0.0, 0.78, 0.50, 0.0, 0.30, 0.0};
          update_waste = true;
          waste_local_pos = gz::math::Vector3d(2.50, 0.0, 0.78);
          if (this->arm_timer > 900) { 
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 2) {
          // STEP 2: Lift waste high up out of roadside dustbin into clear air
          target_angles = {0.0, -0.35, -0.45, 0.0, 0.80, 0.0};
          update_waste = true;
          double lift_progress = std::min(1.0, (double)this->arm_timer / 900.0);
          double z_interp = 0.78 + (1.50 * lift_progress);
          double x_interp = 2.50 - (1.30 * lift_progress);
          waste_local_pos = gz::math::Vector3d(x_interp, 0.0, z_interp);

          if (this->arm_timer > 1100) { 
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 3) {
          // STEP 3: Swing backward 180 deg towards the designated segregation compartment
          double swing_progress = std::min(1.0, (double)this->arm_timer / 1200.0);

          if (this->current_waste_type == "BIODEGRADABLE") {
            // Compartment 1 (Bio, Green) at X = +0.525
            target_angles = {3.14159, 0.45, 0.65, 0.0, 0.45, 0.0};
            waste_local_pos = gz::math::Vector3d(1.20 - (0.675 * swing_progress), 0.0, 2.28 - (1.48 * swing_progress));
          } else if (this->current_waste_type == "NON_BIODEGRADABLE") {
            // Compartment 2 (Non-Bio, Blue) at X = -0.175
            target_angles = {3.14159, 0.65, 0.35, 0.0, 0.55, 0.0};
            waste_local_pos = gz::math::Vector3d(1.20 - (1.375 * swing_progress), 0.0, 2.28 - (1.48 * swing_progress));
          } else {
            // Compartment 3 (Plastic + Crusher) at X = -1.54 / Crusher Hopper
            target_angles = {3.14159, 0.75, 0.40, 0.0, 0.40, 0.0};
            waste_local_pos = gz::math::Vector3d(1.20 - (2.74 * swing_progress), 0.0, 2.28 - (1.40 * swing_progress));
            // Activate plastic crusher
            this->crusher_status = "CRUSHING";
            this->crusher_rpm = 450.0;
            this->crusher_torque = 125.0;
          }
          update_waste = true;

          if (this->arm_timer > 1400) { 
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 4) {
          // STEP 4: Drop waste into the targeted compartment
          if (this->current_waste_type == "BIODEGRADABLE") {
            target_angles = {3.14159, 0.50, 0.70, 0.0, 0.40, 0.0};
            waste_local_pos = gz::math::Vector3d(0.525, 0.0, 0.55);
          } else if (this->current_waste_type == "NON_BIODEGRADABLE") {
            target_angles = {3.14159, 0.70, 0.40, 0.0, 0.50, 0.0};
            waste_local_pos = gz::math::Vector3d(-0.175, 0.0, 0.55);
          } else {
            target_angles = {3.14159, 0.80, 0.45, 0.0, 0.35, 0.0};
            waste_local_pos = gz::math::Vector3d(-0.875, 0.0, 0.55);
          }
          update_waste = true;
          
          if (this->arm_timer == 250) {
            this->DepositWasteIntoCompartment(_ecm, pose);
            this->crusher_status = "IDLE";
            this->crusher_rpm = 0.0;
            this->crusher_torque = 12.0;
          }

          if (this->arm_timer > 1000) { 
            this->arm_seq_step++; 
            this->arm_timer = 0; 
          }
        } else if (this->arm_seq_step == 5) {
          // STEP 5: Return arm to stowed transit position
          target_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          if (this->arm_timer > 1000) { 
            this->state = "IDLE"; 
            this->PublishState("DONE");
            std::cout << "[CivicLens Truck Plugin] Finished PICKUP sequence for Bin " 
                      << this->current_bin_id << std::endl;
          }
        }

        // Kinematically pin waste to claw during pickup manipulation
        if (update_waste && this->active_waste_entity != gz::sim::kNullEntity) {
          gz::math::Vector3d world_w = pose.Pos() + pose.Rot().RotateVector(waste_local_pos);
          gz::math::Pose3d w_pose(world_w, pose.Rot());
          this->SetEntityWorldPose(_ecm, this->active_waste_entity, w_pose);
          this->SetEntityWorldPose(_ecm, this->active_waste_link_entity, w_pose);
        }

        // Apply smooth kinematic joint interpolation
        for (size_t i = 0; i < 6; i++) {
          if (this->arm_joints[i] != gz::sim::kNullEntity) {
            auto posComp = _ecm.Component<gz::sim::components::JointPosition>(this->arm_joints[i]);
            double cur = (posComp && !posComp->Data().empty()) ? posComp->Data()[0] : 0.0;
            double step = (target_angles[i] - cur) * 0.025;
            _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
                this->arm_joints[i], {cur + step});
          }
        }
      }
      else if (this->state == "UNLOADING") {
        this->arm_timer++;
        this->current_velocity = 0.0;
        this->motor_speed_kmh = 0.0;
        this->motor_rpm = 0.0;

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

        // Discharge all segregated payloads to base recycling facility
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

    void DepositWasteIntoCompartment(gz::sim::EntityComponentManager &_ecm, const gz::math::Pose3d &truck_pose) {
      if (this->active_waste_entity == gz::sim::kNullEntity) {
        std::string waste_name = "waste_" + std::to_string(this->current_bin_id);
        this->active_waste_entity = _ecm.EntityByComponents(gz::sim::components::Name(waste_name));
        if (this->active_waste_entity != gz::sim::kNullEntity) {
          gz::sim::Model wm(this->active_waste_entity);
          this->active_waste_link_entity = wm.LinkByName(_ecm, "link");
        }
      }

      gz::math::Vector3d comp_offset;
      if (this->current_waste_type == "BIODEGRADABLE") {
        comp_offset = gz::math::Vector3d(0.525, 0.0, 0.52);
        this->bio_count++;
      } else if (this->current_waste_type == "NON_BIODEGRADABLE") {
        comp_offset = gz::math::Vector3d(-0.175, 0.0, 0.52);
        this->nonbio_count++;
      } else {
        comp_offset = gz::math::Vector3d(-0.875, 0.0, 0.52);
        this->plastic_count++;
      }

      if (this->active_waste_entity != gz::sim::kNullEntity) {
        gz::math::Vector3d world_p = truck_pose.Pos() + truck_pose.Rot().RotateVector(comp_offset);
        gz::math::Pose3d final_pose(world_p, truck_pose.Rot());
        this->SetEntityWorldPose(_ecm, this->active_waste_entity, final_pose);
        this->SetEntityWorldPose(_ecm, this->active_waste_link_entity, final_pose);

        // Register loaded waste item to be tracked while driving
        LoadedWaste item;
        item.model_entity = this->active_waste_entity;
        item.link_entity = this->active_waste_link_entity;
        item.local_offset = comp_offset;
        item.waste_type = this->current_waste_type;
        item.bin_id = this->current_bin_id;
        this->loaded_wastes.push_back(item);
      }
    }

    void UnloadAllWasteAtBase(gz::sim::EntityComponentManager &_ecm) {
      int idx = 0;
      for (const auto &item : this->loaded_wastes) {
        double ux = 4.5 + (idx * 0.7);
        double uy = -10.0 + ((idx % 2 == 0) ? 0.6 : -0.6);
        double uz = 0.35;

        gz::math::Pose3d pad_pose(ux, uy, uz, 0, 0, 0);
        this->SetEntityWorldPose(_ecm, item.model_entity, pad_pose);
        this->SetEntityWorldPose(_ecm, item.link_entity, pad_pose);
        idx++;
      }
      this->loaded_wastes.clear();
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
    double wheel_rot_angle;
    double current_velocity;
    double battery_soc;
    double motor_rpm;
    double motor_speed_kmh;
    double ultrasonic_dist;
    std::string crusher_status;
    double crusher_torque;
    double crusher_rpm;

    gz::sim::Entity active_waste_entity;
    gz::sim::Entity active_waste_link_entity;
    std::vector<gz::sim::Entity> arm_joints;
    std::vector<gz::sim::Entity> wheel_joints;
    std::vector<LoadedWaste> loaded_wastes;
  };
}

GZ_ADD_PLUGIN(civiclens::TruckGazeboPlugin,
              gz::sim::System,
              civiclens::TruckGazeboPlugin::ISystemConfigure,
              civiclens::TruckGazeboPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(civiclens::TruckGazeboPlugin, "civiclens::TruckGazeboPlugin")
