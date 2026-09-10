#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

struct MissionStep {
  std::string command;
  int round;
  std::string street_name;
  std::string bin_code;
  std::string description;
};

class CivicLensAutonomy : public rclcpp::Node
{
public:
  CivicLensAutonomy() : Node("civiclens_autonomy"), current_mission_step_(0), waiting_for_truck_(false)
  {
    cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/civiclens/truck_command", 10);
    mission_status_pub_ = this->create_publisher<std_msgs::msg::String>("/civiclens/mission_status", 10);

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/civiclens/truck_state", 10,
      std::bind(&CivicLensAutonomy::state_callback, this, std::placeholders::_1));

    build_mission_sequence();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CivicLensAutonomy::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "==========================================================");
    RCLCPP_INFO(this->get_logger(), "CIVICLENS AI AUTONOMOUS CENTRAL MISSION PLANNER INITIALIZED");
    RCLCPP_INFO(this->get_logger(), "Total Missions: 5 Round Trips (Streets 1 -> 5 <-> Base Station)");
    RCLCPP_INFO(this->get_logger(), "==========================================================");

    // Initial wait for simulation physics to settle
    rclcpp::sleep_for(std::chrono::seconds(3));
  }

private:
  void build_mission_sequence()
  {
    // =========================================================================
    // ROUND 1: BASE -> STREET 1 (DB-01, Biodegradable) -> BASE UNLOAD
    // =========================================================================
    add_step("GOTO 0 0 1.57", 1, "Street 1: Greenfield Rd", "DB-01", "Leaving Base towards Intersection");
    add_step("GOTO 30 0 0.0", 1, "Street 1: Greenfield Rd", "DB-01", "Cruising East along Greenfield Road");
    add_step("GOTO 30 -1.5 -1.57", 1, "Street 1: Greenfield Rd", "DB-01", "Aligning Truck with Smart Dustbin DB-01");
    add_step("PICKUP 1 BIODEGRADABLE", 1, "Street 1: Greenfield Rd", "DB-01", "Robotic Arm Grasp & Sorting into Bio Compartment");
    add_step("GOTO 30 0 3.14", 1, "Street 1: Greenfield Rd", "DB-01", "Returning to Road Centerline");
    add_step("GOTO 0 0 3.14", 1, "Street 1: Greenfield Rd", "DB-01", "Cruising West towards Central Depot");
    add_step("GOTO 0 -10 -1.57", 1, "Central Base Station", "DB-01", "Docking at Base Station Recycling Bay");
    add_step("UNLOAD", 1, "Central Base Station", "DB-01", "Discharging Segregated Bio Payload at Base Station");

    // =========================================================================
    // ROUND 2: BASE -> STREET 2 (DB-02, Non-Biodegradable) -> BASE UNLOAD
    // =========================================================================
    add_step("GOTO 0 0 1.57", 2, "Street 2: Market Lane", "DB-02", "Leaving Base towards Market Lane");
    add_step("GOTO 50 0 0.0", 2, "Street 2: Market Lane", "DB-02", "Navigating to Market Lane Junction");
    add_step("GOTO 50 30 1.57", 2, "Street 2: Market Lane", "DB-02", "Cruising North along Market Lane");
    add_step("GOTO 51.5 30 0.0", 2, "Street 2: Market Lane", "DB-02", "Aligning Truck with Smart Dustbin DB-02");
    add_step("PICKUP 2 NON_BIODEGRADABLE", 2, "Street 2: Market Lane", "DB-02", "Robotic Arm Grasp & Sorting into Non-Bio Compartment");
    add_step("GOTO 50 30 -1.57", 2, "Street 2: Market Lane", "DB-02", "Returning to Market Lane Centerline");
    add_step("GOTO 50 0 -1.57", 2, "Street 2: Market Lane", "DB-02", "Navigating South to Main Junction");
    add_step("GOTO 0 0 3.14", 2, "Street 2: Market Lane", "DB-02", "Cruising West towards Central Depot");
    add_step("GOTO 0 -10 -1.57", 2, "Central Base Station", "DB-02", "Docking at Base Station Recycling Bay");
    add_step("UNLOAD", 2, "Central Base Station", "DB-02", "Discharging Segregated Non-Bio Payload at Base Station");

    // =========================================================================
    // ROUND 3: BASE -> STREET 3 (DB-03, Plastic + Crusher) -> BASE UNLOAD
    // =========================================================================
    add_step("GOTO 0 0 1.57", 3, "Street 3: Heritage Ave", "DB-03", "Leaving Base towards Heritage Avenue");
    add_step("GOTO 0 50 1.57", 3, "Street 3: Heritage Ave", "DB-03", "Cruising North to Heritage Avenue Junction");
    add_step("GOTO 30 50 0.0", 3, "Street 3: Heritage Ave", "DB-03", "Cruising East along Heritage Avenue");
    add_step("GOTO 30 51.5 1.57", 3, "Street 3: Heritage Ave", "DB-03", "Aligning Truck with Smart Dustbin DB-03");
    add_step("PICKUP 3 PLASTIC", 3, "Street 3: Heritage Ave", "DB-03", "Robotic Arm Grasp, Crusher Compaction & Plastic Sorting");
    add_step("GOTO 30 50 3.14", 3, "Street 3: Heritage Ave", "DB-03", "Returning to Heritage Avenue Centerline");
    add_step("GOTO 0 50 3.14", 3, "Street 3: Heritage Ave", "DB-03", "Navigating West to Main Junction");
    add_step("GOTO 0 0 -1.57", 3, "Street 3: Heritage Ave", "DB-03", "Cruising South towards Central Depot");
    add_step("GOTO 0 -10 -1.57", 3, "Central Base Station", "DB-03", "Docking at Base Station Shredder Plant");
    add_step("UNLOAD", 3, "Central Base Station", "DB-03", "Discharging Compacted Plastic Payload into Shredder Hopper");

    // =========================================================================
    // ROUND 4: BASE -> STREET 4 (DB-04, Biodegradable) -> BASE UNLOAD
    // =========================================================================
    add_step("GOTO 0 0 1.57", 4, "Street 4: Sunrise Blvd", "DB-04", "Leaving Base towards Sunrise Boulevard");
    add_step("GOTO 0 30 1.57", 4, "Street 4: Sunrise Blvd", "DB-04", "Cruising North along Sunrise Boulevard");
    add_step("GOTO -1.5 30 3.14", 4, "Street 4: Sunrise Blvd", "DB-04", "Aligning Truck with Smart Dustbin DB-04");
    add_step("PICKUP 4 BIODEGRADABLE", 4, "Street 4: Sunrise Blvd", "DB-04", "Robotic Arm Grasp & Sorting into Bio Compartment");
    add_step("GOTO 0 30 -1.57", 4, "Street 4: Sunrise Blvd", "DB-04", "Returning to Sunrise Boulevard Centerline");
    add_step("GOTO 0 0 -1.57", 4, "Street 4: Sunrise Blvd", "DB-04", "Cruising South towards Central Depot");
    add_step("GOTO 0 -10 -1.57", 4, "Central Base Station", "DB-04", "Docking at Base Station Recycling Bay");
    add_step("UNLOAD", 4, "Central Base Station", "DB-04", "Discharging Segregated Bio Payload at Base Station");

    // =========================================================================
    // ROUND 5: BASE -> STREET 5 (DB-05, Plastic + Crusher) -> BASE UNLOAD
    // =========================================================================
    add_step("GOTO 0 0 1.57", 5, "Street 5: Tech Park Way", "DB-05", "Leaving Base towards Tech Park Way");
    add_step("GOTO 80 0 0.0", 5, "Street 5: Tech Park Way", "DB-05", "Cruising East along Tech Park Way");
    add_step("GOTO 80 -1.5 -1.57", 5, "Street 5: Tech Park Way", "DB-05", "Aligning Truck with Smart Dustbin DB-05");
    add_step("PICKUP 5 PLASTIC", 5, "Street 5: Tech Park Way", "DB-05", "Robotic Arm Grasp, Crusher Compaction & Plastic Sorting");
    add_step("GOTO 80 0 3.14", 5, "Street 5: Tech Park Way", "DB-05", "Returning to Tech Park Way Centerline");
    add_step("GOTO 0 0 3.14", 5, "Street 5: Tech Park Way", "DB-05", "Cruising West towards Central Depot");
    add_step("GOTO 0 -10 -1.57", 5, "Central Base Station", "DB-05", "Final Docking at Central Recycling Plant");
    add_step("UNLOAD", 5, "Central Base Station", "DB-05", "Final Discharging & Resetting Autonomous Truck");
  }

  void add_step(const std::string &cmd, int round, const std::string &street, 
                const std::string &bin, const std::string &desc)
  {
    MissionStep step;
    step.command = cmd;
    step.round = round;
    step.street_name = street;
    step.bin_code = bin;
    step.description = desc;
    mission_sequence_.push_back(step);
  }

  void publish_mission_status(bool complete = false)
  {
    std::ostringstream ss;
    int cur_round = 1;
    std::string street = "Complete";
    std::string bin = "N/A";
    std::string cmd = "DONE";
    std::string desc = "ALL 5 STREETS SERVICED & RECYCLED";

    if (current_mission_step_ < mission_sequence_.size()) {
      const auto &s = mission_sequence_[current_mission_step_];
      cur_round = s.round;
      street = s.street_name;
      bin = s.bin_code;
      cmd = s.command;
      desc = s.description;
    }

    double progress = (100.0 * current_mission_step_) / mission_sequence_.size();
    if (complete) progress = 100.0;

    ss << "{"
       << "\"round\":" << cur_round << ","
       << "\"total_rounds\":5,"
       << "\"street\":\"" << street << "\","
       << "\"bin_code\":\"" << bin << "\","
       << "\"command\":\"" << cmd << "\","
       << "\"description\":\"" << desc << "\","
       << "\"step_index\":" << current_mission_step_ << ","
       << "\"total_steps\":" << mission_sequence_.size() << ","
       << "\"progress\":" << std::fixed << std::setprecision(1) << progress << ","
       << "\"complete\":" << (complete ? "true" : "false")
       << "}";

    std_msgs::msg::String msg;
    msg.data = ss.str();
    mission_status_pub_->publish(msg);
  }

  void state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "DONE" && waiting_for_truck_) {
      RCLCPP_INFO(this->get_logger(), ">> Truck action finished.");
      waiting_for_truck_ = false;
      current_mission_step_++;
      publish_mission_status(current_mission_step_ >= mission_sequence_.size());
    }
  }

  void timer_callback()
  {
    if (!waiting_for_truck_ && current_mission_step_ < mission_sequence_.size()) {
      const auto &step = mission_sequence_[current_mission_step_];
      auto msg = std_msgs::msg::String();
      msg.data = step.command;
      
      RCLCPP_INFO(this->get_logger(), 
        "[Round %d/5] Executing: %s | Target: %s (%s) | %s",
        step.round, step.command.c_str(), step.bin_code.c_str(), 
        step.street_name.c_str(), step.description.c_str());

      cmd_pub_->publish(msg);
      waiting_for_truck_ = true;
      publish_mission_status(false);
    }
    else if (!waiting_for_truck_ && current_mission_step_ >= mission_sequence_.size()) {
      RCLCPP_INFO(this->get_logger(), "==========================================================");
      RCLCPP_INFO(this->get_logger(), "CIVICLENS AI MISSION FLEET: ALL 5 STREETS SUCCESSFULLY SERVICED!");
      RCLCPP_INFO(this->get_logger(), "ALL PAYLOADS SEGREGATED, CRUSHED & UNLOADED AT BASE.");
      RCLCPP_INFO(this->get_logger(), "==========================================================");
      publish_mission_status(true);
      timer_->cancel();
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<MissionStep> mission_sequence_;
  size_t current_mission_step_;
  bool waiting_for_truck_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CivicLensAutonomy>());
  rclcpp::shutdown();
  return 0;
}

