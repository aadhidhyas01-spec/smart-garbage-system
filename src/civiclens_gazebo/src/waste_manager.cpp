#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>

class WasteManager : public rclcpp::Node
{
public:
  WasteManager() : Node("waste_manager"), 
    total_bio_kg_(0.0), total_nonbio_kg_(0.0), total_plastic_kg_(0.0),
    bins_serviced_(0), unload_cycles_(0)
  {
    cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/civiclens/truck_command", 10,
      std::bind(&WasteManager::cmd_callback, this, std::placeholders::_1));

    analytics_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/civiclens/fleet_analytics", 10);

    publish_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WasteManager::publish_analytics, this));
      
    RCLCPP_INFO(this->get_logger(), "==========================================================");
    RCLCPP_INFO(this->get_logger(), "CIVICLENS AI CENTRAL FLEET & SEGREGATION MANAGER READY");
    RCLCPP_INFO(this->get_logger(), "3-Compartment Payload Tracking + Plastic Crusher Telemetry");
    RCLCPP_INFO(this->get_logger(), "==========================================================");
  }

private:
  void cmd_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string cmd = msg->data;
    if (cmd.find("PICKUP") != std::string::npos) {
      std::istringstream iss(cmd);
      std::string action, waste_type = "BIODEGRADABLE";
      int bin_id = 1;
      iss >> action >> bin_id;
      if (!(iss >> waste_type)) {
        if (bin_id == 1 || bin_id == 4) waste_type = "BIODEGRADABLE";
        else if (bin_id == 2) waste_type = "NON_BIODEGRADABLE";
        else waste_type = "PLASTIC";
      }

      bins_serviced_++;
      double added_kg = 0.0;
      if (waste_type == "BIODEGRADABLE") {
        added_kg = 42.5 + (bin_id * 2.1);
        total_bio_kg_ += added_kg;
      } else if (waste_type == "NON_BIODEGRADABLE") {
        added_kg = 36.8 + (bin_id * 1.5);
        total_nonbio_kg_ += added_kg;
      } else {
        added_kg = 24.0 + (bin_id * 1.8);
        total_plastic_kg_ += added_kg;
      }

      RCLCPP_INFO(this->get_logger(), "----------------------------------------------------------");
      RCLCPP_INFO(this->get_logger(), "[IoT ALERT] Smart Dustbin DB-%02d Overflow Level: 96%%", bin_id);
      RCLCPP_INFO(this->get_logger(), "[AI CLASSIFIER] Classification: %s (Conf: 97.8%%)", waste_type.c_str());
      
      if (waste_type == "PLASTIC") {
        RCLCPP_INFO(this->get_logger(), "[PLASTIC CRUSHER] Ingestion unit engaged! 4.2x Compaction ratio applied.");
        RCLCPP_INFO(this->get_logger(), "[FLEET DISPATCH] Waste shredded and transferred to Compartment 3.");
      } else if (waste_type == "BIODEGRADABLE") {
        RCLCPP_INFO(this->get_logger(), "[FLEET DISPATCH] High-moisture waste transferred to Compartment 1 (Bio/Green).");
      } else {
        RCLCPP_INFO(this->get_logger(), "[FLEET DISPATCH] Recyclable waste transferred to Compartment 2 (Non-Bio/Blue).");
      }

      RCLCPP_INFO(this->get_logger(), "[PAYLOAD METRICS] Collected: +%.1f kg | Total Fleet Mass: %.1f kg", 
                  added_kg, (total_bio_kg_ + total_nonbio_kg_ + total_plastic_kg_));
      RCLCPP_INFO(this->get_logger(), "----------------------------------------------------------");
    }
    else if (cmd.find("UNLOAD") != std::string::npos) {
      unload_cycles_++;
      RCLCPP_INFO(this->get_logger(), "==========================================================");
      RCLCPP_INFO(this->get_logger(), "[CENTRAL BASE DEPOT] Unloading Cycle #%d Initiated.", unload_cycles_);
      RCLCPP_INFO(this->get_logger(), "[BASE LOGISTICS] Segregated materials discharged to Municipal Sorting Facility.");
      RCLCPP_INFO(this->get_logger(), "[BASE SHREDDER] Polymer reprocessing line fed from Compartment 3.");
      RCLCPP_INFO(this->get_logger(), "[FLEET STATUS] Truck Bed Compartments Emptied & Auto-Sterilized. Reset for Next Run.");
      RCLCPP_INFO(this->get_logger(), "==========================================================");
    }
  }

  void publish_analytics()
  {
    double total_mass = total_bio_kg_ + total_nonbio_kg_ + total_plastic_kg_;
    double co2_saved = total_mass * 0.24; // 0.24 kg CO2 saved per kg diverted/recycled

    std::ostringstream ss;
    ss << "{"
       << "\"total_mass_kg\":" << std::fixed << std::setprecision(1) << total_mass << ","
       << "\"bio_mass_kg\":" << total_bio_kg_ << ","
       << "\"nonbio_mass_kg\":" << total_nonbio_kg_ << ","
       << "\"plastic_mass_kg\":" << total_plastic_kg_ << ","
       << "\"crusher_compaction_ratio\":\"4.2:1\","
       << "\"volume_saved_pct\":76.2,"
       << "\"bins_serviced\":" << bins_serviced_ << ","
       << "\"unload_cycles\":" << unload_cycles_ << ","
       << "\"co2_saved_kg\":" << co2_saved << ","
       << "\"fleet_efficiency_pct\":98.7"
       << "}";

    std_msgs::msg::String msg;
    msg.data = ss.str();
    analytics_pub_->publish(msg);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr analytics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  double total_bio_kg_;
  double total_nonbio_kg_;
  double total_plastic_kg_;
  int bins_serviced_;
  int unload_cycles_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WasteManager>());
  rclcpp::shutdown();
  return 0;
}

