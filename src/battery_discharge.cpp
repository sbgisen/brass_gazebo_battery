#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"
#include "battery_discharge.hh"
#include <kobuki_msgs/MotorPower.h>
#include "std_msgs/Float64.h"
#include "ROS_debugging.h"

enum power
{
  OFF = 0,
  ON = 1
};

template <typename T>
T max(T x, T y)
{
  return x < y ? y : x;
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin);

BatteryPlugin::BatteryPlugin()
{
  this->c = 0.0;
  this->r = 0.0;
  this->tau = 0.0;

  this->et = 0.0;
  this->e0 = 0.0;
  this->e1 = 0.0;

  this->q0 = 0.0;
  this->q = 0.0;
  this->qt = 0.0;

  this->iraw = 0.0;
  this->ismooth = 0.0;
  this->charging = false;
  this->battery_full = false;

#ifdef BATTERY_DEBUG
  gzdbg << "Constructed BatteryPlugin and initialized parameters. \n";
#endif

  ROS_INFO_STREAM("BRASS CP1 battery is constructed.");
}

BatteryPlugin::~BatteryPlugin()
{
#ifdef BATTERY_DEBUG
  gzdbg << "Destructing BatteryPlugin and removing the ros node. \n";
#endif
  this->rosNode->shutdown();
}

void BatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
#ifdef BATTERY_DEBUG
  gzdbg << "Loading the BatteryPlugin \n";
#endif

  // check if the ros is up!
  if (!ros::isInitialized())
  {
    ROS_INFO_STREAM("Initializing ROS...");
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, _sdf->Get<std::string>("ros_node"), ros::init_options::NoSigintHandler);
  }

  this->model = _model;
  this->world = _model->GetWorld();

#if GAZEBO_MAJOR_VERSION < 8
  this->sim_time_now = this->world->GetSimTime().Double();
#else
  this->sim_time_now = this->world->SimTime().Double();
#endif

  // Create ros node and publish stuff there!
  this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("ros_node")));
  if (this->rosNode->ok())
  {
    ROS_GREEN_STREAM("ROS node is up");
  }

  // Publish a topic for motor power and charge level
  std::string topicNamespace = _sdf->Get<std::string>("topic_namespace");
  this->motor_power_pub = this->rosNode->advertise<kobuki_msgs::MotorPower>(topicNamespace + "/motor_power", 1);
  this->charge_state_pub = this->rosNode->advertise<std_msgs::Float64>(topicNamespace + "/charge_level", 1);
  this->charge_state_mwh_pub = this->rosNode->advertise<std_msgs::Float64>(topicNamespace + "/charge_level_mwh", 1);
  this->charge_current_pub = this->rosNode->advertise<std_msgs::Float64>(topicNamespace + "/charge_current", 1);
  this->battery_voltage_pub = this->rosNode->advertise<std_msgs::Float64>(topicNamespace + "/battery_voltage", 1);
  this->battery_remaining_pub = this->rosNode->advertise<std_msgs::Float64>(topicNamespace + "/battery_remaining", 1);

  this->set_charging_srv =
      this->rosNode->advertiseService(this->model->GetName() + "/set_charging", &BatteryPlugin::SetCharging, this);
  this->set_charging_rate_srv = this->rosNode->advertiseService(this->model->GetName() + "/set_charge_rate",
                                                                &BatteryPlugin::SetChargingRate, this);
  this->set_charge_srv =
      this->rosNode->advertiseService(this->model->GetName() + "/set_charge", &BatteryPlugin::SetCharge, this);
  this->set_coefficients_srv = this->rosNode->advertiseService(this->model->GetName() + "/set_model_coefficients",
                                                               &BatteryPlugin::SetModelCoefficients, this);

  std::string linkName = _sdf->Get<std::string>("link_name");
  this->link = this->model->GetLink(linkName);

  this->e0 = _sdf->Get<double>("constant_coef");
  this->e1 = _sdf->Get<double>("linear_coef");
  this->q0 = _sdf->Get<double>("initial_charge");
  this->qt = _sdf->Get<double>("charge_rate");
  this->c = _sdf->Get<double>("capacity");
  this->r = _sdf->Get<double>("resistance");
  this->tau = _sdf->Get<double>("smooth_current_tau");

  std::string batteryName = _sdf->Get<std::string>("battery_name");

  if (this->link->BatteryCount() > 0)
  {
    // Creates the battery
    this->battery = this->link->Battery(batteryName);
    ROS_GREEN_STREAM("Created a battery");
  }
  else
  {
    ROS_RED_STREAM("There is no battery specification in the link");
  };

  // Specifying a custom update function
  this->battery->SetUpdateFunc(std::bind(&BatteryPlugin::OnUpdateVoltage, this, std::placeholders::_1));

#if GAZEBO_MAJOR_VERSION < 8
  this->sim_time_now = this->world->GetSimTime().Double();
#else
  this->sim_time_now = this->world->SimTime().Double();
#endif

#ifdef BATTERY_DEBUG
  gzdbg << "Loaded the BatteryPlugin at time:" << this->sim_time_now << "\n";
#endif

  ROS_GREEN_STREAM("Plugin is fully loaded.");
}

// This is for in initialization purposes and is called once after Load
// So explanation about Gazebo Init are here:
// http://playerstage.sourceforge.net/doc/Gazebo-manual-0.5-html/plugin_models.html
void BatteryPlugin::Init()
{
  ROS_GREEN_STREAM("Init Battery");
  this->q = this->q0;
  this->charging = false;
}

void BatteryPlugin::Reset()
{
  this->iraw = 0.0;
  this->ismooth = 0.0;
  this->Init();
}

double BatteryPlugin::OnUpdateVoltage(const common::BatteryPtr& _battery)
{
#if GAZEBO_MAJOR_VERSION < 8
  double dt = this->world->GetPhysicsEngine()->GetMaxStepSize();
#else
  double dt = this->world->Physics()->GetMaxStepSize();
#endif
  double totalpower = 0.0;
  double k = dt / this->tau;

  if (fabs(_battery->Voltage()) < 1e-3)
  {
    ROS_GREEN_STREAM("BatteryPlugin::OnUpdateVoltage -> Battery Voltage is too low: " << _battery->Voltage());
    return 0.0;
  }
  for (auto powerLoad : _battery->PowerLoads())
    totalpower += powerLoad.second;

  // current = power(Watts)/Voltage
  this->iraw = totalpower / _battery->Voltage();

  this->ismooth = this->ismooth + k * (this->iraw - this->ismooth);

  if (!this->charging)
  {
    this->q = this->q - GZ_SEC_TO_HOUR(dt * this->ismooth);
  }
  else
  {
    this->q = this->q + GZ_SEC_TO_HOUR(dt * this->qt);
  }

#if GAZEBO_MAJOR_VERSION < 8
  this->sim_time_now = this->world->GetSimTime().Double();
#else
  this->sim_time_now = this->world->SimTime().Double();
#endif

#ifdef BATTERY_DEBUG
  gzdbg << "Current charge:" << this->q << ", at:" << this->sim_time_now << "\n";
#endif
  //    ROS_INFO_STREAM(this->q);

  this->et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

#ifdef BATTERY_DEBUG
  gzdbg << "Current voltage:" << this->et << ", at:" << this->sim_time_now << "\n";
#endif

  // Turn off the motor
  if (this->q <= 0)
  {
#if GAZEBO_MAJOR_VERSION < 8
    this->sim_time_now = this->world->GetSimTime().Double();
#else
    this->sim_time_now = this->world->SimTime().Double();
#endif

#ifdef BATTERY_DEBUG
    gzdbg << "Out of juice at:" << this->sim_time_now << "\n";
#endif
    this->q = 0;
    kobuki_msgs::MotorPower power_msg;
    power_msg.state = power::OFF;
    this->motor_power_pub.publish(power_msg);
  }
  else if (this->q >= this->c)
  {
    this->q = this->c;
    this->battery_full = true;
  }
  else
  {
    this->battery_full = false;
  }

  std_msgs::Float64 charge_msg, charge_msg_mwh;
  charge_msg.data = this->q;
  charge_msg_mwh.data = this->q * 1000 * this->et;

  std_msgs::Float64 charge_cur_msg;
  charge_cur_msg.data = 0.0;
  if (this->charging && !this->battery_full)
  {
    charge_cur_msg.data = this->qt;
  }
  std_msgs::Float64 battery_voltage_msg, battery_remaining_msg;
  battery_voltage_msg.data = _battery->Voltage();
  battery_remaining_msg.data = this->q / this->c;

  this->charge_state_pub.publish(charge_msg);
  this->charge_state_mwh_pub.publish(charge_msg_mwh);
  this->charge_current_pub.publish(charge_cur_msg);
  this->battery_voltage_pub.publish(battery_voltage_msg);
  this->battery_remaining_pub.publish(battery_remaining_msg);
  return et;
}

bool BatteryPlugin::SetCharging(brass_gazebo_battery::SetCharging::Request& req,
                                brass_gazebo_battery::SetCharging::Response& res)
{
  lock.lock();
  this->charging = req.charging;
  if (this->charging)
  {
#ifdef BATTERY_DEBUG
    gzdbg << "Bot is charging"
          << "\n";
#endif
    ROS_GREEN_STREAM("Bot is charging");
  }
  else
  {
#ifdef BATTERY_DEBUG
    gzdbg << "Bot disconnected from the charging station"
          << "\n";
#endif
    ROS_GREEN_STREAM("Bot disconnected from the charging station");
  }
  res.result = true;
  lock.unlock();
  return true;
}

bool BatteryPlugin::SetChargingRate(brass_gazebo_battery::SetChargingRate::Request& req,
                                    brass_gazebo_battery::SetChargingRate::Response& res)
{
  lock.lock();
  this->qt = req.charge_rate;
#ifdef BATTERY_DEBUG
  gzdbg << "Charging rate has been changed to: " << this->qt << "\n";
#endif
  ROS_GREEN_STREAM("Charging rate has been changed to: " << this->qt);
  res.result = true;
  lock.unlock();
  return true;
}

bool BatteryPlugin::SetCharge(brass_gazebo_battery::SetCharge::Request& req,
                              brass_gazebo_battery::SetCharge::Response& res)
{
  lock.lock();
  if (req.charge <= this->c)
  {
    this->q = req.charge;
#ifdef BATTERY_DEBUG
    gzdbg << "Received charge:" << this->q << "\n";
#endif
    ROS_GREEN_STREAM("A new charge is set: " << this->q);
  }
  else
  {
    this->q = this->c;
    ROS_RED_STREAM("The charge cannot be higher than the capacity of the battery");
  }
  res.result = true;
  lock.unlock();
  return true;
}

bool BatteryPlugin::SetModelCoefficients(brass_gazebo_battery::SetCoef::Request& req,
                                         brass_gazebo_battery::SetCoef::Response& res)
{
  lock.lock();
  this->e0 = req.constant_coef;
  this->e1 = req.linear_coef;
#ifdef BATTERY_DEBUG
  gzdbg << "Power model is changed, new coefficients (constant, linear):" << this->e0 << this->e1 << "\n";
#endif
  ROS_GREEN_STREAM("Power model is changed, new coefficients (constant, linear):" << this->e0 << this->e1);
  res.result = true;
  lock.unlock();
  return true;
}
