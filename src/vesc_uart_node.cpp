// Main program to test UART communication from BBB to VESC
// Last modified on 7/4/2017 by: Ryan Owens
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include "vesc_uart/bldc.h"
// #include "vesc_uart/motortypes.h"
#include <unistd.h> // for usleep
// #include "bldc_interface.h"
// #include "comm_uart.h"

//keyboard message include
#include <keyboard/Key.h>

enum Key
{
  UP        = 273,
  DOWN      = 274,
  RIGHT     = 275,
  LEFT      = 276,
  SPACE     = 32,
  ENTER     = 13,
  BACKSPACE = 8,
  KEY_0     = 48,
  KEY_1     = 49,
  KEY_2     = 50,
  KEY_3     = 51,
  KEY_4     = 52,
  KEY_5     = 53,
  KEY_6     = 54,
  KEY_l     = 108,
  KEY_W     = 119,
  KEY_S     = 115,
};

class VescUart
{
public:
	VescUart(ros::NodeHandle &nh, ros::NodeHandle &pnh);
	~VescUart(){};
	void publishCurrent(); 
	void keyDownCallback(const keyboard::KeyConstPtr &msg);
  	void keyUpCallback(const keyboard::KeyConstPtr &msg);

private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	ros::Publisher current_pub_;
	ros::Subscriber down_sub_, up_sub_;
	std::unique_ptr<BLDC> bldc_;

	float current_ = 0.0;
	float current_increment_ = 0.5;

	bool is_alive_bldc_ = true;
};

VescUart::VescUart(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
{
	std::string serial_port = "/dev/ttyAMA0";
	bool enable_brake; // Enable/Disable braking
	int max_erpm; // Maximum RPM of Motor
	int min_erpm; // Minimum RPM of motor
	float max_amps; // Maximum Current of Motor
	float min_amps; // Minimum Current of Motor
	float max_duty; // Maximum Duty cycle of Motor
	float min_duty; // Minimum Duty cycle of Motor
	float brake_current; // Brake current to use for motor
	// For Analog/Digital Inputs
	float scale_max; // Maximum input value
	float scale_min; // Minimum input value
	// pnh_.param("vesc/serial_port", serial_port, std::string("/dev/ttyAMA0"));
	// pnh_.param("vesc/motor/enable_brake", enable_brake, bool(true));
	// pnh_.param("vesc/motor/max_erpm", max_erpm, int(100000));
	// pnh_.param("vesc/motor/min_erpm", min_erpm, int(900));
	// pnh_.param("vesc/motor/max_amps", max_amps, float(50));
	// pnh_.param("vesc/motor/min_amps", min_amps, float(1));
	// pnh_.param("vesc/motor/max_duty", max_duty, float(0.95));
	// pnh_.param("vesc/motor/min_duty", min_duty, float(0.05));
	// pnh_.param("vesc/motor/brake_current", brake_current, float(18));
	// pnh_.param("vesc/motor/scale_max", scale_max, float(1.0));
	// pnh_.param("vesc/motor/scale_min", scale_min, float(-1.0));

	pnh_.getParam("/vesc/motor/enable_brake", enable_brake);
	pnh_.getParam("/vesc/motor/max_erpm", max_erpm);
	pnh_.getParam("/vesc/motor/min_erpm", min_erpm);
	pnh_.getParam("/vesc/motor/max_amps", max_amps);
	pnh_.getParam("/vesc/motor/min_amps", min_amps);
	pnh_.getParam("/vesc/motor/max_duty", max_duty);
	pnh_.getParam("/vesc/motor/min_duty", min_duty);
	pnh_.getParam("/vesc/motor/brake_current", brake_current);
	pnh_.getParam("/vesc/motor/scale_max", scale_max);
	pnh_.getParam("/vesc/motor/scale_min", scale_min);

	down_sub_ = nh_.subscribe("/keyboard/keydown", 1, &VescUart::keyDownCallback, this);
  	up_sub_ = nh_.subscribe("/keyboard/keyup", 1, &VescUart::keyUpCallback, this);

	current_pub_ = nh_.advertise<std_msgs::Float32>("/bldc_current",10);

	// Motor parameter definitions
	// Change and define new motors as needed
	const Motor_Config motor_config = {
		enable_brake, // enable_brake;
		max_erpm, // Max_Erpm
		min_erpm, // Min_Erpm
		max_amps, // Max_Amps
		min_amps, // Min_Amps
		max_duty, // Max_Duty
		min_duty, // Min_Duty
		brake_current, // Brake_Current
		scale_max, // Scale_Max
		scale_min // Scale_Min
	};

	BLDC::init((char*)serial_port.c_str());
	bldc_.reset(new BLDC(VESC, motor_config));
}

void VescUart::keyDownCallback(const keyboard::KeyConstPtr &msg)
{
    int key_code = msg->code;
    switch (key_code)
    {
	    case Key::KEY_0:
		    current_ = 0.0;
		    ROS_INFO_STREAM("set current = " << current_ << " [A]");
		    break;
		case Key::KEY_1:
		    current_ = 1.0;
		    ROS_INFO_STREAM("set current = " << current_ << " [A]");
		    break;
		case Key::KEY_2:
		    current_ = 2.0;
		    ROS_INFO_STREAM("set current = " << current_ << " [A]");
		    break;
		case Key::KEY_3:
		    current_ = 3.0;
		    ROS_INFO_STREAM("set current = " << current_ << " [A]");
		    break;
		case Key::KEY_4:
		    current_ = 4.0;
		    ROS_INFO_STREAM("set current = " << current_ << " [A]");
		    break;
		case Key::KEY_5:
		    current_ = 5.0;
		    ROS_INFO_STREAM("set current = " << current_ << " [A]");
		    break;
		case Key::KEY_6:
		    current_ = 6.0;
		    ROS_INFO_STREAM("set current = " << current_ << " [A]");
		    break;
		case Key::ENTER:
		    ROS_INFO_STREAM("apply current = " << current_ << " [A]");
			bldc_->set_Current(current_);
			break;
		case Key::SPACE:
		    bldc_->apply_Brake(3);
			BLDC::close();
			is_alive_bldc_ = false;
			break;
		case Key::UP:
			current_ = current_ + current_increment_;
			if (current_ > bldc_->get_Config().Max_Amps)
			{
				current_ = bldc_->get_Config().Max_Amps;
			}
		    ROS_INFO_STREAM("set current = " << current_ << " [A]");
			break;
		case Key::DOWN:
			current_ = current_ - current_increment_;
			if (current_ < bldc_->get_Config().Min_Amps)
			{
				current_ = bldc_->get_Config().Min_Amps;
			}
			else if (current_ < 0)
			{
				current_ = 0.0;
			}
		    ROS_INFO_STREAM("set current = " << current_ << " [A]");
			break;
  }
}

void VescUart::keyUpCallback(const keyboard::KeyConstPtr &msg)
{
}

void VescUart::publishCurrent()
{
	if (is_alive_bldc_)
	{
		int reads = 0;
		while (reads < BLDC::num_Motors())
		{
			if(BLDC::sample_Data())
				reads++;
		}
		std_msgs::Float32 current_msg;
		current_msg.data = bldc_->get_Current();
		ROS_INFO_STREAM("BLDC current = " << current_msg.data << " [A]");
		current_pub_.publish(current_msg);
	}
	else
	{
		ROS_WARN_STREAM("UART communication is closed");
	}
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "vesc_uart_node");
    ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
    ros::Rate rate(10);

	VescUart vesc(nh, pnh);

	while(ros::ok())
	{
		vesc.publishCurrent();
		ros::spinOnce();
		rate.sleep();
	}
    BLDC::close();
    return 0;
}
