/*
* openrobot_control_6omni.cpp
* 
*  Created on: Jan. 07, 2021
*      Author: cdi
*/

#include "openrobot_control_6omni.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include <time.h>

#include <iostream>
#include <fstream>
#include <list>

// Settings
#define VESC_UART_DEV1			"/dev/ttyACM1"
#define VESC_UART_DEV2			"/dev/ttyACM2"
#define VESC_UART_DEV3			"/dev/ttyACM3"
#define NO_OF_UDEV				3				// Max. UART Device number : 3
#define CAN_FORWARD_OFF			0
#define CAN_FORWARD_ON			1
#define BRAKE_CURRENT			10.
#define BRAKE_THRESHOLD			8.	// bigger than (BRAKE_CURRENT/2)

// COMM_SET Types - VESC Original
#define COMM_ALIVE				30
#define COMM_SET_DUTY			5
#define COMM_SET_CURRENT		6
#define COMM_SET_CURRENT_BRAKE	7
#define COMM_SET_RPM			8
#define COMM_SET_POS			9
#define COMM_SET_HANDBRAKE		10

// OpenRobot App
#define TARGET_VESC_ID			255

int tr_on = 0;
int dps_enable = 0;
float x_pos = 0.0;
float y_pos = 0.0;
float v_bx = 0.0;
bool ex_mode  =  false;
bool trajectory = false;

// COMM_SET Types
typedef enum {
	COMM_SET_RELEASE = 100,
	COMM_SET_DPS,
	COMM_SET_DPS_VMAX,
	COMM_SET_DPS_AMAX,
	COMM_SET_GOTO
} COMM_PACKET_ID_OPENROBOT;

// Omniwheel Robot Platform
typedef enum {
	DISABLED = 0,
	DUTY_MODE,
	DPS_MODE
} OMNI_CONTROL_MODE;

// Conversions
#define RAD2DEG         		180.0/M_PI  // radian to deg
#define RPS2DPS					RAD2DEG	


// Uncomment this only when you want to see the below infomations.
//#define PRINT_SENSOR_CORE
// #define PRINT_SENSOR_CUSTOMS

void TeleopInput::keyboardCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	/*
	//ROS_INFO("lin x:%.2f, y:%.2f, z:%.2f", cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->linear.z);
	//ROS_INFO("ang x:%.2f, y:%.2f, z:%.2f", cmd_vel->angular.x, cmd_vel->angular.y, cmd_vel->angular.z);
	dps[0] = cmd_vel->linear.x*2.;//cmd_vel->linear.x*200.;
	speed[0] = cmd_vel->angular.z*1.;
	if(cmd_vel->angular.z < 0) {
		enable.data = false;
		dps[0] = 0.;
		speed[0] = 0.;
	}
	else 
	{
		startTime = ros::Time::now();
		enable.data = true;
	}
	*/
}

void TeleopInput::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	static int joy_cont_mode;
	double joy_cmd_forward, joy_cmd_lateral, joy_cmd_steering, joy_cmd_brake;

	//ROS_INFO("%.3f, %.3f, %.3f, %.3f", joy->axes[0], joy->axes[1], joy->axes[2], joy->axes[3]);
	//ROS_INFO("%d, %d, %d, %d", joy->buttons[0], joy->buttons[1], joy->buttons[2], joy->buttons[3]);	// 0, 1, 2, 3 : X, A, B, Y

	joy_cmd_forward = -(joy->axes[0])*(5.5);
	joy_cmd_lateral = (joy->axes[1])*(1.5);
	joy_cmd_steering = (joy->axes[3])*(3.5);

	vh1_->speed[0] = joy_cmd_forward;
	vh1_->speed[1] = joy_cmd_lateral;
	vh1_->speed[2] = joy_cmd_steering;

	// control enable / disable
	if(joy->buttons[0]==0 && joy->buttons[3]==0 && joy_cont_mode != DISABLED) {
		joy_cont_mode = DISABLED;
		vh1_->enable.data = false;
		tr_on = 0;
		dps_enable = 0;
		ROS_INFO("Control Disabled");
	}
	else if(joy->buttons[0]==1 && joy->buttons[1]==0 && joy->buttons[3]==0 && joy_cont_mode!=DUTY_MODE) {
		joy_cont_mode = DUTY_MODE;
		vh1_->custom_cmd_type[0] = vh1_->custom_cmd_type[1] = vh1_->custom_cmd_type[2] = vh1_->custom_cmd_type[3] = COMM_SET_DUTY;
		vh1_->enable.data = true;
		tr_on = 0;
		dps_enable = 0;
		ROS_INFO("Duty Control Mode On");
	}
	else if(joy->buttons[0]==1 && joy->buttons[1]==1 && joy->buttons[3]==0 && joy_cont_mode!=DPS_MODE) {
		joy_cont_mode = DPS_MODE;
		vh1_->custom_cmd_type[0] = vh1_->custom_cmd_type[1] = vh1_->custom_cmd_type[2] = vh1_->custom_cmd_type[3] = COMM_SET_DPS;
		vh1_->enable.data = true;
		tr_on = 0;
		dps_enable = 1;
		ROS_INFO("DPS Control Mode On");
	}
	else if(joy->buttons[0]==0 && joy->buttons[1]==0 && joy->buttons[3]==1) {
		joy_cont_mode = DPS_MODE;
		vh1_->custom_cmd_type[0] = vh1_->custom_cmd_type[1] = vh1_->custom_cmd_type[2] = vh1_->custom_cmd_type[3] = COMM_SET_DPS;
		vh1_->enable.data = true;
		tr_on = 1;
		dps_enable = 0;
		ROS_INFO("Trajectory Mode On");
	}

}

void TrapezoidalVelProfile::GenProfile(float v_ref, float *vout)
{
	float da = 0;
	float dv = 0;

	// Profile Deg
	if(v_ref == *vout) {
		dv = 0;
	}
	else {
		da = (v_ref - *vout)/dt;
		if(fabs(da) >= (double)Amax) {
			if(da>0) da = Amax;
			else 	 da = -Amax;
		}
	}
	dv = da*dt;
	*vout += dv;

	//ROS_INFO("dv:%.2f", dv);
}

void TeleopVesc::customsCallback(const openrobot_vesc_msgs::VescGetCustomApp::ConstPtr& custom_rx_msg)
{
#ifdef PRINT_SENSOR_CUSTOMS
	ROS_INFO("##########################################################");
	ROS_INFO("header:%6.4f", custom_rx_msg->header.stamp.toSec());
	ROS_INFO("vesc dev number:%d", custom_rx_msg->can_devs_num);

	// CAN connected VESC
	for(int i=0; i<this->NO_VESC; i++) {
		ROS_INFO("---------< index: %d, can id : %d >---------", i, custom_rx_msg->can_id[i]);
		//ROS_INFO("voltage input:%.1f V", custom_rx_msg->voltage_input[i]);
		//ROS_INFO("temperature pcb:%.1f C", custom_rx_msg->temperature_pcb[i]);
		ROS_INFO("temperature motor:%.1f C", custom_rx_msg->temperature_motor[i]);
		ROS_INFO("current motor:%.2f A", custom_rx_msg->current_motor[i]);
		//ROS_INFO("current input:%.2f A", custom_rx_msg->current_input[i]);
		//ROS_INFO("watt hours:%.4f Wh", custom_rx_msg->watt_hours[i]);
		//ROS_INFO("watt hours charged:%.4f Wh", custom_rx_msg->watt_hours_charged[i]);
		//ROS_INFO("duty:%.3f", custom_rx_msg->duty_cycle[i]);
		ROS_INFO("accum. deg now:%.2f deg", custom_rx_msg->accum_deg_now[i]);
		ROS_INFO("dps now:%.2f deg/sec", custom_rx_msg->diff_deg_now[i]);	// diff_deg_now(differential degree in 0.1ms)
	}
#endif
}

void TeleopVesc::stateCallback(const openrobot_vesc_msgs::VescStateStamped::ConstPtr& state_msg)
{
#ifdef PRINT_SENSOR_CORE
	ROS_INFO("------------------------------------------");
	ROS_INFO("header:%6.4f", state_msg->header.stamp.toSec());
	ROS_INFO("voltage input:%.2f V", state_msg->state.voltage_input);
	ROS_INFO("temperature pcb:%.2f C", state_msg->state.temperature_pcb);
	ROS_INFO("current motor:%.2f A", state_msg->state.current_motor);
	ROS_INFO("current input:%.2f A", state_msg->state.current_input);
	ROS_INFO("erpm:%.2f", state_msg->state.speed);
	ROS_INFO("duty:%.2f", state_msg->state.duty_cycle);
	ROS_INFO("amp_hours:%.2f", state_msg->state.charge_drawn);
	ROS_INFO("amp_hours_charged:%.2f", state_msg->state.charge_regen);
	ROS_INFO("watt_hours:%.2f", state_msg->state.energy_drawn);
	ROS_INFO("watt_hours_charged:%.2f", state_msg->state.energy_regen);
	ROS_INFO("tachometer:%.2f", state_msg->state.displacement);
	ROS_INFO("tachometer_abs:%.2f", state_msg->state.distance_traveled);
	ROS_INFO("fault code:%d", state_msg->state.fault_code);
	ROS_INFO("pid_pos_now:%.2f", state_msg->state.pid_pos_now);
	ROS_INFO("controller_id:%d", state_msg->state.controller_id);
#endif
}

void TeleopVesc::setCmdMsg(double data, int send_can, int can_id)
{
	cmd_msg.data = data;
	cmd_msg.send_can = send_can;
	cmd_msg.can_id = can_id;
}

void TeleopVesc::setCustomMsg(int can_id, int send_can, int cmd_type, double data)
{
	//
	custom_tx_msg.id_set.push_back(can_id);
	custom_tx_msg.can_forward_set.push_back(send_can);
	custom_tx_msg.comm_set.push_back(cmd_type);
	custom_tx_msg.value_set.push_back(data);
}

void TeleopVesc::requestCustoms()
{
	std_msgs::Bool msg;
	msg.data = true;
	vesc_cmd_get_customs.publish(msg);
}

void TeleopVesc::setCustomOut()
{
	int num_of_id = 0;
	int can_forw = 0;

	// Clear Custom Message
	custom_tx_msg.id_set.clear();
	custom_tx_msg.can_forward_set.clear();
	custom_tx_msg.comm_set.clear();
	custom_tx_msg.value_set.clear();

	// Custom Command
	for(int i=0; i<this->NO_VESC; i++) {
		if(i==0) can_forw = CAN_FORWARD_OFF;
		else     can_forw = CAN_FORWARD_ON;
		setCustomMsg(controller_id[i], can_forw, custom_cmd_type[i], custom_cmd_value[i]);
		num_of_id++;
	}
	custom_tx_msg.num_of_id = num_of_id;
	custom_tx_msg.data_bytes = 2 + 6*num_of_id;
	vesc_cmd_set_customs.publish(custom_tx_msg);
}

void TeleopVesc::setCurrentOut()
{
	int can_forw = 0;

	if(enable.data)
	{
		if(this->port_name==VESC_UART_DEV1)
		{
			// current
			for(int i=0; i<=1; i++) {
				if(i==0) can_forw = CAN_FORWARD_OFF;
				else     can_forw = CAN_FORWARD_ON;
				setCmdMsg(this->current[i], can_forw, i);
				this->vesc_cmd_current.publish(cmd_msg);
			}
		}
	}
}

void TeleopVesc::setBrakeOut()
{
	int can_forw = 0;

	if(enable.data)
	{
		// current
		for(int i=0; i<=1; i++) {
			if(i==0) can_forw = CAN_FORWARD_OFF;
			else     can_forw = CAN_FORWARD_ON;
			setCmdMsg(brake[i], can_forw, i);
			vesc_cmd_brake.publish(cmd_msg);
		}
	}
}

void TeleopVesc::setDutyCycleOut()
{
	int can_forw = 0;

	if(this->enable.data)
	{
		if(this->port_name==VESC_UART_DEV1)
		{
			// duty
			for(int i=0; i<=this->NO_VESC; i++) {
				if(i==0) can_forw = CAN_FORWARD_OFF;
				else     can_forw = CAN_FORWARD_ON;
				setCmdMsg(this->duty[i], can_forw, controller_id[i]);
				this->vesc_cmd_duty.publish(cmd_msg);
				}
		}
		/*
		else if(this->port_name=="/dev/ttyVESC2")
		{
		// duty
		setCmdMsg(this->duty[0], 0, 0);
		this->vesc_cmd_duty.publish(cmd_msg);
		}*/
	}
}

void TeleopVesc::setSpeedOut()
{
	int can_forw = 0;

	if(enable.data)
	{
		// speed
		for(int i=0; i<=1; i++) {
			if(i==0) can_forw = CAN_FORWARD_OFF;
			else     can_forw = CAN_FORWARD_ON;
			setCmdMsg(speed[i], can_forw, i);
			vesc_cmd_speed.publish(cmd_msg);
		}
	}
}

void TeleopVesc::setPositionOut()
{
	int can_forw = 0;

	if(enable.data)
	{
		// position
		for(int i=0; i<=1; i++) {
			if(i==0) can_forw = CAN_FORWARD_OFF;
			else     can_forw = CAN_FORWARD_ON;
			setCmdMsg(enc_deg[i], can_forw, i);
			vesc_cmd_position.publish(cmd_msg);
		}
	}
}

double duty_limit(double input_duty)
{
	double duty_limit = 0.95;
	double output_duty;

	if(input_duty>=duty_limit)	output_duty = duty_limit;
	else if(input_duty<=-duty_limit)	output_duty = -duty_limit;
	else output_duty = input_duty;

	return output_duty;
}

void mecanum_robot_jacobian_duty(double vx, double vy, double wz, double *duty1, double *duty2, double *duty3, double *duty4)
{
	// robot parameter
	static double radius_wheel = 75.2/1000.;
	static double alpha = 0.48225;	// alpha = L+l
	static double VEL2DUTY = 0.014;	//0.02;
	static int 	  MOTOR_DIR[4] = {1, 1, 1, 1};
	double u1, u2, u3, u4;

	u1 = MOTOR_DIR[0]*(1.0*vx - 1.0*vy - alpha*wz)/radius_wheel;
	u2 = MOTOR_DIR[1]*(1.0*vx + 1.0*vy - alpha*wz)/radius_wheel;
	u3 = MOTOR_DIR[2]*(1.0*vx - 1.0*vy + alpha*wz)/radius_wheel;
	u4 = MOTOR_DIR[3]*(1.0*vx + 1.0*vy + alpha*wz)/radius_wheel;

	*duty1 = duty_limit(u1*VEL2DUTY);
	*duty2 = duty_limit(u2*VEL2DUTY);
	*duty3 = duty_limit(u3*VEL2DUTY);
	*duty4 = duty_limit(u4*VEL2DUTY);
}

void mecanum_robot_jacobian_dps(double vx, double vy, double wz, double *dps1, double *dps2, double *dps3, double *dps4)
{
	// robot parameter
	static double radius_wheel = (71.0/1000.);
	static double alpha = 0.48225;	// alpha = L+l
	static double VEL2DPS = 1./radius_wheel*RAD2DEG;
	static int 	  MOTOR_DIR[4] = {-1, 1, 1, 1};
	double u1, u2, u3, u4;

	u1 = MOTOR_DIR[0]*(1.0*vx - 1.0*vy - alpha*wz)/radius_wheel;
	u2 = MOTOR_DIR[1]*(1.0*vx + 1.0*vy - alpha*wz)/radius_wheel;
	u3 = MOTOR_DIR[2]*(1.0*vx - 1.0*vy + alpha*wz)/radius_wheel;
	u4 = MOTOR_DIR[3]*(1.0*vx + 1.0*vy + alpha*wz)/radius_wheel;

	*dps1 = u1*VEL2DPS*(1.26/1.24);  // 0.5/0.464
	*dps2 = u2*VEL2DPS*(1.26/1.24);
	*dps3 = u3*VEL2DPS*(1.26/1.24);
	*dps4 = u4*VEL2DPS*(1.26/1.24);


	ROS_INFO("u1 speed = %f", *dps1 / 14 * 3.141592 / 180);
	ROS_INFO("u2 speed = %f", *dps2 / 14);
	ROS_INFO("u3 speed = %f", *dps3 / 14);
	ROS_INFO("u4 speed = %f", *dps4 / 14);
	ROS_INFO("###########################");
}

void odometry_callback(const std_msgs::Float64MultiArray::ConstPtr& odometry_msg)
{
	// ROS_INFO("%0.10f", odometry_msg->data[0]);
	// ROS_INFO("%0.10f", odometry_msg->data[1]);

	x_pos = odometry_msg->data[0];
	y_pos = odometry_msg->data[1];
}

void robot_velocity_callback(const std_msgs::Float64MultiArray::ConstPtr& odometry_msg)
{
	// ROS_INFO("%0.10f", odometry_msg->data[0]);
	// ROS_INFO("%0.10f", odometry_msg->data[1]);

	v_bx = odometry_msg->data[0];
	ROS_INFO("v_bx = %f", v_bx);
}	


/*
* Main Function
* 
*/
int main(int argc, char **argv)            
{
	ros::init(argc, argv, "vesc_control_node");

	// loop freq
	//   int rate_hz = 500;	//hz
	int rate_hz = 200;	//hz

	ros::NodeHandle nh("~");
	ros::NodeHandle k;
	ros::NodeHandle rv;

	std::string port;
	int no_vesc;
	TeleopVesc *teleop_vesc[NO_OF_UDEV];	// TeleopVesc Class	
	// TeleopVesc Class
	for(int i=0; i<NO_OF_UDEV; i++)
	{
		std::string p = "port" + std::to_string(i);
		std::string v = "no_vesc" + std::to_string(i);

		if(nh.getParam(p, port) && nh.param(v, no_vesc, 1)) {
			teleop_vesc[i] = new TeleopVesc(no_vesc, port);
			ROS_INFO("VESC Driver %d activated at port:%s, number of vesc:%d", i, port.c_str(), no_vesc);
		}
	}

	// TeleInput Class	
	TeleopInput tele_input(teleop_vesc[0], teleop_vesc[1], teleop_vesc[2]);

	// Velocity Profile
	float amax = 2.0;	// m/s^2
	static float vout_x, vout_y, vout_z;
	TrapezoidalVelProfile v_prof_x(2.0*amax, 1./rate_hz);
	TrapezoidalVelProfile v_prof_y(1.0*amax, 1./rate_hz);
	TrapezoidalVelProfile v_prof_z(4.*amax, 1./rate_hz);

	// ROS Loop
	uint32_t cnt_lp = 0;
	ros::Rate loop_rate(rate_hz); //Hz
	ROS_INFO("Start Tele-operation");
	teleop_vesc[0]->startTime = ros::Time::now();

	// set VESC can id here
	teleop_vesc[0]->controller_id[0] = TARGET_VESC_ID;//0;// Set CAN ID as TARGET_VESC_ID only at USB connected VESC
	teleop_vesc[0]->controller_id[1] = 2;
	teleop_vesc[0]->controller_id[2] = 3;
	teleop_vesc[0]->controller_id[3] = 4;

	bool traj1 = false;
	bool traj2 = false;
	bool traj3 = false;
	bool traj4 = false;

	double init_x_position = 0.0;
	double init_y_position = 0.0;
	double target_x_position = 0.0;
	double target_y_position = 0.0;
	double ros_info_x = 0.0;
	double ros_info_y = 0.0;
	double dt = 1./rate_hz;

	double time = 0.0;
	double count_path = 5.0; // same path 5 repeat
	double target_distance = 3.0;
	
	/////// robot speed command ///////
	double robot_speed = 0.5;
	///////////////////////////////////
	
	ros::Publisher pub = k.advertise<std_msgs::Int32>("/odometry/path_count", 10);
	ros::Subscriber sub = nh.subscribe("/odometry/wheel_velocity", 10, odometry_callback);
	ros::Subscriber sub1 = nh.subscribe("/odometry/robot_velocity", 10, robot_velocity_callback);

	std_msgs::Int32 msg;

	while (ros::ok())
	{ 
		msg.data = time;
		
		v_prof_x.GenProfile(teleop_vesc[0]->speed[0], &vout_x);
		v_prof_y.GenProfile(teleop_vesc[0]->speed[1], &vout_y);
		v_prof_z.GenProfile(teleop_vesc[0]->speed[2], &vout_z);

		if(dps_enable == 0 && teleop_vesc[0]->custom_cmd_type[0] == COMM_SET_DUTY) {
			mecanum_robot_jacobian_duty(vout_x, vout_y, vout_z,
									&(teleop_vesc[0]->custom_cmd_value[0]), &(teleop_vesc[0]->custom_cmd_value[1]), 
									&(teleop_vesc[0]->custom_cmd_value[2]), &(teleop_vesc[0]->custom_cmd_value[3]));
		}
		else if(tr_on == 1 && dps_enable == 0) {

			double x_velocity = 0.25;  // 0.5
			double y_velocity = 0.0;
			double z_velocity = 0.0;

			if (traj1 == false && traj2 == false && traj3 == false && traj4 == false && float(time) < count_path){            // path 1
				
				target_x_position = init_x_position + x_velocity * dt;
		
				init_x_position = target_x_position;

				ros_info_x = target_x_position;
				ros_info_y = target_y_position;
				mecanum_robot_jacobian_dps(x_velocity, y_velocity, z_velocity,
									&(teleop_vesc[0]->custom_cmd_value[0]), &(teleop_vesc[0]->custom_cmd_value[1]), 
									&(teleop_vesc[0]->custom_cmd_value[2]), &(teleop_vesc[0]->custom_cmd_value[3]));

				ROS_INFO("x distance = %f", ros_info_x);
				ROS_INFO("y distance = %f", ros_info_y);
				if (float(ros_info_x) == target_distance) {
					traj1 = true;
				}
			}
			else if (traj1 == true && traj2 == false && traj3 == false && traj4 == false && float(time) < count_path){              // path2
				x_velocity = 0.0;
				y_velocity = -0.25; // cw : 0.5, ccw : -0.5

				target_y_position = -init_y_position - y_velocity * dt;

				init_y_position = -target_y_position;

				ros_info_x = target_x_position;
				ros_info_y = target_y_position;
				mecanum_robot_jacobian_dps(x_velocity, -y_velocity, z_velocity,
									&(teleop_vesc[0]->custom_cmd_value[0]), &(teleop_vesc[0]->custom_cmd_value[1]), 
									&(teleop_vesc[0]->custom_cmd_value[2]), &(teleop_vesc[0]->custom_cmd_value[3]));
				ROS_INFO("x distance = %f", ros_info_x);
				ROS_INFO("y distance = %f", ros_info_y);
				if (float(ros_info_y) == target_distance){ // -target_distance
					traj2 = true;
				}
			}
			else if (traj1 == true && traj2 == true && traj3 == false && traj4 == false && float(time) < count_path){             // path3
				x_velocity = -0.25; // cw : 0.5, ccw : - 0.5
				y_velocity = 0;

				target_x_position = init_x_position + x_velocity* dt;

				init_x_position = target_x_position;

				ros_info_x = target_x_position;
				ros_info_y = target_y_position;

				mecanum_robot_jacobian_dps(x_velocity, y_velocity, z_velocity,
									&(teleop_vesc[0]->custom_cmd_value[0]), &(teleop_vesc[0]->custom_cmd_value[1]), 
									&(teleop_vesc[0]->custom_cmd_value[2]), &(teleop_vesc[0]->custom_cmd_value[3]));
				ROS_INFO("x distance = %f", ros_info_x);
				ROS_INFO("y distance = %f", ros_info_y);
				// ROS_INFO("go go go");
				if (float(ros_info_x) <= 0.0) {
					traj3 = true;
					// ROS_INFO("stop");
				}
			}
			else if (traj1 == true && traj2 == true && traj3 == true && traj4 == false && float(time) < count_path){
				x_velocity = 0;
				y_velocity = 0.25; // cw : -0.5, ccw : 0.5

				init_y_position = ros_info_y;

				target_y_position = init_y_position - y_velocity * dt;

				init_y_position = target_y_position;

				ros_info_x = target_x_position;
				ros_info_y = target_y_position;

				mecanum_robot_jacobian_dps(x_velocity, -y_velocity, z_velocity,
									&(teleop_vesc[0]->custom_cmd_value[0]), &(teleop_vesc[0]->custom_cmd_value[1]), 
									&(teleop_vesc[0]->custom_cmd_value[2]), &(teleop_vesc[0]->custom_cmd_value[3]));
				ROS_INFO("x distance = %f", ros_info_x);
				ROS_INFO("y distance = %f", ros_info_y);

				if (float(ros_info_y) <= 0.0){ 
					traj4 = true;
				}
			}
			else if (traj1 == true && traj2 == true && traj3 == true && traj4 == true && float(time) < count_path){
				x_velocity = 0;
				y_velocity = 0;

				ros_info_x = target_x_position;
				ros_info_y = target_y_position;

				mecanum_robot_jacobian_dps(x_velocity, y_velocity, z_velocity,
									&(teleop_vesc[0]->custom_cmd_value[0]), &(teleop_vesc[0]->custom_cmd_value[1]), 
									&(teleop_vesc[0]->custom_cmd_value[2]), &(teleop_vesc[0]->custom_cmd_value[3]));
				ROS_INFO("x distance = %f", ros_info_x);
				ROS_INFO("y distance = %f", ros_info_y);
				
				time +=1.0;
		
				traj1 = false;
				traj2 = false;
				traj3 = false;
				traj4 = false;
				init_x_position = 0.0;
				init_y_position = 0.0;
				target_x_position = 0.0;
				target_y_position = 0.0;
				ros_info_x = 0.0;
				ros_info_y = 0.0;
				
				ROS_INFO("TIME = %f, 00000", time);


			}
			else if (traj1 == false && traj2 == false && traj3 == false && traj4 == false && float(time)>= count_path){
				x_velocity = 0;
				y_velocity = 0;

				ros_info_x = target_x_position;
				ros_info_y = target_y_position;

				mecanum_robot_jacobian_dps(x_velocity, y_velocity, z_velocity,
									&(teleop_vesc[0]->custom_cmd_value[0]), &(teleop_vesc[0]->custom_cmd_value[1]), 
									&(teleop_vesc[0]->custom_cmd_value[2]), &(teleop_vesc[0]->custom_cmd_value[3]));
				ROS_INFO("x distance = %f", ros_info_x);
				ROS_INFO("y distance = %f", ros_info_y);

				ROS_INFO("TIME = %f, 1111", time);
			}

		}
		else if(tr_on == 0 && dps_enable == 1 && teleop_vesc[0]->custom_cmd_type[0] == COMM_SET_DPS) {
			mecanum_robot_jacobian_dps(vout_x, vout_y, vout_z,
									&(teleop_vesc[0]->custom_cmd_value[0]), &(teleop_vesc[0]->custom_cmd_value[1]), 
									&(teleop_vesc[0]->custom_cmd_value[2]), &(teleop_vesc[0]->custom_cmd_value[3]));
		}

		if(teleop_vesc[0]->enable.data == true) teleop_vesc[0]->setCustomOut();

		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
