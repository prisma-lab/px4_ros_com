#include "offboard_control.hpp"

OffboardControl::OffboardControl() : Node("offboard_control"), _state(STOPPED) {
#ifdef ROS_DEFAULT_API
	_offboard_control_mode_publisher =
		this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
	_trajectory_setpoint_publisher =
		this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
	_vehicle_command_publisher =
		this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
#else
	_offboard_control_mode_publisher =
		this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
	_trajectory_setpoint_publisher =
		this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
	_vehicle_command_publisher =
		this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
#endif

	// get common timestamp
	_timesync_sub =
		this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
			[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
				_timestamp.store(msg->timestamp);
			});

	_odom_sub =
		this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/vehicle_odometry/out", 1,
			[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
				_first_odom = true;
				_attitude = matrix::Quaternionf(msg->q.data()[0], msg->q.data()[1], msg->q.data()[2], msg->q.data()[3]);
				_position = matrix::Vector3f(msg->x, msg->y, msg->z);
			});

	_offboard_setpoint_counter = 0;


	_timer = this->create_wall_timer(_timer_period, std::bind(&OffboardControl::timer_callback, this));

	boost::thread key_input_t( &OffboardControl::key_input, this );
}

void OffboardControl::timer_callback() {
	if(!_first_odom)
		return;

	if(!_first_traj){
		firstTraj();
		_first_traj = true;
	}

	if (_offboard_setpoint_counter == 20) {
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 10, 1);
	}

	_trajectory.getNext(_x, _xd, _xdd);

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();
	publish_trajectory_setpoint();

	// stop the counter after reaching 101
	if (_offboard_setpoint_counter < 21) {
		_offboard_setpoint_counter++;
	}

}

void OffboardControl::key_input() {
	bool exit = false;
	std::string cmd;
	while(!exit && rclcpp::ok()) {
		std::cout << "Enter command [arm | go | acc | vel | vely | stop]: \n"; 
		std::cin >> cmd;
		if(cmd == "go") {
			matrix::Vector3f sp;
			double duration;
			float roll, pitch, yaw;
			std::cout << "Enter X coordinate: "; 
			std::cin >> sp(0);
			std::cout << "Enter Y coordinate: "; 
			std::cin >> sp(1);
			std::cout << "Enter Z coordinate: "; 
			std::cin >> sp(2);
			std::cout << "Enter roll: "; 
			std::cin >> roll;
			std::cout << "Enter pitch: "; 
			std::cin >> pitch;
			std::cout << "Enter yaw: "; 
			std::cin >> yaw;
			std::cout << "Enter duration: "; 
			std::cin >> duration;
			startTraj(sp, roll, pitch, yaw, duration);

		}
		else if(cmd == "stop") {
			exit = true;
			rclcpp::shutdown();
		}
		else if(cmd == "arm") {
			this->arm();
		}
		else {
			std::cout << "Unknown command;\n";
		}

	}

}

void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = _timestamp.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = true;
	msg.attitude = false;
	msg.body_rate = false;

	_offboard_control_mode_publisher->publish(msg);
}


void OffboardControl::publish_trajectory_setpoint() const {
	TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();

	msg.x = _x.pose.position.x;
	msg.y = _x.pose.position.y;
	msg.z = _x.pose.position.z;

	msg.vx = _xd.twist.linear.x;
	msg.vy = _xd.twist.linear.y;
	msg.vz = _xd.twist.linear.z;

	msg.acceleration[0] = _xdd.accel.linear.x;
	msg.acceleration[1] = _xdd.accel.linear.y;
	msg.acceleration[2] = _xdd.accel.linear.z;

	matrix::Quaternionf des_att(_x.pose.orientation.w, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z);
	msg.yaw = matrix::Eulerf(des_att).psi();
	msg.yawspeed = 0.0f;

	// msg.x = _current_position_setpoint(0);
	// msg.y = _current_position_setpoint(1);
	// msg.z = _current_position_setpoint(2);
	// msg.vx = _current_velocity_setpoint(0);
	// msg.vy = _current_velocity_setpoint(1);
	// msg.vz = _current_velocity_setpoint(2);
	// msg.acceleration[0] = _current_acceleration_setpoint(0);
	// msg.acceleration[1] = _current_acceleration_setpoint(1);
	// msg.acceleration[2] = _current_acceleration_setpoint(2);
	// msg.yaw = _current_yaw_setpoint; // [-PI:PI]
	// msg.yawspeed = 0.0f;

	_trajectory_setpoint_publisher->publish(msg);
}


void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param3) const {
	VehicleCommand msg{};
	msg.timestamp = _timestamp.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	// std::cout << "Sending command\n";

	_vehicle_command_publisher->publish(msg);
}

void OffboardControl::firstTraj() {
	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t;

	p.pose.position.x = _position(0);
	p.pose.position.y = _position(1);
	p.pose.position.z = _position(2); 

	p.pose.orientation.w = _attitude(0);
	p.pose.orientation.x = _attitude(1);
	p.pose.orientation.y = _attitude(2);
	p.pose.orientation.z = _attitude(3);
	t = 0.0f;

	poses.push_back(p);
	times.push_back(t);
	
	p.pose.orientation.w = 1.0f;
	p.pose.orientation.x = 0.0f;
	p.pose.orientation.y = 0.0f;
	p.pose.orientation.z = 0.0f;
	poses.push_back(p);
	times.push_back(0.1);

	_trajectory.set_waypoints(poses, times);
	_trajectory.compute();
}

void OffboardControl::startTraj(matrix::Vector3f pos, float roll, float pitch, float yaw, double d) {

	while(_trajectory.isReady()) {
		usleep(0.1e6);
	}
	
	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t;

	roll = abs(roll) > 0.5f ? 0.5f * matrix::sign(roll) : roll;
	pitch = abs(pitch) > 0.5f ? 0.5f * matrix::sign(pitch) : roll;

	matrix::Quaternionf att(matrix::Eulerf(roll, pitch, yaw));

	/* */
	p.pose.position.x = _position(0);
	p.pose.position.y = _position(1);
	p.pose.position.z = _position(2); 

	p.pose.orientation.w = _attitude(0);
	p.pose.orientation.x = _attitude(1);
	p.pose.orientation.y = _attitude(2);
	p.pose.orientation.z = _attitude(3);

	t = 0.0;
	
	poses.push_back(p);
	times.push_back(t);
	
	/* */
	p.pose.position.x = pos(0);
	p.pose.position.y = pos(1);
	p.pose.position.z = pos(2); 

	p.pose.orientation.w = att(0);
	p.pose.orientation.x = att(1);
	p.pose.orientation.y = att(2);
	p.pose.orientation.z = att(3);

	t = d;

	poses.push_back(p);
	times.push_back(t);

	_trajectory.set_waypoints(poses, times);

	_trajectory.compute();

}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	auto offboardCtrlPtr = std::make_shared<OffboardControl>();
	
	rclcpp::spin(offboardCtrlPtr);

	rclcpp::shutdown();
	return 0;
}
