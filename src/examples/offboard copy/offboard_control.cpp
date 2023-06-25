#include "offboard_control.hpp"

using namespace matrix;

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
				_mtx.lock();
				_attitude = Quaternionf(msg->q.data()[0], msg->q.data()[1], msg->q.data()[2], msg->q.data()[3]);
				_position = Vector3f(msg->x, msg->y, msg->z);
				// std::cout << "Quaternion:\n" << _attitude.vec() << "\n" << _attitude(0) << std::endl;
				// std::cout << "Roll: " << Eulerf(_attitude).phi() << std::endl;
				// std::cout << "Pitch: " << Eulerf(_attitude).theta() << std::endl;
				// std::cout << "Yaw: " << Eulerf(_attitude).psi() << std::endl;
				_mtx.unlock();
			});

	_offboard_setpoint_counter = 0;


	_timer = this->create_wall_timer(_timer_period, std::bind(&OffboardControl::timer_callback, this));

	boost::thread key_input_t( &OffboardControl::key_input, this );

	// _trajectory.setStartingPoint(_position, Matrix3f(_attitude).eulerAngles(0,1,2).z());
	// _trajectory.setGoal(_position);
	// _trajectory.start();
}

void OffboardControl::timer_callback() {

	if(!_first_odom)
		return;

	if (_offboard_setpoint_counter == 50) {
		_mtx.lock();
		_trajectory.setStartingPoint(_position, Eulerf(_attitude).psi());
		_trajectory.setGoal(_position);
		_mtx.unlock();
		_trajectory.start();
		// std::cout << "Quaternion:\n" << _attitude.w() << "\n" << _attitude.vec() << std::endl;
		// std::cout << "Yaw:\n" << Matrix3f(_attitude).eulerAngles(0,1,2).z() << std::endl;
	}
	if (_offboard_setpoint_counter == 100) {
		// Change to Offboard mode after 10 setpoints
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 10, 1);
	}
	// else if (_offboard_setpoint_counter == 52) {
	// 	// Arm the vehicle
	// 	this->arm();
	// }

	_trajectory.getNextPoint(0.05f, _current_position_setpoint,
		_current_velocity_setpoint, _current_acceleration_setpoint, _current_yaw_setpoint);

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();
	publish_trajectory_setpoint();

	// stop the counter after reaching 51
	if (_offboard_setpoint_counter < 101) {
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
			Vector3f sp;
			float duration;
			std::cout << "Enter X coordinate: "; 
			std::cin >> sp(0);
			std::cout << "Enter Y coordinate: "; 
			std::cin >> sp(1);
			std::cout << "Enter Z coordinate: "; 
			std::cin >> sp(2);
			// std::cout << "Enter duration: "; 
			// std::cin >> duration;
			_mtx.lock();
			_trajectory.setStartingPoint(_position, Eulerf(_attitude).psi());
			_mtx.unlock();
			_trajectory.setGoal(sp);
			// _trajectory.setDuration(duration);

			_trajectory.start();
		}
		else if(cmd == "test") {
			Eulerf rpy_s, rpy_g;
			Quaternionf start;//(rpy_s);
			Quaternionf goal;//(rpy_g);
			float duration;
			std::cout << "Enter starting quaternion:\n"; 
			std::cin >> start(0) >> start(1) >> start(2) >> start(3);
			std::cout << "Enter goal quaternion:\n"; 
			std::cin >> goal(0) >> goal(1) >> goal(2) >> goal(3);
			Quaternionf err;
			Vector3f start_vec(start(1), start(2), start(3));
			Vector3f goal_vec(goal(1), goal(2), goal(3));
			Vector3f err_vec;
			// err(0) = goal(0) * start(0) + goal_vec.dot(start_vec);
			// err_vec = start(0) * goal_vec - goal(0) * start_vec + goal_vec.cross(start_vec*(-1.0f));
			// err(1) = err_vec(0);
			// err(2) = err_vec(1);
			// err(3) = err_vec(2);

			err = goal * start.inversed();
			Quaternionf err1 = start.inversed() * goal;
			
			Eulerf err_eul(err);
			Eulerf err_eul1(err1);
			std::cout << "Error quaternion q1*q2:\n";
			std::cout << err(0) << ",   "<< err(1) << ",   "<< err(2) << ",   "<< err(3) << std::endl;
			std::cout << "Error quaternion q2*q1:\n";
			std::cout << err1(0) << ",   "<< err1(1) << ",   "<< err1(2) << ",   "<< err1(3) << std::endl;
			std::cout << "Error RPY q1*q2:\n";
			std::cout << err_eul(0) << ",   "<< err_eul(1) << ",   "<< err_eul(2) << std::endl;
			std::cout << "Error RPY q2*q1:\n";
			std::cout << err_eul1(0) << ",   "<< err_eul1(1) << ",   "<< err_eul1(2) << std::endl;
		}
		else if(cmd == "stop") {
			exit = true;
			rclcpp::shutdown();
		}
		else if(cmd == "arm") {
			this->arm();
		}
		else if(cmd == "acc") {
			float acc;
			std::cout << "Enter maximum acceleration: "; 
			std::cin >> acc;
			_trajectory.setMaxAcc(acc);
		}
		else if(cmd == "vel") {
			float vel;
			std::cout << "Enter maximum velocity: "; 
			std::cin >> vel;
			_trajectory.setMaxVel(vel);
		}
		else if(cmd == "vely") {
			float vel;
			std::cout << "Enter yaw velocity: "; 
			std::cin >> vel;
			_trajectory.setYawVel(vel);
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
	msg.x = _current_position_setpoint(0);
	msg.y = _current_position_setpoint(1);
	msg.z = _current_position_setpoint(2);
	msg.vx = _current_velocity_setpoint(0);
	msg.vy = _current_velocity_setpoint(1);
	msg.vz = _current_velocity_setpoint(2);
	msg.acceleration[0] = _current_acceleration_setpoint(0);
	msg.acceleration[1] = _current_acceleration_setpoint(1);
	msg.acceleration[2] = _current_acceleration_setpoint(2);
	msg.yaw = _current_yaw_setpoint; // [-PI:PI]
	msg.yawspeed = 0.0f;

	//std::cout << "Position setpoint:     " << _current_position_setpoint(0) << ", " \
			 << _current_position_setpoint(1) << ", " << _current_position_setpoint(2) << std::endl;
	//std::cout << "Velocity setpoint:     " << _current_velocity_setpoint(0) << ", " \
			 << _current_velocity_setpoint(1) << ", " << _current_velocity_setpoint(2) << std::endl;
	//std::cout << "Acceleration setpoint: " << _current_acceleration_setpoint(0) << ", " \
			 << _current_acceleration_setpoint(1) << ", " << _current_acceleration_setpoint(2) << std::endl;

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

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	auto offboardCtrlPtr = std::make_shared<OffboardControl>();
	
	rclcpp::spin(offboardCtrlPtr);

	rclcpp::shutdown();
	return 0;
}
