#include "offboard_control.hpp"

#include <cmath>

OffboardControl::OffboardControl() : Node("offboard_control"), _state(STOPPED) {
#ifdef ROS_DEFAULT_API
	_offboard_control_mode_publisher =
		this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
	_trajectory_setpoint_publisher =
		this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
	_vehicle_command_publisher =
		this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
	_tilting_attitude_setpoint_publisher =
		this->create_publisher<TiltingAttitudeSetpoint>("fmu/tilting_attitude_setpoint/in", 10);
#else
	_offboard_control_mode_publisher =
		this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
	_trajectory_setpoint_publisher =
		this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
	_vehicle_command_publisher =
		this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
	_tilting_attitude_setpoint_publisher =
		this->create_publisher<TiltingAttitudeSetpoint>("fmu/tilting_attitude_setpoint/in");
#endif

	this->declare_parameter<std::vector<double>>("traj_points", {});
	rclcpp::Parameter traj_param;
	_traj_present = this->get_parameter("traj_points", traj_param);
	if(_traj_present) {
		_traj_points = traj_param.as_double_array();
		if(_traj_points.size() % 7 != 0)
			_traj_present = false;
	}


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
				if(isnanf(_position(0)) || 
						isnanf(_position(1)) ||
						isnanf(_position(2))) {
					RCLCPP_WARN(rclcpp::get_logger("OFFBOARD"), "INVALID POSITION: %10.5f, %10.5f, %10.5f",
						_position(0), _position(1), _position(2));
				}
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
		std::cout << "Enter command [arm | go | takeoff | stop]: \n"; 
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
		else if(cmd == "traj") {
			if(!_traj_present) {
				std::cout << "Trajectory not loaded correctly!\n";
				continue;
			}
			for(int i = 0; i<_traj_points.size(); i+=7) {
				matrix::Vector3f sp(_traj_points[i], _traj_points[i+1], _traj_points[i+2]);
				float roll  = _traj_points[i+3];
				float pitch = _traj_points[i+4];
				float yaw   = _traj_points[i+5];
				double duration = _traj_points[i+6];
				startTraj(sp, roll, pitch, yaw, duration);
			}
		}
		else if(cmd == "takeoff") {
			float alt;
			std::cout << "Enter altitude: "; 
			std::cin >> alt;
			takeoffTraj(alt);
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

	TiltingAttitudeSetpoint att_sp{};
	att_sp.timestamp = msg.timestamp;

	att_sp.q_d[0] = des_att(0);
	att_sp.q_d[1] = des_att(1);
	att_sp.q_d[2] = des_att(2);
	att_sp.q_d[3] = des_att(3);
	
	// std::cout << att_sp.q_d[0] << ", ";
	// std::cout << att_sp.q_d[1] << ", ";
	// std::cout << att_sp.q_d[2] << ", ";
	// std::cout << att_sp.q_d[3] << "\n";

	_tilting_attitude_setpoint_publisher->publish(att_sp);


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

	_last_pos_sp = _position;
	_last_att_sp = _attitude;

	p.pose.position.x = _last_pos_sp(0);
	p.pose.position.y = _last_pos_sp(1);
	p.pose.position.z = _last_pos_sp(2); 

	p.pose.orientation.w = _last_att_sp(0);
	p.pose.orientation.x = _last_att_sp(1);
	p.pose.orientation.y = _last_att_sp(2);
	p.pose.orientation.z = _last_att_sp(3);
	t = 0.0f;

	poses.push_back(p);
	times.push_back(t);
	
	poses.push_back(p);
	times.push_back(0.1);

	_trajectory.set_waypoints(poses, times);
	_trajectory.compute();

	// for (int i=0; i<_trajectory._x.size(); i++) {
	// 	std::cout << _trajectory._x[i].pose.orientation.w << ", ";
	// 	std::cout << _trajectory._x[i].pose.orientation.x << ", ";
	// 	std::cout << _trajectory._x[i].pose.orientation.y << ", ";
	// 	std::cout << _trajectory._x[i].pose.orientation.z << "\n";
	// }
}

void OffboardControl::takeoffTraj(float alt) {

	while(_trajectory.isReady()) {
		usleep(0.1e6);
	}

	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t;

	p.pose.position.x = _last_pos_sp(0);
	p.pose.position.y = _last_pos_sp(1);
	p.pose.position.z = _last_pos_sp(2); 

	p.pose.orientation.w = _last_att_sp(0);
	p.pose.orientation.x = _last_att_sp(1);
	p.pose.orientation.y = _last_att_sp(2);
	p.pose.orientation.z = _last_att_sp(3);
	t = 0.0f;

	poses.push_back(p);
	times.push_back(t);
	
	alt = alt > 0 ? -alt : alt;
	p.pose.position.z = alt; 
	poses.push_back(p);
	times.push_back(3.0f * abs(alt));

	_last_pos_sp(2) = alt;

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

	roll = abs(roll) > 0.3f ? 0.3f * matrix::sign(roll) : roll;
	pitch = abs(pitch) > 0.3f ? 0.3f * matrix::sign(pitch) : pitch;
	if(pos(2) > 0.0f)
		pos(2) *= -1;

	matrix::Quaternionf att(matrix::Eulerf(roll, pitch, yaw));

	/* */
	p.pose.position.x = _last_pos_sp(0);
	p.pose.position.y = _last_pos_sp(1);
	p.pose.position.z = _last_pos_sp(2); 

	p.pose.orientation.w = _last_att_sp(0);
	p.pose.orientation.x = _last_att_sp(1);
	p.pose.orientation.y = _last_att_sp(2);
	p.pose.orientation.z = _last_att_sp(3);

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

	_last_pos_sp = pos;
	_last_att_sp = att;

	poses.push_back(p);
	times.push_back(t);

	auto start = steady_clock::now();

	_trajectory.set_waypoints(poses, times);

	_trajectory.compute();

	auto end = steady_clock::now();
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
