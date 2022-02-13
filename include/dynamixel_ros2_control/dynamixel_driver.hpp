#pragma once
#include <stddef.h>

#include <bitset>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <functional>
#include <map>
#include <memory>
#include <vector>
#include <limits>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "iostream"
#include "servo_register_definitions.h"

namespace dynamixel {

class Driver {
   public:
	typedef std::function<void(const std::string& joint_name)> hw_error_callback;
   private:
   	class Motor;
	struct GroupSyncReader {
		GroupSyncRead syncRead;
		std::vector<std::function<void(GroupSyncRead&)>> read_functions;
		std::vector<Motor*> motors;
		GroupSyncReader(
			std::unique_ptr<PortHandler>& port,
			std::unique_ptr<PacketHandler>& ph,
			uint16_t& start_address,
			uint16_t& data_length) : syncRead(port.get(), ph.get(), start_address, data_length){};
		GroupSyncReader(
			std::unique_ptr<PortHandler>& port,
			std::unique_ptr<PacketHandler>& ph,
			field& f) : GroupSyncReader(port, ph, f.address, f.data_length){};
	};

	class Motor {
	   public:
		const motor_id id;
		const std::string name;
		const model_info* model;

		bool led;
		bool rebooting;
		bool rebooted;
		
		double goal_position;

		double position;

		//double goal_velocity;
		double velocity;
		double effort;
		bool torque;

		uint8_t hw_error;

		double p_gain, i_gain, d_gain;
		double p_gain_target, i_gain_target, d_gain_target;

		std::vector<std::function<void()>> after_reboot_functions;

		typedef std::function<void(dynamixel::hardware_status)> hw_error_callback;
		//hw_error_callback error_callback;

		void get_command(const dynamixel::field_name& command, field& f) const {
			try {
				f = model->controll_table->at(command);
			} catch (std::exception& e) {
				throw command_type_not_implemented(command);
			}
		};

		double velocity_from_raw(uint32_t raw_position) const {
			const double RPM2RADPERSEC = 0.104719755f;
			//FIXME this calculation is not always correct
			//In the official Dynamixel-Toolbox there is a function which handles all the edge cases
			//in which negative directions are marked by the 10th bit.
			return double(static_cast<int32_t>(raw_position)) * model->rpm_factor * RPM2RADPERSEC;
		}
		
		double position_from_raw(uint32_t raw_position) const {
			return double(double(double(raw_position) - double(model->min_value)) / double(model->max_value - model->min_value)) * double(model->max_radian - model->min_radian) + double(model->min_radian);
		};

		uint32_t raw_from_position(double position) const {
			return uint64_t((position - model->min_radian) / double(model->max_radian - model->min_radian) * double(model->max_value - model->min_value)) + double(model->min_value);
		};

		void set_position_from_raw(uint32_t raw) { position = position_from_raw(raw); }
		void set_velocity_from_raw(uint32_t raw) { velocity = velocity_from_raw(raw); }
		void set_effort_from_raw(uint32_t raw) { effort = double(static_cast<int16_t>(raw))*0.1; }

		Motor(const motor_id p_id, std::string p_name, const model_t type) : 
				id(p_id), 
				name(p_name), 
				rebooting(true), 
				rebooted(false), 
				goal_position(std::numeric_limits<double>::quiet_NaN()), 
				position(std::numeric_limits<double>::quiet_NaN()), 
				p_gain(std::numeric_limits<double>::quiet_NaN()), 
				i_gain(std::numeric_limits<double>::quiet_NaN()), 
				d_gain(std::numeric_limits<double>::quiet_NaN()) 
		{
			try {
				model = &model_infos.at(type);
			} catch (const std::exception &e) {
				throw motor_type_not_implemented(type);
			}
		};

		uint32_t get_position_raw() const {
			return raw_from_position(position);
		}

		void setup_sync_readers(
			std::map<field, std::unique_ptr<GroupSyncReader>>& syncReaders,
			std::unique_ptr<PortHandler>& portHandler,
			std::unique_ptr<PacketHandler>& packetHandler, 
			hw_error_callback error_callback = [](auto){}) {
			//Torque enabled
			setup_sync_reader(
				syncReaders, portHandler, packetHandler,
				command::Torque_Enable,
				[this](uint32_t data) {
					this->torque = bool(data);
					if (this->torque == false) {
						goal_position = std::numeric_limits<double>::quiet_NaN();
					}
				});

			//Present Position
			setup_sync_reader(
				syncReaders, portHandler, packetHandler,
				command::Present_Position,
				[this](uint32_t data) {
					this->set_position_from_raw(data);
				});

			//Present Velocity
			setup_sync_reader(
				syncReaders, portHandler, packetHandler,
				command::Present_Velocity,
				[this](uint32_t data) {
					this->set_velocity_from_raw(data);
				});

			//Present Effort (Load if available else Current)
			try {
				setup_sync_reader(
					syncReaders, portHandler, packetHandler,
					command::Present_Load,
					[this](uint32_t data) {
						this->set_effort_from_raw(data);
					});
			} catch (command_type_not_implemented& e) {
				setup_sync_reader(
					syncReaders, portHandler, packetHandler,
					command::Present_Current,
					[this](uint32_t data) {
						this->set_effort_from_raw(data);
					});
			}

			//Hardware Error Status
			if(error_callback) {
				setup_sync_reader(
					syncReaders, portHandler, packetHandler,
					command::Hardware_Error_Status,
					[this, error_callback](uint32_t data) {
						uint8_t error = uint8_t(data);
						this->hw_error = error;
						if (error != 0) {
							//std::bitset<8> error(data);	
							if (error & hardware_status::Input_Voltage_Error) {
								error_callback(hardware_status::Input_Voltage_Error);
							}
							if (error & hardware_status::OverHeating_Error) {
								error_callback(hardware_status::OverHeating_Error);
							}
							if (error & hardware_status::Motor_Encoder_Error) {
								error_callback(hardware_status::Motor_Encoder_Error);
							}
							if (error & hardware_status::Electrical_Shock_Error) {
								error_callback(hardware_status::Electrical_Shock_Error);
							}
							if (error & hardware_status::Overload_Error) {
								error_callback(hardware_status::Overload_Error);
							}
							//https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#hardware-error-status
						} else {
							this->rebooting = false;
							this->rebooted = true;
						}
					});
			}
		}

		void setup_init_sync_readers(
			std::map<field, std::unique_ptr<GroupSyncReader>>& initSyncReaders,
			std::unique_ptr<PortHandler>& portHandler,
			std::unique_ptr<PacketHandler>& packetHandler) 
		{
			//Load PID gains if available
			try {
				setup_sync_reader(
					initSyncReaders, portHandler, packetHandler,
					command::Position_P_Gain,
					[this](uint32_t raw) {
						this->p_gain = double(static_cast<int16_t>(raw));
					});
				setup_sync_reader(
					initSyncReaders, portHandler, packetHandler,
					command::Position_I_Gain,
					[this](uint32_t raw) {
						this->i_gain = double(static_cast<int16_t>(raw));
					});
				setup_sync_reader(
					initSyncReaders, portHandler, packetHandler,
					command::Position_D_Gain,
					[this](uint32_t raw) {
						this->d_gain = double(static_cast<int16_t>(raw));
					});
			} catch (command_type_not_implemented& e) {}
		}

		void setup_sync_reader(
			std::map<field, std::unique_ptr<GroupSyncReader>>& syncReaders,
			std::unique_ptr<PortHandler>& portHandler,
			std::unique_ptr<PacketHandler>& packetHandler,
			const dynamixel::field_name& field_name,
			std::function<void(uint32_t)> callback) {
			field sr_field;
			this->get_command(field_name, sr_field);
			auto sync_reader_it = syncReaders.find(sr_field);
			//if there is no reader for the command field create it
			if (sync_reader_it == syncReaders.end()) {
				sync_reader_it = syncReaders.insert(
					sync_reader_it,
					{sr_field,
					 std::make_unique<GroupSyncReader>(portHandler, packetHandler, sr_field)});
			}
			sync_reader_it->second->syncRead.addParam(this->id);
			sync_reader_it->second->motors.push_back(this);
			sync_reader_it->second->read_functions.push_back(
				[this, sr_field, callback](GroupSyncRead& read) {
					if (read.isAvailable(this->id, sr_field.address, sr_field.data_length)) {
						uint32_t data = read.getData(this->id, sr_field.address, sr_field.data_length);
						callback(data);
					} else {
						// No data available
					}
				});
		}
		typedef std::shared_ptr<Motor> Ptr;
	};

	std::map<std::string, Motor::Ptr> motors;
	std::unique_ptr<PortHandler> portHandler;
	std::unique_ptr<PacketHandler> packetHandler;

	std::map<field, std::unique_ptr<GroupSyncReader>> initSyncReaders;
	std::map<field, std::unique_ptr<GroupSyncReader>> syncReaders;
	std::map<field, std::unique_ptr<GroupSyncWrite>> syncWrites;

	model_t get_motor_type(motor_id id) const {
		uint16_t model_number = 0;
		uint8_t dxl_error = 0;
		int dxl_comm_result = packetHandler->ping(portHandler.get(), id, &model_number, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) {
			//error<<"Error pinging "<<m.joint_name<<" on id "<<std::to_string(id)<<":\n"<<packetHandler_->getRxPacketError(dxl_error);
			//std::cerr << "Error pinging motor (id: " << std::to_string(id) << ") :\n"
			//		  << packetHandler->getRxPacketError(dxl_error) << std::endl;
			throw dynamixel_bus_error(id, std::string(packetHandler->getRxPacketError(dxl_error)));
		}
		return model_t(model_number);
	}

	/**
	 * Adds the data to a syncwrite (and creates one if there is none for the specific command field)
	 * @param command_field the field (i.e. address and data length of the controll table)
	 * @param data the data to write to the Dynamixel
	 * @return true if successfull, false otherwise
	 */
	bool add_syncwrite_data(motor_id id, field& command_field, uint8_t* data) {
		auto find = syncWrites.find(command_field);
		if (find == syncWrites.end()) {
			find = syncWrites.insert(
				find,
				{command_field,
				 std::make_unique<GroupSyncWrite>(
					 portHandler.get(),
					 packetHandler.get(),
					 command_field.address,
					 command_field.data_length)});
		}
		find->second->addParam(id, data);
		return true;
	}

	/**
	 * Searches the motor name given the id
	 * @param id the id to get the name for
	 * @return the name of the motor
	 */
	std::string get_motor_name_by_id(const motor_id& id) const {
		for (const auto& [joint_name, motor_check] : motors) {
			if (motor_check->id == id)
				return joint_name;
		}
		throw std::invalid_argument(
			"Joint with id \"" + std::to_string(id) + 
			"\" not added to driver!");
	}

	/**
	 * adds the specified motors torque state into the specific GroupSyncWriter 
	 * to prepare it for the writeing over the bus.
	 */
	void set_torque(const dynamixel::Driver::Motor::Ptr& motor, bool torque) {
		if(torque && motor->hw_error) {
			if(motor->rebooting) 
				return;
			throw hardware_error(motor, "Can't enable torque!");
		}
		field command_field;
		motor->get_command(command::Torque_Enable, command_field);
		uint32_t p = torque;
		add_syncwrite_data(motor->id, command_field, reinterpret_cast<uint8_t*>(&p));
	}

    void reboot(dynamixel::Driver::Motor::Ptr& motor) {
		motor->after_reboot_functions.clear();
		motor->after_reboot_functions.push_back([this, motor](){
			this->pid_gains(motor->name, motor->p_gain_target, motor->i_gain_target, motor->d_gain_target);
		});
		packetHandler->reboot(portHandler.get(),motor->id);
	}

	hw_error_callback overload_error_callback;
	hw_error_callback input_voltage_error_callback;
	hw_error_callback overheating_error_callback;
	hw_error_callback electrical_shock_error_callback;
	hw_error_callback motor_encoder_error_callback;

   public:
	struct command_type_not_implemented : public std::logic_error
    {
        command_type_not_implemented (const dynamixel::field_name type) : std::logic_error{"Command type \""+type+"\" not implemented."} {}
    };

	struct motor_type_not_implemented : public std::logic_error
    {
        motor_type_not_implemented (const model_t type) : std::logic_error{"Motor type \""+std::to_string(type)+"\" not implemented."} {}
    };

	struct torque_not_enabled : public std::runtime_error {
		torque_not_enabled(const dynamixel::Driver::Motor::Ptr& motor) : std::runtime_error("torque of motor " + motor->name + "(id: " + std::to_string(motor->id) + ") is not enabled!") {}
	};

	struct dynamixel_bus_error : public std::runtime_error {
		dynamixel_bus_error(const motor_id& id, std::string dynamixel_error) : std::runtime_error("dynamixel bus error while communicating with motor (id: " + std::to_string(id) + ") " + dynamixel_error) {}
	};

	struct hardware_error : public std::runtime_error {
		hardware_error(const dynamixel::Driver::Motor::Ptr& motor, std::string consequence) : std::runtime_error("motor " + motor->name + "(id: " + std::to_string(motor->id) + ") has hardware error! " + consequence) {}
	};

	/**
	 * Dynamixel Driver class. 
	 * Only works with dynmaixel-protocol version 2.0 as syncwrites and reads are used.
	 * @param port the name of the serial port the dynamixel actuators are connected to
	 * @param baudrate the baudrate of the dynamixels
	 */
	Driver(std::string port, uint baudrate = 57600) : portHandler(PortHandler::getPortHandler(port.c_str())),
													  packetHandler(PacketHandler::getPacketHandler(2.0f)) {
		portHandler->setBaudRate(baudrate);
		//syncRead = std::make_shared<GroupSyncRead>(portHandler_.get(),packetHandler_.get(),0,);
	}

	void set_hardware_error_callbacks(
			hw_error_callback overload_error_callback = [](auto){},
			hw_error_callback input_voltage_error_callback = [](auto){},
			hw_error_callback overheating_error_callback = [](auto){},
			hw_error_callback electrical_shock_error_callback = [](auto){},
			hw_error_callback motor_encoder_error_callback = [](auto){}) {
		this->overload_error_callback = overload_error_callback;
		this->input_voltage_error_callback = input_voltage_error_callback;
		this->overheating_error_callback = overheating_error_callback;
		this->electrical_shock_error_callback = electrical_shock_error_callback;
		this->motor_encoder_error_callback = motor_encoder_error_callback;
	}

	/**
	 * Tryes to add a motor to this driver by pinging the id over the bus.
	 * @param joint_name the name of the actuator
	 * @param id the id the dynamixel-actuator has
	 */
	void add_motor(const std::string& joint_name, motor_id id) {
		for (const auto& [joint_name_check, motor_check] : motors) {
			if (motor_check->id == id)
				throw std::invalid_argument(
					"Dynamixel-ID \"" + std::to_string(id) +
					"\" already in use for joint \"" + joint_name_check + "\"!");
			if (joint_name_check == joint_name)
				if (id != motor_check->id)
					throw std::invalid_argument(
						"Can not add Dynamixel \"" + joint_name + "\" with id " + std::to_string(id) +
						".\nIt is already added under different id (id:\"" + joint_name_check + "\")!");
		}
		auto type = get_motor_type(id);
		if (type == model_t::None)
			throw std::invalid_argument("Joint \"" + joint_name + "\" not added to driver!");

		auto motor = std::make_shared<Motor>(id, joint_name, type);
		motors[joint_name] = motor;
		std::weak_ptr<Motor> weak_motor(motors[joint_name]);

		motor->setup_init_sync_readers(initSyncReaders, portHandler, packetHandler);
		motor->setup_sync_readers(syncReaders, portHandler, packetHandler, 
			[this, weak_motor](dynamixel::hardware_status s)
			{
				switch (s) {
				case dynamixel::hardware_status::Overload_Error:
					overload_error_callback(weak_motor.lock()->name);
					break;
				case dynamixel::hardware_status::Input_Voltage_Error:
					input_voltage_error_callback(weak_motor.lock()->name);
					break;
				case dynamixel::hardware_status::OverHeating_Error:
					overheating_error_callback(weak_motor.lock()->name);
					break;
				case dynamixel::hardware_status::Motor_Encoder_Error:
					motor_encoder_error_callback(weak_motor.lock()->name);
					break;
				case dynamixel::hardware_status::Electrical_Shock_Error:
					electrical_shock_error_callback(weak_motor.lock()->name);
					break;
				default:
					break;
				}
			});
	}

	/**
	 * pushes the commands over the bus and reads the states back.
	 */
	void write(bool enable_torque = false) {
		for (const auto& [joint_name, motor]: motors) {
			if(motor->torque || enable_torque) {
				if(std::isnan(motor->goal_position))
					motor->goal_position = motor->position;
				set_position(joint_name, motor->goal_position, enable_torque);
			}
		}

		for (const auto& [field, syncWrite] : syncWrites) {
			//(void)field;
			/*auto result = */ syncWrite->txPacket();
			//syncWrite->clearParam();
			//std::cout<<"SyncWrite"<<std::endl;
			//std::cout<<packetHandler->getTxRxResult(result)<<std::endl;
		}
		syncWrites.clear();
	}

	void read() {
		bool init_done = true;
		for (auto it = initSyncReaders.begin(); it != initSyncReaders.end(); std::advance(it,1)) {
			const auto& [field, syncReader] = *it;
			(void)field;
			/*auto result = */ syncReader->syncRead.txRxPacket();
			//std::cout<<packetHandler->getTxRxResult(result)<<std::endl;
			for (auto read_f : syncReader->read_functions) {
				read_f(syncReader->syncRead);
			}
			// If no motor is rebooting then the initial read was succesfull.
			if (!std::all_of(syncReader->motors.cbegin(), syncReader->motors.cend(), 
					[](const auto& m){ return !m->rebooting; })) {
				init_done = false;
			}
		}

		for (const auto& [field, syncReader] : syncReaders) {
			(void)field;
			/*auto result = */ syncReader->syncRead.txRxPacket();
			//std::cout<<packetHandler->getTxRxResult(result)<<std::endl;
			for (auto read_f : syncReader->read_functions) {
				read_f(syncReader->syncRead);
			}
		}

		if(init_done) {
			initSyncReaders.clear();
			for (const auto& [joint_name, motor]: motors) {
				if(motor->rebooted) {
					motor->rebooted = false;
					for(auto after_reboot_f:motor->after_reboot_functions) {
						after_reboot_f();
					}
				}
			}
		}
	}

	/**
	 * Tries to ping every Dynamixel to check it they are reachable.
	 */
	void ping_all() const {
		for (const auto& [joint_name, motor] : motors) {
			get_motor_type(motor->id);
		}
	}

	void for_each_joint(std::function<void(const std::string&)> f) {
		for (const auto& [joint_name, motor]: motors) {
			f(joint_name);
		}
	}

	/**
	 * Garanteed to have the same order as 
	 * get_positions, get_velocities and get_efforts.
	 * 
	 * Use to construct the JointStates message.
	 */
	void get_joint_names(std::vector<std::string>& names) const {
		names.resize(motors.size());
		int i = 0;
		for (const auto& [joint_name, motor] : motors) {
			(void)joint_name;
			names[i++] = joint_name;
		}
	}

	double* get_goal_position_ptr(const std::string& joint_name) const {
		auto find = motors.find(joint_name);
		if (find == motors.end())
			throw std::invalid_argument("Joint \"" + joint_name + "\" not added to driver!");
		return &find->second->goal_position;
	}

	double* get_position_ptr(const std::string& joint_name) const {
		auto find = motors.find(joint_name);
		if (find == motors.end())
			throw std::invalid_argument("Joint \"" + joint_name + "\" not added to driver!");
		return &find->second->position;
	}
	
	double* get_velocity_ptr(const std::string& joint_name) const {
		auto find = motors.find(joint_name);
		if (find == motors.end())
			throw std::invalid_argument("Joint \"" + joint_name + "\" not added to driver!");
		return &find->second->velocity;
	}
	
	double* get_effort_ptr(const std::string& joint_name) const {
		auto find = motors.find(joint_name);
		if (find == motors.end())
			throw std::invalid_argument("Joint \"" + joint_name + "\" not added to driver!");
		return &find->second->effort;
	}

	/**
	 * Garanteed to have the same order as 
	 * get_joint_names, get_velocities and get_efforts.
	 * 
	 * Use to construct the JointStates message.
	 * @param positions the array that gets filled with the data
	 */
	void get_positions(std::vector<double>& positions) const {
		positions.resize(motors.size());
		int i = 0;
		for (const auto& [joint_name, motor] : motors) {
			(void)joint_name;
			positions[i++] = motor->position;
		}
	}

	/**
	 * Garanteed to have the same order as 
	 * get_joint_names, get_positions and get_efforts.
	 * 
	 * Use to construct the JointStates message.
	 * @param velocities the array that gets filled with the data
	 */
	void get_velocities(std::vector<double>& velocities) const {
		velocities.resize(motors.size());
		int i = 0;
		for (const auto& [joint_name, motor] : motors) {
			(void)joint_name;
			velocities[i++] = motor->velocity;
		}
	}

	/**
	 * Garanteed to have the same order as 
	 * get_joint_names, get_positions and get_velocities.
	 * 
	 * Use to construct the JointStates message.
	 * @param efforts the array that gets filled with the data
	 */
	void get_efforts(std::vector<double>& efforts) const {
		efforts.resize(motors.size());
		int i = 0;
		for (const auto& [joint_name, motor] : motors) {
			(void)joint_name;
			efforts[i++] = 0;
		}
	}

	/**
	 * returns the number of actuators this controller handles.
	 * add_motor(...) to add more.
	 * @return returns the number of actuators
	 */
	std::size_t get_motor_count() const {
		return motors.size();
	}

	/**
	 * adds the specified motors torque state into the specific GroupSyncWriter 
	 * to prepare it for the writeing over the bus.		
	 * @param joint_name the name of the joint	
	 * @param torque true -> enable torque, false -> no torque
	 */
	void set_torque(const std::string& joint_name, bool torque) {
		auto it_motor = motors.find(joint_name);
		if (it_motor == motors.end())
			throw std::invalid_argument("Joint \"" + joint_name + "\" not added to driver!");
		set_torque(it_motor->second, torque);
	}

	/**
	 * adds all motors torque state into the GroupSyncWriter 
	 * to prepare it for the writeing over the bus.	
	 * @param torque true -> enable torque, false -> no torque
	 */
	void set_torque_all(bool torque) {
		std::map<Motor::Ptr,bool> old_torques;
		try{
			for (const auto& [joint_name, motor]: motors) {
				old_torques[motor] = motor->torque;
				set_torque(motor, torque);
			}
		} catch (hardware_error& e) {
			for (const auto& [motor, old_torque]: old_torques) {
				set_torque(motor, old_torque);
			}
			throw e;
		}
	}

	/**
	 * adds the specified motors position-data into the specific GroupSyncWriter 
	 * to prepare it for the writeing over the bus.		
	 * @param joint_name the name of the joint
	 * @param position the position in radians the actuator should move to
	 * @param enable_torque enable the torque if not enabled already
	 */
	void set_position(const std::string& joint_name, double position, bool enable_torque = false) {
		if (std::isnan(position))
			throw std::invalid_argument("Can't set position to NaN for Joint \"" + joint_name + "\"!");
		auto it_motor = motors.find(joint_name);
		if (it_motor == motors.end())
			throw std::invalid_argument("Joint \"" + joint_name + "\" not added to driver!");
		auto motor = it_motor->second;	//motors[joint_name];

		if (!motor->torque) {
			if (enable_torque)
				set_torque(motor, true);
			else
				throw torque_not_enabled(motor);
		}

		field command_field;
		motor->get_command(command::Goal_Position, command_field);
		uint32_t p = motor->raw_from_position(position);
		motor->goal_position = position;
		add_syncwrite_data(motor->id, command_field, reinterpret_cast<uint8_t*>(&p));
	}

	/**
	 * Sets the pid gains for a motor. This is only possible after the first read 
	 * as the initial values need to be read from the motor.
	 * @param joint_name the name of the joint
	 */

	void pid_gains(const std::string& joint_name, double& p, double& i, double& d) {
		auto it_motor = motors.find(joint_name);
		if (it_motor == motors.end())
			throw std::invalid_argument("Joint \"" + joint_name + "\" not added to driver!");
		auto motor = it_motor->second;
		//std::cout <<joint_name<<" target p:"<< p<<"\ti:"<< i<<"\td:" << d << std::endl;
		//std::cout <<joint_name<<" current p:"<< motor->p_gain<<"\ti:"<< motor->i_gain<<"\td:" << motor->d_gain << std::endl;
		if(std::isnan( motor->p_gain) || std::isnan( motor->i_gain) || std::isnan( motor->d_gain)) {
			throw std::invalid_argument("Cant set pid gains for \"" + joint_name + "\" because motor doesn't have pid control!");
		}
		if(std::isnan(p)) {
			p = motor->p_gain;
		}
		if(std::isnan(i)) {
			i = motor->i_gain;
		}
		if(std::isnan(d)) {
			d = motor->d_gain;
		}
		if( p != motor->p_gain) {
			field command_field;
			motor->get_command(command::Position_P_Gain, command_field);
			uint32_t data = static_cast<uint32_t>(p);
			motor->p_gain_target = p;
			add_syncwrite_data(motor->id, command_field, reinterpret_cast<uint8_t*>(&data));
		}
		if( i != motor->i_gain) {
			field command_field;
			motor->get_command(command::Position_I_Gain, command_field);
			uint32_t data = static_cast<uint32_t>(i);
			motor->i_gain_target = i;
			add_syncwrite_data(motor->id, command_field, reinterpret_cast<uint8_t*>(&data));
		}
		if( d != motor->d_gain) {
			field command_field;
			motor->get_command(command::Position_D_Gain, command_field);
			uint32_t data = static_cast<uint32_t>(d);
			motor->d_gain_target = d;
			add_syncwrite_data(motor->id, command_field, reinterpret_cast<uint8_t*>(&data));
		}
	}

	/**
	 * reboots a joint.
	 * @param joint_name the name of the joint	
	 */
	void reboot(const std::string& joint_name) {
		auto it_motor = motors.find(joint_name);
		if (it_motor == motors.end())
			throw std::invalid_argument("Joint \"" + joint_name + "\" not added to driver!");
		it_motor->second->rebooting = true;
		reboot(it_motor->second);
	}
};
}  // namespace dynamixel
