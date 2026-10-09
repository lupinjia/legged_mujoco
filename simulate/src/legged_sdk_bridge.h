#pragma once

#include <mujoco/mujoco.h>

#include <legged_sdk/dds/dds_publisher.hpp>
#include <legged_sdk/dds/dds_subscriber.hpp>
#include <legged_sdk/common/legged_joystick.hpp>
#include <legged_sdk/common/periodic_task.hpp>
#include <legged_sdk/idl/agilepf/LowCmd/LowCmd.hpp>
#include <legged_sdk/idl/agilepf/LowCmd/LowCmdPubSubTypes.hpp>
#include <legged_sdk/idl/agilepf/LowState/LowState.hpp>
#include <legged_sdk/idl/agilepf/LowState/LowStatePubSubTypes.hpp>
#include <mutex>
#include <legged_sdk/dds_wrapper/robot/agilepf/agilepf_pub.h>

#include "param.h"
#include "physics_joystick.h"
#include "keyboard_joystick.h"

using namespace legged::dds;

#define MOTOR_SENSOR_NUM 2

class LeggedSDKBridgeBase
{
public:
    LeggedSDKBridgeBase(mjModel *model, mjData *data, const KeyboardGlfwAdapter *keyboard = nullptr)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "yahboom") {
                joystick  = std::make_shared<YahboomJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "betop") {
                joystick  = std::make_shared<BeTopJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "keyboard") {
                joystick  = std::make_shared<KeyboardJoystick>(keyboard);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    int have_imu_ = false;

    std::shared_ptr<legged::common::LeggedJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
        {
            const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
            if (strcmp(name, "imu_quat") == 0) {
                have_imu_ = true;
            }
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public LeggedSDKBridgeBase
{
public:
    RobotBridge(mjModel *model, mjData *data, int domain_id, std::string interface_name,
                const KeyboardGlfwAdapter *keyboard = nullptr) 
               : LeggedSDKBridgeBase(model, data, keyboard)
    {
        lowcmd = std::make_shared<LowCmd_t>(domain_id, interface_name, "rt/lowcmd",
                                            std::bind(&RobotBridge::lowcmdHandler, this, std::placeholders::_1),
                                            2000.0f);
        lowcmd->init();
        // LowState_t and JoystickData_t will init in constructor
        lowstate = std::make_unique<LowState_t>(domain_id, interface_name, 2000.0f);
        lowstate->joystick = joystick;
        thread_ = std::make_shared<legged::common::PeriodicTask>(
            "unitree_bridge", 2000, std::bind(&RobotBridge::run, this),
            1, SCHED_FIFO, 99);
    }

    void lowcmdHandler(const void* msg)
    {
        if(!mj_data_) 
        {
            spdlog::warn("mj_data_ is null in lowcmdHandler");
            return;
        }
        const typename LowCmd_t::MsgType msg_ = *(const typename LowCmd_t::MsgType*)msg;
        for(int i(0); i<num_motor_; i++) {
            auto & m = msg_.motor_cmd()[i];
            mj_data_->ctrl[i] = m.tau() +
                                m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }

    void run()
    {
        // Print debug info every 500 runs
        // if (run_count_ == 500)
        // {
        //     run_count_ = 0;
        //     spdlog::info("CPU ID of unitree_bridge thread: {}", sched_getcpu());
        // } else {
        //     run_count_++;
        // }
        
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
            }
            if(have_imu_) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
            }
            lowstate->unlockAndPublish();
        }
    }

    std::shared_ptr<LowCmd_t> lowcmd;
    std::shared_ptr<LowState_t> lowstate;
    
private:
    legged::common::PeriodicTaskPtr thread_;
    uint64_t run_count_ = 0;
};

using AgilepfBridge = RobotBridge<
    legged::dds::DDSSubscriber<legged::idl::agilepf::LowCmd, legged::idl::agilepf::LowCmdPubSubType>,
    legged::dds_wrapper::agilepf::publisher::LowStatePublisher>;  