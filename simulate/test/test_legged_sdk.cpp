#include <spdlog/spdlog.h>
#include <legged_sdk/dds/dds_publisher.hpp>
#include <legged_sdk/dds/dds_subscriber.hpp>
#include <legged_sdk/idl/agilepf/LowState/LowState.hpp>
#include <legged_sdk/idl/agilepf/LowState/LowStatePubSubTypes.hpp>
#include <legged_sdk/idl/agilepf/LowCmd/LowCmd.hpp>
#include <legged_sdk/idl/agilepf/LowCmd/LowCmdPubSubTypes.hpp>
#include <legged_sdk/common/periodic_task.hpp>

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LOWCMD "rt/lowcmd"

using namespace legged::dds;
using namespace legged::common;

void LowStateHandler(const void *msg)
{
    const legged::idl::agilepf::LowState s = *(legged::idl::agilepf::LowState *)msg;

    std::cout << "Quaternion: "
              << s.imu_state().quaternion()[0] << " "
              << s.imu_state().quaternion()[1] << " "
              << s.imu_state().quaternion()[2] << " "
              << s.imu_state().quaternion()[3] << " " << std::endl;
    
    std::cout << "Motor States: " << std::endl;
    for (size_t i = 0; i < s.motor_state().size(); i++)
    {
        std::cout << "  Motor " << i << ": q=" << s.motor_state()[i].q()
                  << ", dq=" << s.motor_state()[i].dq() << std::endl;
    }
}

int main()
{
    int domain_id = 1;
    std::string interface = "lo";

    DDSSubscriberPtr<legged::idl::agilepf::LowState, 
        legged::idl::agilepf::LowStatePubSubType> lowstate_suber = 
        std::make_shared<DDSSubscriber<legged::idl::agilepf::LowState, 
        legged::idl::agilepf::LowStatePubSubType>>(domain_id,
        interface, TOPIC_LOWSTATE, LowStateHandler, 2000.0);
    lowstate_suber->init();

    DDSPublisherPtr<legged::idl::agilepf::LowCmd, 
        legged::idl::agilepf::LowCmdPubSubType> low_cmd_puber = 
        std::make_shared<DDSPublisher<legged::idl::agilepf::LowCmd, 
        legged::idl::agilepf::LowCmdPubSubType>>(domain_id,
        interface, TOPIC_LOWCMD);
    low_cmd_puber->init();

    spdlog::info("Start sending lowcmd and receiving lowstate. Press q to exit the program.");

    while (true)
    {   
        legged::idl::agilepf::LowCmd low_cmd{};
        for (int i = 0; i < 6; i++)
        {
            low_cmd.motor_cmd()[i].q() = 0;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kd() = 0;
            low_cmd.motor_cmd()[i].tau() = 1;
        }
        low_cmd_puber->publish(&low_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    delete lowstate_suber.get();
    delete low_cmd_puber.get();

    return 0;
}
