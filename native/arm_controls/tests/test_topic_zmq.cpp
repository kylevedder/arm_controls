#include <atomic>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>
#include <zmq.hpp>

#include "arm_controls_device.hpp"
#include "arm_controls_device_arm.hpp"
#include "arm_controls_profile.hpp"

#define private public
#include "arm_controls_topic_zmq.hpp"
#undef private

namespace {

class TestDevice : public Device {
   public:
    TestDevice(const CommandLineArgs& cla, int dof_total) : Device(cla) {
        role_ = cla.role;
        dof_total_ = dof_total;
    }

    ReturnCode apply_action(const MsgJoints&) override {
        ++joint_action_count;
        return ReturnCode::SUCCESS;
    }
    ReturnCode apply_action(const MsgJoystick&) override {
        ++joystick_action_count;
        return ReturnCode::SUCCESS;
    }
    ReturnCode get_observation(MsgJoints&) override { return ReturnCode::SUCCESS; }
    ReturnCode process_follower_msg(const MsgJoints& msg) override {
        ++follower_action_count;
        last_follower_msg = msg;
        return ReturnCode::SUCCESS;
    }
    ReturnCode read_hardware_values() override { return ReturnCode::SUCCESS; }
    ReturnCode write_hardware_values() override { return ReturnCode::SUCCESS; }
    ReturnCode move_to_ready_position() override { return ReturnCode::SUCCESS; }
    ReturnCode operate_as_leader() override { return ReturnCode::SUCCESS; }
    ReturnCode operate_as_follower() override { return ReturnCode::SUCCESS; }
    ReturnCode get_servo_ids(std::vector<int>&) override { return ReturnCode::SUCCESS; }
    ReturnCode set_control_mode(Role, ControlModeIntent) override { return ReturnCode::SUCCESS; }
    ReturnCode park_safely() override { ++park_count; return ReturnCode::SUCCESS; }

    void expire_client_watchdog() {
        is_ready_ = true;
        last_client_heartbeat_ = std::chrono::steady_clock::now() - std::chrono::seconds(6);
        check_client_heartbeat_watchdog();
    }
    int park_count = 0;
    int joint_action_count = 0;
    int joystick_action_count = 0;
    int follower_action_count = 0;
    MsgJoints last_follower_msg;
};

// Exercise the production arm handlers with an empty, hardware-free joint set.
class RuntimeModeTestArm : public DeviceArm {
   public:
    explicit RuntimeModeTestArm(const CommandLineArgs& cla) : DeviceArm(cla) {
        role_ = Role::FOLLOWER;
        is_ready_ = true;
        dof_total_ = 0;
    }
    void attach_topic(std::shared_ptr<Topic> topic) { p_topic_ = std::move(topic); }
    void latch_feedback_hold(bool value) { feedback_hold_latched_ = value; }
};

CommandLineArgs make_cla() {
    static std::atomic<unsigned int> next_id{0};
    const std::string suffix = std::to_string(getpid()) + "_" + std::to_string(next_id.fetch_add(1));

    CommandLineArgs cla{};
    cla.device_model = "topic_zmq_test";
    cla.device_id = suffix;
    cla.role = Role::FOLLOWER;
    cla.control_frequency = 100;
    cla.moving_mode = MovingMode::PARALLEL;
    cla.msg_type = MsgType::JOINT_INFO;
    cla.force_feedback = -1.0f;
    cla.topic_joint = "topic_zmq_test_joint_" + suffix;
    cla.topic_state = "topic_zmq_test_state_" + suffix;
    cla.topic_lifecycle = "topic_zmq_test_lifecycle_" + suffix;
    cla.topic_status = "topic_zmq_test_status_" + suffix;
    cla.topic_joystick = OPT_DEFAULT_NONE;
    return cla;
}

class TopicZmqPublishBoundsTest : public testing::Test {
   protected:
    CommandLineArgs cla_ = make_cla();
    TestDevice device_{cla_, 11};
    TopicZmq topic_{&device_, cla_, 0, nullptr};
};

TEST(CoherentFeedbackWire, RawAbiAndReservedTailRemainExplicit) {
    ZmqJointState state{};
    state.raw.joint_num = 7;
    state.raw.joint_pos[6] = 0.75f;
    state.feedback[6] = FeedbackSample{0.75f, -0.25f, 123456789, 987};
    EXPECT_EQ(sizeof(state.raw), 252u);
    EXPECT_EQ(sizeof(state), 496u);
    EXPECT_EQ(reinterpret_cast<const char*>(&state.feedback[0]) - reinterpret_cast<const char*>(&state), 256);
    EXPECT_EQ(state.feedback_version, 1u);
    EXPECT_FLOAT_EQ(state.feedback[6].position, state.raw.joint_pos[6]);
    EXPECT_EQ(state.feedback[6].received_mono_ns, 123456789u);
    // The legacy joint ABI has capacity ten; unused state records stay zero.
    for (int i = 7; i < 10; ++i) {
        EXPECT_EQ(state.feedback[i].sample_id, 0u);
        EXPECT_EQ(state.feedback[i].received_mono_ns, 0u);
    }
}

TEST(RuntimeModeStatus, NativeAcceptedTargetAndHoldProduceSequencedModeBeforeAck) {
    CommandLineArgs cla = make_cla();
    cla.no_recovery_motion = true;
    cla.topic_direct_command = "runtime_mode_direct_" + cla.device_id;
    RuntimeModeTestArm device(cla);
    auto topic = std::make_shared<TopicZmq>(&device, cla, 0, nullptr);
    device.attach_topic(topic);

    auto pair = [&](zmq::socket_t& socket, const std::string& name) {
        socket.close();
        socket = zmq::socket_t(topic->context_, ZMQ_PAIR);
        const std::string endpoint = "inproc://" + name + cla.device_id;
        socket.bind(endpoint);
        zmq::socket_t peer(topic->context_, ZMQ_PAIR);
        peer.connect(endpoint);
        return peer;
    };
    auto statuses = pair(topic->pub_device_info_, "runtime_status_");
    auto commands = pair(topic->sub_command_, "runtime_lifecycle_");
    auto direct = pair(topic->sub_direct_joint_, "runtime_direct_");
    auto receive = [&]() -> std::optional<ZmqDeviceInfo> {
        zmq::message_t name, payload;
        if (!statuses.recv(name, zmq::recv_flags::dontwait)) return std::nullopt;
        if (!statuses.recv(payload, zmq::recv_flags::dontwait) || payload.size() != sizeof(ZmqDeviceInfo))
            return std::nullopt;
        ZmqDeviceInfo result{};
        std::memcpy(&result, payload.data(), sizeof(result));
        return result;
    };
    auto sequence = [](const ZmqDeviceInfo& status) {
        return static_cast<uint64_t>(static_cast<uint32_t>(status.param_int[1])) |
               (static_cast<uint64_t>(static_cast<uint32_t>(status.param_int[2])) << 32);
    };
    auto send_target = [&](int joints) {
        ZmqJointInfo target{};
        target.msg_type = static_cast<int>(MsgType::JOINT_INFO);
        target.joint_num = joints;
        direct.send(zmq::buffer(cla.topic_direct_command), zmq::send_flags::sndmore);
        direct.send(zmq::buffer(&target, sizeof(target)), zmq::send_flags::none);
    };
    auto send_hold = [&]() {
        ZmqCommand hold{};
        hold.command = DEVICE_COMMAND_HOLD;
        hold.num_param_int = 1;
        hold.param_int[0] = 17;
        commands.send(zmq::buffer(cla.topic_lifecycle), zmq::send_flags::sndmore);
        commands.send(zmq::buffer(&hold, sizeof(hold)), zmq::send_flags::none);
    };

    ASSERT_EQ(topic->step(), ReturnCode::SUCCESS);
    auto state = receive();
    ASSERT_TRUE(state.has_value()) << "native state 32 must be produced, not only consumed";
    ASSERT_EQ(state->info_key, DEVICE_INFO_RUNTIME_MODE);
    ASSERT_EQ(state->num_param_int, 3);
    EXPECT_EQ(state->param_int[0], 2);
    EXPECT_EQ(sequence(*state), 1u);

    send_target(0);
    ASSERT_EQ(topic->step(), ReturnCode::SUCCESS);
    state = receive();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->param_int[0], 3);
    EXPECT_EQ(sequence(*state), 2u);
    // The automatic gap handler clears buffers but keeps the base mode; its
    // separate status 34 owns that pause instead of inventing a second edge.
    ASSERT_EQ(device.runtime_hold(true), ReturnCode::SUCCESS);
    device.latch_feedback_hold(true);
    send_target(0);
    ASSERT_EQ(topic->step(), ReturnCode::SUCCESS);
    while (auto gap_repeat = receive()) {
        ASSERT_EQ(gap_repeat->info_key, DEVICE_INFO_RUNTIME_MODE);
        EXPECT_EQ(gap_repeat->param_int[0], 3);
        EXPECT_EQ(sequence(*gap_repeat), 2u);
    }
    send_hold();
    ASSERT_EQ(topic->step(), ReturnCode::SUCCESS);
    state = receive();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->info_key, DEVICE_INFO_RUNTIME_MODE);
    EXPECT_EQ(state->param_int[0], 2);
    EXPECT_EQ(sequence(*state), 3u);
    auto ack = receive();
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->info_key, DEVICE_INFO_COMMAND_ACK);
    EXPECT_EQ(ack->param_int[0], 17);
    EXPECT_EQ(ack->param_int[1], static_cast<int>(ReturnCode::SUCCESS));

    send_hold();
    ASSERT_EQ(topic->step(), ReturnCode::SUCCESS);
    // Repeated explicit HOLD and periodic re-announcements retain the counter.
    while (auto repeated = receive()) {
        if (repeated->info_key == DEVICE_INFO_RUNTIME_MODE) EXPECT_EQ(sequence(*repeated), 3u);
    }
    device.latch_feedback_hold(true);
    send_target(0); // Native drops this without applying target buffers.
    ASSERT_EQ(topic->step(), ReturnCode::SUCCESS);
    device.latch_feedback_hold(false);
    send_target(1); // Native rejects the wrong joint count.
    ASSERT_EQ(topic->step(), ReturnCode::INVALID_PARAM);
    std::this_thread::sleep_for(std::chrono::milliseconds(110));
    ASSERT_EQ(topic->step(), ReturnCode::SUCCESS);
    state = receive();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->param_int[0], 2);
    EXPECT_EQ(sequence(*state), 3u);

    // The native counter retains a complete intermediate direct/HOLD episode
    // even if a subscriber only keeps its final (same-mode) announcement.
    send_target(0);
    ASSERT_EQ(topic->step(), ReturnCode::SUCCESS);
    ASSERT_TRUE(receive().has_value());
    send_hold();
    ASSERT_EQ(topic->step(), ReturnCode::SUCCESS);
    state = receive();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->info_key, DEVICE_INFO_RUNTIME_MODE);
    EXPECT_EQ(state->param_int[0], 2);
    EXPECT_EQ(sequence(*state), 5u);
}

TEST(CoherentFeedbackWire, PublishedExtendedStateReachesPairedSubscriberAndCommandsStayLegacy) {
    CommandLineArgs follower_cla = make_cla();
    TestDevice follower_device(follower_cla, 1);
    TopicZmq follower(&follower_device, follower_cla, 0, nullptr);
    follower.set_pause_leader_command_listening(false);

    CommandLineArgs leader_cla = make_cla();
    leader_cla.role = Role::LEADER;
    leader_cla.topic_joint = follower_cla.topic_joint;
    leader_cla.paired_follower_state_topic = follower_cla.topic_state;
    TestDevice leader_device(leader_cla, 1);
    TopicZmq leader(&leader_device, leader_cla, 0, nullptr);

    MsgJoints msg;
    msg.add_joint_info(0.25f, -0.5f, 0.75f, 23.0f, 1.5f, 12.0f,
                       FeedbackSample{0.25f, -0.5f, 123456789, 42});
    msg.measured_idc_current_ = 2.5f;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (leader_device.follower_action_count == 0 || follower_device.joint_action_count == 0) {
        ASSERT_LT(std::chrono::steady_clock::now(), deadline);
        ASSERT_EQ(follower.publish(msg), ReturnCode::SUCCESS);
        ASSERT_EQ(leader.publish(msg), ReturnCode::SUCCESS);
        ASSERT_EQ(leader.step(), ReturnCode::SUCCESS);
        ASSERT_EQ(follower.step(), ReturnCode::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // Both messages went through the production publisher and socket drains.
    // The command drain above still rejects every size other than 252 bytes.
    ASSERT_EQ(leader_device.last_follower_msg.joints_.size(), 1u);
    const auto& joint = leader_device.last_follower_msg.joints_[0];
    EXPECT_FLOAT_EQ(joint.curr_pos_, 0.25f);
    EXPECT_FLOAT_EQ(joint.curr_vel_, -0.5f);
    EXPECT_FLOAT_EQ(joint.curr_tor_, 0.75f);
    EXPECT_FLOAT_EQ(joint.temperature_, 23.0f);
    EXPECT_FLOAT_EQ(joint.idc_current_, 1.5f);
    EXPECT_FLOAT_EQ(joint.frame_age_ms_, 12.0f);
    EXPECT_FLOAT_EQ(leader_device.last_follower_msg.measured_idc_current_, 2.5f);
}

TEST(CoherentFeedbackWire, PairedSubscriberPreservesRawPrefixForLegacyAndUnknownExtension) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.sub_follower_joint_.close();
    topic.sub_follower_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string endpoint = "inproc://follower_state_compat_" + cla.device_id;
    topic.sub_follower_joint_.bind(endpoint);
    zmq::socket_t sender(topic.context_, ZMQ_PAIR);
    sender.connect(endpoint);

    ZmqJointState state{};
    state.raw.msg_type = static_cast<int>(MsgType::JOINT_INFO);
    state.raw.joint_num = 1;
    state.raw.joint_pos[0] = 0.25f;
    state.feedback_version = 99;
    for (const size_t size : {sizeof(ZmqJointInfo), sizeof(ZmqJointState)}) {
        sender.send(zmq::buffer(topic.topic_follower_joint_), zmq::send_flags::sndmore);
        sender.send(zmq::buffer(&state, size), zmq::send_flags::none);
        ASSERT_EQ(topic.step(), ReturnCode::SUCCESS);
        ASSERT_EQ(device.last_follower_msg.joints_.size(), 1u);
        EXPECT_FLOAT_EQ(device.last_follower_msg.joints_[0].curr_pos_, 0.25f);
    }
    EXPECT_EQ(device.follower_action_count, 2);
}

TEST(CoherentFeedbackWire, CommandDrainsRejectEmptyAndExtendedPayloads) {
    for (const bool direct : {false, true}) {
        CommandLineArgs cla = make_cla();
        cla.topic_direct_command = "topic_zmq_test_direct_" + cla.device_id;
        TestDevice device(cla, 1);
        TopicZmq topic(&device, cla, 0, nullptr);
        topic.set_pause_leader_command_listening(false);
        auto& sub = direct ? topic.sub_direct_joint_ : topic.sub_joint_;
        const auto& name = direct ? topic.topic_direct_command_ : topic.topic_joint_;
        sub.close();
        sub = zmq::socket_t(topic.context_, ZMQ_PAIR);
        const std::string endpoint = "inproc://command_size_rejection_" + cla.device_id;
        sub.bind(endpoint);
        zmq::socket_t sender(topic.context_, ZMQ_PAIR);
        sender.connect(endpoint);

        ZmqJointState state{};
        state.raw.msg_type = static_cast<int>(MsgType::JOINT_INFO);
        state.raw.joint_num = 1;
        for (const size_t size : {size_t{0}, sizeof(state)}) {
            sender.send(zmq::buffer(name), zmq::send_flags::sndmore);
            sender.send(zmq::buffer(&state, size), zmq::send_flags::none);
            EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
            EXPECT_EQ(device.joint_action_count, 0);
        }
    }
}

TEST_F(TopicZmqPublishBoundsTest, RejectsJointPayloadLargerThanWireArray) {
    MsgJoints msg;
    for (int i = 0; i < 11; ++i) {
        msg.add_joint_info(i, i, i, i, i, -1.0f);
    }

    EXPECT_EQ(topic_.publish(msg), ReturnCode::INVALID_PARAM);
}

TEST_F(TopicZmqPublishBoundsTest, RejectsEffectorPayloadLargerThanWireArray) {
    std::vector<float> params(31, 1.0f);
    MsgEffectorInfo msg(&params);

    EXPECT_EQ(topic_.publish(msg), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqPublishAvailability, RejectsMissingFollowerStatePublisher) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.pub_follower_joint_.close();
    MsgJoints msg;
    msg.add_joint_info(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f);

    EXPECT_EQ(topic.publish(msg), ReturnCode::NOT_INITIALIZED);
}

TEST(TopicZmqPublishAvailability, RejectsMissingEffectorInfoPublisher) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.pub_effector_info_.close();
    std::vector<float> params{0.0f};
    MsgEffectorInfo msg(&params);

    EXPECT_EQ(topic.publish(msg), ReturnCode::NOT_INITIALIZED);
}

TEST_F(TopicZmqPublishBoundsTest, RejectsLeaderJointPayloadLargerThanWireArray) {
    ZmqJointInfo msg{};
    msg.msg_type = (int)MsgType::JOINT_INFO;
    msg.joint_num = 11;

    EXPECT_EQ(topic_.process_leader_msg_joint(&msg), ReturnCode::INVALID_PARAM);
}

TEST_F(TopicZmqPublishBoundsTest, RejectsFollowerJointPayloadLargerThanWireArray) {
    ZmqJointInfo msg{};
    msg.msg_type = (int)MsgType::JOINT_INFO;
    msg.joint_num = 11;

    EXPECT_EQ(topic_.process_follower_msg_joint(&msg), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqJointCount, RejectsLeaderPayloadForDifferentDof) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.set_pause_leader_command_listening(false);
    ZmqJointInfo msg{};
    msg.msg_type = (int)MsgType::JOINT_INFO;
    msg.joint_num = 2;

    EXPECT_EQ(topic.process_leader_msg_joint(&msg), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(device.joint_action_count, 0);
}

TEST(TopicZmqJointCount, RejectsFollowerPayloadForDifferentDof) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    ZmqJointInfo msg{};
    msg.msg_type = (int)MsgType::JOINT_INFO;
    msg.joint_num = 2;

    EXPECT_EQ(topic.process_follower_msg_joint(&msg), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqStep, PreservesEarlierDispatchFailureAcrossStreams) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);

    topic.sub_joint_.close();
    topic.sub_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string leader_endpoint =
        "inproc://topic_zmq_step_leader_" + cla.device_id;
    topic.sub_joint_.bind(leader_endpoint);
    zmq::socket_t leader_sender(topic.context_, ZMQ_PAIR);
    leader_sender.connect(leader_endpoint);

    topic.sub_follower_joint_.close();
    topic.sub_follower_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string follower_endpoint =
        "inproc://topic_zmq_step_follower_" + cla.device_id;
    topic.sub_follower_joint_.bind(follower_endpoint);
    zmq::socket_t follower_sender(topic.context_, ZMQ_PAIR);
    follower_sender.connect(follower_endpoint);

    ZmqJointInfo invalid_leader{};
    invalid_leader.msg_type = -1;
    invalid_leader.joint_num = 1;
    leader_sender.send(
        zmq::buffer(topic.topic_joint_), zmq::send_flags::sndmore);
    leader_sender.send(zmq::buffer(&invalid_leader, sizeof(invalid_leader)),
                       zmq::send_flags::none);

    ZmqJointInfo valid_follower{};
    valid_follower.msg_type = static_cast<int>(MsgType::JOINT_INFO);
    valid_follower.joint_num = 1;
    follower_sender.send(
        zmq::buffer(topic.topic_follower_joint_), zmq::send_flags::sndmore);
    follower_sender.send(zmq::buffer(&valid_follower, sizeof(valid_follower)),
                         zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(device.follower_action_count, 1);
}

TEST(TopicZmqStep, ReportsMalformedLifecyclePayload) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);

    topic.sub_command_.close();
    topic.sub_command_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string endpoint =
        "inproc://topic_zmq_step_lifecycle_" + cla.device_id;
    topic.sub_command_.bind(endpoint);
    zmq::socket_t sender(topic.context_, ZMQ_PAIR);
    sender.connect(endpoint);

    const uint8_t malformed_payload = 0;
    sender.send(zmq::buffer(topic.topic_lifecycle_), zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&malformed_payload, sizeof(malformed_payload)),
                zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqStep, StopDoesNotDispatchQueuedJointCommand) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.set_pause_leader_command_listening(false);

    topic.sub_command_.close();
    topic.sub_command_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string lifecycle_endpoint =
        "inproc://topic_zmq_step_stop_lifecycle_" + cla.device_id;
    topic.sub_command_.bind(lifecycle_endpoint);
    zmq::socket_t lifecycle_sender(topic.context_, ZMQ_PAIR);
    lifecycle_sender.connect(lifecycle_endpoint);

    topic.sub_joint_.close();
    topic.sub_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string joint_endpoint =
        "inproc://topic_zmq_step_stop_joint_" + cla.device_id;
    topic.sub_joint_.bind(joint_endpoint);
    zmq::socket_t joint_sender(topic.context_, ZMQ_PAIR);
    joint_sender.connect(joint_endpoint);

    ZmqCommand stop{};
    stop.command = DEVICE_COMMAND_STOP;
    lifecycle_sender.send(
        zmq::buffer(topic.topic_lifecycle_), zmq::send_flags::sndmore);
    lifecycle_sender.send(zmq::buffer(&stop, sizeof(stop)), zmq::send_flags::none);

    ZmqJointInfo joint{};
    joint.msg_type = static_cast<int>(MsgType::JOINT_INFO);
    joint.joint_num = 1;
    joint_sender.send(zmq::buffer(topic.topic_joint_), zmq::send_flags::sndmore);
    joint_sender.send(zmq::buffer(&joint, sizeof(joint)), zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::SUCCESS);
    EXPECT_FALSE(topic.is_running());
    EXPECT_EQ(device.joint_action_count, 0);
}

TEST(TopicZmqStep, HoldDiscardsAlreadyQueuedJointCommand) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.set_pause_leader_command_listening(false);

    topic.sub_command_.close();
    topic.sub_command_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string lifecycle_endpoint =
        "inproc://topic_zmq_step_stop_lifecycle_" + cla.device_id;
    topic.sub_command_.bind(lifecycle_endpoint);
    zmq::socket_t lifecycle_sender(topic.context_, ZMQ_PAIR);
    lifecycle_sender.connect(lifecycle_endpoint);

    topic.sub_joint_.close();
    topic.sub_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string joint_endpoint =
        "inproc://topic_zmq_step_stop_joint_" + cla.device_id;
    topic.sub_joint_.bind(joint_endpoint);
    zmq::socket_t joint_sender(topic.context_, ZMQ_PAIR);
    joint_sender.connect(joint_endpoint);

    ZmqCommand stop{};
    stop.command = DEVICE_COMMAND_HOLD;
    lifecycle_sender.send(
        zmq::buffer(topic.topic_lifecycle_), zmq::send_flags::sndmore);
    lifecycle_sender.send(zmq::buffer(&stop, sizeof(stop)), zmq::send_flags::none);

    ZmqJointInfo joint{};
    joint.msg_type = static_cast<int>(MsgType::JOINT_INFO);
    joint.joint_num = 1;
    joint_sender.send(zmq::buffer(topic.topic_joint_), zmq::send_flags::sndmore);
    joint_sender.send(zmq::buffer(&joint, sizeof(joint)), zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::SUCCESS);
    EXPECT_EQ(device.joint_action_count, 0);
}

TEST(TopicZmqStep, StrictStopParksBeforeReturning) {
    CommandLineArgs cla = make_cla();
    cla.no_recovery_motion = true;
    TestDevice device(cla, 1);
    class ExposedTopic : public TopicZmq {
       public:
        using TopicZmq::TopicZmq;
        using Topic::process_leader_msg;
    };
    ExposedTopic topic(&device, cla, 0, nullptr);
    MsgCommand stop{};
    stop.command_ = DEVICE_COMMAND_STOP;
    EXPECT_EQ(topic.process_leader_msg(stop), ReturnCode::SUCCESS);
    EXPECT_EQ(device.park_count, 1);
    EXPECT_FALSE(topic.is_running());
}

TEST(TopicZmqStep, ReportsMalformedJointPayload) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);

    topic.sub_joint_.close();
    topic.sub_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string endpoint =
        "inproc://topic_zmq_step_malformed_joint_" + cla.device_id;
    topic.sub_joint_.bind(endpoint);
    zmq::socket_t sender(topic.context_, ZMQ_PAIR);
    sender.connect(endpoint);

    const uint8_t malformed_payload = 0;
    sender.send(zmq::buffer(topic.topic_joint_), zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&malformed_payload, sizeof(malformed_payload)),
                zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(device.joint_action_count, 0);
}

TEST(TopicZmqStep, EmptyJointTopicDoesNotStarveLaterValidFrame) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.set_pause_leader_command_listening(false);

    topic.sub_joint_.close();
    topic.sub_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string endpoint =
        "inproc://topic_zmq_step_empty_joint_topic_" + cla.device_id;
    topic.sub_joint_.bind(endpoint);
    zmq::socket_t sender(topic.context_, ZMQ_PAIR);
    sender.connect(endpoint);

    zmq::message_t empty_topic;
    ZmqJointInfo discarded_payload{};
    sender.send(empty_topic, zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&discarded_payload, sizeof(discarded_payload)),
                zmq::send_flags::none);

    ZmqJointInfo valid_payload{};
    valid_payload.msg_type = static_cast<int>(MsgType::JOINT_INFO);
    valid_payload.joint_num = 1;
    sender.send(zmq::buffer(topic.topic_joint_), zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&valid_payload, sizeof(valid_payload)),
                zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(device.joint_action_count, 1);
}

TEST(TopicZmqStep, ReportsEmptyLifecycleTopic) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);

    topic.sub_command_.close();
    topic.sub_command_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string endpoint =
        "inproc://topic_zmq_step_empty_lifecycle_topic_" + cla.device_id;
    topic.sub_command_.bind(endpoint);
    zmq::socket_t sender(topic.context_, ZMQ_PAIR);
    sender.connect(endpoint);

    zmq::message_t empty_topic;
    ZmqCommand payload{};
    sender.send(empty_topic, zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&payload, sizeof(payload)), zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqJoystickCount, RejectsInvalidCountsBeforeDispatch) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 0);
    TopicZmq topic(&device, cla, 0, nullptr);
    ZmqJoystickInfo msg{};

    msg.channel_num = static_cast<int>(sizeof(msg.channel) / sizeof(msg.channel[0])) + 1;
    EXPECT_EQ(
        topic.process_leader_msg_joystick(&msg, MsgJoystick::Side::LEFT),
        ReturnCode::INVALID_PARAM);

    msg.channel_num = 0;
    msg.button_num = -1;
    EXPECT_EQ(
        topic.process_leader_msg_joystick(&msg, MsgJoystick::Side::LEFT),
        ReturnCode::INVALID_PARAM);

    EXPECT_EQ(device.joystick_action_count, 0);
}

TEST(TopicZmqInitializationDeathTest, RejectsZeroControlFrequency) {
    CommandLineArgs cla = make_cla();
    cla.control_frequency = 0;
    TestDevice device(cla, 1);

    EXPECT_EXIT(
        {
            try {
                TopicZmq topic(&device, cla, 0, nullptr);
            } catch (const std::invalid_argument&) {
                _exit(0);
            } catch (...) {
                _exit(2);
            }
            _exit(1);
        },
        testing::ExitedWithCode(0), "");
}

TEST(TopicZmqInitializationDeathTest, RejectsNegativeControlFrequency) {
    CommandLineArgs cla = make_cla();
    cla.control_frequency = -1;
    TestDevice device(cla, 1);

    EXPECT_EXIT(
        {
            try {
                TopicZmq topic(&device, cla, 0, nullptr);
            } catch (const std::invalid_argument&) {
                _exit(0);
            } catch (...) {
                _exit(2);
            }
            _exit(1);
        },
        testing::ExitedWithCode(0), "");
}

}  // namespace

TEST(StrictClientWatchdog, ExpiredClientHeartbeatParksOnceWithoutRecovery) {
    auto cla = make_cla(); cla.no_recovery_motion = true;
    TestDevice device(cla, 1);
    // This unit test covers terminal retirement after an armed watchdog expires.
    // Process startup arming and exit codes require the separate node/client harness.
    device.note_client_heartbeat();
    device.expire_client_watchdog();
    EXPECT_EQ(device.park_count, 1);
    EXPECT_TRUE(device.is_in_emergency_recovery());
    EXPECT_TRUE(device.rejects_direct_commands());
    device.expire_client_watchdog();
    EXPECT_EQ(device.park_count, 1);
    EXPECT_EQ(device.joint_action_count, 0);
}
