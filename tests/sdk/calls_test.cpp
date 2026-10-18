// Every typed call: parameters against the manual's request templates (key
// names and order, values from its examples where it has them), replies of the
// shapes that need their own decoding, and the registry of all calls.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ec/sdk/all.hpp"
#include "ec/sdk/codec.hpp"

namespace {

using namespace ec::sdk;

template <Command C>
std::string params(const C& command) {
    return to_request(command).params.dump();
}

// Parses a response line as the controller sends it and decodes it as C's reply.
template <Command C>
std::expected<typename C::Reply, Error> decode_line(std::string_view line) {
    const auto response = parse_response(line);
    if (!response) return std::unexpected(response.error());
    if (!response->result) return std::unexpected(response->result.error());
    return decode_reply<C>(*response->result);
}

std::string reply(std::string_view result) {
    return std::string(R"({"jsonrpc":"2.0","result":)") + Json(std::string(result)).dump() + R"(,"id":1})";
}

constexpr JointsDeg kHome{0, -90, 0, -90, 90, 0};
constexpr const char* kHomeJson = "[0.0,-90.0,0.0,-90.0,90.0,0.0]";
constexpr PoseMmRad kPose{400, 0, 300, 3.14, 0, 1.57};
constexpr const char* kPoseJson = "[400.0,0.0,300.0,3.14,0.0,1.57]";

std::string with(std::string text) {  // substitutes the joint and pose placeholders
    for (auto [key, value] : {std::pair{"HOME", kHomeJson}, std::pair{"POSE", kPoseJson}}) {
        for (auto at = text.find(key); at != std::string::npos; at = text.find(key))
            text.replace(at, std::string_view(key).size(), value);
    }
    return text;
}

TEST(Calls, ServoAndParameterServiceParams) {
    EXPECT_EQ(params(SetServoStatus{.on = true}), R"({"status":1})");
    EXPECT_EQ(params(GetTcpPose{}), R"({"coordinate_num":-1,"tool_num":-1,"unit_type":1})");
    EXPECT_EQ(params(SetToolNumber{.tool = 3}), R"({"tool_num":3})");
    EXPECT_EQ(params(SetUserNumber{.user_frame = 2}), R"({"user_num":2})");
    EXPECT_EQ(params(SetCurrentCoord{.system = CoordinateSystem::Base}), R"({"coord_mode":1})");
    EXPECT_EQ(params(DragTeachSwitch{.on = true}), R"({"switch":1})");
    EXPECT_EQ(params(CmdSetPayload{.tool = 1, .mass_kg = 2.5, .cog_mm = {1, 2, 3}}),
              R"({"tool_num":1,"m":2.5,"cog":[1.0,2.0,3.0]})");
    EXPECT_EQ(params(CmdSetTcp{.tool = 0, .point = kPose}), with(R"({"point":POSE,"tool_num":0,"unit_type":1})"));
    EXPECT_EQ(params(GetUserFrame{.user_frame = 1}), R"({"user_num":1,"unit_type":1})");
    EXPECT_EQ(params(SetCycleMode{.mode = ec::CycleMode::Continuous}), R"({"cycle_mode":2})");
    EXPECT_EQ(params(SetUserFrame{.user_frame = 1, .frame = kPose}),
              with(R"({"user_num":1,"user_frame":POSE,"unit_type":1})"));
    EXPECT_EQ(params(GetTcpPos{.tool = 2}), R"({"tool_num":2,"unit_type":1})");
    EXPECT_EQ(params(GetPayload{.tool = 1}), R"({"tool_num":1})");
    EXPECT_EQ(params(GetCentreMass{.tool = 1}), R"({"tool_num":1})");
    EXPECT_EQ(params(SetCollisionEnable{.on = false}), R"({"enable":0})");
    EXPECT_EQ(params(SetCollisionSensitivity{.value_pct = 10}), R"({"value":10})");
    EXPECT_EQ(params(SetSafetyParams{.password = "123456",
                                     .power_w = 1500,
                                     .momentum_kg_m_s = 90,
                                     .tool_force_n = 400,
                                     .elbow_force_n = 400,
                                     .speed_pct = 100,
                                     .collision_detection = true,
                                     .collision_sensitivity_pct = 10}),
              R"({"password":"123456","enable":1,"mode":0,"power":1500.0,"momentum":90.0,"tool_force":400.0,)"
              R"("elbow_force":400.0,"speed":100.0,"collision_enable":1,"collision_sensitivity":10})");
    EXPECT_EQ(params(SetAutoRunToolNumber{.tool = 1}), R"({"tool_num":1})");
    EXPECT_EQ(params(GetBaseFlangePose{}), R"({"unit_type":1})");
    EXPECT_EQ(params(GetUserFlangePose{}), R"({"unit_type":1})");
    EXPECT_EQ(params(GetTcpForce{.in_tool_frame = false}), R"({"ref_tcp":0})");
    EXPECT_EQ(params(SetFlangeButton{.button = FlangeButton::Green, .function = FlangeButtonFunction::Drag}),
              R"({"button_num":1,"state":1})");
    EXPECT_EQ(params(CheckFlangeButton{.button = FlangeButton::Blue}), R"({"button_num":0})");
    EXPECT_EQ(params(GetTcpPoseInUser{}), R"({"unit_type":1})");
    EXPECT_EQ(params(GetToolPayload{.tool = 0}), R"({"tool_num":0})");
    EXPECT_EQ(params(GetActualTcp{}), "{}");
    EXPECT_EQ(params(GetTargetTcp{.tool = 1, .user_frame = 2}), R"({"tool_num":1,"user_num":2})");
    EXPECT_EQ(params(GetInterpPose{.from = kPose, .to = kPose, .ratio = 0.5}),
              with(R"({"data1":POSE,"data2":POSE,"ratio":0.5})"));
    EXPECT_EQ(params(InverseKinematicAll{.target = kPose}), with(R"({"targetPose":POSE})"));
    EXPECT_EQ(params(SetSystemTime{.time = {2026, 10, 12, 9, 30, 0}}), R"({"time":[2026,10,12,9,30,0]})");
    EXPECT_EQ(params(GetJointVersion{.axis = 0}), R"({"axis":0})");
}

TEST(Calls, MovementServiceParams) {
    // The vendor's example: moveByJoint with an input as the stop condition.
    EXPECT_EQ(params(MoveByJoint{.target = kHome,
                                 .speed_pct = 30,
                                 .acc_pct = 10,
                                 .dec_pct = 10,
                                 .until = StopCondition{.address = 7, .value = true}}),
              with(R"({"targetPos":HOME,"speed":30.0,"acc":10,"dec":10,"cond_type":0,"cond_num":7,"cond_value":1})"));
    EXPECT_EQ(params(MoveByJoint{
                  .target = kHome,
                  .speed_pct = 30,
                  .until = StopCondition{.source = StopCondition::Source::Expression, .expression = "IN#(4)==1"}}),
              with(R"({"targetPos":HOME,"speed":30.0,"cond_type":2,"cond_judgment":"IN#(4)==1"})"));
    EXPECT_EQ(params(MoveByLine{.target = kHome, .speed = 100}),
              with(R"({"targetPos":HOME,"speed_type":0,"speed":100.0})"));
    EXPECT_EQ(params(MoveByArc{.middle = kHome, .target = kHome, .speed = 50, .acc_pct = 20}),
              with(R"({"midPos":HOME,"targetPos":HOME,"speed_type":0,"speed":50.0,"acc":20})"));
    EXPECT_EQ(params(MoveByRotate{.target = kHome, .speed = 20}),
              with(R"({"targetPos":HOME,"speed_type":3,"speed":20.0})"));
    // The vendor's example: a joint waypoint blended at 50 mm, then a linear one.
    EXPECT_EQ(
        params(AddPathPoint{.point = kHome, .move_type = PathMoveType::Joint, .speed = 50, .blend_radius_mm = 50}),
        with(R"({"wayPoint":HOME,"moveType":0,"speed":50.0,"circular_radius":50.0})"));
    EXPECT_EQ(params(AddPathPoint{.point = kHome, .move_type = PathMoveType::Line, .speed = 50}),
              with(R"({"wayPoint":HOME,"moveType":1,"speed_type":0,"speed":50.0,"circular_radius":0.0})"));
    EXPECT_EQ(params(Jog{.index = 0, .speed_pct = 10}), R"({"index":0,"speed":10.0})");
    EXPECT_EQ(params(Jog{.index = 11}), R"({"index":11})");
    EXPECT_EQ(params(CheckJbiExist{.filename = "test"}), R"({"filename":"test"})");
    EXPECT_EQ(params(RunJbi{.filename = "test"}), R"({"filename":"test"})");
    EXPECT_EQ(params(SetSpeed{.value_pct = 50}), R"({"value":50.0})");
    // The vendor's example: every joint at 1 deg/s, acc 20, for 5 s.
    EXPECT_EQ(params(MoveBySpeedj{.velocity_deg_s = {1, 1, 1, 1, 1, 1}, .acc_deg_s2 = 20, .duration_s = 5}),
              R"({"vj":[1.0,1.0,1.0,1.0,1.0,1.0],"acc":20,"t":5.0})");
    EXPECT_EQ(params(Stopj{.acc_deg_s2 = 100}), R"({"acc":100})");
    EXPECT_EQ(params(MoveBySpeedl{
                  .velocity = {10, 0, 0, 0, 0, 0}, .acc_mm_s2 = 100, .acc_rot_deg_s2 = 50, .duration_s = 0.1}),
              R"({"v":[10.0,0.0,0.0,0.0,0.0,0.0],"acc":100,"arot":50,"t":0.1})");
    EXPECT_EQ(params(Stopl{.acc_mm_s2 = 100}), R"({"acc":100})");
    EXPECT_EQ(params(MoveByLineCoord{.target = kPose, .speed = 100, .user_frame = kPose}),
              with(R"({"targetUserPose":POSE,"speed_type":2,"speed":100.0,"user_coord":POSE,"unit_type":1})"));
}

TEST(Calls, StreamingAndTimestampServiceParams) {
    // The vendor's examples.
    EXPECT_EQ(
        params(TransparentTransmissionInit{.lookahead_ms = 400, .period_ms = 10, .smoothness = 0.1, .replies = true}),
        R"({"lookahead":400,"t":10,"smoothness":0.1,"response_enable":1})");
    EXPECT_EQ(params(TransparentTransmissionInit{}), R"({"lookahead":40,"t":8,"smoothness":0.1})");
    EXPECT_EQ(params(TtPutServoJointToBuf{.target = kHome}), with(R"({"targetPos":HOME})"));
    EXPECT_EQ(params(TtSetCurrentServoJoint{.target = kHome}), with(R"({"targetPos":HOME})"));
    EXPECT_EQ(params(StartPushPos{.point_count = 10, .point_type = PointType::Joint, .reference = kHome}),
              with(R"({"path_lenth":10,"pos_type":0,"ref_joint_pos":HOME,)"
                   R"("ref_frame":[0.0,0.0,0.0,0.0,0.0,0.0],"ret_flag":1})"));
    EXPECT_EQ(params(PushPos{.time_s = 0.002, .point = kHome}), with(R"({"timestamp":0.002,"pos":HOME})"));
    EXPECT_EQ(params(StartTrajectory{.speed_pct = 50}), R"({"speed_percent":50.0})");
}

TEST(Calls, KinematicsIoAndVariableParams) {
    EXPECT_EQ(params(InverseKinematic{.target = kPose, .reference = kHome}),
              with(R"({"targetPose":POSE,"referencePos":HOME,"unit_type":1})"));
    EXPECT_EQ(params(InverseKinematic{.target = kPose}), with(R"({"targetPose":POSE,"unit_type":1})"));
    EXPECT_EQ(params(PositiveKinematic{.joints = kHome}), with(R"({"targetPos":HOME,"unit_type":1})"));
    EXPECT_EQ(params(ConvertPoseFromCartToUser{.pose = kPose, .user_frame = 1}),
              with(R"({"targetPose":POSE,"userNo":1,"unit_type":1})"));
    EXPECT_EQ(params(ConvertPoseFromUserToCart{.pose = kPose, .user_frame = 2}),
              with(R"({"targetPose":POSE,"userNo":2,"unit_type":1})"));
    EXPECT_EQ(params(PoseMul{.first = kPose, .second = kPose}), with(R"({"pose1":POSE,"pose2":POSE,"unit_type":1})"));
    EXPECT_EQ(params(PoseInv{.pose = kPose}), with(R"({"pose":POSE,"unit_type":1})"));

    EXPECT_EQ(params(GetInput{.address = 3}), R"({"addr":3})");
    EXPECT_EQ(params(SetOutput{.address = 3, .on = true}), R"({"addr":3,"status":1})");
    EXPECT_EQ(params(SetVirtualOutput{.address = 600, .on = false}), R"({"addr":600,"status":0})");
    EXPECT_EQ(params(GetRegisters{.address = 0, .length = 2}), R"({"addr":0,"len":2})");
    EXPECT_EQ(params(GetAnalogOutput{.address = 1}), R"({"addr":1})");
    EXPECT_EQ(params(SetAnalogOutput{.address = 4, .value_v = 5}), R"({"addr":4,"value":5.0})");

    EXPECT_EQ(params(SetSysVarB{.address = 1, .value = 7}), R"({"addr":1,"value":7})");
    EXPECT_EQ(params(SetSysVarD{.address = 2, .value = 0.5}), R"({"addr":2,"value":0.5})");
    EXPECT_EQ(params(SetSysVarP{.address = 0, .joints = kHome}), with(R"({"addr":0,"pos":HOME})"));  // vendor's example
    EXPECT_EQ(params(SetSysVarV{.address = 0, .pose = kPose}), with(R"({"addr":0,"pose":POSE})"));
}

TEST(Calls, FieldbusSensorPortAndForceControlParams) {
    EXPECT_EQ(params(GetProfinetIntInputRegisters{}), R"({"addr":0,"length":1})");
    EXPECT_EQ(params(SetProfinetFloatOutputRegisters{.address = 0, .values = {1.5, 2.5}}),
              R"({"addr":0,"length":2,"value":[1.5,2.5]})");
    EXPECT_EQ(to_request(SetProfinetFloatOutputRegisters{}).method, "set_profinet_float_output_registers");
    // The vendor's example.
    EXPECT_EQ(params(SetEipIntOutputRegisters{.address = 1, .values = {1, 1}}),
              R"({"addr":1,"length":2,"value":[1,1]})");

    EXPECT_EQ(params(StartPushForce{.replies = true}), R"({"return_flag":true})");
    EXPECT_EQ(params(PushExternalForce{.index = 3, .wrench_kg_kgm = {1, 2, 3, 0.1, 0.2, 0.3}}),
              R"({"index":3,"torque_arry":[1.0,2.0,3.0,0.1,0.2,0.3]})");

    EXPECT_EQ(params(OpenSerialPort{.rs485 = true}), R"({"device_type":1})");
    EXPECT_EQ(params(SetoptSerialPort{.settings = {.baud_rate = 9600, .data_bits = 8, .parity = 'N', .stop_bits = 1}}),
              R"({"baud_rate":9600,"bits":8,"event":"N","stop":1})");  // the template's values
    EXPECT_EQ(params(SetoptTci{}), "{}");
    // The vendor's examples.
    EXPECT_EQ(params(RecvSerialPort{.count = 100, .hex = false, .timeout_ms = 10}),
              R"({"count":100,"hex":0,"timeout":10})");
    EXPECT_EQ(params(SendSerialPort{.data = "1024", .hex = false}), R"({"send_buf":"1024","hex":0})");
    EXPECT_EQ(params(RecvTci{.count = 5}), R"({"count":5})");

    EXPECT_EQ(params(StartForceMode{}), "{}");
    EXPECT_EQ(params(StartForceMode{.mode = 3,
                                    .frame = kPose,
                                    .compliance = std::array<int, 6>{6, 1, 5, 1, 0, 1},
                                    .target_wrench = std::array<double, 6>{-10, -10, -10, 1.3, 1.3, 1.3},
                                    .speed_limits = std::array<double, 6>{100, 100, 100, 5.73, 5.73, 5.73}}),
              with(R"({"mode":3,"arr_frame":POSE,"arr_optional":[6,1,5,1,0,1],)"
                   R"("arr_torque":[-10.0,-10.0,-10.0,1.3,1.3,1.3],"arr_speed":[100.0,100.0,100.0,5.73,5.73,5.73]})"));
}

TEST(Calls, SerialPortReplies) {
    EXPECT_EQ(decode_line<RecvSerialPort>(reply(R"({"result":true,"size":4,"buf":"1024"})")),
              (PortRead{true, 4, "1024"}));
    EXPECT_EQ(decode_line<RecvTci>(reply(R"({"result":false,"size":0,"buf":""})")), (PortRead{false, 0, ""}));
    EXPECT_EQ(decode_line<SendSerialPort>(reply(R"({"result":true,"size":4})")), (PortWritten{true, 4}));
    EXPECT_FALSE(decode_line<SendTci>(reply("true")));
}

TEST(Calls, SafetyLimitsLeftOutAreZeroSoTheControllerRejectsThem) {
    SetSafetyParams omitted{};  // the compiler warns about omitted limits in a designated initializer
    omitted.password = "123456";
    const auto p = to_request(omitted).params;
    EXPECT_EQ(p["enable"], 1);
    EXPECT_EQ(p["power"], 0.0);  // outside 80..1500
    EXPECT_EQ(p["tool_force"], 0.0);
}

TEST(Calls, CommandsWithoutParamsSendNone) {
    EXPECT_TRUE(to_request(ClearAlarm{}).params.is_null());
    EXPECT_TRUE(to_request(TtClearServoJointBuf{}).params.is_null());
    EXPECT_TRUE(to_request(FlushTrajectory{}).params.is_null());
    EXPECT_EQ(encode(to_request(SyncMotorStatus{}), 3), R"({"jsonrpc":"2.0","method":"syncMotorStatus","id":3})"
                                                        "\n");
}

TEST(Calls, ZeroOrOneFlags) {
    EXPECT_EQ(decode_line<GetCollisionState>(reply("1")), Flag::On);
    EXPECT_EQ(decode_line<GetEstopStatus>(reply("0")), Flag::Off);
    EXPECT_EQ(decode_line<GetRobotSafetyParamsEnabled>(reply("0")), Flag::Off);  // as read on the EA66
    for (std::string_view bad : {"2", "true", "\"1\""}) {
        const auto f = decode_line<GetCollisionState>(reply(bad));
        ASSERT_FALSE(f) << bad;
        EXPECT_EQ(f.error().kind, Error::Kind::BadReply) << bad;
    }
}

TEST(Calls, EnumsRejectUndefinedValues) {
    EXPECT_EQ(decode_line<GetCurrentCoord>(reply("4")), CoordinateSystem::Cylindrical);
    EXPECT_FALSE(decode_line<GetCurrentCoord>(reply("5")));
    EXPECT_EQ(decode_line<CheckTrajectory>(reply("0")), TrajectoryCheck::Valid);
    EXPECT_EQ(decode_line<CheckTrajectory>(reply("-3")), TrajectoryCheck::BadTimestamps);
    EXPECT_FALSE(decode_line<CheckTrajectory>(reply("1")));
    EXPECT_FALSE(decode_line<CheckTrajectory>(reply("-4")));
    EXPECT_EQ(decode_line<CheckFlangeButton>(reply("2")), FlangeButtonFunction::RecordPoint);
}

TEST(Calls, RepliesWithTheirOwnShape) {
    EXPECT_EQ(decode_line<GetJbiState>(reply(R"({"jbiName":"test","runState":3})")),
              (JbiState{"test", JbiRunState::Running}));
    EXPECT_FALSE(decode_line<GetJbiState>(reply(R"({"jbiName":"test"})")));
    EXPECT_FALSE(decode_line<GetJbiState>(reply(R"({"jbiName":"test","runState":7})")));

    EXPECT_EQ(decode_line<GetAlarmNum>(reply("[11,12,13,14,15]")), (std::vector<std::int64_t>{11, 12, 13, 14, 15}));
    const auto no_alarms = decode_line<GetAlarmNum>(reply("false"));
    ASSERT_FALSE(no_alarms);
    EXPECT_EQ(no_alarms.error().kind, Error::Kind::Refused);

    const ToolPayload payload{1.5, {0, 0, 50}};
    EXPECT_EQ(decode_line<GetToolPayload>(reply(R"({"m":1.5,"tool_cog":[0,0,50]})")), payload);
    EXPECT_EQ(decode_line<GetToolPayload>(reply("[1.5,[0,0,50]]")), payload);
    EXPECT_FALSE(decode_line<GetToolPayload>(reply("1.5")));

    EXPECT_EQ(decode_line<MoveByPath>(reply("4")), 4);
    const auto failed = decode_line<MoveByPath>(reply("-1"));
    ASSERT_FALSE(failed);
    EXPECT_EQ(failed.error().kind, Error::Kind::Refused);

    EXPECT_EQ(decode_line<GetJointVersion>(reply("\"V11.2\"")), "V11.2");
    EXPECT_EQ(decode_line<GetJointVersion>(reply("11")), "11");

    // Normal mode, then reduced mode, as read on the EA66.
    EXPECT_EQ(decode_line<GetRobotSafetyPower>(reply("[1500,1500]")), (std::array<double, 2>{1500, 1500}));
    EXPECT_EQ(decode_line<GetRobotSpeedPercentage>(reply("[100,30]")), (std::array<double, 2>{100, 30}));
    EXPECT_EQ(decode_line<InverseKinematicAll>(reply("[[1,2,3,4,5,6]]")), Json::parse("[[1,2,3,4,5,6]]"));
}

TEST(Calls, TheRegistryHasEveryTypedCallOnce) {
    const auto all = all_commands();
    EXPECT_EQ(all.size(), AllCommands::size);
    EXPECT_EQ(all.size(), 184u);  // every call of the 3.11.2 manual
    std::set<std::string_view> names;
    for (const auto& c : all) {
        EXPECT_FALSE(c.method.empty());
        EXPECT_TRUE(names.insert(c.method).second) << c.method << " is listed twice";
    }
    std::size_t visited = 0;
    AllCommands::for_each([&]<Command C>() {
        if (find_command(C::method) != nullptr) ++visited;
    });
    EXPECT_EQ(visited, all.size());
}

TEST(Calls, TheRegistryFindsByNameAndChecksReplies) {
    const auto* state = find_command("getRobotState");
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->kind, Kind::Query);
    EXPECT_TRUE(state->check_reply(Json(3)));
    EXPECT_FALSE(state->check_reply(Json(9)));  // not a robot state
    EXPECT_FALSE(state->check_reply(Json("x")));

    const auto* move = find_command("moveByJoint");
    ASSERT_NE(move, nullptr);
    EXPECT_EQ(move->kind, Kind::Motion);
    EXPECT_FALSE(is_read_only(move->kind));
    EXPECT_EQ(move->example().method, "moveByJoint");
    EXPECT_TRUE(is_read_only(find_command("poseMul")->kind));
    EXPECT_EQ(find_command("getDH")->example().params, Json::parse(R"({"index":0})"));
    EXPECT_EQ(find_command("noSuchCall"), nullptr);
}

}  // namespace
