#include <AP_gtest.h>

#include "Vektor_AssignmentMatrix.h"
#include "Vektor_Attitude.h"
#include "Vektor_Protocol.h"
#include "Vektor_PwmInput.h"
#include "Vektor_PwmOutput.h"
#include "Vektor_Rcin.h"
#include "Vektor_RequestCache.h"
#include "Vektor_Runtime.h"
#include "Vektor_Schema.h"
#include "Vektor_SerialCatalog.h"
#include "Vektor_SerialProtocol.h"
#include "Vektor_Subscription.h"
#include "Vektor_Vsp.h"

#include <AP_HAL/AP_HAL.h>

#include <math.h>
#include <string.h>
#include <unistd.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace {

constexpr Vektor::TimerGroup h743_pwm_timer_groups[] = {
    { "TIM2", 2 },
    { "TIM4", 4 },
    { "TIM8", 4 },
    { "TIM1", 2 },
};

constexpr Vektor::TimerGroup h743_flex_timer_groups[] = {
    { "TIM5", 2 },
    { "TIM3", 4 },
};

constexpr Vektor::TimerGroup f405_pwm_timer_groups[] = {
    { "TIM3", 2 },
    { "TIM2", 4 },
};

constexpr Vektor::BoardCapability h743_test_capability {
    "Vektor Core Evo H743",
    "test",
    "STM32H743VIT6",
    0,
    12,
    6,
    1,
    4,
    2,
    1,
    0,
    "ICM-20602",
    "MMC5983MA",
    Vektor::CAP_NATIVE_USB |
        Vektor::CAP_PWM_OUTPUTS |
        Vektor::CAP_FLEX_TIMER_CHANNELS |
        Vektor::CAP_DEDICATED_RECEIVER_ROW |
        Vektor::CAP_EXTERNAL_UARTS |
        Vektor::CAP_CLASSIC_CAN |
        Vektor::CAP_ADC_OBSERVABLES |
        Vektor::CAP_ONBOARD_IMU |
        Vektor::CAP_ONBOARD_COMPASS,
    h743_pwm_timer_groups,
    4,
    h743_flex_timer_groups,
    2,
    -1,
};

constexpr Vektor::BoardCapability f405_test_capability {
    "Vektor Core Reduced F405",
    "revo-mini",
    "STM32F405xx",
    124,
    6,
    0,
    0,
    3,
    0,
    3,
    2,
    "BMI088",
    "Onboard Compass",
    Vektor::CAP_NATIVE_USB |
        Vektor::CAP_PWM_OUTPUTS |
        Vektor::CAP_EXTERNAL_UARTS |
        Vektor::CAP_ADC_OBSERVABLES |
        Vektor::CAP_STORAGE |
        Vektor::CAP_ONBOARD_IMU |
        Vektor::CAP_SD_STORAGE |
        Vektor::CAP_DATAFLASH_STORAGE,
    f405_pwm_timer_groups,
    2,
    nullptr,
    0,
    0,
};

template<typename ReferenceType, size_t count>
void expect_catalog_fields_match_schema(
    const ReferenceType *const (&references)[count])
{
    const Vektor::SchemaRegistry &registry = Vektor::schema_registry();
    for (const ReferenceType *reference : references) {
        ASSERT_NE(reference, nullptr);
        const Vektor::FieldDescriptor *field =
            registry.field_by_id(reference->id);
        ASSERT_NE(field, nullptr) << reference->path;
        EXPECT_STREQ(field->path, reference->path);
        EXPECT_EQ(reference->id, Vektor::Protocol::fnv1a32(reference->path));
        EXPECT_EQ(reference->type, field->type);
    }
}

class TestUart : public AP_HAL::UARTDriver {
public:
    TestUart()
    {
        lock_write_key = 0;
        lock_read_key = 0;
        parity = 0;
        _last_options = 0;
    }

    bool is_initialized() override { return _initialized; }
    bool tx_pending() override { return false; }
    uint32_t txspace() override { return sizeof(_tx) - _tx_len; }

    void reset()
    {
        lock_write_key = 0;
        lock_read_key = 0;
        parity = 0;
        _last_options = 0;
        _initialized = false;
        _rx_offset = 0;
        _rx_len = 0;
        _tx_len = 0;
    }

    bool push_rx(const uint8_t *data, uint16_t length)
    {
        if (data == nullptr || length > sizeof(_rx) - _rx_len) {
            return false;
        }
        memcpy(&_rx[_rx_len], data, length);
        _rx_len += length;
        return true;
    }

    void clear_tx() { _tx_len = 0; }
    const uint8_t *tx_data() const { return _tx; }
    uint16_t tx_length() const { return _tx_len; }

protected:
    void _begin(uint32_t, uint16_t, uint16_t) override
    {
        _initialized = true;
    }

    size_t _write(const uint8_t *buffer, size_t size) override
    {
        if (buffer == nullptr || size > sizeof(_tx) - _tx_len) {
            return 0;
        }
        memcpy(&_tx[_tx_len], buffer, size);
        _tx_len += size;
        return size;
    }

    ssize_t _read(uint8_t *buffer, uint16_t count) override
    {
        const uint16_t available = _rx_len - _rx_offset;
        const uint16_t to_read = count < available ? count : available;
        if (to_read != 0) {
            memcpy(buffer, &_rx[_rx_offset], to_read);
            _rx_offset += to_read;
        }
        if (_rx_offset == _rx_len) {
            _rx_offset = 0;
            _rx_len = 0;
        }
        return to_read;
    }

    void _end() override { _initialized = false; }
    void _flush() override {}
    uint32_t _available() override { return _rx_len - _rx_offset; }

    bool _discard_input() override
    {
        _rx_offset = 0;
        _rx_len = 0;
        return true;
    }

private:
    bool _initialized = false;
    uint8_t _rx[1024] {};
    uint16_t _rx_offset = 0;
    uint16_t _rx_len = 0;
    uint8_t _tx[2048] {};
    uint16_t _tx_len = 0;
};

bool push_request(TestUart &uart,
                  Vektor::Protocol::MessageType type,
                  uint16_t sequence,
                  const uint8_t *payload,
                  uint16_t payload_len)
{
    uint8_t decoded[Vektor::Protocol::MAX_DECODED_FRAME_SIZE];
    uint8_t encoded[Vektor::Protocol::MAX_ENCODED_STREAM_SIZE];
    uint16_t encoded_len = 0;
    if (!Vektor::Protocol::build_frame(type,
                                       0,
                                       sequence,
                                       payload,
                                       payload_len,
                                       decoded,
                                       sizeof(decoded),
                                       encoded,
                                       sizeof(encoded),
                                       encoded_len)) {
        return false;
    }
    return uart.push_rx(encoded, encoded_len);
}

bool parse_single_frame(const TestUart &uart,
                        Vektor::Protocol::Parser &parser,
                        Vektor::Protocol::FrameView &frame)
{
    uint16_t frames = 0;
    for (uint16_t i = 0; i < uart.tx_length(); i++) {
        const Vektor::Protocol::ParseResult result =
            parser.consume(uart.tx_data()[i], frame);
        if (result == Vektor::Protocol::ParseResult::FRAME) {
            frames++;
        } else if (result != Vektor::Protocol::ParseResult::NONE &&
                   result != Vektor::Protocol::ParseResult::EMPTY) {
            return false;
        }
    }
    return frames == 1;
}

bool skip_str8(Vektor::Protocol::PayloadReader &reader)
{
    uint8_t length = 0;
    const uint8_t *bytes = nullptr;
    return reader.u8(length) && reader.bytes(bytes, length);
}

bool hello_hashes_for_capability(const Vektor::BoardCapability &capability,
                                 uint64_t &schema_hash,
                                 uint64_t &capability_hash)
{
    static TestUart uart;
    uart.reset();
    Vektor::Parameters parameters;
    parameters.sys_protocol_baud.set(Vektor::protocol_baud);
    Vektor::RuntimeState runtime;
    runtime.init(Vektor::default_service_rate_hz, AP_HAL::micros64());
    Vektor::AttitudeSource attitude;
    attitude.reset();
    Vektor::VspComponent vsp;
    vsp.reset();
    Vektor::RcinSource rcin;
    rcin.reset();
    Vektor::PwmInput pwm_input;
    Vektor::PwmOutput pwm_output;
    Vektor::AssignmentMatrix assignments;
    Vektor::SerialProtocol serial;
    serial.init(&uart,
                capability,
                parameters,
                runtime,
                attitude,
                vsp,
                rcin,
                pwm_input,
                pwm_output,
                assignments);
    if (!uart.is_initialized()) {
        return false;
    }
    uart.clear_tx();

    uint8_t payload[Vektor::Protocol::MAX_PAYLOAD_SIZE];
    Vektor::Protocol::PayloadWriter hello(payload, sizeof(payload));
    hello.u8(1);
    hello.u8(1);
    hello.u16(0);
    hello.u32(0);
    hello.u32(1);
    if (!push_request(uart,
                      Vektor::Protocol::MessageType::HELLO,
                      1,
                      hello.data(),
                      hello.length())) {
        return false;
    }
    serial.update();

    Vektor::Protocol::Parser parser;
    Vektor::Protocol::FrameView response {};
    if (!parse_single_frame(uart, parser, response) ||
        response.message_type != Vektor::Protocol::MessageType::HELLO) {
        return false;
    }

    Vektor::Protocol::PayloadReader reader(response.payload,
                                           response.payload_len);
    uint8_t major = 0;
    uint16_t minor = 0;
    uint32_t board_id = 0;
    return reader.u8(major) &&
           reader.u16(minor) &&
           skip_str8(reader) &&
           skip_str8(reader) &&
           skip_str8(reader) &&
           reader.u32(board_id) &&
           reader.u64(schema_hash) &&
           reader.u64(capability_hash);
}

void expect_stream_parses(const uint8_t *stream,
                          uint16_t stream_len,
                          Vektor::Protocol::MessageType expected_type,
                          uint16_t expected_sequence,
                          const uint8_t *expected_payload,
                          uint16_t expected_payload_len)
{
    Vektor::Protocol::Parser parser;
    Vektor::Protocol::FrameView frame {};
    Vektor::Protocol::ParseResult result =
        Vektor::Protocol::ParseResult::NONE;

    for (uint16_t i = 0; i < stream_len; i++) {
        result = parser.consume(stream[i], frame);
    }

    EXPECT_EQ(result, Vektor::Protocol::ParseResult::FRAME);
    EXPECT_EQ(frame.version, Vektor::Protocol::MAJOR_VERSION);
    EXPECT_EQ(frame.message_type, expected_type);
    EXPECT_EQ(frame.flags, 0);
    EXPECT_EQ(frame.sequence, expected_sequence);
    EXPECT_EQ(frame.payload_len, expected_payload_len);
    EXPECT_EQ(memcmp(frame.payload, expected_payload, expected_payload_len), 0);
}

} // namespace

TEST(VektorProtocol, SelfTestVectorPasses)
{
    EXPECT_TRUE(Vektor::Protocol::self_test());
}

TEST(VektorProtocol, CobsRoundTripPreservesZerosAndLongRuns)
{
    uint8_t decoded[260];
    for (uint16_t i = 0; i < sizeof(decoded); i++) {
        decoded[i] = uint8_t(i + 1);
    }
    decoded[0] = 0;
    decoded[17] = 0;
    decoded[254] = 0;

    uint8_t encoded[Vektor::Protocol::MAX_ENCODED_BODY_SIZE];
    uint16_t encoded_len = 0;
    ASSERT_TRUE(Vektor::Protocol::cobs_encode(decoded,
                                              sizeof(decoded),
                                              encoded,
                                              sizeof(encoded),
                                              encoded_len));

    for (uint16_t i = 0; i < encoded_len; i++) {
        EXPECT_NE(encoded[i], 0);
    }

    uint8_t roundtrip[sizeof(decoded)];
    uint16_t roundtrip_len = 0;
    ASSERT_TRUE(Vektor::Protocol::cobs_decode(encoded,
                                              encoded_len,
                                              roundtrip,
                                              sizeof(roundtrip),
                                              roundtrip_len));
    EXPECT_EQ(roundtrip_len, sizeof(decoded));
    EXPECT_EQ(memcmp(decoded, roundtrip, sizeof(decoded)), 0);
}

TEST(VektorProtocol, BuildFrameMatchesCanonicalPingVector)
{
    static const uint8_t payload[] = {
        0x78, 0x56, 0x34, 0x12,
        0x08, 0x07, 0x06, 0x05,
        0x04, 0x03, 0x02, 0x01,
    };
    static const uint8_t expected_stream[] = {
        0x03, 0x01, 0x03, 0x04, 0x34, 0x12, 0x0C, 0x11,
        0x78, 0x56, 0x34, 0x12, 0x08, 0x07, 0x06, 0x05,
        0x04, 0x03, 0x02, 0x01, 0x56, 0xB8, 0xD9, 0x95,
        0x00,
    };

    uint8_t decoded[Vektor::Protocol::MAX_DECODED_FRAME_SIZE];
    uint8_t encoded[Vektor::Protocol::MAX_ENCODED_STREAM_SIZE];
    uint16_t encoded_len = 0;
    ASSERT_TRUE(Vektor::Protocol::build_frame(
        Vektor::Protocol::MessageType::PING,
        0,
        0x1234,
        payload,
        sizeof(payload),
        decoded,
        sizeof(decoded),
        encoded,
        sizeof(encoded),
        encoded_len));

    EXPECT_EQ(encoded_len, sizeof(expected_stream));
    EXPECT_EQ(memcmp(encoded, expected_stream, sizeof(expected_stream)), 0);
    expect_stream_parses(encoded,
                         encoded_len,
                         Vektor::Protocol::MessageType::PING,
                         0x1234,
                         payload,
                         sizeof(payload));
}

TEST(VektorProtocol, ParserIgnoresLeadingDelimiterAndParsesSequentialFrames)
{
    static const uint8_t payload_a[] = { 1, 2, 3 };
    static const uint8_t payload_b[] = { 4, 5 };
    uint8_t decoded[Vektor::Protocol::MAX_DECODED_FRAME_SIZE];
    uint8_t frame_a[Vektor::Protocol::MAX_ENCODED_STREAM_SIZE];
    uint8_t frame_b[Vektor::Protocol::MAX_ENCODED_STREAM_SIZE];
    uint16_t frame_a_len = 0;
    uint16_t frame_b_len = 0;

    ASSERT_TRUE(Vektor::Protocol::build_frame(
        Vektor::Protocol::MessageType::PING,
        0,
        7,
        payload_a,
        sizeof(payload_a),
        decoded,
        sizeof(decoded),
        frame_a,
        sizeof(frame_a),
        frame_a_len));
    ASSERT_TRUE(Vektor::Protocol::build_frame(
        Vektor::Protocol::MessageType::GET,
        0,
        8,
        payload_b,
        sizeof(payload_b),
        decoded,
        sizeof(decoded),
        frame_b,
        sizeof(frame_b),
        frame_b_len));

    Vektor::Protocol::Parser parser;
    Vektor::Protocol::FrameView frame {};
    EXPECT_EQ(parser.consume(Vektor::Protocol::DELIMITER, frame),
              Vektor::Protocol::ParseResult::EMPTY);

    for (uint16_t i = 0; i < frame_a_len - 1; i++) {
        EXPECT_EQ(parser.consume(frame_a[i], frame),
                  Vektor::Protocol::ParseResult::NONE);
    }
    EXPECT_EQ(parser.consume(frame_a[frame_a_len - 1], frame),
              Vektor::Protocol::ParseResult::FRAME);
    EXPECT_EQ(frame.message_type, Vektor::Protocol::MessageType::PING);
    EXPECT_EQ(frame.sequence, 7);
    EXPECT_EQ(frame.payload_len, sizeof(payload_a));
    EXPECT_EQ(memcmp(frame.payload, payload_a, sizeof(payload_a)), 0);

    for (uint16_t i = 0; i < frame_b_len - 1; i++) {
        EXPECT_EQ(parser.consume(frame_b[i], frame),
                  Vektor::Protocol::ParseResult::NONE);
    }
    EXPECT_EQ(parser.consume(frame_b[frame_b_len - 1], frame),
              Vektor::Protocol::ParseResult::FRAME);
    EXPECT_EQ(frame.message_type, Vektor::Protocol::MessageType::GET);
    EXPECT_EQ(frame.sequence, 8);
    EXPECT_EQ(frame.payload_len, sizeof(payload_b));
    EXPECT_EQ(memcmp(frame.payload, payload_b, sizeof(payload_b)), 0);
}

TEST(VektorProtocol, ParserRejectsBadCrcAndRecovers)
{
    static const uint8_t payload[] = { 9, 8, 7, 6 };
    uint8_t decoded[Vektor::Protocol::MAX_DECODED_FRAME_SIZE];
    uint8_t encoded[Vektor::Protocol::MAX_ENCODED_STREAM_SIZE];
    uint16_t encoded_len = 0;
    ASSERT_TRUE(Vektor::Protocol::build_frame(
        Vektor::Protocol::MessageType::PING,
        0,
        11,
        payload,
        sizeof(payload),
        decoded,
        sizeof(decoded),
        encoded,
        sizeof(encoded),
        encoded_len));

    uint8_t decoded_bad[Vektor::Protocol::MAX_DECODED_FRAME_SIZE];
    uint16_t decoded_bad_len = 0;
    ASSERT_TRUE(Vektor::Protocol::cobs_decode(encoded,
                                              encoded_len - 1,
                                              decoded_bad,
                                              sizeof(decoded_bad),
                                              decoded_bad_len));
    decoded_bad[decoded_bad_len - 1] ^= 0x01;

    uint8_t bad_stream[Vektor::Protocol::MAX_ENCODED_STREAM_SIZE];
    uint16_t bad_body_len = 0;
    ASSERT_TRUE(Vektor::Protocol::cobs_encode(decoded_bad,
                                              decoded_bad_len,
                                              bad_stream,
                                              sizeof(bad_stream) - 1,
                                              bad_body_len));
    bad_stream[bad_body_len] = Vektor::Protocol::DELIMITER;

    Vektor::Protocol::Parser parser;
    Vektor::Protocol::FrameView frame {};
    for (uint16_t i = 0; i < bad_body_len; i++) {
        EXPECT_EQ(parser.consume(bad_stream[i], frame),
                  Vektor::Protocol::ParseResult::NONE);
    }
    EXPECT_EQ(parser.consume(Vektor::Protocol::DELIMITER, frame),
              Vektor::Protocol::ParseResult::BAD_CRC);

    for (uint16_t i = 0; i < encoded_len - 1; i++) {
        EXPECT_EQ(parser.consume(encoded[i], frame),
                  Vektor::Protocol::ParseResult::NONE);
    }
    EXPECT_EQ(parser.consume(encoded[encoded_len - 1], frame),
              Vektor::Protocol::ParseResult::FRAME);
    EXPECT_EQ(frame.sequence, 11);
}

TEST(VektorProtocol, PayloadReaderWriterRoundTrip)
{
    uint8_t buffer[32];
    Vektor::Protocol::PayloadWriter writer(buffer, sizeof(buffer));

    EXPECT_TRUE(writer.u8(0x12));
    EXPECT_TRUE(writer.u16(0x3456));
    EXPECT_TRUE(writer.u32(0x789ABCDEU));
    EXPECT_TRUE(writer.u64(0x0102030405060708ULL));
    EXPECT_TRUE(writer.bytes(reinterpret_cast<const uint8_t *>("ok"), 2));
    ASSERT_TRUE(writer.ok());

    Vektor::Protocol::PayloadReader reader(writer.data(), writer.length());
    uint8_t u8 = 0;
    uint16_t u16 = 0;
    uint32_t u32 = 0;
    uint64_t u64 = 0;
    const uint8_t *bytes = nullptr;

    EXPECT_TRUE(reader.u8(u8));
    EXPECT_TRUE(reader.u16(u16));
    EXPECT_TRUE(reader.u32(u32));
    EXPECT_TRUE(reader.u64(u64));
    EXPECT_TRUE(reader.bytes(bytes, 2));
    EXPECT_EQ(u8, 0x12);
    EXPECT_EQ(u16, 0x3456);
    EXPECT_EQ(u32, 0x789ABCDEU);
    EXPECT_EQ(u64, 0x0102030405060708ULL);
    EXPECT_EQ(memcmp(bytes, "ok", 2), 0);
    EXPECT_EQ(reader.remaining(), 0);
}

TEST(VektorProtocol, PayloadWriterMarksOverflow)
{
    uint8_t buffer[3];
    Vektor::Protocol::PayloadWriter writer(buffer, sizeof(buffer));
    EXPECT_FALSE(writer.u32(1));
    EXPECT_FALSE(writer.ok());
}

TEST(VektorProtocol, Fnv64SupportsDeterministicIncrementalHashing)
{
    static const uint8_t value[] = "descriptor-stream";
    const uint16_t value_len = sizeof(value) - 1;
    const uint64_t one_shot = Vektor::Protocol::fnv1a64(value, value_len);

    uint64_t incremental = 0xCBF29CE484222325ULL;
    incremental = Vektor::Protocol::fnv1a64_update(incremental, value, 5);
    incremental = Vektor::Protocol::fnv1a64_update(incremental,
                                                   &value[5],
                                                   value_len - 5);
    EXPECT_EQ(incremental, one_shot);
    EXPECT_NE(one_shot, 0U);
}

TEST(VektorProtocol, StableIdsMustBeUniqueAndNonzero)
{
    const uint32_t valid[] = { 1, 0x12345678, UINT32_MAX };
    const uint32_t duplicate[] = { 1, 2, 1 };
    const uint32_t zero[] = { 1, 0, 2 };

    EXPECT_TRUE(Vektor::Protocol::stable_ids_unique_nonzero(valid, 3));
    EXPECT_FALSE(Vektor::Protocol::stable_ids_unique_nonzero(duplicate, 3));
    EXPECT_FALSE(Vektor::Protocol::stable_ids_unique_nonzero(zero, 3));
    EXPECT_FALSE(Vektor::Protocol::stable_ids_unique_nonzero(nullptr, 1));
    EXPECT_TRUE(Vektor::Protocol::stable_ids_unique_nonzero(nullptr, 0));
}

TEST(VektorSchemaRegistry, ExposesStableComponentsAndFields)
{
    const Vektor::SchemaRegistry &registry = Vektor::schema_registry();
    EXPECT_EQ(registry.component_count(), 7);
    EXPECT_EQ(registry.field_count(), 102);
    EXPECT_EQ(registry.parameter_count(), 22);

    EXPECT_EQ(registry.component_id(0),
              Vektor::Protocol::fnv1a32("component/system/0"));
    EXPECT_EQ(registry.component_type_id(0),
              Vektor::Protocol::fnv1a32(
                  "component_type/system/protocol"));
    EXPECT_EQ(registry.component_id(registry.component_count()), 0U);

    const uint32_t uptime_id = Vektor::Protocol::fnv1a32(
        "component/system/1/observable/uptime_ms");
    const Vektor::FieldDescriptor *uptime = registry.field_by_id(uptime_id);
    ASSERT_NE(uptime, nullptr);
    EXPECT_EQ(uptime->slot, Vektor::FieldSlot::UPTIME_MS);
    EXPECT_EQ(uptime->kind, Vektor::Protocol::FieldKind::OBSERVABLE);
    EXPECT_EQ(uptime->type, Vektor::Protocol::PrimitiveType::U32);
    EXPECT_NE(uptime->flags & Vektor::FIELD_READABLE, 0U);
    EXPECT_NE(uptime->flags & Vektor::FIELD_REALTIME, 0U);
    EXPECT_EQ(registry.field_by_id(0), nullptr);

    const uint32_t servo_a_id = Vektor::Protocol::fnv1a32(
        "component/vsp/1/output/servo_a");
    const Vektor::FieldDescriptor *servo_a =
        registry.field_by_id(servo_a_id);
    ASSERT_NE(servo_a, nullptr);
    EXPECT_EQ(servo_a->kind, Vektor::Protocol::FieldKind::OUTPUT);
    EXPECT_EQ(servo_a->type, Vektor::Protocol::PrimitiveType::FLOAT32);
    EXPECT_NE(servo_a->flags & Vektor::FIELD_ROUTABLE, 0U);

    const uint32_t rcin_channel_id = Vektor::Protocol::fnv1a32(
        "component/rcin/0/output/channel_1");
    const Vektor::FieldDescriptor *rcin_channel =
        registry.field_by_id(rcin_channel_id);
    ASSERT_NE(rcin_channel, nullptr);
    EXPECT_EQ(rcin_channel->slot, Vektor::FieldSlot::RCIN_CHANNEL_1);
    EXPECT_EQ(rcin_channel->kind, Vektor::Protocol::FieldKind::OUTPUT);
    EXPECT_EQ(rcin_channel->type,
              Vektor::Protocol::PrimitiveType::FLOAT32);
    EXPECT_NE(rcin_channel->flags & Vektor::FIELD_ROUTABLE, 0U);

    const Vektor::FieldDescriptor *pwmin_channel = registry.field_by_id(
        Vektor::Protocol::fnv1a32(
            "component/pwm_input/0/output/channel_1"));
    ASSERT_NE(pwmin_channel, nullptr);
    EXPECT_EQ(pwmin_channel->kind, Vektor::Protocol::FieldKind::OUTPUT);
    EXPECT_NE(pwmin_channel->flags & Vektor::FIELD_ROUTABLE, 0U);

    const Vektor::FieldDescriptor *pwmout_channel = registry.field_by_id(
        Vektor::Protocol::fnv1a32(
            "component/pwm_output/0/input/channel_1"));
    ASSERT_NE(pwmout_channel, nullptr);
    EXPECT_EQ(pwmout_channel->kind, Vektor::Protocol::FieldKind::INPUT);
    EXPECT_NE(pwmout_channel->flags & Vektor::FIELD_ROUTABLE, 0U);

    const Vektor::FieldDescriptor *attitude_quaternion = registry.field_by_id(
        Vektor::SerialCatalog::Observable::ATTITUDE_QUATERNION.id);
    ASSERT_NE(attitude_quaternion, nullptr);
    EXPECT_EQ(attitude_quaternion->kind,
              Vektor::Protocol::FieldKind::OBSERVABLE);
    EXPECT_EQ(attitude_quaternion->type,
              Vektor::Protocol::PrimitiveType::QUATERNIONF);
    EXPECT_NE(attitude_quaternion->flags & Vektor::FIELD_REALTIME, 0U);

    const Vektor::FieldDescriptor *rcin_pwm = registry.field_by_id(
        Vektor::SerialCatalog::Observable::RCIN_CHANNEL_1_US.id);
    ASSERT_NE(rcin_pwm, nullptr);
    EXPECT_EQ(rcin_pwm->type, Vektor::Protocol::PrimitiveType::U16);

    const Vektor::FieldDescriptor *pwmout_pulse = registry.field_by_id(
        Vektor::SerialCatalog::Output::PWMOUT_CHANNEL_1.id);
    ASSERT_NE(pwmout_pulse, nullptr);
    EXPECT_EQ(pwmout_pulse->kind, Vektor::Protocol::FieldKind::OUTPUT);
    EXPECT_EQ(pwmout_pulse->type, Vektor::Protocol::PrimitiveType::U16);
    EXPECT_EQ(pwmout_pulse->flags & Vektor::FIELD_ROUTABLE, 0U);
}

TEST(VektorSchemaRegistry, SerializesCanonicalFieldRecord)
{
    const Vektor::SchemaRegistry &registry = Vektor::schema_registry();
    uint8_t record[128];
    uint16_t record_len = 0;
    ASSERT_TRUE(registry.build_field_record(3,
                                            record,
                                            sizeof(record),
                                            record_len));

    Vektor::Protocol::PayloadReader reader(record, record_len);
    uint8_t version = 0;
    uint32_t field_id = 0;
    uint32_t owner_id = 0;
    uint8_t kind = 0;
    uint8_t type = 0;
    uint32_t flags = 0;
    ASSERT_TRUE(reader.u8(version));
    ASSERT_TRUE(reader.u32(field_id));
    ASSERT_TRUE(reader.u32(owner_id));
    ASSERT_TRUE(reader.u8(kind));
    ASSERT_TRUE(reader.u8(type));
    ASSERT_TRUE(reader.u32(flags));
    ASSERT_TRUE(skip_str8(reader));
    ASSERT_TRUE(skip_str8(reader));
    ASSERT_TRUE(skip_str8(reader));
    EXPECT_EQ(reader.remaining(), 0);
    EXPECT_EQ(version, 1);
    EXPECT_EQ(field_id,
              Vektor::Protocol::fnv1a32(
                  "component/system/1/observable/uptime_ms"));
    EXPECT_EQ(owner_id,
              Vektor::Protocol::fnv1a32("component/system/1"));
    EXPECT_EQ(kind,
              uint8_t(Vektor::Protocol::FieldKind::OBSERVABLE));
    EXPECT_EQ(type,
              uint8_t(Vektor::Protocol::PrimitiveType::U32));
    EXPECT_EQ(flags,
              uint32_t(Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME));

    EXPECT_FALSE(registry.build_field_record(registry.field_count(),
                                             record,
                                             sizeof(record),
                                             record_len));
    EXPECT_EQ(record_len, 0);
}

TEST(VektorSerialCatalog, MatchesCompleteSchemaRegistry)
{
    namespace Catalog = Vektor::SerialCatalog;

    const Vektor::SchemaRegistry &registry = Vektor::schema_registry();
    EXPECT_EQ(Catalog::Component::COUNT, registry.component_count());
    EXPECT_EQ(Catalog::Parameter::COUNT, registry.parameter_count());
    EXPECT_EQ(Catalog::FIELD_COUNT, registry.field_count());

    for (uint16_t i = 0; i < Catalog::Component::COUNT; i++) {
        const Catalog::Reference *reference = Catalog::Component::ALL[i];
        ASSERT_NE(reference, nullptr);
        const Vektor::ComponentDescriptor *component =
            registry.component_by_index(i);
        ASSERT_NE(component, nullptr);
        EXPECT_STREQ(reference->path, component->path);
        EXPECT_EQ(reference->id, registry.component_id(i));
    }

    expect_catalog_fields_match_schema(Catalog::Parameter::ALL);
    expect_catalog_fields_match_schema(Catalog::Observable::ALL);
    expect_catalog_fields_match_schema(Catalog::Input::ALL);
    expect_catalog_fields_match_schema(Catalog::Output::ALL);

    EXPECT_EQ(Catalog::Stream::ATTITUDE_COUNT, 5);
    EXPECT_EQ(Catalog::Stream::RC_INPUT_NORMALIZED_COUNT, 16);
    EXPECT_EQ(Catalog::Stream::RC_INPUT_PWM_US_COUNT, 16);
    EXPECT_EQ(Catalog::Stream::RC_OUTPUT_PWM_US_COUNT, 12);
    EXPECT_EQ(Catalog::Stream::RC_OUTPUT_COMMAND_NORMALIZED_COUNT, 12);
    EXPECT_EQ(Catalog::Stream::ATTITUDE_RC_IO_COUNT, 33);
    EXPECT_LE(Catalog::Stream::ATTITUDE_RC_IO_COUNT,
              Vektor::SubscriptionTable::max_fields);

    for (const Catalog::ParameterReference *parameter :
         Catalog::Parameter::ALL) {
        ASSERT_NE(parameter, nullptr);
        EXPECT_NE(parameter->ap_param_name, nullptr);
        EXPECT_NE(parameter->ap_param_name[0], '\0');
    }
}

TEST(VektorAttitudeSource, TracksEstimatorValueAndQuality)
{
    Vektor::AttitudeSource attitude;
    attitude.reset();
    EXPECT_EQ(attitude.quality(), Vektor::SignalQuality::INVALID);

    const Vektor::AttitudeValue value {
        10.0F, -20.0F, 30.0F,
        1.0F, 0.0F, 0.0F, 0.0F,
        0.1F, 0.2F, 0.3F,
    };
    attitude.ingest(value, 1234, true);
    EXPECT_EQ(attitude.quality(), Vektor::SignalQuality::VALID);
    EXPECT_EQ(attitude.timestamp_us(), 1234U);
    EXPECT_FLOAT_EQ(attitude.value().roll_deg, 10.0F);
    EXPECT_FLOAT_EQ(attitude.value().body_rate_z_rad_s, 0.3F);

    Vektor::AttitudeValue invalid = value;
    invalid.yaw_deg = NAN;
    attitude.ingest(invalid, 2345, true);
    EXPECT_EQ(attitude.quality(), Vektor::SignalQuality::INVALID);
}

TEST(VektorRcinSource, TracksFreshStaleAndFailsafeQuality)
{
    Vektor::RcinSource rcin;
    rcin.reset(500000);
    ASSERT_NE(rcin.channel(0), nullptr);
    EXPECT_EQ(rcin.channel(0)->quality, Vektor::SignalQuality::INVALID);
    EXPECT_EQ(rcin.channel(Vektor::RcinSource::max_channels), nullptr);

    const float values[] = { 0.25f, -2.0f, NAN };
    const uint16_t pwm[] = { 1600, 1000, 1500 };
    rcin.ingest_normalized(values, pwm, 3, false, 100);
    rcin.update(100);
    EXPECT_TRUE(rcin.healthy());
    EXPECT_FALSE(rcin.failsafe());
    EXPECT_EQ(rcin.channel_count(), 3);
    EXPECT_EQ(rcin.frame_count(), 1U);
    EXPECT_EQ(rcin.last_frame_us(), 100U);
    EXPECT_FLOAT_EQ(rcin.channel(0)->value, 0.25f);
    EXPECT_EQ(rcin.channel(0)->quality, Vektor::SignalQuality::VALID);
    EXPECT_FLOAT_EQ(rcin.channel(1)->value, -1.0f);
    EXPECT_EQ(rcin.channel(1)->quality, Vektor::SignalQuality::VALID);
    EXPECT_EQ(rcin.channel(2)->quality, Vektor::SignalQuality::INVALID);
    EXPECT_EQ(rcin.raw_pwm(0), 1600);

    rcin.update(500101);
    EXPECT_FALSE(rcin.healthy());
    EXPECT_EQ(rcin.channel(0)->quality, Vektor::SignalQuality::STALE);
    EXPECT_EQ(rcin.channel(1)->quality, Vektor::SignalQuality::STALE);
    EXPECT_EQ(rcin.channel(2)->quality, Vektor::SignalQuality::INVALID);

    rcin.ingest_normalized(values, pwm, 2, true, 600000);
    rcin.update(600000);
    EXPECT_FALSE(rcin.healthy());
    EXPECT_TRUE(rcin.failsafe());
    EXPECT_EQ(rcin.channel(0)->quality, Vektor::SignalQuality::INVALID);
}

TEST(VektorRcinSource, NormalizesPwmAndRejectsInvalidPulses)
{
    Vektor::RcinSource rcin;
    rcin.reset();
    const uint16_t pwm[] = { 1000, 1500, 2000, 850 };
    rcin.ingest_pwm(pwm, 4, false, 1000);
    rcin.update(1000);

    EXPECT_FLOAT_EQ(rcin.channel(0)->value, -1.0f);
    EXPECT_FLOAT_EQ(rcin.channel(1)->value, 0.0f);
    EXPECT_FLOAT_EQ(rcin.channel(2)->value, 1.0f);
    EXPECT_EQ(rcin.channel(0)->quality, Vektor::SignalQuality::VALID);
    EXPECT_EQ(rcin.channel(3)->quality, Vektor::SignalQuality::INVALID);
}

TEST(VektorPwmInput, NormalizesIndependentPulsesAndTracksFreshness)
{
    Vektor::PwmInput input;
    input.ingest_pulse(0, 1250, 1000);
    input.ingest_pulse(1, 1750, 1000);
    input.update(1000);

    ASSERT_NE(input.channel(0), nullptr);
    EXPECT_FLOAT_EQ(input.channel(0)->value, -0.5F);
    EXPECT_FLOAT_EQ(input.channel(1)->value, 0.5F);
    EXPECT_EQ(input.channel(0)->quality, Vektor::SignalQuality::VALID);
    EXPECT_EQ(input.raw_pwm(1), 1750);

    input.update(101001);
    EXPECT_EQ(input.channel(0)->quality, Vektor::SignalQuality::STALE);

    input.ingest_pulse(0, 2500, 102000);
    input.update(102000);
    EXPECT_EQ(input.channel(0)->quality, Vektor::SignalQuality::INVALID);
}

TEST(VektorPwmOutput, MapsNormalizedCommandsToCalibratedPulses)
{
    EXPECT_EQ(Vektor::PwmOutput::normalized_to_pwm(
                  -1.0F, 1000, 1500, 2000, false),
              1000);
    EXPECT_EQ(Vektor::PwmOutput::normalized_to_pwm(
                  0.0F, 1000, 1500, 2000, false),
              1500);
    EXPECT_EQ(Vektor::PwmOutput::normalized_to_pwm(
                  0.5F, 1000, 1500, 2000, false),
              1750);
    EXPECT_EQ(Vektor::PwmOutput::normalized_to_pwm(
                  1.0F, 1000, 1500, 2000, true),
              1000);
    EXPECT_TRUE(Vektor::PwmOutput::supported_rate(330));
    EXPECT_FALSE(Vektor::PwmOutput::supported_rate(250));

    Vektor::PwmOutput output;
    output.init(2, 1U << 1);
    const Vektor::SignalSample<float> commands[] = {
        { 0.5F, 1000, Vektor::SignalQuality::VALID },
        { -0.5F, 1000, Vektor::SignalQuality::VALID },
    };
    output.update(commands, 2, (1U << 0) | (1U << 1));
    EXPECT_TRUE(output.active(0));
    EXPECT_EQ(output.pwm_us(0), 1750);
    EXPECT_FALSE(output.active(1));
    EXPECT_EQ(output.pwm_us(1), 0);

    output.update(commands, 2, 0);
    EXPECT_FALSE(output.active(0));
    EXPECT_EQ(output.pwm_us(0), 0);
}

TEST(VektorAssignmentMatrix, ValidatesReplacesAndRemovesAssignments)
{
    const uint32_t channel_1 = Vektor::Protocol::fnv1a32(
        "component/rcin/0/output/channel_1");
    const uint32_t channel_2 = Vektor::Protocol::fnv1a32(
        "component/rcin/0/output/channel_2");
    const uint32_t vsp_x = Vektor::Protocol::fnv1a32(
        "component/vsp/1/input/x");
    const uint32_t servo_a = Vektor::Protocol::fnv1a32(
        "component/vsp/1/output/servo_a");
    const uint32_t pwm_input_1 = Vektor::Protocol::fnv1a32(
        "component/pwm_input/0/output/channel_1");
    const uint32_t pwm_output_1 = Vektor::Protocol::fnv1a32(
        "component/pwm_output/0/input/channel_1");
    const uint32_t pwm_output_7 = Vektor::Protocol::fnv1a32(
        "component/pwm_output/0/input/channel_7");

    Vektor::AssignmentMatrix assignments;
    const Vektor::AssignmentMatrix::Entry *accepted = nullptr;
    EXPECT_EQ(assignments.set(channel_1, vsp_x, 0, accepted),
              Vektor::AssignmentMatrix::SetResult::OK);
    ASSERT_NE(accepted, nullptr);
    EXPECT_EQ(assignments.count(), 1);
    EXPECT_NE(accepted->route_id, 0U);
    EXPECT_EQ(accepted->route_id,
              Vektor::AssignmentMatrix::make_route_id(channel_1, vsp_x));
    EXPECT_EQ(accepted->route_id, 0x92359BB4U);
    EXPECT_EQ(assignments.to_destination(vsp_x), accepted);

    EXPECT_EQ(assignments.set(channel_2, vsp_x, 0, accepted),
              Vektor::AssignmentMatrix::SetResult::OK);
    ASSERT_NE(accepted, nullptr);
    EXPECT_EQ(assignments.count(), 1);
    EXPECT_EQ(accepted->source_output_id, channel_2);

    EXPECT_EQ(assignments.set(vsp_x, vsp_x, 0, accepted),
              Vektor::AssignmentMatrix::SetResult::SOURCE_NOT_ROUTABLE);
    EXPECT_EQ(assignments.set(servo_a, vsp_x, 0, accepted),
              Vektor::AssignmentMatrix::SetResult::CYCLE);
    EXPECT_EQ(assignments.set(channel_1, vsp_x, 1, accepted),
              Vektor::AssignmentMatrix::SetResult::UNSUPPORTED_FLAGS);

    const Vektor::AssignmentMatrix::Entry *route =
        assignments.to_destination(vsp_x);
    ASSERT_NE(route, nullptr);
    const uint32_t route_id = route->route_id;
    EXPECT_TRUE(assignments.remove(route_id));
    EXPECT_EQ(assignments.count(), 0);
    EXPECT_EQ(assignments.by_id(route_id), nullptr);
    EXPECT_FALSE(assignments.remove(route_id));

    EXPECT_EQ(assignments.set(channel_1, pwm_output_1, 0, accepted),
              Vektor::AssignmentMatrix::SetResult::OK);
    EXPECT_EQ(assignments.set(pwm_input_1, pwm_output_1, 0, accepted),
              Vektor::AssignmentMatrix::SetResult::OK);
    ASSERT_NE(accepted, nullptr);
    EXPECT_EQ(assignments.count(), 1);
    EXPECT_EQ(accepted->source_output_id, pwm_input_1);

    assignments.configure_channel_limits(Vektor::PwmInput::max_channels, 6);
    EXPECT_EQ(assignments.set(channel_1, pwm_output_7, 0, accepted),
              Vektor::AssignmentMatrix::SetResult::BAD_DESTINATION);
    assignments.configure_channel_limits(Vektor::PwmInput::max_channels,
                                         6,
                                         1U << 0);
    EXPECT_EQ(assignments.set(channel_1, pwm_output_1, 0, accepted),
              Vektor::AssignmentMatrix::SetResult::BAD_DESTINATION);
}

TEST(VektorRequestReplayCache, ReplaysIdenticalRequest)
{
    Vektor::RequestReplayCache cache;
    cache.reset();

    static const uint8_t response[] = { 1, 2, 3, 0 };
    const uint8_t payload[] = { 9, 9 };
    const uint32_t payload_crc =
        Vektor::Protocol::crc32_iso_hdlc(payload, sizeof(payload));
    cache.remember(Vektor::Protocol::MessageType::SET,
                   42,
                   payload_crc,
                   response,
                   sizeof(response));

    Vektor::RequestReplayCache::CachedResponse cached {};
    EXPECT_EQ(cache.lookup(Vektor::Protocol::MessageType::SET,
                           42,
                           payload_crc,
                           cached),
              Vektor::RequestReplayCache::LookupResult::REPLAY);
    ASSERT_EQ(cached.encoded_len, sizeof(response));
    EXPECT_EQ(memcmp(cached.encoded, response, sizeof(response)), 0);
}

TEST(VektorRequestReplayCache, RejectsSequenceReuseWithDifferentPayload)
{
    Vektor::RequestReplayCache cache;
    cache.reset();

    static const uint8_t response[] = { 7, 0 };
    cache.remember(Vektor::Protocol::MessageType::SET,
                   5,
                   0x11111111U,
                   response,
                   sizeof(response));

    Vektor::RequestReplayCache::CachedResponse cached {};
    EXPECT_EQ(cache.lookup(Vektor::Protocol::MessageType::SET,
                           5,
                           0x22222222U,
                           cached),
              Vektor::RequestReplayCache::LookupResult::CONFLICT);
}

TEST(VektorRequestReplayCache, EvictsOldestEntry)
{
    Vektor::RequestReplayCache cache;
    cache.reset();

    static const uint8_t response[] = { 1, 0 };
    for (uint8_t i = 0; i < Vektor::RequestReplayCache::cache_size + 1; i++) {
        cache.remember(Vektor::Protocol::MessageType::GET,
                       i,
                       i,
                       response,
                       sizeof(response));
    }

    Vektor::RequestReplayCache::CachedResponse cached {};
    EXPECT_EQ(cache.lookup(Vektor::Protocol::MessageType::GET, 0, 0, cached),
              Vektor::RequestReplayCache::LookupResult::MISS);
    EXPECT_EQ(cache.lookup(Vektor::Protocol::MessageType::GET, 1, 1, cached),
              Vektor::RequestReplayCache::LookupResult::REPLAY);
}

TEST(VektorRuntimeState, TracksLoopTimingAndUptime)
{
    Vektor::RuntimeState runtime;
    runtime.init(100, 1000);

    EXPECT_EQ(runtime.service_rate_hz(), 100);
    EXPECT_EQ(runtime.uptime_ms(2500), 1U);
    EXPECT_EQ(runtime.loop_count(), 0U);

    runtime.begin_loop(3000);
    runtime.end_loop(3300);
    EXPECT_EQ(runtime.loop_count(), 1U);
    EXPECT_EQ(runtime.last_loop_dt_us(), 0U);
    EXPECT_EQ(runtime.last_loop_work_us(), 300U);
    EXPECT_EQ(runtime.max_loop_work_us(), 300U);

    runtime.begin_loop(13000);
    runtime.end_loop(13450);
    EXPECT_EQ(runtime.loop_count(), 2U);
    EXPECT_EQ(runtime.last_loop_dt_us(), 10000U);
    EXPECT_EQ(runtime.last_loop_work_us(), 450U);
    EXPECT_EQ(runtime.max_loop_work_us(), 450U);
}

TEST(VektorRuntimeState, ClampsWideValues)
{
    Vektor::RuntimeState runtime;
    runtime.init(70000, 0);
    EXPECT_EQ(runtime.service_rate_hz(), UINT16_MAX);

    runtime.begin_loop(0);
    runtime.end_loop(uint64_t(UINT32_MAX) + 10ULL);
    EXPECT_EQ(runtime.last_loop_work_us(), UINT32_MAX);
    EXPECT_EQ(runtime.max_loop_work_us(), UINT32_MAX);
    EXPECT_EQ(runtime.uptime_ms((uint64_t(UINT32_MAX) + 10ULL) * 1000ULL),
              UINT32_MAX);
}

TEST(VektorSubscriptionTable, NegotiatesRateAndSchedulesWithoutBacklog)
{
    Vektor::SubscriptionTable subscriptions;
    subscriptions.reset(10000);
    const uint32_t fields[] = { 0x11, 0x22 };
    const Vektor::SubscriptionTable::Entry *created = nullptr;

    EXPECT_EQ(subscriptions.add(10001,
                                fields,
                                2,
                                1000,
                                created),
              Vektor::SubscriptionTable::AddResult::OK);
    ASSERT_NE(created, nullptr);
    EXPECT_EQ(created->id, 1);
    EXPECT_EQ(created->period_us, 20000U);
    EXPECT_EQ(created->field_count, 2);
    EXPECT_EQ(created->field_ids[0], fields[0]);
    EXPECT_EQ(created->field_ids[1], fields[1]);
    EXPECT_EQ(subscriptions.claim_due(20999), nullptr);

    Vektor::SubscriptionTable::Entry *due = subscriptions.claim_due(21000);
    ASSERT_NE(due, nullptr);
    EXPECT_EQ(due->id, created->id);
    EXPECT_EQ(subscriptions.claim_due(21000), nullptr);

    // Missing several periods produces one fresh sample, not a backlog.
    due = subscriptions.claim_due(101000);
    ASSERT_NE(due, nullptr);
    EXPECT_EQ(subscriptions.claim_due(101000), nullptr);
    EXPECT_EQ(due->next_sample_us, 121000U);
}

TEST(VektorSubscriptionTable, FastestRateAndRemoval)
{
    Vektor::SubscriptionTable subscriptions;
    subscriptions.reset(5000);
    const uint32_t field = 7;
    const Vektor::SubscriptionTable::Entry *created = nullptr;

    ASSERT_EQ(subscriptions.add(0, &field, 1, 0, created),
              Vektor::SubscriptionTable::AddResult::OK);
    ASSERT_NE(created, nullptr);
    EXPECT_EQ(created->period_us, 5000U);
    EXPECT_EQ(subscriptions.active_count(), 1);
    EXPECT_NE(subscriptions.find(created->id), nullptr);
    EXPECT_TRUE(subscriptions.remove(created->id));
    EXPECT_EQ(subscriptions.active_count(), 0);
    EXPECT_FALSE(subscriptions.remove(created->id));
}

TEST(VektorSubscriptionTable, EnforcesBoundsAndCapacity)
{
    Vektor::SubscriptionTable subscriptions;
    subscriptions.reset(10000);
    const uint32_t field = 1;
    const Vektor::SubscriptionTable::Entry *created = nullptr;

    EXPECT_EQ(subscriptions.add(10000, nullptr, 1, 0, created),
              Vektor::SubscriptionTable::AddResult::INVALID_FIELD_COUNT);
    EXPECT_EQ(subscriptions.add(10000, &field, 0, 0, created),
              Vektor::SubscriptionTable::AddResult::INVALID_FIELD_COUNT);

    for (uint16_t i = 0;
         i < Vektor::SubscriptionTable::max_subscriptions;
         i++) {
        ASSERT_EQ(subscriptions.add(10000, &field, 1, 0, created),
                  Vektor::SubscriptionTable::AddResult::OK);
        ASSERT_NE(created, nullptr);
        EXPECT_NE(created->id, 0);
    }
    EXPECT_EQ(subscriptions.add(10000, &field, 1, 0, created),
              Vektor::SubscriptionTable::AddResult::FULL);
    EXPECT_EQ(created, nullptr);
}

TEST(VektorSerialProtocol, DescriptorHashesAreStableAndBoardSpecific)
{
    uint64_t h743_schema = 0;
    uint64_t h743_capability = 0;
    uint64_t repeated_schema = 0;
    uint64_t repeated_capability = 0;
    uint64_t f405_schema = 0;
    uint64_t f405_capability = 0;

    ASSERT_TRUE(hello_hashes_for_capability(
        h743_test_capability, h743_schema, h743_capability));
    ASSERT_TRUE(hello_hashes_for_capability(
        h743_test_capability,
        repeated_schema,
        repeated_capability));
    ASSERT_TRUE(hello_hashes_for_capability(
        f405_test_capability, f405_schema, f405_capability));

    EXPECT_NE(h743_schema, 0U);
    EXPECT_NE(h743_capability, 0U);
    EXPECT_EQ(repeated_schema, h743_schema);
    EXPECT_EQ(repeated_capability, h743_capability);
    EXPECT_EQ(f405_schema, h743_schema);
    EXPECT_NE(f405_capability, h743_capability);
}

TEST(VektorSerialProtocol, ManagesAssignmentsOverRouteMessages)
{
    static TestUart uart;
    uart.reset();
    Vektor::Parameters parameters;
    parameters.sys_protocol_baud.set(Vektor::protocol_baud);
    Vektor::RuntimeState runtime;
    runtime.init(Vektor::default_service_rate_hz, AP_HAL::micros64());
    Vektor::AttitudeSource attitude;
    attitude.reset();
    Vektor::VspComponent vsp;
    vsp.reset();
    Vektor::RcinSource rcin;
    rcin.reset();
    Vektor::PwmInput pwm_input;
    Vektor::PwmOutput pwm_output;
    Vektor::AssignmentMatrix assignments;
    Vektor::SerialProtocol serial;
    serial.init(&uart,
                h743_test_capability,
                parameters,
                runtime,
                attitude,
                vsp,
                rcin,
                pwm_input,
                pwm_output,
                assignments);
    ASSERT_TRUE(uart.is_initialized());
    uart.clear_tx();

    uint8_t payload[Vektor::Protocol::MAX_PAYLOAD_SIZE];
    Vektor::Protocol::PayloadWriter hello(payload, sizeof(payload));
    hello.u8(1);
    hello.u8(1);
    hello.u16(0);
    hello.u32(0);
    hello.u32(0xA55A);
    ASSERT_TRUE(push_request(uart,
                             Vektor::Protocol::MessageType::HELLO,
                             1,
                             hello.data(),
                             hello.length()));
    serial.update();

    Vektor::Protocol::Parser parser;
    Vektor::Protocol::FrameView response {};
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    ASSERT_EQ(response.message_type, Vektor::Protocol::MessageType::HELLO);

    const uint32_t source_id = Vektor::Protocol::fnv1a32(
        "component/rcin/0/output/channel_1");
    const uint32_t destination_id = Vektor::Protocol::fnv1a32(
        "component/pwm_output/0/input/channel_1");

    uart.clear_tx();
    Vektor::Protocol::PayloadWriter route_set(payload, sizeof(payload));
    route_set.u32(source_id);
    route_set.u32(destination_id);
    route_set.u16(0);
    ASSERT_TRUE(push_request(uart,
                             Vektor::Protocol::MessageType::ROUTE_SET,
                             2,
                             route_set.data(),
                             route_set.length()));
    serial.update();
    parser.reset();
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    ASSERT_EQ(response.message_type, Vektor::Protocol::MessageType::ROUTES);
    EXPECT_EQ(response.flags, Vektor::Protocol::FLAG_RESPONSE);

    Vektor::Protocol::PayloadReader set_response(response.payload,
                                                 response.payload_len);
    uint32_t next_cursor = 1;
    uint16_t route_count = 0;
    uint32_t route_id = 0;
    uint32_t returned_source = 0;
    uint32_t returned_destination = 0;
    uint16_t returned_flags = UINT16_MAX;
    ASSERT_TRUE(set_response.u32(next_cursor));
    ASSERT_TRUE(set_response.u16(route_count));
    ASSERT_TRUE(set_response.u32(route_id));
    ASSERT_TRUE(set_response.u32(returned_source));
    ASSERT_TRUE(set_response.u32(returned_destination));
    ASSERT_TRUE(set_response.u16(returned_flags));
    EXPECT_EQ(set_response.remaining(), 0);
    EXPECT_EQ(next_cursor, 0U);
    EXPECT_EQ(route_count, 1);
    EXPECT_EQ(route_id,
              Vektor::AssignmentMatrix::make_route_id(source_id,
                                                      destination_id));
    EXPECT_EQ(returned_source, source_id);
    EXPECT_EQ(returned_destination, destination_id);
    EXPECT_EQ(returned_flags, 0);

    uart.clear_tx();
    Vektor::Protocol::PayloadWriter route_list(payload, sizeof(payload));
    route_list.u32(0);
    route_list.u16(1);
    ASSERT_TRUE(push_request(uart,
                             Vektor::Protocol::MessageType::ROUTE_LIST,
                             3,
                             route_list.data(),
                             route_list.length()));
    serial.update();
    parser.reset();
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    ASSERT_EQ(response.message_type, Vektor::Protocol::MessageType::ROUTES);
    Vektor::Protocol::PayloadReader list_response(response.payload,
                                                  response.payload_len);
    ASSERT_TRUE(list_response.u32(next_cursor));
    ASSERT_TRUE(list_response.u16(route_count));
    ASSERT_TRUE(list_response.u32(route_id));
    ASSERT_TRUE(list_response.u32(returned_source));
    ASSERT_TRUE(list_response.u32(returned_destination));
    ASSERT_TRUE(list_response.u16(returned_flags));
    EXPECT_EQ(list_response.remaining(), 0);
    EXPECT_EQ(route_count, 1);

    uart.clear_tx();
    Vektor::Protocol::PayloadWriter route_delete(payload, sizeof(payload));
    route_delete.u32(route_id);
    ASSERT_TRUE(push_request(uart,
                             Vektor::Protocol::MessageType::ROUTE_DELETE,
                             4,
                             route_delete.data(),
                             route_delete.length()));
    serial.update();
    parser.reset();
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    ASSERT_EQ(response.message_type, Vektor::Protocol::MessageType::ROUTES);
    Vektor::Protocol::PayloadReader delete_response(response.payload,
                                                    response.payload_len);
    ASSERT_TRUE(delete_response.u32(next_cursor));
    ASSERT_TRUE(delete_response.u16(route_count));
    EXPECT_EQ(delete_response.remaining(), 0);
    EXPECT_EQ(next_cursor, 0U);
    EXPECT_EQ(route_count, 0);
    EXPECT_EQ(assignments.count(), 0);
}

TEST(VektorSerialProtocol, NegotiatesAndStreamsRuntimeTelemetry)
{
    static TestUart uart;
    uart.reset();
    Vektor::Parameters parameters;
    parameters.sys_protocol_baud.set(Vektor::protocol_baud);
    Vektor::RuntimeState runtime;
    runtime.init(Vektor::default_service_rate_hz, AP_HAL::micros64());
    Vektor::AttitudeSource attitude;
    attitude.reset();
    const Vektor::AttitudeValue attitude_value {
        10.0F, -20.0F, 30.0F,
        1.0F, 0.0F, 0.0F, 0.0F,
        0.1F, 0.2F, 0.3F,
    };
    attitude.ingest(attitude_value, AP_HAL::micros64(), true);
    Vektor::VspComponent vsp;
    vsp.reset();
    Vektor::RcinSource rcin;
    rcin.reset();
    const float rcin_values[] = { 0.25F };
    const uint16_t rcin_pwm[] = { 1600 };
    rcin.ingest_normalized(rcin_values,
                           rcin_pwm,
                           1,
                           false,
                           AP_HAL::micros64());
    Vektor::PwmInput pwm_input;
    Vektor::PwmOutput pwm_output;
    pwm_output.init(1, 0);
    const Vektor::SignalSample<float> pwm_commands[] = {
        { 0.5F, AP_HAL::micros64(), Vektor::SignalQuality::VALID },
    };
    pwm_output.update(pwm_commands, 1, 1U);
    Vektor::AssignmentMatrix assignments;
    Vektor::SerialProtocol serial;
    serial.init(&uart,
                h743_test_capability,
                parameters,
                runtime,
                attitude,
                vsp,
                rcin,
                pwm_input,
                pwm_output,
                assignments);
    ASSERT_TRUE(uart.is_initialized());
    uart.clear_tx();

    uint8_t payload[Vektor::Protocol::MAX_PAYLOAD_SIZE];
    Vektor::Protocol::PayloadWriter hello(payload, sizeof(payload));
    hello.u8(1);
    hello.u8(1);
    hello.u16(0);
    hello.u32(0);
    hello.u32(0x12345678);
    ASSERT_TRUE(push_request(uart,
                             Vektor::Protocol::MessageType::HELLO,
                             1,
                             hello.data(),
                             hello.length()));
    serial.update();

    Vektor::Protocol::Parser parser;
    Vektor::Protocol::FrameView response {};
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    EXPECT_EQ(response.message_type, Vektor::Protocol::MessageType::HELLO);
    EXPECT_EQ(response.flags, Vektor::Protocol::FLAG_RESPONSE);
    EXPECT_EQ(response.sequence, 1);

    Vektor::Protocol::PayloadReader hello_response(response.payload,
                                                   response.payload_len);
    uint8_t selected_major = 0;
    uint16_t server_minor = 0;
    uint32_t board_id = 0;
    uint64_t schema_hash = 0;
    uint64_t capability_hash = 0;
    uint64_t ignored64 = 0;
    uint64_t capability_flags = 0;
    uint16_t max_payload = 0;
    uint16_t max_file_chunk = 0;
    uint16_t max_subscriptions = 0;
    uint16_t max_realtime_hz = 0;
    uint16_t control_update_hz = 0;
    uint16_t attitude_update_hz = 0;
    uint32_t server_nonce = 0;
    ASSERT_TRUE(hello_response.u8(selected_major));
    ASSERT_TRUE(hello_response.u16(server_minor));
    ASSERT_TRUE(skip_str8(hello_response));
    ASSERT_TRUE(skip_str8(hello_response));
    ASSERT_TRUE(skip_str8(hello_response));
    ASSERT_TRUE(hello_response.u32(board_id));
    ASSERT_TRUE(hello_response.u64(schema_hash));
    ASSERT_TRUE(hello_response.u64(capability_hash));
    ASSERT_TRUE(hello_response.u64(ignored64)); // device ID
    ASSERT_TRUE(hello_response.u64(capability_flags));
    ASSERT_TRUE(hello_response.u16(max_payload));
    ASSERT_TRUE(hello_response.u16(max_file_chunk));
    ASSERT_TRUE(hello_response.u16(max_subscriptions));
    ASSERT_TRUE(hello_response.u16(max_realtime_hz));
    ASSERT_TRUE(hello_response.u16(control_update_hz));
    ASSERT_TRUE(hello_response.u16(attitude_update_hz));
    ASSERT_TRUE(hello_response.u32(server_nonce));
    EXPECT_EQ(hello_response.remaining(), 0);
    EXPECT_EQ(selected_major, Vektor::Protocol::MAJOR_VERSION);
    EXPECT_EQ(server_minor, 0);
    EXPECT_EQ(max_payload, Vektor::Protocol::MAX_PAYLOAD_SIZE);
    EXPECT_EQ(max_file_chunk, 0);
    EXPECT_EQ(max_subscriptions,
              Vektor::SubscriptionTable::max_subscriptions);
    EXPECT_EQ(max_realtime_hz, Vektor::max_realtime_rate_hz);
    EXPECT_EQ(control_update_hz, Vektor::default_service_rate_hz);
    EXPECT_EQ(attitude_update_hz, Vektor::attitude_update_rate_hz);
    EXPECT_NE(capability_flags & (1ULL << 11), 0U);
    EXPECT_NE(capability_flags & (1ULL << 13), 0U);
    EXPECT_NE(schema_hash, 0U);
    EXPECT_NE(capability_hash, 0U);
    EXPECT_NE(schema_hash, capability_hash);

    uart.clear_tx();
    Vektor::Protocol::PayloadWriter describe(payload, sizeof(payload));
    describe.u8(uint8_t(Vektor::Protocol::DescriptorDomain::RUNTIME_LIMIT));
    describe.u32(0);
    describe.u16(0);
    ASSERT_TRUE(push_request(uart,
                             Vektor::Protocol::MessageType::DESCRIBE,
                             2,
                             describe.data(),
                             describe.length()));
    serial.update();
    parser.reset();
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    EXPECT_EQ(response.message_type, Vektor::Protocol::MessageType::DESCRIBE);
    EXPECT_EQ(response.flags, Vektor::Protocol::FLAG_RESPONSE);
    EXPECT_EQ(response.sequence, 2);

    Vektor::Protocol::PayloadReader description(response.payload,
                                                response.payload_len);
    uint8_t descriptor_domain = 0;
    uint32_t next_cursor = 1;
    uint16_t record_count = 0;
    uint16_t record_len = 0;
    const uint8_t *record_bytes = nullptr;
    ASSERT_TRUE(description.u8(descriptor_domain));
    ASSERT_TRUE(description.u32(next_cursor));
    ASSERT_TRUE(description.u16(record_count));
    ASSERT_TRUE(description.u16(record_len));
    ASSERT_TRUE(description.bytes(record_bytes, record_len));
    EXPECT_EQ(description.remaining(), 0);
    EXPECT_EQ(descriptor_domain,
              uint8_t(Vektor::Protocol::DescriptorDomain::RUNTIME_LIMIT));
    EXPECT_EQ(next_cursor, 0U);
    EXPECT_EQ(record_count, 1);

    Vektor::Protocol::PayloadReader runtime_limit(record_bytes, record_len);
    uint8_t record_version = 0;
    uint32_t limit_id = 0;
    uint64_t limit_capability_flags = 0;
    uint16_t limit_max_payload = 0;
    uint16_t limit_max_file_chunk = 0;
    uint16_t limit_max_subscriptions = 0;
    uint16_t limit_max_realtime_hz = 0;
    uint16_t limit_control_update_hz = 0;
    uint16_t limit_attitude_update_hz = 0;
    ASSERT_TRUE(runtime_limit.u8(record_version));
    ASSERT_TRUE(runtime_limit.u32(limit_id));
    ASSERT_TRUE(runtime_limit.u64(limit_capability_flags));
    ASSERT_TRUE(runtime_limit.u16(limit_max_payload));
    ASSERT_TRUE(runtime_limit.u16(limit_max_file_chunk));
    ASSERT_TRUE(runtime_limit.u16(limit_max_subscriptions));
    ASSERT_TRUE(runtime_limit.u16(limit_max_realtime_hz));
    ASSERT_TRUE(runtime_limit.u16(limit_control_update_hz));
    ASSERT_TRUE(runtime_limit.u16(limit_attitude_update_hz));
    EXPECT_EQ(runtime_limit.remaining(), 0);
    EXPECT_EQ(record_version, 1);
    EXPECT_EQ(limit_id,
              Vektor::Protocol::fnv1a32("runtime_limit/protocol/0"));
    EXPECT_EQ(limit_capability_flags, capability_flags);
    EXPECT_EQ(limit_max_payload, max_payload);
    EXPECT_EQ(limit_max_file_chunk, max_file_chunk);
    EXPECT_EQ(limit_max_subscriptions, max_subscriptions);
    EXPECT_EQ(limit_max_realtime_hz, max_realtime_hz);
    EXPECT_EQ(limit_control_update_hz, control_update_hz);
    EXPECT_EQ(limit_attitude_update_hz, attitude_update_hz);

    uart.clear_tx();
    const uint32_t uptime_id = Vektor::Protocol::fnv1a32(
        "component/system/1/observable/uptime_ms");
    const uint32_t service_rate_id = Vektor::Protocol::fnv1a32(
        "component/system/1/observable/service_rate_hz");
    const uint32_t servo_a_id = Vektor::Protocol::fnv1a32(
        "component/vsp/1/output/servo_a");
    const uint32_t attitude_quaternion_id =
        Vektor::SerialCatalog::Observable::ATTITUDE_QUATERNION.id;
    const uint32_t rcin_pwm_id =
        Vektor::SerialCatalog::Observable::RCIN_CHANNEL_1_US.id;
    const uint32_t pwmout_pulse_id =
        Vektor::SerialCatalog::Output::PWMOUT_CHANNEL_1.id;
    Vektor::Protocol::PayloadWriter subscribe(payload, sizeof(payload));
    subscribe.u32(0); // fastest supported rate
    subscribe.u16(6);
    subscribe.u32(uptime_id);
    subscribe.u32(service_rate_id);
    subscribe.u32(servo_a_id);
    subscribe.u32(attitude_quaternion_id);
    subscribe.u32(rcin_pwm_id);
    subscribe.u32(pwmout_pulse_id);
    ASSERT_TRUE(push_request(uart,
                             Vektor::Protocol::MessageType::SUBSCRIBE,
                             3,
                             subscribe.data(),
                             subscribe.length()));
    serial.update();

    parser.reset();
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    EXPECT_EQ(response.message_type, Vektor::Protocol::MessageType::SUBSCRIBE);
    EXPECT_EQ(response.flags, Vektor::Protocol::FLAG_RESPONSE);
    Vektor::Protocol::PayloadReader subscribe_response(response.payload,
                                                       response.payload_len);
    uint16_t subscription_id = 0;
    uint32_t accepted_period_us = 0;
    uint16_t field_count = 0;
    uint32_t returned_field_id = 0;
    ASSERT_TRUE(subscribe_response.u16(subscription_id));
    ASSERT_TRUE(subscribe_response.u32(accepted_period_us));
    ASSERT_TRUE(subscribe_response.u16(field_count));
    EXPECT_NE(subscription_id, 0);
    EXPECT_EQ(accepted_period_us,
              1000000U / Vektor::max_realtime_rate_hz);
    EXPECT_EQ(field_count, 6);
    ASSERT_TRUE(subscribe_response.u32(returned_field_id));
    EXPECT_EQ(returned_field_id, uptime_id);
    ASSERT_TRUE(subscribe_response.u32(returned_field_id));
    EXPECT_EQ(returned_field_id, service_rate_id);
    ASSERT_TRUE(subscribe_response.u32(returned_field_id));
    EXPECT_EQ(returned_field_id, servo_a_id);
    ASSERT_TRUE(subscribe_response.u32(returned_field_id));
    EXPECT_EQ(returned_field_id, attitude_quaternion_id);
    ASSERT_TRUE(subscribe_response.u32(returned_field_id));
    EXPECT_EQ(returned_field_id, rcin_pwm_id);
    ASSERT_TRUE(subscribe_response.u32(returned_field_id));
    EXPECT_EQ(returned_field_id, pwmout_pulse_id);
    EXPECT_EQ(subscribe_response.remaining(), 0);

    uart.clear_tx();
    usleep(20000);
    serial.update();
    parser.reset();
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    EXPECT_EQ(response.message_type, Vektor::Protocol::MessageType::TELEMETRY);
    EXPECT_EQ(response.flags, Vektor::Protocol::FLAG_VOLATILE);
    Vektor::Protocol::PayloadReader telemetry(response.payload,
                                              response.payload_len);
    uint16_t telemetry_subscription_id = 0;
    uint16_t sample_sequence = 0;
    uint64_t timestamp_us = 0;
    uint8_t quality_len = 0;
    uint8_t quality_0 = 0xFF;
    uint8_t quality_1 = 0xFF;
    uint32_t uptime_ms = 0;
    uint16_t service_rate_hz = 0;
    uint32_t servo_a_raw = UINT32_MAX;
    uint32_t quaternion_w_raw = 0;
    uint32_t quaternion_x_raw = UINT32_MAX;
    uint32_t quaternion_y_raw = UINT32_MAX;
    uint32_t quaternion_z_raw = UINT32_MAX;
    uint16_t received_rcin_pwm = 0;
    uint16_t received_pwmout_pulse = 0;
    ASSERT_TRUE(telemetry.u16(telemetry_subscription_id));
    ASSERT_TRUE(telemetry.u16(sample_sequence));
    ASSERT_TRUE(telemetry.u64(timestamp_us));
    ASSERT_TRUE(telemetry.u8(quality_len));
    ASSERT_TRUE(telemetry.u8(quality_0));
    ASSERT_TRUE(telemetry.u8(quality_1));
    ASSERT_TRUE(telemetry.u32(uptime_ms));
    ASSERT_TRUE(telemetry.u16(service_rate_hz));
    ASSERT_TRUE(telemetry.u32(servo_a_raw));
    ASSERT_TRUE(telemetry.u32(quaternion_w_raw));
    ASSERT_TRUE(telemetry.u32(quaternion_x_raw));
    ASSERT_TRUE(telemetry.u32(quaternion_y_raw));
    ASSERT_TRUE(telemetry.u32(quaternion_z_raw));
    ASSERT_TRUE(telemetry.u16(received_rcin_pwm));
    ASSERT_TRUE(telemetry.u16(received_pwmout_pulse));
    EXPECT_EQ(telemetry.remaining(), 0);
    EXPECT_EQ(telemetry_subscription_id, subscription_id);
    EXPECT_EQ(sample_sequence, 0);
    EXPECT_NE(timestamp_us, 0U);
    EXPECT_EQ(quality_len, 2);
    EXPECT_EQ(quality_0, 0x20); // VSP output is INVALID until routed inputs exist
    EXPECT_EQ(quality_1, 0x00);
    EXPECT_EQ(service_rate_hz, Vektor::default_service_rate_hz);
    EXPECT_EQ(servo_a_raw, 0U);
    EXPECT_EQ(quaternion_w_raw, 0x3F800000U);
    EXPECT_EQ(quaternion_x_raw, 0U);
    EXPECT_EQ(quaternion_y_raw, 0U);
    EXPECT_EQ(quaternion_z_raw, 0U);
    EXPECT_EQ(received_rcin_pwm, 1600);
    EXPECT_EQ(received_pwmout_pulse, 1750);

    uart.clear_tx();
    Vektor::Protocol::PayloadWriter unsubscribe(payload, sizeof(payload));
    unsubscribe.u16(subscription_id);
    ASSERT_TRUE(push_request(uart,
                             Vektor::Protocol::MessageType::UNSUBSCRIBE,
                             4,
                             unsubscribe.data(),
                             unsubscribe.length()));
    serial.update();
    parser.reset();
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    EXPECT_EQ(response.message_type,
              Vektor::Protocol::MessageType::UNSUBSCRIBE);
    EXPECT_EQ(response.flags, Vektor::Protocol::FLAG_RESPONSE);
    Vektor::Protocol::PayloadReader unsubscribe_response(response.payload,
                                                         response.payload_len);
    uint16_t removed_id = 0;
    ASSERT_TRUE(unsubscribe_response.u16(removed_id));
    EXPECT_EQ(removed_id, subscription_id);
    EXPECT_EQ(unsubscribe_response.remaining(), 0);

    uart.clear_tx();
    usleep(20000);
    serial.update();
    EXPECT_EQ(uart.tx_length(), 0);

    // The complete attitude + raw RC input + physical RC output programming
    // aid fits in one negotiated subscription and one telemetry frame.
    Vektor::Protocol::PayloadWriter full_subscribe(payload, sizeof(payload));
    full_subscribe.u32(0);
    full_subscribe.u16(
        Vektor::SerialCatalog::Stream::ATTITUDE_RC_IO_COUNT);
    for (const Vektor::SerialCatalog::Reference *field :
         Vektor::SerialCatalog::Stream::ATTITUDE_RC_IO) {
        full_subscribe.u32(field->id);
    }
    ASSERT_TRUE(full_subscribe.ok());
    ASSERT_TRUE(push_request(uart,
                             Vektor::Protocol::MessageType::SUBSCRIBE,
                             5,
                             full_subscribe.data(),
                             full_subscribe.length()));
    serial.update();
    parser.reset();
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    ASSERT_EQ(response.message_type,
              Vektor::Protocol::MessageType::SUBSCRIBE);

    Vektor::Protocol::PayloadReader full_response(response.payload,
                                                  response.payload_len);
    uint16_t full_subscription_id = 0;
    uint32_t full_period_us = 0;
    uint16_t full_field_count = 0;
    ASSERT_TRUE(full_response.u16(full_subscription_id));
    ASSERT_TRUE(full_response.u32(full_period_us));
    ASSERT_TRUE(full_response.u16(full_field_count));
    EXPECT_EQ(full_period_us, 1000000U / Vektor::max_realtime_rate_hz);
    EXPECT_EQ(full_field_count,
              Vektor::SerialCatalog::Stream::ATTITUDE_RC_IO_COUNT);

    uart.clear_tx();
    usleep(20000);
    serial.update();
    parser.reset();
    ASSERT_TRUE(parse_single_frame(uart, parser, response));
    ASSERT_EQ(response.message_type,
              Vektor::Protocol::MessageType::TELEMETRY);
    Vektor::Protocol::PayloadReader full_telemetry(response.payload,
                                                   response.payload_len);
    uint16_t ignored_subscription_id = 0;
    uint16_t ignored_sample_sequence = 0;
    uint64_t ignored_timestamp_us = 0;
    uint8_t full_quality_len = 0;
    const uint8_t *full_quality = nullptr;
    const uint8_t *full_values = nullptr;
    ASSERT_TRUE(full_telemetry.u16(ignored_subscription_id));
    ASSERT_TRUE(full_telemetry.u16(ignored_sample_sequence));
    ASSERT_TRUE(full_telemetry.u64(ignored_timestamp_us));
    ASSERT_TRUE(full_telemetry.u8(full_quality_len));
    ASSERT_EQ(full_quality_len, 9);
    ASSERT_TRUE(full_telemetry.bytes(full_quality, full_quality_len));
    ASSERT_TRUE(full_telemetry.bytes(full_values, 96));
    EXPECT_EQ(full_telemetry.remaining(), 0);
    EXPECT_EQ(ignored_subscription_id, full_subscription_id);
    EXPECT_EQ(ignored_sample_sequence, 0);
    EXPECT_NE(ignored_timestamp_us, 0U);
    EXPECT_NE(full_quality, nullptr);
    EXPECT_NE(full_values, nullptr);
}

AP_GTEST_MAIN()
