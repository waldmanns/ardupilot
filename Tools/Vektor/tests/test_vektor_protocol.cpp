#include <AP_gtest.h>

#include "Vektor_Protocol.h"
#include "Vektor_RequestCache.h"
#include "Vektor_Runtime.h"

#include <string.h>

namespace {

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

AP_GTEST_MAIN()
