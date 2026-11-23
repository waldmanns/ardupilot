#include "Sequencer.h"
#include "SequencerDocument.h"

#include <cassert>
#include <cstdint>
#include <cstring>

using Vektor2::Sequencer;
namespace Codec = Vektor2::SequencerDocument;

static Sequencer::Document demo()
{
    Sequencer::Document doc;
    doc.lanes[0].enabled = 1;
    doc.lanes[0].idle = 1250;
    doc.blocks[8] = {100, 0, 0, 1, 1, Sequencer::Type::Wait};
    doc.blocks[2] = {300, 1100, 1700, 1, 2, Sequencer::Type::Ramp};
    doc.blocks[13] = {100, 0, 1600, 1, 3, Sequencer::Type::Set};

    doc.lanes[1].enabled = 1;
    doc.lanes[1].output = Sequencer::Output::Binary;
    doc.lanes[1].idle = 0;
    doc.blocks[0] = {100, 0, 1, 2, 1, Sequencer::Type::Set};

    doc.lanes[2].enabled = 1;
    doc.lanes[2].clock = Sequencer::Clock::Independent;
    doc.lanes[2].idle = 2000;
    doc.blocks[4] = {800, 1750, 1350, 3, 1, Sequencer::Type::Ramp};

    assert(doc.valid());
    return doc;
}

static void playback()
{
    auto doc = demo();
    Sequencer seq;
    const auto shared = Sequencer::shared_slot;
    seq.load(&doc);
    assert(!seq.status(0).valid);
    assert(!seq.power(true));
    seq.activate(true);
    assert(seq.status(0).pulse == 1250);
    assert(seq.status(1).valid && seq.status(1).logical == 0 && seq.status(1).pulse == 1000);
    assert(!seq.start(shared, 0));
    assert(seq.power(true));
    seq.tick(0);
    assert(seq.start(shared, 0));
    assert(seq.status(0).block == 9 && seq.status(0).pulse == 1250);
    assert(seq.status(1).pulse == 2000 && seq.status(1).run == seq.status(0).run);
    assert(seq.status(2).state == Sequencer::State::Idle);
    seq.tick(99);
    assert(seq.status(0).block_elapsed == 99);
    seq.tick(100);
    assert(seq.status(0).block == 3 && seq.status(0).pulse == 1100);
    assert(seq.status(1).holding && seq.status(1).elapsed == 100 &&
           seq.status(1).block_elapsed == 100);
    assert(seq.start(2, 100));
    assert(seq.status(2).elapsed == 0 && seq.status(2).pulse == 1750 &&
           seq.status(2).run != seq.status(0).run);
    seq.tick(101);
    assert(seq.status(2).pulse == 1749); // nearest with away-from-From half step
    seq.tick(200);
    assert(seq.status(0).pulse == 1300 && seq.status(2).pulse == 1700);
    seq.tick(300);
    seq.tick(399);
    assert(seq.status(0).pulse == 1698);
    seq.tick(400);
    assert(seq.status(0).block == 14 && seq.status(0).pulse == 1600);
    seq.stop(shared);
    assert(seq.status(0).pulse == 1250 && seq.status(1).pulse == 1000);
    assert(seq.status(2).elapsed == 300 && seq.status(2).state == Sequencer::State::Running);
    seq.tick(500);
    seq.tick(600);
    seq.tick(700);
    seq.tick(800);
    seq.tick(900);
    assert(seq.status(2).state == Sequencer::State::Complete && seq.status(2).block == 0);
    assert(seq.status(2).elapsed == 800 && seq.status(2).pulse == 2000);
    assert(seq.start(shared, 900));
    for (uint32_t t = 1000; t <= 1400; t += 100) {
        seq.tick(t);
    }
    assert(seq.status(0).state == Sequencer::State::Complete && seq.status(0).elapsed == 500);
    assert(seq.status(1).state == Sequencer::State::Complete && seq.status(1).elapsed == 100);
    seq.power(false);
    assert(!seq.powered() && seq.status(1).valid && seq.status(1).pulse == 1000);
    seq.activate(false);
    assert(!seq.status(0).valid && !seq.status(1).valid && !seq.running());
}

static void trigger_baselines()
{
    auto doc = demo();
    doc.lanes[0].repeat = 1;
    doc.lanes[2].repeat = 1;
    doc.shared.edge = Sequencer::Edge::Rising;
    doc.lanes[2].trigger.edge = Sequencer::Edge::Falling;
    const uint8_t shared = Sequencer::shared_slot;
    Sequencer seq;
    seq.load(&doc);
    seq.activate(true);
    seq.power(true);
    assert(!seq.start(shared, 0));

    const auto tick = [&](uint32_t t, uint16_t shared_input, uint16_t lane_input, uint32_t suppress = 0) {
        seq.sample(shared, {shared_input, shared_input != 0});
        seq.sample(2, {lane_input, lane_input != 0});
        seq.tick(t, suppress);
    };
    tick(UINT32_MAX - 20, 1700, 1700);
    assert(seq.shared_status().state == Sequencer::State::Armed);
    assert(!seq.running());
    tick(UINT32_MAX - 10, 1200, 1700);
    tick(0, 1500, 1200);
    assert(seq.shared_status().run != 0 && seq.status(2).run != 0 && seq.status(0).elapsed == 0);
    const auto run = seq.status(0).run;
    tick(40, 1200, 1200);
    tick(80, 1700, 1200);
    assert(seq.status(0).run == run && seq.status(0).elapsed == 80);
    tick(90, 0, 1200);
    assert(seq.shared_status().reason == Sequencer::Reason::InputLost);
    assert(seq.status(2).state == Sequencer::State::Running);
    tick(100, 1700, 1200);
    assert(seq.status(0).run == run);
    tick(110, 1200, 1200);
    tick(120, 1700, 1200, 1UL << shared);
    assert(seq.status(0).run == run);
    tick(130, 1700, 1200);
    assert(seq.status(0).run == run);
    tick(140, 1200, 1200);
    tick(150, 1700, 1200);
    assert(seq.status(0).run != run);
    seq.routes_changed(1UL << shared);
    assert(seq.shared_status().reason == Sequencer::Reason::RouteChanged);
    tick(160, 1700, 1200);
    assert(seq.shared_status().state == Sequencer::State::Armed);
    tick(261, 1700, 1200);
    assert(seq.status(2).reason == Sequencer::Reason::LoopGap && !seq.running());
    assert(!seq.start(shared, 261));
    tick(271, 1700, 1200);
    assert(!seq.running());
}

static void long_run()
{
    Sequencer::Document doc;
    doc.lanes[0].enabled = 1;
    doc.lanes[0].clock = Sequencer::Clock::Independent;
    for (uint8_t n = 0; n < Sequencer::per_lane; n++) {
        doc.blocks[n] = {60000, 0, uint16_t(1000 + n), 1, uint8_t(n + 1), Sequencer::Type::Set};
    }
    assert(doc.valid());
    Sequencer seq;
    seq.load(&doc);
    seq.activate(true);
    seq.power(true);
    seq.tick(0);
    assert(seq.start(0, 0));
    const uint32_t end = uint32_t(Sequencer::per_lane) * 60000;
    for (uint32_t t = 100; t < end; t += 100) {
        seq.tick(t);
    }
    assert(seq.status(0).block == Sequencer::per_lane && seq.status(0).block_elapsed == 59900);
    seq.tick(end);
    assert(seq.status(0).elapsed == end && seq.status(0).state == Sequencer::State::Complete);
}

static void boot_autostart()
{
    auto doc = demo();
    doc.lanes[2].autostart = 1;
    doc.lanes[3].enabled = 1;
    doc.lanes[3].clock = Sequencer::Clock::Independent;
    doc.lanes[3].autostart = 1;
    doc.blocks[5] = {800, 0, 1600, 4, 1, Sequencer::Type::Set};
    assert(doc.valid());
    Sequencer seq;
    seq.activate(true);
    seq.load(&doc);
    seq.tick(0);
    assert(!seq.powered() && !seq.running()); // A regular document Apply.
    seq.schedule_boot_start(0);
    assert(seq.boot_start_pending());
    seq.tick(10);
    assert(seq.powered() && seq.running() && !seq.boot_start_pending());
    assert(seq.status(2).run != 0 && seq.status(2).elapsed == 0 && seq.status(2).pulse == 1750);
    assert(seq.status(3).run != 0 && seq.status(3).elapsed == 0 && seq.status(3).pulse == 1600);
    assert(seq.status(0).run == 0 && seq.shared_status().run == 0);
    const auto run = seq.status(2).run;
    for (uint32_t t = 110; t <= 910; t += 100) {
        seq.tick(t);
    }
    assert(seq.status(2).state == Sequencer::State::Complete && seq.status(2).run == run);

    doc.lanes[0].autostart = 1;
    doc.lanes[3].enabled = 0;
    doc.shared.edge = Sequencer::Edge::Rising;
    assert(doc.valid());
    constexpr auto shared = Sequencer::shared_slot;
    Sequencer triggered;
    triggered.activate(true);
    triggered.load(&doc);
    triggered.schedule_boot_start(UINT32_MAX - 20);
    triggered.tick(UINT32_MAX - 10);
    assert(triggered.powered() && triggered.status(2).run != 0 && triggered.boot_start_pending());
    assert(triggered.shared_status().run == 0 && triggered.status(3).run == 0);
    triggered.sample(shared, {1700, true});
    triggered.tick(0); // The first valid sample satisfies the start without an edge.
    assert(triggered.shared_status().run != 0 && triggered.status(0).elapsed == 0);
    assert(triggered.status(1).run == 0 && !triggered.boot_start_pending());
    triggered.sample(shared, {});
    triggered.tick(10);
    assert(triggered.shared_status().reason == Sequencer::Reason::InputLost);
    assert(triggered.status(2).state == Sequencer::State::Running);
    triggered.sample(shared, {1700, true});
    triggered.tick(20);
    assert(triggered.shared_status().state == Sequencer::State::Armed && triggered.status(1).run == 0);
    triggered.sample(shared, {1200, true});
    triggered.tick(30);
    triggered.sample(shared, {1700, true});
    triggered.tick(40);
    assert(triggered.status(1).run == triggered.status(0).run);

    doc.lanes[2].autostart = 0;
    Sequencer cancelled;
    cancelled.load(&doc);
    cancelled.schedule_boot_start(0); // Activation was off at boot.
    cancelled.activate(true);
    cancelled.tick(0);
    assert(!cancelled.powered() && !cancelled.boot_start_pending());
    cancelled.schedule_boot_start(0, 1UL << shared); // Stop issued while loading.
    assert(!cancelled.boot_start_pending());
    cancelled.schedule_boot_start(0);
    cancelled.stop(shared);
    cancelled.sample(shared, {1700, true});
    cancelled.tick(10);
    assert(!cancelled.powered() && !cancelled.boot_start_pending());
    cancelled.schedule_boot_start(10);
    cancelled.power(false);
    assert(!cancelled.boot_start_pending());
    cancelled.schedule_boot_start(10);
    cancelled.activate(false);
    assert(!cancelled.boot_start_pending());
    cancelled.activate(true);
    cancelled.schedule_boot_start(10);
    for (uint32_t t = 110; t < 10 + Sequencer::autostart_window_ms; t += 100) {
        cancelled.tick(t);
    }
    cancelled.sample(shared, {1700, true});
    cancelled.tick(10 + Sequencer::autostart_window_ms);
    assert(!cancelled.powered() && !cancelled.boot_start_pending());
}

static void repeat_playback()
{
    auto doc = demo();
    for (uint8_t lane = 0; lane < 4; lane++) {
        doc.lanes[lane].autostart = 1;
    }
    doc.lanes[1].repeat = 1;
    doc.lanes[2].repeat = 1;
    doc.lanes[3].repeat = 1;
    doc.blocks[5] = {800, 0, 1600, 4, 1, Sequencer::Type::Set};
    assert(doc.valid());
    Sequencer seq;
    seq.activate(true);
    seq.load(&doc);
    const uint32_t start = UINT32_MAX - 250;
    seq.schedule_boot_start(start);
    seq.tick(start);
    const uint32_t shared_run = seq.shared_status().run;
    const uint32_t independent_run = seq.status(2).run;
    assert(shared_run != 0 && independent_run != 0 && independent_run != shared_run);
    assert(seq.status(3).state == Sequencer::State::Disabled && seq.status(3).run == 0);
    for (uint32_t elapsed = 100; elapsed <= 407; elapsed += 100) {
        seq.tick(start + elapsed);
    }
    seq.tick(start + 407);
    seq.tick(start + 507);
    assert(seq.status(0).state == Sequencer::State::Complete && seq.status(0).pulse == 1250);
    assert(seq.status(1).state == Sequencer::State::Running && seq.status(1).elapsed == 7);
    assert(seq.status(1).run == shared_run && seq.shared_status().run == shared_run);
    for (uint32_t elapsed = 607; elapsed <= 1007; elapsed += 100) {
        seq.tick(start + elapsed);
    }
    assert(seq.status(0).state == Sequencer::State::Complete);
    assert(seq.status(1).elapsed == 7 && seq.status(2).elapsed == 207);
    assert(seq.status(2).run == independent_run);
    seq.stop(Sequencer::shared_slot);
    seq.tick(start + 1107);
    assert(seq.status(1).state == Sequencer::State::Idle && seq.status(1).pulse == 1000);
    assert(seq.status(2).state == Sequencer::State::Running);
    seq.power(false);
    assert(!seq.running() && seq.status(2).reason == Sequencer::Reason::PowerOff);
}

static void document_validation()
{
    auto doc = demo();
    doc.revision = UINT32_MAX;
    doc.lanes[1].autostart = 1;
    doc.lanes[2].autostart = 1;
    doc.lanes[1].repeat = 1;
    memcpy(doc.lanes[0].name, "Ramp", 5);
    uint8_t bytes[Codec::max_bytes];
    uint8_t encoded[Codec::max_bytes];
    size_t size = Codec::encode(doc, bytes);
    Sequencer::Document decoded;
    assert(Codec::decode(bytes, size, decoded) == Codec::Decode::Valid);
    assert(Codec::encode(decoded, encoded) == size && memcmp(encoded, bytes, size) == 0);
    assert(decoded.revision == UINT32_MAX && strcmp(decoded.lanes[0].name, "Ramp") == 0);
    assert(decoded.lanes[0].autostart == 0);
    assert(decoded.lanes[1].autostart == 1 && decoded.lanes[2].autostart == 1);
    assert(decoded.lanes[1].repeat == 1 && decoded.lanes[2].repeat == 0);
    const size_t blocks_offset = Codec::header_size + Sequencer::lane_count * Codec::lane_size;
    for (uint8_t version = 1; version <= 2; version++) {
        const size_t legacy_lane_size = Codec::legacy_lane_size + version - 1;
        memcpy(encoded, bytes, Codec::header_size);
        encoded[4] = version;
        for (uint8_t i = 0; i < Sequencer::lane_count; i++) {
            memcpy(encoded + Codec::header_size + i * legacy_lane_size,
                   bytes + Codec::header_size + i * Codec::lane_size, legacy_lane_size);
        }
        const size_t legacy_offset = Codec::header_size + Sequencer::lane_count * legacy_lane_size;
        const size_t legacy_size = size - (blocks_offset - legacy_offset);
        memcpy(encoded + legacy_offset, bytes + blocks_offset, size - blocks_offset);
        for (uint8_t i = 0; i < 4; i++) {
            encoded[8 + i] = legacy_size >> (8 * i);
        }
        const auto legacy_crc = Codec::checksum(encoded, legacy_size);
        for (uint8_t i = 0; i < 4; i++) {
            encoded[16 + i] = legacy_crc >> (8 * i);
        }
        assert(Codec::decode(encoded, legacy_size, decoded) == Codec::Decode::Valid);
        assert(decoded.lanes[1].autostart == version - 1 && decoded.lanes[1].repeat == 0);
        assert(decoded.revision == UINT32_MAX);
    }
    for (size_t cut = 0; cut < size; cut++) {
        assert(Codec::decode(bytes, cut, decoded) != Codec::Decode::Valid);
    }
    bytes[size - 1] ^= 1;
    assert(Codec::decode(bytes, size, decoded) == Codec::Decode::Invalid);
    bytes[4] = Codec::schema_version + 1;
    assert(Codec::decode(bytes, size, decoded) == Codec::Decode::Unsupported);

    doc.lanes[1].autostart = 2;
    assert(!doc.valid());
    doc.lanes[1].autostart = 1;
    doc.lanes[1].repeat = 2;
    assert(!doc.valid());

    doc = demo();
    doc.blocks[0].type = Sequencer::Type::Ramp;
    assert(!doc.valid());
    doc = demo();
    doc.blocks[13].position = 2;
    assert(!doc.valid());
    doc.blocks[13].position = 4;
    assert(!doc.valid());
    doc = demo();
    doc.blocks[13].duration = 60001;
    assert(!doc.valid());
    doc = demo();
    doc.lanes[0].name[2] = 'A';
    assert(!doc.valid());
    doc = demo();
    doc.lanes[1].trigger.edge = Sequencer::Edge::Rising;
    assert(!doc.valid());

    // Valid checksums do not override duplicate stable identities.
    doc = demo();
    size = Codec::encode(doc, bytes);
    bytes[Codec::header_size + Codec::lane_size] = 1;
    const auto crc = Codec::checksum(bytes, size);
    for (uint8_t i = 0; i < 4; i++) {
        bytes[16 + i] = crc >> (8 * i);
    }
    assert(Codec::decode(bytes, size, decoded) == Codec::Decode::Invalid);
}

int main()
{
    playback();
    trigger_baselines();
    long_run();
    boot_autostart();
    repeat_playback();
    document_validation();
}
