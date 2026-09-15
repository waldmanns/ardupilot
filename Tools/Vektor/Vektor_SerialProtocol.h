#pragma once

#include "Vektor_AssignmentMatrix.h"
#include "Vektor_Attitude.h"
#include "Vektor_Capability.h"
#include "Vektor_Parameters.h"
#include "Vektor_Protocol.h"
#include "Vektor_PwmInput.h"
#include "Vektor_PwmOutput.h"
#include "Vektor_RequestCache.h"
#include "Vektor_Rcin.h"
#include "Vektor_Runtime.h"
#include "Vektor_Subscription.h"
#include "Vektor_Vsp.h"

#include <AP_HAL/UARTDriver.h>

namespace Vektor {

class SerialProtocol {
public:
    void init(AP_HAL::UARTDriver *uart,
              const BoardCapability &capability,
              Parameters &parameters,
              const RuntimeState &runtime,
              const AttitudeSource &attitude,
              const VspComponent &vsp,
              const RcinSource &rcin,
              PwmInput &pwm_input,
              PwmOutput &pwm_output,
              AssignmentMatrix &assignments);
    void update();

private:
    struct ParameterUpdate {
        uint32_t field_id;
        uint8_t type_id;
        uint32_t raw;
    };

    void handle_frame(const Protocol::FrameView &frame);
    void handle_hello(const Protocol::FrameView &frame);
    void handle_ping(const Protocol::FrameView &frame);
    void handle_describe(const Protocol::FrameView &frame);
    void handle_get(const Protocol::FrameView &frame);
    void handle_set(const Protocol::FrameView &frame);
    void handle_get_many(const Protocol::FrameView &frame);
    void handle_set_many(const Protocol::FrameView &frame);
    void handle_get_all_params(const Protocol::FrameView &frame);
    void handle_subscribe(const Protocol::FrameView &frame);
    void handle_unsubscribe(const Protocol::FrameView &frame);
    void handle_route_list(const Protocol::FrameView &frame);
    void handle_route_set(const Protocol::FrameView &frame);
    void handle_route_delete(const Protocol::FrameView &frame);
    void service_telemetry(uint64_t now_us);
    bool send_payload(Protocol::MessageType message_type,
                      uint8_t flags,
                      uint16_t sequence,
                      const uint8_t *payload,
                      uint16_t payload_len);
    bool send_error(const Protocol::FrameView &request,
                    Protocol::ErrorCode code,
                    uint32_t object_id,
                    const char *detail);
    bool replay_cached_response_or_reject(const Protocol::FrameView &frame);
    void begin_request_cache_capture(const Protocol::FrameView &frame);
    void end_request_cache_capture();
    void remember_cached_response(const uint8_t *encoded,
                                  uint16_t encoded_len);
    bool send_board_description(uint16_t sequence,
                                uint8_t domain,
                                uint32_t cursor,
                                uint16_t max_records);
    bool send_endpoint_descriptions(uint16_t sequence,
                                    uint8_t domain,
                                    uint32_t cursor,
                                    uint16_t max_records);
    bool send_component_descriptions(uint16_t sequence,
                                     uint8_t domain,
                                     uint32_t cursor,
                                     uint16_t max_records);
    bool send_field_descriptions(uint16_t sequence,
                                 uint8_t domain,
                                 uint32_t cursor,
                                 uint16_t max_records);
    bool send_timer_group_descriptions(uint16_t sequence,
                                       uint8_t domain,
                                       uint32_t cursor,
                                       uint16_t max_records);
    bool send_runtime_limit_descriptions(uint16_t sequence,
                                         uint8_t domain,
                                         uint32_t cursor,
                                         uint16_t max_records);
    bool send_empty_description(uint16_t sequence, uint8_t domain);
    bool send_description_page(uint16_t sequence,
                               uint8_t domain,
                               uint32_t cursor,
                               uint16_t max_records,
                               uint16_t total_records);
    bool build_descriptor_record(uint8_t domain,
                                 uint16_t index,
                                 uint8_t *record,
                                 uint16_t record_capacity,
                                 uint16_t &record_len);
    bool build_endpoint_record(uint16_t index,
                               uint8_t *record,
                               uint16_t record_capacity,
                               uint16_t &record_len);
    bool build_component_record(uint16_t index,
                                uint8_t *record,
                                uint16_t record_capacity,
                                uint16_t &record_len);
    bool build_field_record(uint16_t index,
                            uint8_t *record,
                            uint16_t record_capacity,
                            uint16_t &record_len);
    bool build_timer_group_record(uint16_t index,
                                  uint8_t *record,
                                  uint16_t record_capacity,
                                  uint16_t &record_len);
    bool build_board_record(uint16_t index,
                            uint8_t *record,
                            uint16_t record_capacity,
                            uint16_t &record_len);
    bool build_runtime_limit_record(uint16_t index,
                                    uint8_t *record,
                                    uint16_t record_capacity,
                                    uint16_t &record_len);
    bool send_value(uint16_t sequence, uint32_t field_id);
    bool send_values(uint16_t sequence,
                     const uint32_t *field_ids,
                     uint16_t field_count);
    bool send_all_parameters(uint16_t sequence);
    bool send_routes(uint16_t sequence,
                     uint32_t cursor,
                     uint16_t max_records);
    bool send_single_route(uint16_t sequence,
                           const AssignmentMatrix::Entry &route);
    bool write_field_value(Protocol::PayloadWriter &writer,
                           uint32_t field_id) const;
    bool write_field_payload(Protocol::PayloadWriter &writer,
                             uint32_t field_id) const;
    uint8_t field_quality_code(uint32_t field_id) const;
    bool apply_parameter_value(uint32_t field_id, uint8_t type_id, uint32_t raw);
    bool validate_parameter_value(uint32_t field_id,
                                  uint8_t type_id,
                                  uint32_t raw,
                                  Protocol::ErrorCode &code,
                                  const char *&detail) const;
    bool validate_parameter_set(const ParameterUpdate *updates,
                                uint16_t count,
                                Protocol::ErrorCode &code,
                                uint32_t &failing_field_id,
                                const char *&detail) const;
    bool read_typed_value(Protocol::PayloadReader &reader,
                          Protocol::PrimitiveType type,
                          uint32_t &raw) const;
    uint64_t coarse_capability_flags() const;
    uint64_t device_id() const;
    bool initialize_descriptor_identity();
    bool refresh_capability_hash();
    bool validate_descriptor_ids();
    bool calculate_descriptor_hash(const uint8_t *domains,
                                   uint8_t domain_count,
                                   uint64_t &hash);
    bool descriptor_id(uint8_t domain,
                       uint16_t index,
                       uint32_t &id);
    uint16_t descriptor_count(uint8_t domain) const;
    uint32_t protocol_field_id(uint16_t index) const;
    uint16_t parameter_field_count() const;
    uint16_t descriptor_page_limit() const;
    uint16_t component_count() const;
    uint16_t field_count() const;
    uint16_t endpoint_count() const;
    uint16_t timer_group_count() const;
    uint16_t runtime_limit_count() const;
    bool field_available(const FieldDescriptor &field) const;
    const FieldDescriptor *available_field_by_index(uint16_t index,
                                                    uint16_t *schema_index = nullptr) const;

    AP_HAL::UARTDriver *_uart = nullptr;
    const BoardCapability *_capability = nullptr;
    Parameters *_parameters = nullptr;
    const RuntimeState *_runtime = nullptr;
    const AttitudeSource *_attitude = nullptr;
    const VspComponent *_vsp = nullptr;
    const RcinSource *_rcin = nullptr;
    PwmInput *_pwm_input = nullptr;
    PwmOutput *_pwm_output = nullptr;
    AssignmentMatrix *_assignments = nullptr;
    Protocol::Parser _parser;
    bool _ready = false;
    bool _hello_seen = false;
    bool _capture_response = false;
    Protocol::MessageType _capture_request_type =
        Protocol::MessageType::ERROR;
    uint16_t _capture_sequence = 0;
    uint32_t _capture_payload_crc = 0;
    uint32_t _server_nonce = 0;
    uint16_t _hello_sequence = 0;
    uint32_t _hello_payload_crc = 0;
    uint64_t _schema_hash = 0;
    uint64_t _capability_hash = 0;
    uint32_t _rx_frames = 0;
    uint32_t _rx_drops = 0;
    uint32_t _tx_drops = 0;
    uint16_t _telemetry_sequence = 0;
    uint8_t _payload[Protocol::MAX_PAYLOAD_SIZE];
    uint8_t _tx_decoded[Protocol::MAX_DECODED_FRAME_SIZE];
    uint8_t _tx_encoded[Protocol::MAX_ENCODED_STREAM_SIZE];
    RequestReplayCache _request_cache;
    SubscriptionTable _subscriptions;
};

} // namespace Vektor
