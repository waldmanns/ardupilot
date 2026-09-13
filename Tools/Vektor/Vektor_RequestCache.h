#pragma once

#include "Vektor_Protocol.h"

#include <stdint.h>

namespace Vektor {

class RequestReplayCache {
public:
    static constexpr uint8_t cache_size = 4;

    enum class LookupResult : uint8_t {
        MISS,
        REPLAY,
        CONFLICT,
    };

    struct CachedResponse {
        const uint8_t *encoded;
        uint16_t encoded_len;
    };

    void reset();
    LookupResult lookup(Protocol::MessageType request_type,
                        uint16_t sequence,
                        uint32_t payload_crc,
                        CachedResponse &response) const;
    void remember(Protocol::MessageType request_type,
                  uint16_t sequence,
                  uint32_t payload_crc,
                  const uint8_t *encoded,
                  uint16_t encoded_len);

private:
    struct Entry {
        bool valid;
        Protocol::MessageType request_type;
        uint16_t sequence;
        uint32_t payload_crc;
        uint16_t encoded_len;
        uint8_t encoded[Protocol::MAX_ENCODED_STREAM_SIZE];
    };

    uint8_t _next = 0;
    Entry _entries[cache_size] {};
};

} // namespace Vektor
