#include "Vektor_RequestCache.h"

#include <string.h>

namespace Vektor {

void RequestReplayCache::reset()
{
    _next = 0;
    for (uint8_t i = 0; i < cache_size; i++) {
        _entries[i].valid = false;
    }
}

RequestReplayCache::LookupResult RequestReplayCache::lookup(
    Protocol::MessageType request_type,
    uint16_t sequence,
    uint32_t payload_crc,
    CachedResponse &response) const
{
    response.encoded = nullptr;
    response.encoded_len = 0;

    for (uint8_t i = 0; i < cache_size; i++) {
        const Entry &entry = _entries[i];
        if (!entry.valid ||
            entry.request_type != request_type ||
            entry.sequence != sequence) {
            continue;
        }

        if (entry.payload_crc != payload_crc) {
            return LookupResult::CONFLICT;
        }

        response.encoded = entry.encoded;
        response.encoded_len = entry.encoded_len;
        return LookupResult::REPLAY;
    }

    return LookupResult::MISS;
}

void RequestReplayCache::remember(Protocol::MessageType request_type,
                                  uint16_t sequence,
                                  uint32_t payload_crc,
                                  const uint8_t *encoded,
                                  uint16_t encoded_len)
{
    if (encoded == nullptr ||
        encoded_len == 0 ||
        encoded_len > Protocol::MAX_ENCODED_STREAM_SIZE) {
        return;
    }

    Entry &entry = _entries[_next];
    entry.valid = true;
    entry.request_type = request_type;
    entry.sequence = sequence;
    entry.payload_crc = payload_crc;
    entry.encoded_len = encoded_len;
    memcpy(entry.encoded, encoded, encoded_len);

    _next++;
    if (_next >= cache_size) {
        _next = 0;
    }
}

} // namespace Vektor
