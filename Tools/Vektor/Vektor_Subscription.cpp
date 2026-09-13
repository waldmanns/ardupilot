#include "Vektor_Subscription.h"

#include <limits.h>
#include <string.h>

namespace Vektor {

constexpr uint16_t SubscriptionTable::max_subscriptions;
constexpr uint16_t SubscriptionTable::max_fields;

void SubscriptionTable::reset(uint32_t scheduler_period_us)
{
    _scheduler_period_us = scheduler_period_us == 0 ? 1 : scheduler_period_us;
    _next_id = 1;
    for (uint16_t i = 0; i < max_subscriptions; i++) {
        _entries[i].active = false;
    }
}

SubscriptionTable::AddResult SubscriptionTable::add(
    uint32_t requested_period_us,
    const uint32_t *field_ids,
    uint16_t field_count,
    uint64_t now_us,
    const Entry *&created)
{
    created = nullptr;
    if (field_ids == nullptr || field_count == 0 || field_count > max_fields) {
        return AddResult::INVALID_FIELD_COUNT;
    }

    Entry *free_entry = nullptr;
    for (uint16_t i = 0; i < max_subscriptions; i++) {
        if (!_entries[i].active) {
            free_entry = &_entries[i];
            break;
        }
    }
    if (free_entry == nullptr) {
        return AddResult::FULL;
    }

    const uint32_t period_us = accepted_period(requested_period_us);
    const uint16_t subscription_id = allocate_id();
    if (subscription_id == 0) {
        return AddResult::FULL;
    }
    free_entry->id = subscription_id;
    free_entry->active = true;
    free_entry->period_us = period_us;
    free_entry->next_sample_us =
        now_us > UINT64_MAX - period_us ? UINT64_MAX : now_us + period_us;
    free_entry->sample_sequence = 0;
    free_entry->field_count = field_count;
    memcpy(free_entry->field_ids,
           field_ids,
           field_count * sizeof(field_ids[0]));
    created = free_entry;
    return AddResult::OK;
}

bool SubscriptionTable::remove(uint16_t subscription_id)
{
    for (uint16_t i = 0; i < max_subscriptions; i++) {
        if (_entries[i].active && _entries[i].id == subscription_id) {
            _entries[i].active = false;
            return true;
        }
    }
    return false;
}

SubscriptionTable::Entry *SubscriptionTable::claim_due(uint64_t now_us)
{
    for (uint16_t i = 0; i < max_subscriptions; i++) {
        Entry &entry = _entries[i];
        if (!entry.active || now_us < entry.next_sample_us) {
            continue;
        }

        const uint64_t periods_elapsed =
            ((now_us - entry.next_sample_us) / entry.period_us) + 1U;
        if (periods_elapsed >
            (UINT64_MAX - entry.next_sample_us) / entry.period_us) {
            entry.next_sample_us = UINT64_MAX;
        } else {
            entry.next_sample_us += periods_elapsed * entry.period_us;
        }
        return &entry;
    }
    return nullptr;
}

const SubscriptionTable::Entry *SubscriptionTable::find(
    uint16_t subscription_id) const
{
    for (uint16_t i = 0; i < max_subscriptions; i++) {
        if (_entries[i].active && _entries[i].id == subscription_id) {
            return &_entries[i];
        }
    }
    return nullptr;
}

uint16_t SubscriptionTable::active_count() const
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < max_subscriptions; i++) {
        if (_entries[i].active) {
            count++;
        }
    }
    return count;
}

uint16_t SubscriptionTable::allocate_id()
{
    for (uint16_t attempts = 0; attempts <= max_subscriptions; attempts++) {
        const uint16_t candidate = _next_id++;
        if (_next_id == 0) {
            _next_id = 1;
        }
        if (candidate != 0 && find(candidate) == nullptr) {
            return candidate;
        }
    }

    // There is a free table entry, so at most max_subscriptions IDs can be in
    // use. This is unreachable unless the table is corrupted.
    return 0;
}

uint32_t SubscriptionTable::accepted_period(uint32_t requested_period_us) const
{
    if (requested_period_us == 0 ||
        requested_period_us <= _scheduler_period_us) {
        return _scheduler_period_us;
    }

    const uint64_t periods =
        (uint64_t(requested_period_us) + _scheduler_period_us - 1U) /
        _scheduler_period_us;
    const uint64_t rounded = periods * _scheduler_period_us;
    return rounded > UINT32_MAX ? UINT32_MAX : uint32_t(rounded);
}

} // namespace Vektor
