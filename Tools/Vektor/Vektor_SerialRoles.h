#pragma once

#include "Config.h"
#include "Vektor_Capability.h"

#include <AP_Param/AP_Param.h>

#include <stdint.h>

namespace Vektor {

enum class SerialRole : int16_t {
    NONE = 0,
    RCIN = 1,
    VEKTOR = 2,
    MAVLINK1 = 3,
    MAVLINK2 = 4,
};

enum class UsbMode : int16_t {
    VEKTOR_ONLY = 0,
    VEKTOR_MAVLINK = 1,
};

enum class SerialEndpointStatus : int16_t {
    OK = 0,
    RESERVED_NOT_IMPLEMENTED = 1,
    INVALID_ROLE = 2,
    UNSUPPORTED_ROLE = 3,
    DUPLICATE_RCIN = 4,
    DUPLICATE_VEKTOR = 5,
    INVALID_BAUD = 6,
    ENDPOINT_UNAVAILABLE = 7,
    CONFIGURATION_INVALID = 8,
};

class SerialRoleParameters {
public:
    static constexpr uint8_t max_endpoints = 10;

    SerialRoleParameters();

    AP_Int16 role[max_endpoints];
    AP_Int32 baud[max_endpoints];
    AP_Int16 usb_mode;

    static const AP_Param::GroupInfo var_info[];
};

bool validate_serial_roles(
    const BoardCapability &capability,
    const int16_t roles[SerialRoleParameters::max_endpoints],
    SerialEndpointStatus statuses[SerialRoleParameters::max_endpoints]);

class SerialRoleManager {
public:
    void init(const BoardCapability &capability,
              SerialRoleParameters &parameters);

    bool boot_configuration_valid() const { return _boot_valid; }
    SerialRole configured_role(uint8_t serial_index) const;
    SerialRole boot_configured_role(uint8_t serial_index) const;
    SerialRole active_role(uint8_t serial_index) const;
    uint32_t configured_baud(uint8_t serial_index) const;
    uint32_t active_baud(uint8_t serial_index) const;
    SerialEndpointStatus status(uint8_t serial_index) const;
    bool reboot_required(uint8_t serial_index) const;

    UsbMode configured_usb_mode() const;
    UsbMode boot_configured_usb_mode() const;
    UsbMode active_usb_mode() const { return UsbMode::VEKTOR_ONLY; }
    SerialEndpointStatus usb_status() const;
    bool usb_reboot_required() const;

    void set_configured_role(uint8_t serial_index, SerialRole role);
    void set_configured_baud(uint8_t serial_index, uint32_t baud);
    void set_configured_usb_mode(UsbMode mode);
    void mark_endpoint_unavailable(uint8_t serial_index);

private:
    bool index_valid(uint8_t serial_index) const;
    bool current_roles_valid(SerialEndpointStatus *statuses) const;

    const BoardCapability *_capability = nullptr;
    SerialRoleParameters *_parameters = nullptr;
    int16_t _boot_roles[SerialRoleParameters::max_endpoints] {};
    uint32_t _boot_baud[SerialRoleParameters::max_endpoints] {};
    SerialRole _active_roles[SerialRoleParameters::max_endpoints] {};
    SerialEndpointStatus
        _boot_status[SerialRoleParameters::max_endpoints] {};
    SerialEndpointStatus
        _runtime_status[SerialRoleParameters::max_endpoints] {};
    int16_t _boot_usb_mode = int16_t(UsbMode::VEKTOR_ONLY);
    bool _boot_valid = false;
};

} // namespace Vektor
