#include "Vektor_SerialRoles.h"

namespace {

template <typename Parameter, typename Value>
void save_parameter(Parameter &parameter, Value value)
{
    if (parameter.get() != value) {
        parameter.set(value);
        parameter.save(true);
    }
}

bool valid_role_value(int16_t value)
{
    return value >= int16_t(Vektor::SerialRole::NONE) &&
           value <= int16_t(Vektor::SerialRole::MAVLINK2);
}

bool valid_usb_mode_value(int16_t value)
{
    return value >= int16_t(Vektor::UsbMode::VEKTOR_ONLY) &&
           value <= int16_t(Vektor::UsbMode::VEKTOR_MAVLINK);
}

bool valid_baud(int32_t baud)
{
    return baud >= Vektor::protocol_baud_min &&
           baud <= Vektor::protocol_baud_max;
}

} // namespace

namespace Vektor {

#define VEKTOR_SERIAL_PARAM_INFO(INDEX, BASE)                               \
    AP_GROUPINFO(#INDEX "_ROLE", BASE, SerialRoleParameters,              \
                 role[INDEX], int16_t(SerialRole::NONE)),                  \
    AP_GROUPINFO(#INDEX "_BAUD", BASE + 1, SerialRoleParameters,          \
                 baud[INDEX], protocol_baud)

const AP_Param::GroupInfo SerialRoleParameters::var_info[] = {
    // @Param: 0_ROLE
    // @DisplayName: SERIAL0 Vektor role
    // @Description: Owner selected for this logical ArduPilot serial endpoint. 0=None, 1=RC input, 2=Vektor Protocol, 3=MAVLink 1 (reserved), 4=MAVLink 2 (reserved). Changes take effect after reboot.
    // @Values: 0:None,1:RC Input,2:Vektor,3:MAVLink 1 (reserved),4:MAVLink 2 (reserved)
    // @RebootRequired: True
    VEKTOR_SERIAL_PARAM_INFO(0, 1),

    // @Param: 1_ROLE
    // @CopyFieldsFrom: SER0_ROLE
    VEKTOR_SERIAL_PARAM_INFO(1, 3),
    // @Param: 2_ROLE
    // @CopyFieldsFrom: SER0_ROLE
    VEKTOR_SERIAL_PARAM_INFO(2, 5),
    // @Param: 3_ROLE
    // @CopyFieldsFrom: SER0_ROLE
    VEKTOR_SERIAL_PARAM_INFO(3, 7),
    // @Param: 4_ROLE
    // @CopyFieldsFrom: SER0_ROLE
    VEKTOR_SERIAL_PARAM_INFO(4, 9),
    // @Param: 5_ROLE
    // @CopyFieldsFrom: SER0_ROLE
    VEKTOR_SERIAL_PARAM_INFO(5, 11),
    // @Param: 6_ROLE
    // @CopyFieldsFrom: SER0_ROLE
    VEKTOR_SERIAL_PARAM_INFO(6, 13),
    // @Param: 7_ROLE
    // @CopyFieldsFrom: SER0_ROLE
    VEKTOR_SERIAL_PARAM_INFO(7, 15),
    // @Param: 8_ROLE
    // @CopyFieldsFrom: SER0_ROLE
    VEKTOR_SERIAL_PARAM_INFO(8, 17),
    // @Param: 9_ROLE
    // @CopyFieldsFrom: SER0_ROLE
    VEKTOR_SERIAL_PARAM_INFO(9, 19),

    // @Param: USB_MODE
    // @DisplayName: USB protocol mode
    // @Description: USB always provides the Vektor recovery/configuration protocol. Vektor plus MAVLink is reserved for future firmware and currently leaves only Vektor active.
    // @Values: 0:Vektor only,1:Vektor plus MAVLink (reserved)
    // @RebootRequired: True
    AP_GROUPINFO("USB_MODE", 21, SerialRoleParameters, usb_mode,
                 int16_t(UsbMode::VEKTOR_ONLY)),

    AP_GROUPEND
};

#undef VEKTOR_SERIAL_PARAM_INFO

SerialRoleParameters::SerialRoleParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool validate_serial_roles(
    const BoardCapability &capability,
    const int16_t roles[SerialRoleParameters::max_endpoints],
    SerialEndpointStatus statuses[SerialRoleParameters::max_endpoints])
{
    if (roles == nullptr || statuses == nullptr ||
        capability.serial_endpoint_count > SerialRoleParameters::max_endpoints) {
        return false;
    }

    bool valid = true;
    uint8_t rcin_count = 0;
    uint8_t vektor_count = 0;
    for (uint8_t i = 0; i < SerialRoleParameters::max_endpoints; i++) {
        statuses[i] = SerialEndpointStatus::OK;
        const int16_t value = roles[i];
        if (!valid_role_value(value)) {
            statuses[i] = SerialEndpointStatus::INVALID_ROLE;
            valid = false;
            continue;
        }
        const SerialRole role = SerialRole(value);
        const uint8_t supported =
            serial_endpoint_supported_role_mask(capability, i);
        if ((supported & (1U << uint8_t(role))) == 0) {
            // USB and absent/EMPTY SERIAL_ORDER slots have no UART role
            // parameter surface. Their stored default NONE is valid.
            if (role != SerialRole::NONE) {
                statuses[i] = SerialEndpointStatus::UNSUPPORTED_ROLE;
                valid = false;
            }
            continue;
        }
        if (role == SerialRole::RCIN) {
            rcin_count++;
        } else if (role == SerialRole::VEKTOR) {
            vektor_count++;
        } else if (role == SerialRole::MAVLINK1 ||
                   role == SerialRole::MAVLINK2) {
            statuses[i] = SerialEndpointStatus::RESERVED_NOT_IMPLEMENTED;
        }
    }

    if (rcin_count > 1) {
        valid = false;
        for (uint8_t i = 0; i < capability.serial_endpoint_count; i++) {
            if (roles[i] == int16_t(SerialRole::RCIN)) {
                statuses[i] = SerialEndpointStatus::DUPLICATE_RCIN;
            }
        }
    }
    if (vektor_count > 1) {
        valid = false;
        for (uint8_t i = 0; i < capability.serial_endpoint_count; i++) {
            if (roles[i] == int16_t(SerialRole::VEKTOR)) {
                statuses[i] = SerialEndpointStatus::DUPLICATE_VEKTOR;
            }
        }
    }
    return valid;
}

void SerialRoleManager::init(const BoardCapability &capability,
                             SerialRoleParameters &parameters)
{
    _capability = &capability;
    _parameters = &parameters;
    _boot_usb_mode = parameters.usb_mode.get();

    for (uint8_t i = 0; i < SerialRoleParameters::max_endpoints; i++) {
        _boot_roles[i] = parameters.role[i].get();
        const int32_t baud = parameters.baud[i].get();
        _boot_baud[i] = valid_baud(baud) ? uint32_t(baud) : protocol_baud;
        _active_roles[i] = SerialRole::NONE;
        _runtime_status[i] = SerialEndpointStatus::OK;
    }

    _boot_valid = validate_serial_roles(capability,
                                        _boot_roles,
                                        _boot_status) &&
                  valid_usb_mode_value(_boot_usb_mode);
    for (uint8_t i = 0; i < capability.serial_endpoint_count; i++) {
        if (serial_endpoint_usable(capability, i) &&
            !serial_endpoint_is_usb(capability, i) &&
            !valid_baud(parameters.baud[i].get())) {
            _boot_status[i] = SerialEndpointStatus::INVALID_BAUD;
            _boot_valid = false;
        }
    }
    if (!_boot_valid) {
        return;
    }

    for (uint8_t i = 0; i < capability.serial_endpoint_count; i++) {
        const SerialRole configured = SerialRole(_boot_roles[i]);
        if (configured == SerialRole::RCIN ||
            configured == SerialRole::VEKTOR) {
            _active_roles[i] = configured;
        }
    }
}

bool SerialRoleManager::index_valid(uint8_t serial_index) const
{
    return _capability != nullptr && _parameters != nullptr &&
           serial_index < _capability->serial_endpoint_count &&
           serial_index < SerialRoleParameters::max_endpoints;
}

SerialRole SerialRoleManager::configured_role(uint8_t serial_index) const
{
    if (!index_valid(serial_index)) {
        return SerialRole::NONE;
    }
    const int16_t value = _parameters->role[serial_index].get();
    return valid_role_value(value) ? SerialRole(value) : SerialRole::NONE;
}

SerialRole SerialRoleManager::boot_configured_role(uint8_t serial_index) const
{
    return index_valid(serial_index) && valid_role_value(_boot_roles[serial_index]) ?
        SerialRole(_boot_roles[serial_index]) : SerialRole::NONE;
}

SerialRole SerialRoleManager::active_role(uint8_t serial_index) const
{
    return index_valid(serial_index) ? _active_roles[serial_index] :
                                      SerialRole::NONE;
}

uint32_t SerialRoleManager::configured_baud(uint8_t serial_index) const
{
    if (!index_valid(serial_index)) {
        return 0;
    }
    const int32_t baud = _parameters->baud[serial_index].get();
    return baud > 0 ? uint32_t(baud) : 0;
}

uint32_t SerialRoleManager::active_baud(uint8_t serial_index) const
{
    return index_valid(serial_index) ? _boot_baud[serial_index] : 0;
}

bool SerialRoleManager::current_roles_valid(
    SerialEndpointStatus *statuses) const
{
    if (_capability == nullptr || _parameters == nullptr || statuses == nullptr) {
        return false;
    }
    int16_t roles[SerialRoleParameters::max_endpoints] {};
    for (uint8_t i = 0; i < SerialRoleParameters::max_endpoints; i++) {
        roles[i] = _parameters->role[i].get();
    }
    return validate_serial_roles(*_capability, roles, statuses);
}

SerialEndpointStatus SerialRoleManager::status(uint8_t serial_index) const
{
    if (!index_valid(serial_index)) {
        return SerialEndpointStatus::ENDPOINT_UNAVAILABLE;
    }
    if (_runtime_status[serial_index] != SerialEndpointStatus::OK) {
        return _runtime_status[serial_index];
    }
    SerialEndpointStatus statuses[SerialRoleParameters::max_endpoints] {};
    (void)current_roles_valid(statuses);
    if (statuses[serial_index] != SerialEndpointStatus::OK) {
        return statuses[serial_index];
    }
    if (!serial_endpoint_is_usb(*_capability, serial_index) &&
        !valid_baud(_parameters->baud[serial_index].get())) {
        return SerialEndpointStatus::INVALID_BAUD;
    }
    if (!_boot_valid && !reboot_required(serial_index)) {
        return _boot_status[serial_index] == SerialEndpointStatus::OK ?
            SerialEndpointStatus::CONFIGURATION_INVALID :
            _boot_status[serial_index];
    }
    return statuses[serial_index];
}

bool SerialRoleManager::reboot_required(uint8_t serial_index) const
{
    return index_valid(serial_index) &&
           (_parameters->role[serial_index].get() != _boot_roles[serial_index] ||
            configured_baud(serial_index) != _boot_baud[serial_index]);
}

UsbMode SerialRoleManager::configured_usb_mode() const
{
    if (_parameters == nullptr) {
        return UsbMode::VEKTOR_ONLY;
    }
    const int16_t value = _parameters->usb_mode.get();
    return valid_usb_mode_value(value) ? UsbMode(value) : UsbMode::VEKTOR_ONLY;
}

UsbMode SerialRoleManager::boot_configured_usb_mode() const
{
    return valid_usb_mode_value(_boot_usb_mode) ? UsbMode(_boot_usb_mode) :
                                                 UsbMode::VEKTOR_ONLY;
}

SerialEndpointStatus SerialRoleManager::usb_status() const
{
    if (_parameters == nullptr || !valid_usb_mode_value(
            _parameters->usb_mode.get())) {
        return SerialEndpointStatus::INVALID_ROLE;
    }
    return configured_usb_mode() == UsbMode::VEKTOR_MAVLINK ?
        SerialEndpointStatus::RESERVED_NOT_IMPLEMENTED :
        SerialEndpointStatus::OK;
}

bool SerialRoleManager::usb_reboot_required() const
{
    return _parameters != nullptr &&
           _parameters->usb_mode.get() != _boot_usb_mode;
}

void SerialRoleManager::set_configured_role(uint8_t serial_index,
                                            SerialRole role)
{
    if (index_valid(serial_index)) {
        save_parameter(_parameters->role[serial_index], int16_t(role));
    }
}

void SerialRoleManager::set_configured_baud(uint8_t serial_index,
                                            uint32_t baud)
{
    if (index_valid(serial_index)) {
        save_parameter(_parameters->baud[serial_index], int32_t(baud));
    }
}

void SerialRoleManager::set_configured_usb_mode(UsbMode mode)
{
    if (_parameters != nullptr) {
        save_parameter(_parameters->usb_mode, int16_t(mode));
    }
}

void SerialRoleManager::mark_endpoint_unavailable(uint8_t serial_index)
{
    if (index_valid(serial_index)) {
        _active_roles[serial_index] = SerialRole::NONE;
        _runtime_status[serial_index] =
            SerialEndpointStatus::ENDPOINT_UNAVAILABLE;
    }
}

} // namespace Vektor
