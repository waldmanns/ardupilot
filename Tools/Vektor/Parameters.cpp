#include "Vektor.h"

#include <StorageManager/StorageManager.h>

namespace Vektor {

const AP_Param::Info App::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Vektor parameter storage format
    // @Description: Internal Vektor storage format marker. Vektor resets parameters when the expected format changes.
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION", 0),

    // @Param: SYS_OPTIONS
    // @DisplayName: Vektor system options
    // @Description: Reserved Vektor system option bitmask for early platform feature switches.
    // @Bitmask: 0:Reserved
    // @User: Advanced
    GSCALAR(sys_options, "SYS_OPTIONS", 0),

    // @Param: SYS_DESC_PAGE
    // @DisplayName: Vektor descriptor page size
    // @Description: Default maximum descriptor records returned per DESCRIBE response when the host does not request a smaller page.
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    GSCALAR(sys_desc_page, "SYS_DESC_PAGE", default_describe_page_records),

    // @Param: SYS_PROTO_BAUD
    // @DisplayName: Vektor protocol UART baud
    // @Description: Baud rate used for the Vektor Serial Protocol stream on boot.
    // @Range: 9600 921600
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    GSCALAR(sys_protocol_baud, "SYS_PROTO_BAUD", protocol_baud),

    AP_VAREND
};

void App::load_parameters()
{
    AP_Param::setup_sketch_defaults();
    AP_Param::check_var_info();

    if (!g.format_version.load() ||
        g.format_version != k_format_version) {
        StorageManager::erase();
        AP_Param::erase_all();
        g.format_version.set_and_save(k_format_version);
    }

    g.format_version.set_default(k_format_version);
    AP_Param::load_all();
}

} // namespace Vektor
