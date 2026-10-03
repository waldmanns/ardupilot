#pragma once

// Included AFTER the generated ap_config.h by this application's wscript.
// SITL unconditionally defines OpenDroneID in that generated header, so a
// command-line -D=0 would be redefined before any source file is compiled.
// Vektor2 owns neither an OpenDroneID instance nor Torqeedo motor backends.
// Disable their GCS/RC hooks as well as omitting their libraries from the link.
#undef AP_OPENDRONEID_ENABLED
#define AP_OPENDRONEID_ENABLED 0
#undef HAL_TORQEEDO_ENABLED
#define HAL_TORQEEDO_ENABLED 0
