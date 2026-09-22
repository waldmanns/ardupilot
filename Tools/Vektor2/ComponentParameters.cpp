#include "ComponentParameters.h"

namespace Vektor2 {

const AP_Param::GroupInfo VSP1ParameterStore::var_info[] = {
    // @Param: LIM
    // @DisplayName: VSP1 limit
    // @Description: Custom VSP1 limit value. Interpretation belongs to VSP1 logic.
    // @User: Standard
    AP_GROUPINFO("LIM", 1, VSP1ParameterStore, lim, 25),

    // @Param: X_C
    // @DisplayName: VSP1 X_C
    // @Description: Custom VSP1 X_C value. Interpretation belongs to VSP1 logic.
    // @User: Standard
    AP_GROUPINFO("X_C", 2, VSP1ParameterStore, x_c, 1500),

    // @Param: Y_C
    // @DisplayName: VSP1 Y_C
    // @Description: Custom VSP1 Y_C value. Interpretation belongs to VSP1 logic.
    // @User: Standard
    AP_GROUPINFO("Y_C", 3, VSP1ParameterStore, y_c, 1500),

    AP_GROUPEND
};

const AP_Param::GroupInfo VSP2ParameterStore::var_info[] = {
    // @Param: LIM
    // @DisplayName: VSP2 limit
    // @Description: Custom VSP2 limit value. Interpretation belongs to VSP2 logic.
    // @User: Standard
    AP_GROUPINFO("LIM", 1, VSP2ParameterStore, lim, 25),

    // @Param: X_C
    // @DisplayName: VSP2 X_C
    // @Description: Custom VSP2 X_C value. Interpretation belongs to VSP2 logic.
    // @User: Standard
    AP_GROUPINFO("X_C", 2, VSP2ParameterStore, x_c, 1500),

    // @Param: Y_C
    // @DisplayName: VSP2 Y_C
    // @Description: Custom VSP2 Y_C value. Interpretation belongs to VSP2 logic.
    // @User: Standard
    AP_GROUPINFO("Y_C", 3, VSP2ParameterStore, y_c, 1500),

    AP_GROUPEND
};

} // namespace Vektor2
