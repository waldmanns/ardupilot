# Vektor2 M3.5 — EKF3 and assignable DP telemetry handover

**Base:** Vektor2 M3 observation-only archive. **Safety:** no automatic output
routing, ownership, RC actions or physical vessel DP activation.

## Changes

- `Vektor2.cpp` uses the **same** AHRS/EKF3 estimated position and velocity
  for DP target capture, subsequent actual position and velocity corrections.
  Position acceptance now also checks the E7 geographic range.
- Adds AHRS reset time monitoring for yaw, horizontal **position** and
  horizontal **velocity**. Any new reset clears targets and requires a new
  capture. The first timestamp sample establishes a baseline. No custom
  estimator, jump detector or antenna/attitude correction was introduced.
- Shared heading output exposes EKF actual heading independently of whether
  the automatic heading controller has authority. Output `valid` and demand
  remain gated; no heading command becomes active in Free mode.
- Preserves VTEL sources 240..256 and adds **257 `HDG_ERR`**, **258
  `DP_CMD_N`**, **259 `DP_CMD_E`**. All are selectable through existing
  `VTEL_SLTn` parameters and optional i-BUS/HoTT receiver telemetry.
- The same three fields join the MAVLink DP diagnostic stream, controlled by
  `DP_TEL_HZ` (0..50 Hz, default off). Existing `NAV_STATE`/`NAV_RBITS`
  timing, masks and source IDs 140/141 remain unchanged.

## Reference point and attitude

The DP point is the ArduPilot AHRS/EKF3 body-origin position/velocity estimate,
not raw GPS antenna coordinates. EKF3 handles sensor offsets as configured.
No custom roll/pitch horizontal compensation is added. This assumes the
intended hull DP control point coincides with the EKF body origin; verify this
and the configured GPS/IMU offsets physically. The geometric hull center and
AHRS body origin are **not automatically the same point**. Do not claim
vessel-center precision without that commissioning check.

`DP_CMD_N/E` are earth-frame normalized requests and `DP_CMD_X/Y` are the
corresponding body-frame surge/sway requests. They do **not** represent
measured physical thrust. Invalid output remains unavailable rather than
being published as a plausible normal zero.

## Rate interpretation

`DP_TEL_HZ` controls MAVLink named-float DP diagnostic reports only.
VTEL already operates a **10 Hz snapshot** and is polled by receivers, so
its sources remain assignable even if `DP_TEL_HZ=0`. High MAVLink report
rates may be limited by bandwidth (20 numeric fields per snapshot at M3.5).

## Verification scope

- Host tests for DP math, heading diagnostics, reset timestamp tracking and
  the VTEL source catalog are included.
- Neither full ArduPilot firmware compilation nor SITL/real vessel validation
  is possible from the application-only upload. AHRS reset API compatibility
  needs confirmation against the exact full ArduPilot tree used for the build.
- Verify EKF body-origin point, antenna offset settings, yaw observability
  while stationary, and GPS/heading reset cases before hardware operation.

M4 remains: assignable RC actions, ownership and neutral handovers, propulsion
routing and real failsafe behavior. All remain disabled/unimplemented here.
