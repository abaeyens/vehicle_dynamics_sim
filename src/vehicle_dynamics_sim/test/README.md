# Vehicle Dynamics Simulator Test Plan

## Overview

This document describes the test suite for the vehicle dynamics simulator.

## Unit tests (`test_all.cpp`)
**Purpose**: Unit tests for core math and utility functions.

**Test Files**:
- `test_Pose2D.cpp` - 2D pose transformations and conversions
- `test_utils.cpp` - Angle wrapping utilities (`mod_pi`)
- `test_vehicles.cpp` - Actuator dynamics (dead time delay, first-order filters), URDF generation

**Run time**: <1 second

## Integration tests
### Smoke testing
**Purpose**: Verify basic simulator functionality with default parameters.

**Tests**:
- ✅ **Output topics published**:
  Verifies all expected topics (`/tf`, `twist_actual` etc.) are published
- ✅ **Input topics advertised**: Verifies simulator subscribes to `cmd_vel_nav`
- ✅ **TF tree structure**: Verifies `map → odom → base_link` chain exists

**Configuration**: Default parameters from launch file

**Run time**: ~10 seconds

---

### Vehicle Velocity Tracking Verification
**Purpose**:
Verify steady state velocity tracking for all vehicle models and configurations.

**Test Implementation**: These tests use shared infrastructure in `test_vehicles.py` which is imported by 6 separate test files:
- `test_vehicles_bicycle_front_steer_rear_drive.py`
- `test_vehicles_bicycle_front_steer_front_drive.py`
- `test_vehicles_bicycle_rear_steer_rear_drive.py`
- `test_vehicles_bicycle_rear_steer_front_drive.py`
- `test_vehicles_differential.py`
- `test_vehicles_omni.py`

**Test Cases Per Vehicle**
| Test | Bicycle<br>(fixed axle drive) | Bicycle</br>(steered axle drive) | Differential | Omni |
|------|-------------------------------|----------------------------------|--------------|------|
| Forward motion | ✅ | ✅ | ✅ | ✅ |
| Curve motion | ✅ | ✅ | ✅ | ✅ |
| In-place rotation | ❌ | ✅ | ✅ | ✅ |
| Lateral motion | ❌ | ❌ | ❌ | ✅ |
| Combined 3-DOF | ❌ | ❌ | ❌ | ✅ |

In addition: for the bicycle vehicle, we also test the reverse configuration
(= steered axle behind fixed axle).

**Test Methodology**
1. Publish constant velocity command at 10 Hz
2. Wait a few seconds for actuator dynamics to settle
3. Collect `twist_actual` messages for a few seconds
4. Compute average velocities
5. Verify within tolerance of commanded values

**Notes**
- Tests use **wall-clock time** (not simulation time)
- **Localization disabled** in velocity tests, as localization simulation block is not tested
- **Fast actuators** configured for faster than usual test execution (higher limits than defaults)
- Tests focus on **steady-state** behavior (transient dynamics are tested as part of the unit tests)
- Bicycle tests cover **all 4 configuration variants** (front/rear steer × front/rear drive)
- Each vehicle configuration runs in a **separate test file**
  such that xUnit files (which contain the test results)
  are available for each configuration.

**Run time**: ≈20 seconds per vehicle config = a few minutes total
