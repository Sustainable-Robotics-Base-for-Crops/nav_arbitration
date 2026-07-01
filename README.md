# nav_arbitration

High-level supervisor of the autonomous navigation stack. It runs a [YASMIN](https://github.com/uleroboticsgroup/yasmin) state machine that drives the **lifecycle states** of the managed navigation nodes (action servers including `cylinder_go_end`, replay, geofencing, recorder, `json_agri_format_parser`) according to operator commands (`conductor_cmd`) and to the vehicle / replay status. It also republishes the resulting drive mode on `conductor_state`.

**Node:** `arbitration` · **Executable:** `nav_arbitration`

## Overview

The node owns a YASMIN `Blackboard` shared by every state and a `StateMachine` started in a dedicated thread (`start_sm()`):

1. Build the blackboard: a [`nav_lifecycle_manager::LifecycleManager`](../nav_lifecycle_manager/README.md) managing the action-server group (including `cylinder_go_end`), plus individual `LifecycleServiceClient` instances for replay, geofencing, recorder and `json_agri_format_parser`.
2. Run the state machine, which on each control cycle (10 Hz) refreshes the blackboard (`conductor_cmd`, `replay_status`, `vehicle_status`) and evaluates whether a transition is allowed.
3. On every transition, drive the managed nodes to the required lifecycle primary state (configure / activate / deactivate / cleanup).
4. In parallel, a timer publishes the current drive mode on `/auto/conductor_state` so that the rest of the system knows the active mode.

Managed nodes are transitioned as a group through the lifecycle manager; replay, geofencing, `json_agri_format_parser` and recorder are transitioned individually through their service clients.

## State machine

| State             | Lifecycle meaning            | Description                                                               |
| ----------------- | ---------------------------- | ------------------------------------------------------------------------- |
| `unconfigured`    | `PRIMARY_STATE_UNCONFIGURED` | Idle / teleop. Action servers kept inactive, recorder active.             |
| `replay_inactive` | `PRIMARY_STATE_INACTIVE`     | Replay configured but paused; ready to start.                             |
| `replay_active`   | `PRIMARY_STATE_ACTIVE`       | Replay running; action servers active and following the recorded mission. |

### Transitions

| From              | Outcome            | To                | Trigger (summary)                                                        |
| ----------------- | ------------------ | ----------------- | ------------------------------------------------------------------------ |
| `unconfigured`    | `replay_configure` | `replay_inactive` | `drive_mode == DRIVE_REPLAY`, no `restart`, vehicle status OK            |
| `unconfigured`    | `cleanup`          | `unconfigured`    | Stays unconfigured (default)                                             |
| `replay_inactive` | `activate`         | `replay_active`   | Not paused / not restart, replay & vehicle status OK                     |
| `replay_inactive` | `cleanup`          | `unconfigured`    | `restart`, mode change, or vehicle error past the emergency-stop timeout |
| `replay_active`   | `deactivate`       | `replay_inactive` | `pause`, end of path, remote e-stop or bumper warning                    |
| `replay_active`   | `cleanup`          | `unconfigured`    | `restart`, mode change or vehicle error                                  |
| `unconfigured`    | `shutdown`         | *(terminal)*      | `rclcpp` shutdown while in `unconfigured`                                |

If a group transition fails, the affected nodes are rolled back to their previous state and the transition is not taken.

## Managed nodes

**Action-server group** (via `nav_lifecycle_manager::LifecycleManager`, transitioned in list order):

| Node                                        | Role                   |
| ------------------------------------------- | ---------------------- |
| `/auto/line/matcher`                        | Line matcher server    |
| `/auto/line/follower`                       | Line follower          |
| `/auto/turn/on_spot`                        | Turn-on-spot server    |
| `/auto/path/matcher`                        | Path matcher server    |
| `/auto/path/follower`                       | Path follower          |
| `/auto/working_zone_action/cylinder_go_end` | Cylinder GO/END action |

**Individually managed** (via `LifecycleServiceClient`):

| Node                                      | Blackboard client                |
| ----------------------------------------- | -------------------------------- |
| `/auto/replay`                            | `client_replay`                  |
| `/auto/recorder`                          | `client_recorder`                |
| `/safety/geofencing/geofencing_publisher` | `client_geofencing`              |
| `/auto/json_agri_format_parser`           | `client_json_agri_format_parser` |

## Parameters

| Parameter                                          | Default           | Description                                                      |
| -------------------------------------------------- | ----------------- | ---------------------------------------------------------------- |
| `rate`                                             | `0.3`             | Period (s) of the `conductor_state` publishing timer             |
| `timeout_end_emergency_stop`                       | `8.0`             | Delay (s) after an emergency stop ends before replay may resume  |
| `lateral_deviation_max`                            | `0.4`             | Max lateral deviation (m) — served to the navigation nodes       |
| `lateral_deviation_max.in_working_zone`            | `0.4`             | Max lateral deviation inside the working zone (m)                |
| `lateral_deviation_max.out_working_zone`           | `0.6`             | Max lateral deviation outside the working zone (m)               |
| `lateral_deviation_max.uturn`                      | `1.5`             | Max lateral deviation during a U-turn (m)                        |
| `cut_line_overshoot`                               | `0.05`            | Cut-line crossing overshoot (m) — served to the navigation nodes |
| `course_deviation_max`                             | `π/8`             | Max course deviation (rad) — served to the navigation nodes      |
| `speed_working_zone_added`                         | `0.0`             | Extra speed applied inside the working zone (m/s)                |
| `working_zone_action.name`                         | `cylinder_go_end` | Name of the working-zone action server                           |
| `working_zone_action.offset_distance_at_the_start` | `0.0`             | Offset distance at the start of the working zone (m)             |
| `working_zone_action.offset_distance_at_the_end`   | `0.0`             | Offset distance at the end of the working zone (m)               |
| `loop_back.return_speed`                           | `0.0`             | Return speed on loop-back segments (m/s)                         |

The navigation and mission parameters are declared here so that downstream nodes (e.g. [`nav_line_matcher`](../nav_line_matcher/README.md)) can read them remotely from `/auto/arbitration`.

## Topics

| Topic                   | Type                           | Direction        | Description                                    |
| ----------------------- | ------------------------------ | ---------------- | ---------------------------------------------- |
| `/auto/conductor_cmd`   | `nav_interfaces/msg/Conductor` | In (best effort) | Operator command (drive mode, pause, restart)  |
| `replay/status`         | `std_msgs/msg/UInt64`          | In               | Replay status bitmask                          |
| `/vehicle/status`       | `std_msgs/msg/UInt64`          | In               | Vehicle status bitmask (warnings / errors)     |
| `/auto/conductor_state` | `nav_interfaces/msg/Conductor` | Out              | Current drive mode reflecting the active state |

### `Conductor` message

| Field        | Values                                 | Description                    |
| ------------ | -------------------------------------- | ------------------------------ |
| `drive_mode` | `DRIVE_TELEOP` (0), `DRIVE_REPLAY` (1) | Requested / current drive mode |
| `pause`      | `bool`                                 | Pause the current mission      |
| `restart`    | `bool`                                 | Request a full reset to teleop |

### Vehicle status bitmask

`/vehicle/status` is a 64-bit field split by severity (`VehicleStatus`):

| Range           | Examples                                                                                       |
| --------------- | ---------------------------------------------------------------------------------------------- |
| Info `[0-15]`   | `normal_operation`                                                                             |
| Warn `[16-31]`  | `warn_emergency_stop`, `warn_emergency_stop_remote`, `warn_bumper`                             |
| Error `[32-63]` | `error_udp_driver`, `error_battery`, `error_cylinder`, `error_vcu`, `error_motor`, `error_imu` |

Any **error** bit (`>= 1 << 32`) blocks activation and forces a return to `unconfigured`; remote e-stop and bumper warnings deactivate replay until cleared and the `timeout_end_emergency_stop` has elapsed.
