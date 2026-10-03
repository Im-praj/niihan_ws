# Deployment acceptance

The repository is a simulation development workspace. Passing unit tests or loading a world does not certify physical deployment. Complete and record the following evidence before unattended use.

| Gate | Evidence needed | Current status |
| --- | --- | --- |
| Hardware bringup | Physical drivers, calibrated URDF/extrinsics, real-time TF and odometry | Not verified; no audited hardware bringup |
| Independent stop | Physical E-stop removes motor power; safe restart procedure | Hardware untested |
| Motor watchdog | Loss of ROS/network/commands produces a bounded physical stop | Software watchdog present; physical response untested |
| Perception | Dust, glare, darkness, rain and obscured ground tests | Unverified |
| Cliff avoidance | Positive/negative obstacle tests at controlled speed and stopping distance | Software regressions only |
| Navigation | Mapped route, obstacle insertion, cancel, replanning and arrival tests | Unit contracts checked; live mission acceptance pending |
| Geofence | Boundary entry/exit and stale localization tests | Live acceptance pending |
| Dashboard security | Authenticated authorized access, TLS and network policy | Loopback default; no application authentication/TLS |
| Offline operation | Local JS assets and dependency availability | Three.js currently uses CDNs |
| Repeatability | Repeated bagged missions within defined tolerances | World seed fixed; output equivalence unverified |
| Release provenance | Owned license, maintainers, upstream revisions and asset rights | Placeholders/proprietary rights remain |

Start field testing on a controlled, level course with a physical stop operator. Record stop distances, command latency, localization error and mission completion criteria for the intended hardware, load and speed. Software stop topics must not substitute for independently functioning motor safety.

The full-system launch always starts simulation. Do not connect its simulator drive outputs to physical motors. A physical bringup needs an explicit separate launch and validated command routing.
