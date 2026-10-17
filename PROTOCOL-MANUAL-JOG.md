# Protocol v2: renewable manual jog

Current firmware: 2.4.48, normal Nano R4 adapter. Normal P2 framing, request sequence, CRC and replay protection apply. `GET_CAPABILITIES` also emits `MANUAL_JOG_CAPABILITIES|1`; unsupported adapters emit `0`.

| Request | Fields | Successful ACK |
| --- | --- | --- |
| `MANUAL_JOG_START` | nonzero uint32 gesture token, axis (0 Pan / 1 Tilt), direction (−1 / +1), tempo (1–1000 ‰) | `JOGGING` |
| `MANUAL_JOG_KEEPALIVE` | active gesture token | `JOG_ALIVE` |
| `MANUAL_JOG_STOP` | gesture token | `JOG_STOPPED` |

Start requires confirmed limits, valid fresh encoders, resting feedback and an admissible idle/paused state. It never queues. The other axis is disabled. The velocity reference ramps towards the configured inner limit and is capped by `manual.jogMaximumTempoPermille`.

The MCU measures the lease from the last matching start/keepalive, using unsigned subtraction across timer wrap. A late renewal cannot revive an expired gesture. Ordinary release/expiry disables both motors and completes `JOG/CANCELLED` while preserving the surrounding job state. Inner-limit completion emits `JOG/REACHED`; a real sensor/motion fault remains a safety fault and emits `JOG/FAULT`.

`MANUAL_STATUS|startRequestSequence|JOG|result|startPanMdeg|startTiltMdeg|elapsedMs|leaseMs` identifies the original start. Result is `ACTIVE`, `CANCELLED`, `REACHED` or `FAULT`. Starting coordinates are not destinations. Active duration may exceed leaseMs because renewals extend permission. Motors off does not certify absence of mechanical coast.

Stop is idempotent and may arrive before start acknowledgement. A stopped token cannot start later; a late old stop cannot cancel a newer gesture. Android cancels pending wire retries before the priority stop and discards old completions. Lost stop delivery is bounded by the lease.

During jog, status/diagnostic reads, matching renewals/releases, STOP and ESTOP remain available. Other movements, camera commands and job/configuration mutations are refused. Sensor/range/no-progress protections remain active. Configurable parameters and cross-checks are in `MotionParameters.h`/`MotionSettings.cpp`; persistence migrates complete 85–97-value prefixes without resetting prior settings.

Coverage: `testHeldJogProtocol()`, `runMotionTests()`, Android `JogHoldTest`, `V2ClientTest`, `ManualServiceChecks`, `JogUiChecks` and `ManualHardwareChecks`.
