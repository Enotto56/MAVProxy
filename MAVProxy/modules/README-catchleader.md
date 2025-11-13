# CatchLeader module

The CatchLeader module provides predictive intercept guidance that steers a
follower aircraft towards a moving leader. It monitors telemetry for both
vehicles, sends `SET_POSITION_TARGET_GLOBAL_INT` guidance targets, and can
optionally drive the follower into FBWA for manual intervention.

## Configuring speed selection

Use the `catchleader set` command to inspect or change module settings. In
addition to the existing `follower_speed` value, the module now supports speed
profiles. They can be adjusted either via the settings command or with the
dedicated shortcut:

```
catchleader set speed_profile <cruise|max|custom>
catchleader speed <cruise|max|custom> [value]
```

* **custom** – default behaviour that relies on the configured
  `follower_speed` value, exactly matching previous releases.
* **cruise** – tries to read `AIRSPEED_CRUISE`, `AIRSPEED_TRIM`, or
  `TRIM_ARSPD_CM` from the follower. When the parameters are unavailable the
  module falls back to `follower_speed` and records the reason in the log.
* **max** – uses `AIRSPEED_MAX` (or `ARSPD_FBW_MAX`) and enables a velocity
  override so that the position target includes North/East/Down velocity
  components aimed directly at the intercept point. If required telemetry is
  missing the module reverts to position-only guidance and reports this in the
  log.

Whenever a profile resolves to a concrete speed the module issues
`MAV_CMD_DO_CHANGE_SPEED` to the follower before pushing a new position target.
This keeps the autopilot airspeed controller aligned with the requested
profile.

## Backward compatibility

Existing configurations continue to function without modification—the
`speed_profile` setting defaults to `custom`, preserving the original behaviour
where only `follower_speed` is used. Logs and CLI status reports now include the
active profile and the resolved speed so that operators can confirm the chosen
behaviour at a glance.
