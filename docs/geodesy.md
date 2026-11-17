# GNSS noise semantics and geodesy

FusionCore uses two related but distinct ideas when turning GNSS measurements into filter noise:

1. the receiver's reported **DOP or covariance**, which describes the quality of the current fix; and
2. FusionCore's **baseline noise parameters**, which scale that information when the message format does not provide the same kind of uncertainty directly.

The important detail is that `gnss.base_noise_xy` does **not** have one physical unit for every input path.

## How the GNSS position noise is derived

### DOP path

When the incoming fix provides receiver-native HDOP/VDOP rather than a usable position covariance, FusionCore uses the baseline parameters as metres per unit DOP.

For horizontal position:

`sigma_xy = gnss.base_noise_xy * HDOP`

For vertical position:

`sigma_z = gnss.base_noise_z * VDOP`

On this path:

- `gnss.base_noise_xy` is in **metres per unit HDOP**.
- `gnss.base_noise_z` is in **metres per unit VDOP**.
- An HDOP of 2.0 with `base_noise_xy: 1.0` therefore produces a 2.0 m horizontal sigma before the rest of the measurement model is applied.

This is the interpretation most users expect from the parameter name and the default configuration.

### Covariance path

A `sensor_msgs/NavSatFix` can carry a position covariance. When FusionCore receives a usable covariance, the receiver has already supplied an uncertainty estimate in metres.

In that case, `gnss.base_noise_xy` acts as a **dimensionless multiplier** on the sigma derived from the receiver covariance rather than as metres-per-DOP.

That means a configuration such as:

`gnss.base_noise_xy: 6.0`

does not mean “6 metres” on this path. It means “multiply the receiver-derived horizontal sigma by 6”.

This distinction matters when moving between receivers or drivers. A value that is sensible for a DOP-based input can be surprisingly large when the same configuration is applied to a driver that supplies covariance.

## How to tell which path you are on

Check the message your driver publishes on `gnss.fix_topic`.

For `sensor_msgs/NavSatFix`:

- the message contains `position_covariance` and `position_covariance_type`;
- a usable covariance can drive the covariance path;
- when no usable covariance is available, FusionCore can fall back to receiver-native DOP information where supported.

For `gps_msgs/GPSFix`:

- receiver-native `hdop`/`vdop` are available directly;
- `err_horz`/`err_vert` and covariance fields may also provide uncertainty information;
- FusionCore chooses the best available source according to the documented covariance priority.

The authoritative parameter and message-format details are in the [Configuration reference](configuration.md).

## Practical tuning rule

Do not copy a `gnss.base_noise_xy` value from another robot without checking its message path.

A useful diagnostic sequence is:

1. inspect `position_covariance_type` and the covariance values, if using `NavSatFix`;
2. inspect HDOP/VDOP when the driver exposes them;
3. determine which uncertainty source FusionCore is actually using;
4. compare the resulting sigma with the receiver's observed behaviour;
5. only then change `gnss.base_noise_xy` or `gnss.base_noise_z`.

If the receiver reports a covariance, first decide whether the receiver's covariance is calibrated for your operating environment. Increasing the multiplier can be appropriate when the reported covariance is systematically optimistic, but it should be an evidence-based calibration choice rather than a unit conversion.

## Geodetic coordinate path

FusionCore's internal GNSS position handling uses a global Cartesian representation and an ENU frame anchored at the configured reference point. The normal conceptual path is:

`latitude / longitude / altitude → ECEF → local ENU`

This avoids making the filter state depend on a particular UTM zone.

UTM-style coordinate output is still possible when a downstream system needs it. The `input.gnss_crs` and `output.crs` parameters control CRS handling; choosing a UTM CRS for output does not change the filter's internal geodetic model.

## Why this distinction is important

There are two different questions:

- **Where is the robot?** The geodetic conversion determines how latitude/longitude/altitude become metric coordinates for the filter.
- **How certain is that position?** The covariance/DOP path determines the measurement uncertainty used by the filter.

Changing a CRS does not make a GNSS measurement more or less accurate. Likewise, changing `gnss.base_noise_xy` does not change the coordinate conversion.

Keeping those two concerns separate makes GNSS tuning easier to reason about and prevents a unit or covariance interpretation error from being mistaken for a geodesy problem.
