# Energy-based swing-up

Serial mode `1` now runs an energy-shaping swing-up controller. It hands off
to the existing upright controller only when angle, pendulum rate, cart speed,
and rail position are all inside the capture region.

## Mass model

The pole is modelled as a uniform rod plus a small attachment near its end.
After weighing the parts, edit these constants in `src/Pendulum.cpp`:

```cpp
TIP_TO_ROD_MASS_RATIO = attachment_mass / bare_rod_mass;
TIP_DISTANCE_RATIO = pivot_to_attachment_center / rod_length;
```

The starting values are `0.10` and `0.95`. Set the mass ratio to zero for a
pure uniform rod.

## Safe bring-up

1. Send rail limit, speed/acceleration limits, upright position, and the
   already-proven balance gains from the panel before starting mode 1.
2. Temporarily set `SWING_ACCEL_MAX_MPS2` to `0.5`.
3. Confirm that positive acceleration increases measured cart position. If
   not, set `SWING_DIRECTION_SIGN` to `-1`.
4. If successive swings shrink, also flip that sign.
5. Raise acceleration gradually: 0.5, 1.0, 1.5, then 2.0 m/s^2.
6. Use physical end stops and an accessible motor-power emergency stop.
   Encoder-derived software position is not a safety device.

The default capture boundary is 12 degrees, 1.5 rad/s pendulum rate, and
0.30 m/s cart speed. Release is at 25 degrees to provide hysteresis.
