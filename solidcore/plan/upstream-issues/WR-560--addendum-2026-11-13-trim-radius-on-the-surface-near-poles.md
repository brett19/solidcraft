# WR-560: a trim's radius is measured on its surface near a pole too

Updates: WR-560

WR-560 maps a face's UV trim by the Cholesky factor of its surface's first fundamental
form at the trim's middle before test 13. That constant map is exact on a plane only.
Near a singular point of the surface it measures the trim at the wrong scale: at a
sphere's pole the u direction collapses, and the exact trim of a smooth curve passing a
distance `d` from the pole turns through up to half a period in u over a few times `d`.
Mapped by the metric at the trim's middle, that turn has a radius of `1e-9` to `1e-5`
against a tolerance of `1e-4`. Its image on the ball has a radius of `2e-4` to `1e-2`.

Trigger: a half roll (r 48..50 about z) less a r 6 ball whose poles lie about the zone
tolerance off the roll's outer face. Every frame was refused as not a solid, and test 13
was the only report in some of them (poles `1.2e-5` inside, frames 2 and 5).

Change (not upstream, no flag): a test-13 solution below the tolerance on a mapped trim
is measured again on the surface, as the radius of curvature of `S(c(t))` from both
sides of `t`, and passed over when that radius is at least the tolerance. Where the
surface is regular, a small radius in (u, v) is a small radius on the surface, so no
real kink is passed over. Where the radius cannot be measured (an evaluation fails, a
singular point, zero speed), the solution counts as before. Curves that are no face's
trim, 3D curves (test 14) and unmapped trims are unchanged.

Code: `radius_on_surface` and the pass-over in `BSplineCurve::assert_valid_with`
(`crates/solidcore/src/nurbs/b_spline_curve/p3.rs`).
