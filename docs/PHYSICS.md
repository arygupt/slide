# Physics model

Everything is SI (m, s, kg, N, rad) and lives in `include/slide/` + `src/physics/`.
The core has no dependencies beyond the C++20 standard library.

## Frames and state

- **World**: x east, y north, z up. Heading ψ is counter-clockwise from +x.
- **Body**: x forward, y left, z up (ISO 8855). Positive pitch is nose-down,
  positive roll is left-side-up.

| Subsystem | Degrees of freedom |
|---|---|
| Chassis (planar) | x, y, ψ and their rates |
| Sprung body | heave z, pitch θ, roll φ and their rates |
| Wheels | 4 spin rates ω, 4 × 2 contact-patch deflections |
| Engine | crank speed, manifold pressure, throttle plate angle |
| Brakes | 4 line pressures (first-order hydraulics) |

## Time integration

`Vehicle::step(input, dt)` advances one fixed step (1 kHz by default) in this order:

1. Steering (rate-limited input, speed-sensitive lock, Ackermann split).
2. Contact-patch kinematics: `v_contact = v_body + ω_z × r_i`, rotated into each wheel frame.
3. Terrain sample under each tire (height, grip, rolling resistance).
4. Suspension → tire normal loads.
5. Tire forces from the relaxed slip state.
6. Driver aids (ABS / TCS / ESC) read wheel speeds and body motion.
7. Engine air path and torque; clutch and differential torques.
8. Wheel spin update, then Coulomb brakes, then tire relaxation using the **new** ω
   (semi-implicit in the stiff wheel-tire loop).
9. Chassis forces + aero → semi-implicit Euler for the planar body.
10. Sprung body integration.

Stiff terms are handled implicitly where it matters: tire relaxation decay, the
manifold filling equation and the clutch are all solved as implicit or
velocity-level updates. That keeps one integrator stable from a standstill to
250 km/h, and results converge as the step shrinks (tested: 1 kHz vs 2 kHz agree
within 3%). The step is deterministic: the same inputs give bit-identical states.

## Tires

**Pure slip.** A sine-wave grip curve, with the peak tied to load. With
`s = |x| / x_peak`:

```
F = D · sin(π/2 · s)                                      s ≤ 1   (rising to the peak)
F = D · (1 − (1 − k_slide) · sin(π/2 · (s − 1)/(s_slide − 1)))   s > 1   (easing down to sliding grip)
D = μ(Fz) · Fz,   μ(Fz) = μ0 · (1 − k_load · (Fz/Fz0 − 1))     (load sensitivity)
```

`x` is slip ratio κ for Fx and slip angle α for Fy. Force peaks at κ = 0.13 and
α = 8.3° (6.6° on the rear), then falls to 78–80% of peak once the tire is fully
sliding (a locked wheel, or 40° of slip angle), which is why ABS shortens stops.
Front and rear axles carry different tires (staggered 245/275, with a sharper
rear), which gives about 1.5°/g of understeer.

**Combined slip** uses the similarity method. Each slip is normalized by its
peak (`s_x = κ/κ_peak`, `s_y = α/α_peak`). Both curves are evaluated at the
combined magnitude `s = |(s_x, s_y)|`, and the result is shared by direction:

```
Fx = D · (s_x/s) · MFx(s·κ_peak)     Fy = D · (s_y/s) · MFy(s·α_peak)
```

The force traces a friction ellipse, and braking while cornering costs grip on
both axes. It isn't clipped to a circle.

**Transient slip.** Slip is not computed as `(ωR − vx)/|vx|`, which is singular
at a standstill. Each tire instead carries contact-patch deflections with
relaxation lengths σ:

```
σx · dκ'/dt + |vx| · κ' = ωR − vx        σy · dα'/dt + |vx| · α' = −vy
```

These are written in deflection form `u = σ·κ'`, and the decay term is
integrated implicitly. Below 3 m/s a fading contact damper is added, so a parked
car settles instead of ringing on tire stiffness.

## Suspension and load transfer

Each corner is a spring (with a progressive bump stop) and an asymmetric
bump/rebound damper between the sprung body and the road. Anti-roll bars couple
left and right. Corner displacement is `z_i = z − x_i·θ + y_i·φ`, and
compression is `δ_i = h_road,i − z_i`.

```
F_strut,i = max(0, F_static,i + k·δ + c(δ̇)·δ̇ + k_bs·max(0, δ−δ_bump)² ± k_arb·(δ_L − δ_R))
Fz_i      = F_strut,i + m_unsprung·g ∓ Fy_axle · h_rc / track     (roll-center geometric transfer)
```

Body equations, with tire forces acting at the ground (`h` = CG height):

```
m_s · z̈  = Σ(F_strut − F_static) − F_downforce
I_yy · θ̈ = −Σ x_i·F_strut,i − h·ΣFx + aero pitch moment
I_xx · φ̈ = Σ y_i·F_strut,i + Σ (h − h_rc)·Fy + m_s·g·(h − h_rc)·φ
```

Load transfer is dynamic: the body has to pitch and roll, through the dampers,
before the tires see it. In steady braking the front gains `m·a·h/L`, matching
the static result (unit-tested within 15%). Road height comes from the terrain:
rumble-strip curbs and sinusoidal grass bumps excite the suspension.

## Engine (mean-value SI model)

No torque lookup table is used. Torque comes from the air path and an energy
balance:

- **Throttle orifice.** Compressible isentropic flow with choking:
  `ṁ = Cd·A(θ)·p0/√(R·T0)·Ψ(p_man/p0)`, with `A(θ) = (πD²/4)(1 − cos θ / cos θ0)`.
  Ψ is linearized above a pressure ratio of 0.98 so its slope stays finite.
  The same equations size rocket injectors and feed-system orifices.
- **Manifold plenum.** `dp/dt = (R·T/V)·(ṁ_throttle − ṁ_cylinders)`, stiff near
  wide-open throttle, so it takes a linearized implicit Euler step:
  `p += dt·f / (1 − dt·∂f/∂p)`.
- **Cylinders.** `ṁ_cyl = VE(N)·ρ_man·V_d·ω/(4π)` (four-stroke).
- **Indicated torque.** `T_i = η(MAP)·LHV·VE·ρ_man·V_d / (4π·AFR)`. Efficiency
  falls at part load, which models dilution and heat loss.
- **Losses.** Friction mean effective pressure is `a + b·N + c·N²`.
  Pumping is `(p_exh − p_man)·V_d/(4π)`.
- **ECU.** Drive-by-wire throttle lag, a PI idle-speed loop on the idle air
  valve, deceleration fuel cut with a dashpot, a rev limiter with hysteresis,
  WOT enrichment (affects fuel flow and BSFC), stall detection and a starter
  motor.

`slide-bench dyno` holds crank speed like a speed-controlled absorber and
reports torque, power, MAP, airflow, VE and BSFC across the rev range.

## Drivetrain

The clutch is solved as a **velocity constraint**, the same way rigid-body
engines treat friction contacts. Each step it computes the torque `T_lock` that
would make engine speed equal gearbox input speed at the end of the step:

```
ω_e + dt·(T_e − T_c)/I_e = G·(ω̄_w + dt·(η·G·T_c + τ_L + τ_R)/(2·I_w))
```

That torque is clamped to the friction capacity `T_max·engagement²`. If
`|T_lock|` fits, the clutch is locked. Otherwise it slips at capacity. This one
rule covers launches, lock-up, gear changes and stalls, with no stiff springs or
mode switching. Efficiency η is applied in the direction power flows (it divides
on overrun), so engine braking cannot create energy.

Around it:

- **Automatic clutch.** Engagement follows engine speed during launch, then
  latches closed once slip is gone.
- **Shift sequencing.** Clutch open, ratio swap, re-engage. Upshifts cut
  ignition until the clutch has synchronized the engine. Downshifts blip the
  throttle to rev-match.
- **Shift logic.** A throttle-dependent shift schedule with skip-downshifts,
  over-rev protection, and reverse only near a standstill.
- **LSD.** Clutch-pack limited-slip differential: viscous bias capped at
  preload plus a power-proportional lock.

Brakes are Coulomb friction: they can stop a wheel but never reverse it, applied
as a clamp after the free spin update.

## Driver aids

- **ABS.** A per-wheel PI controller on brake-pressure modulation targeting 10%
  slip, rate-limited like valve hardware. The proportional term damps the limit
  cycle that the 35 ms brake hydraulics would otherwise cause. Result: 100–0
  km/h in about 36 m, versus 44 m with locked wheels.
- **TCS.** PI controller on worst driven-wheel slip acting on engine torque. It
  switches to slip speed near a standstill, where slip ratio is undefined.
- **ESC.** Yaw rate is compared against a steady-state bicycle model
  `r_ref = vx·δ / (L + K_us·vx²)`, clamped to the friction limit. Oversteer brakes
  the outside front wheel and trims torque. Understeer brakes the inside rear.
  Sideslip beyond 10° counts as oversteer.

## Autopilot and lap simulation

The speed profile comes from curvature, `v ≤ √(μ·g/|κ|)`, followed by backward
(braking) and forward (traction) passes. This is the standard quasi-steady-state
lap-time construction. Steering is pure pursuit from the rear axle. It drives
the demo, the `laps` bench scenario, and a regression test that requires a full
lap without leaving the track.

## Known simplifications

- Flat-plane chassis: no camber, no bump steer, no unsprung vertical mass
  (the tire is rigid vertically), no anti-dive or anti-squat geometry.
- Rear-wheel drive only; a single tire/road friction model for all surfaces,
  scaled by surface grip.
- Engine torque is air-limited at stoichiometric; enrichment only changes fuel
  flow. No turbocharger, knock or thermal model.
- Aero is a constant CdA/ClA with a fixed balance.
