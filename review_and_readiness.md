# Critical Review and Publication-Readiness Assessment

## Overall decision

**Recommendation: major revision before submission.** The package is a credible starting point for a research study, a traceable audit of the source calculations, and a reproducible reduced-order simulation. It is not yet publication-ready because its proposed model cannot presently be identified from the supplied design data, the validation is computational only, and the novelty claim is still provisional.

## Strongest defensible current contribution

The most specific technical result is the sign-aware analysis of the exact proposed planetary member pairings, joined to a parameterized model that checks whether commanded swerve motion is compatible with motor torque, tire friction and chassis yaw dynamics. The second drive stage is especially important: with the carrier fixed and the sun as input, the ring output speed is opposite in sign to the sun speed. The combined signed wheel-to-motor speed ratio is therefore `-1/12`, not simply a positive `12:1` scalar. This is a configuration-specific modelling/feasibility result, but its novelty has not yet been demonstrated against a sufficiently broad prior-art search.

## Candidate contributions considered

1. **Signed planetary transmission plus wheel-force/yaw-feasibility pairing.** Provisionally selected as the core thread because it is anchored to the supplied design and can be audited exactly. Novelty remains unproven.
2. **Sensitivity of yaw acceleration to mass distribution, spacing, gearbox efficiency, and friction.** Useful as a follow-on, but a quantitative sensitivity study should wait until physically defensible parameter ranges are available.
3. **Experimental parameter identification and validation.** The strongest route to a defensible applied robotics/mechatronics paper, but it requires access to the hardware, instrumentation and data, which were not supplied.

## Major issues a reviewer is likely to raise

1. **Novelty:** four-wheel independently steered/driven kinematics, planar dynamics, friction constraints and force allocation have already been studied. The paper must state exactly what the signed carrier-output/ring-output planetary arrangement contributes beyond existing work.
2. **Model identifiability:** assembled mass, module spacing, CG, yaw inertia, selected motor curves and tire properties are missing. A simulation using placeholders cannot establish the real rover's maximum yaw acceleration, turning radius or motor adequacy.
3. **Validation:** there are no experimental measurements from the rover. Analytical and numerical checks are useful, but the model must ultimately be compared against measured trajectories, steering angles, wheel speeds and motor currents on manoeuvres not used for parameter fitting.
4. **Tire model:** the regularized longitudinal/lateral force law is an illustrative low-speed proxy, not a calibrated tire model. Its scope needs to be justified or replaced by an identified slip-angle/combined-slip model.
5. **Mechanical design substantiation:** the preliminary gear tooth calculations do not contain enough traceable rating-standard inputs to certify the reported factors of safety. The bearing-life arithmetic has been corrected, but equivalent-load assumptions must also be verified.
6. **Efficiency:** applying 0.92 to each stage is acceptable for a preliminary operating-point estimate, not a measured universal efficiency. Planetary losses can depend on load, speed, lubrication, bearings and load sharing.
7. **Load allocation:** 30 kg per module represents the nominal payload share, not the complete rover weight. Actual mass and normal load distribution must include the hardware and CG/load-transfer effects where relevant.
8. **Steering scrub:** the source uses a circular-patch scrub-torque equation alongside a rectangular contact-patch area assumption. The equivalent scrub model needs measurement or geometric reconciliation.

## Calculation audit highlights

| Item | Result |
|---|---|
| Stage 1 signed speed ratio | `+1/4` |
| Stage 2 signed speed ratio | `-1/3` |
| Combined wheel/motor speed ratio | `-1/12` |
| Steering carrier/sun speed ratio | `+1/6` |
| Payload-only longitudinal force demand | approximately `55.51 N/module` |
| Payload-only wheel torque | approximately `5.274 N m/module` |
| Payload-only motor torque at stated operating point | approximately `0.519 N m/module` |
| Four-module input power at that point | approximately `314.82 W`, excluding hardware mass |
| Bearing `L10` life recomputation using stated C, P, speed | approximately `7.87 x 10^4 h`, not `4.72 x 10^5 h` |

## Verification actually performed

- Eight analytical assertions for planetary signed ratios, straight-line kinematics, in-place wheel-speed magnitude and power consistency passed.
- Six numerical checks passed: finite output states, friction-circle enforcement at saved samples, no unintended yaw in the straight baseline, zero requested body twist for stationary steering, low-friction stress test reaches the friction boundary, and low-torque stress test activates torque saturation.
- Seven deterministic simulations were generated: straight, crab, constant-command curve, in-place rotation, steering at zero requested body twist, low-friction stress turn, and low-motor-torque stress turn.
- These checks are software/model verification only. No hardware accuracy, motor-selection suitability, gear safety or external predictive accuracy is claimed.

## Data collection required before a submission-quality validation

1. Measure the complete empty and loaded rover mass; obtain CG and yaw inertia from CAD, then verify mass properties where practical.
2. Measure true module centre spacing and module locations relative to CG.
3. Select exact drive and steering motor/controller models; obtain torque-speed curves, current limits, peak-duration limits, motor constants and thermal information.
4. Verify gearbox member connections, direction signs and backlash on a stand. Measure efficiency at multiple loads and speeds if power predictions matter.
5. Characterize steering resistance under representative normal loads in both steering directions and reconcile contact patch geometry with the assumed scrub moment.
6. Conduct repeatable tire traction and rolling-resistance tests on the target floor. Estimate uncertainty and normal-load dependence.
7. Log synchronized wheel speeds, module angles, motor current/voltage, IMU, body heading and independent position/velocity reference at a rate appropriate to the manoeuvre dynamics.
8. Perform straight acceleration, crab motion, constant-radius turns, in-place rotation, steering while stationary, and safe near-limit manoeuvres. Use separate parameter-identification and validation runs.
9. Report bias, RMSE, maximum absolute error, repeatability and uncertainty for velocity, yaw rate, heading, position, steering and motor current.
10. Expand the literature review with database search strings, inclusion/exclusion criteria, and full-text review of the closest prior models before finalizing the novelty claim.

## Journal suitability

- **Current status:** not ready for submission to a peer-reviewed journal.
- **Most plausible future venue family:** applied mobile robotics, mechatronics or vehicle-dynamics journals if the work is supported by a real rover, parameter identification, and independent experimental validation.
- **A transmission-mechanics venue** would require a stronger gearbox-specific contribution, a standards-based gear-rating analysis and/or measured transmission results. Merely embedding known planetary ratios in an established rover dynamics model is unlikely to be sufficient.
- No acceptance likelihood is claimed. The appropriate target venue should be chosen after the novelty contribution and validation evidence are strengthened.

## Pre-submission checklist

- [ ] Replace all illustrative model parameters with traceable CAD, datasheet or measurement values.
- [ ] Verify the signed direction convention physically for each gearbox stage.
- [ ] Recompute normal-load distribution from complete mass/CG geometry, including appropriate load transfer.
- [ ] Identify tire and steering scrub parameters from tests or justify a higher-fidelity tire/contact model.
- [ ] Validate the dynamic model against independent physical manoeuvres.
- [ ] Complete a broader literature review and narrow the originality statement.
- [ ] Rework gear/bearing checks against appropriate accepted rating standards and duty requirements.
- [ ] Quantify uncertainty and repeatability; report no simulated result as experimental.
- [ ] Freeze code and parameter configurations corresponding to every reported result.
- [ ] Follow the chosen journal's scope, reference, figure and data/code policies.
