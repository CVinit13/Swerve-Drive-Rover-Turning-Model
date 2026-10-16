# Four-Wheel Swerve-Drive Rover: Research Draft and Reproducibility Package

## Status

This is a technically grounded **research draft and modelling prototype**, not a validated digital twin and not a publication-ready manuscript. The current source PDF provides preliminary mechanical sizing but omits several inputs needed to identify or validate a full-rover dynamic model. The manuscript explicitly records those gaps rather than filling them with invented data.

## Main findings from the supplied calculation sheet

- Signed planetary speed ratios, using the Willis relationship:
  - Drive stage 1 (sun input, ring fixed, carrier output): `+1/4`.
  - Drive stage 2 (sun input, carrier fixed, ring output): `-1/3`.
  - Combined drive wheel-to-motor speed ratio, assuming stage-1 carrier drives stage-2 sun: `-1/12` (magnitude 12:1).
  - Steering stage (sun input, ring fixed, carrier output): `+1/6`.
- Recalculated payload-only operating point with 30 kg nominal payload share per module, 0.8 m/s^2 acceleration, 5-degree grade, rolling-resistance coefficient 0.02 and 0.92 efficiency per stage: approximately 55.51 N required longitudinal force, 5.274 N m wheel torque, 0.519 N m drive-motor torque and 78.71 W motor input per module.
- Four-module input power at that point is approximately 314.82 W **for the 120 kg payload-only basis**, before adding chassis, batteries, actuators and other hardware mass.
- The source bearing-life calculation is arithmetically high by about a factor of six. With the source's stated `C = 4900 N`, `P = 591.2 N`, and `n = 120.6 rpm`, the basic ball-bearing `L10` relation gives roughly `7.87 x 10^4 h`, not `4.72 x 10^5 h`. This does not establish the load estimate itself is correct.

## Critical missing parameters

The following must be obtained or identified before reporting physical turning-performance predictions: total assembled mass, true module spacing from CAD, loaded centre-of-gravity location and yaw inertia, selected drive and steering motor models/torque-speed/current curves, drive-train inertias, steering rate limits and backlash, tire longitudinal/lateral properties, measured rolling resistance, gearbox efficiency versus operating point, and steering scrub torque under representative normal load.

The model's default total mass of 120 kg is explicitly a payload-only illustrative lower bound. The 0.80 m module spacing, yaw inertia, tire coefficients, inertias, dampings and controller gains are labelled illustrative. The factor of safety is not applied as extra simulated mass.

## Run the numerical model

Python 3.10+ is recommended. From this folder:

```bash
python -m pip install -r requirements.txt
python swerve_rover_model.py --outdir results --duration 2.5
```

The command writes analytical checks, numerical checks, a parameter configuration, case-specific configurations, seven per-case CSV time histories, a summary table, and figures. Four baseline motions are included: straight translation/acceleration, crab motion, a constant-command curve and in-place rotation. Three additional cases exercise stationary steering, a deliberately low-friction turn, and a deliberately low-motor-torque turn. The latter two use altered stress-test parameters, not proposed hardware values.

The manuscript can be rebuilt using:

```bash
python build_manuscript.py
```

The rebuild requires `python-docx` and the already-generated `figures/` and `results/` contents. The PDF was generated from the Word manuscript with LibreOffice; the current Word and PDF copies are included.

## Package contents

- `Swerve_Rover_Research_Draft.docx`: editable manuscript.
- `Swerve_Rover_Research_Draft.pdf`: rendered manuscript.
- `swerve_rover_model.py`: executable reduced-order ODE model.
- `build_manuscript.py`: manuscript, literature matrix and calculation-audit generator.
- `calculation_audit.csv`: traceable source-versus-recalculation table.
- `literature_matrix.csv`: targeted literature matrix with DOIs and relevance notes.
- `figures/`: geometry, planar free-body/force decomposition, architecture, signed gear ratios and illustrative response plots.
- `results/`: analytical/numerical checks, case configurations, simulation histories, summaries and run log.
- `source_design_calculations.pdf`: a copy of the supplied preliminary calculation PDF.
- `review_and_readiness.md`: standalone critical review, publication-readiness assessment and pre-submission checklist.

## Interpretation cautions

- Code verification means selected equations/constraints were implemented consistently. It is not experimental validation.
- Stress-test simulation results are not measured or expected rover performance.
- The literature review is targeted but not exhaustive, and the exact novelty of the compound planetary configuration has not been established.
- The manuscript should not be submitted until the missing parameters have been sourced or identified, the gear/bearing calculations receive proper standard-based review, and a physical validation study supports the model's claims.
