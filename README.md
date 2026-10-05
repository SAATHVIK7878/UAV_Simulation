# UAV Survey Toolkit

Plan, fly, log and analyse survey missions on **PX4 SITL + Gazebo + QGroundControl**
for disaster-area coverage scenarios.

PX4, Gazebo, QGroundControl and MAVSDK are existing open-source tools. This
toolkit is the project's own layer on top of them:

| Module | What it does |
|---|---|
| `config.py` | Validated mission settings (rejects typos like a 300 m altitude) |
| `geometry.py` | Lawnmower sweep generation, lat/lon conversion, path distances |
| `planner.py` | Builds a survey plan and its statistics (distance, time estimate) |
| `export.py` | Exports to QGroundControl `.plan`, GeoJSON and CSV |
| `mission.py` | Uploads and flies the plan with battery, timeout and Ctrl+C return-to-launch failsafes |
| `telemetry.py` | Fixed-rate CSV recording of position, speed, battery and mode |
| `analysis.py` / `plotting.py` | Flight metrics, deviation from the planned path, run comparison, figures |
| `cli.py` | `plan`, `fly`, `log`, `analyze` commands |

## Install

```bash
pip install -e ".[plot]"          # planning, logging analysis, figures
pip install "mavsdk>=3,<4"        # needed only for `fly` and `log`
```

Use MAVSDK **3.x**. A plain `pip install mavsdk` now installs 4.x, which has a
different API; the toolkit detects this and tells you.

## Typical workflow

```bash
# 1. Start the simulator (in PX4-Autopilot)
make px4_sitl gazebo-classic_iris

# 2. Preview a plan without any vehicle (also writes a QGroundControl .plan file)
python -m uav_survey plan --width 80 --height 60 --spacing 20 --altitude 30 --out-dir mission_out

# 3. Terminal A: record telemetry (stops by itself after landing)
python -m uav_survey log --out runs/alt30.csv --stop-on-land

# 4. Terminal B: fly it (plan is generated around the vehicle's actual home position)
python -m uav_survey fly --config configs/default.json --out-dir runs/alt30

# 5. Analyse one run, or compare several, against the plan
python -m uav_survey analyze --log runs/alt30.csv runs/alt15.csv \
    --plan runs/alt30/plan.json --plot-dir figures --summary-csv comparison.csv
```

Repeat step 3-4 with `configs/low_altitude_slow.json` and
`configs/high_altitude_fast.json` to get a real altitude/speed comparison.

## Safety behaviour

* Refuses to take off if GPS/home is not ready, the battery is unreadable, or
  the battery is below `min_start_battery_pct`.
* Refuses a saved plan made for a location more than 50 m from the vehicle.
* Aborts and commands return-to-launch if the battery stays below
  `min_battery_pct` (3 consecutive readings) or the mission exceeds
  `mission_timeout_s`.
* Ctrl+C during a flight commands return-to-launch before exiting.

## Tests

```bash
pip install pytest
pytest
```

The tests cover geometry, configuration validation, planning, exports,
analysis, the CLI, the recorder, and every failsafe path of the mission runner
using a scripted fake vehicle. They do **not** replace flying in the simulator.

## Limits (be upfront about these)

* Simulation only; no real hardware has been used.
* Fixed-pattern coverage: no obstacle avoidance, SLAM or onboard AI.
* The flat-earth conversion is accurate for areas up to a few hundred metres.
* The time estimate ignores wind and acceleration, so it is a lower bound.
* Path deviation is horizontal distance to the planned path, not timing along it.
