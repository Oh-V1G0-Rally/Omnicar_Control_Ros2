# Path Following Test Analysis

This folder contains the reusable analysis code for path following experiments.

## Structure

- `scripts/`: bag extraction, metrics computation, plotting and global summary update.
- `config/`: analysis defaults, map geometry and naming rules.
- `../test_results/`: generated outputs for analyzed tests.

## Main Command

Prerequisites:

```bash
source /opt/ros/jazzy/setup.bash
source /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/install/setup.bash
sudo apt install python3-pandas
```

```bash
python3 /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/Test/test_code/scripts/analyze_path_following_bag.py \
  --bag /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/bags/PF_2026-04-27_RECT_MAP_LINEAR_V01_R01 \
  --test-id PF_2026-04-27_RECT_MAP_LINEAR_V01_R01 \
  --path-name RECT \
  --frame MAP \
  --controller-name LINEAR \
  --controller-version V01 \
  --path-file /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/src/splines/FEUP/FEUP_rectangle.csv \
  --config /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/Test/test_code/config/analysis_config.yaml \
  --output-root /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/Test/test_results
```

## Outputs

For each analyzed test the pipeline generates:

- `test_info.yaml`
- `raw/path_follower_debug.csv`
- `raw/path_reference.csv`
- `figures/01_xy_map.png`
- `figures/02_target_tracking_and_errors.png`
- `figures/03_metrics_summary.png`
- `metrics/metrics.csv`
- `metrics/metrics.json`

It also updates:

- `test_results/path_following_tests_summary.csv`

Controller parameter defaults are loaded from `controller_defaults` in the analysis config, then
overridden from `controller_config` when that key points to the runtime ROS parameter YAML
used by the path follower.

## Multi-Test Comparison

To compare multiple tests on the same `map_xp` figure, first analyze each bag with
`analyze_path_following_bag.py`, then select the result folders or test IDs:

```bash
python3 /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/Test/test_code/scripts/compare_path_following_tests.py \
  --tests \
    PF_2026-04-27_INFINITY_MAP_LINEAR_V01_R01 \
    PF_2026-04-27_INFINITY_MAP_MPCC_V01_R01 \
    PF_2026-04-27_INFINITY_MAP_NONLINEAR_V01_R01 \
  --results-root /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/Test/test_results \
  --title "Infinity trajectory controller comparison"
```

The script writes:

- `figures/map_xp.png`: all selected robot trajectories on the same map, with the legend below the plot.
- `figures/comparison_metrics.png`: combined figure with map, shared legend and grouped metric bars for common-segment contouring and lag errors.
- `tables/comparison_metrics.csv`: comparison table for thesis tables, computed only on the common trajectory segment.
- `recommendation.md`: compact table of the same common-segment metrics.

It can also read raw bag folders directly when `--bags-root` and `--path-file` are provided:

```bash
python3 /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/Test/test_code/scripts/compare_path_following_tests.py \
  --tests PF_2026-04-27_INFINITY_MAP_LINEAR_V01_R01 PF_2026-04-27_INFINITY_MAP_MPCC_V01_R01 \
  --bags-root /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/bags \
  --path-file /home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/src/splines/FEUP/FEUP_infinity.csv
```
