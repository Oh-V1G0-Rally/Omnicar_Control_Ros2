# Path-following comparison

Best score: `PF_2026-05-11_INFIN_MAP_MPCC_V01_R06`

The score is a practical ranking for experiment selection, not a formal proof of optimality.
It penalizes tracking RMSE, maximum tracking error, yaw RMSE, incomplete path progress, and command saturation.

## Ranking

| rank | test_id | controller | desired_speed | rmse_x_m | rmse_y_m | max_x_m | max_y_m | rmse_yaw_rad | mean_abs_e_along_m | progress | score |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | PF_2026-05-11_INFIN_MAP_MPCC_V01_R06 | MPCC |  | 0.06774 | 0.09337 | 0.1447 | 0.2019 | 0.02943 | 0.1038 | 0.9754 | 0.1969 |
| 2 | PF_2026-04-29_INFIN_MAP_LINEAR_V01_R10 | PONLY | 0.5 | 0.03959 | 0.02524 | 0.1104 | 0.08921 | 0.03235 | 0.0001955 | 0.4793 | 0.6868 |
