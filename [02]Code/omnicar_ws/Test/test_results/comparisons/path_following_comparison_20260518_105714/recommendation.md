# Path-following comparison

Best score: `PF_2026-05-11_INFIN_MAP_MPCC_V01_R05`

The score is a practical ranking for experiment selection, not a formal proof of optimality.
It penalizes tracking RMSE, maximum tracking error, yaw RMSE, incomplete path progress, and command saturation.

## Ranking

| rank | test_id | controller | desired_speed | rmse_x_m | rmse_y_m | max_x_m | max_y_m | rmse_yaw_rad | mean_abs_e_along_m | progress | score |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | PF_2026-05-11_INFIN_MAP_MPCC_V01_R05 | MPCC |  | 0.0368 | 0.0583 | 0.09162 | 0.145 | 0.0139 | 0.05892 | 0.4976 | 0.6101 |
| 2 | PF_2026-04-29_INFIN_MAP_LINEAR_V01_R10 | PONLY | 0.5 | 0.03959 | 0.02524 | 0.1104 | 0.08921 | 0.03235 | 0.0001955 | 0.4793 | 0.6868 |
