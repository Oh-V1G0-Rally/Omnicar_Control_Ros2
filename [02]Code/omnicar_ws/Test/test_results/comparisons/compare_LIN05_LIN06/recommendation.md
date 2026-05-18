# Common-segment path-following comparison

All metrics are computed only on the common trajectory segment reached by every selected test.
The reference is the same geometric path for every controller, interpolated at the recorded path coordinate.

## Metrics

| test_id | controller | desired_speed | common_segment_m | rmse_contour_m | mae_contour_m | max_contour_m | rmse_lag_m | mae_lag_m |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| PF_2026-04-29_INFIN_MAP_LINEAR_V01_R08 | PONLY | 0.4 | 4.537 | 0.03542 | 0.02997 | 0.07717 | 0.0008175 | 0.0005088 |
| PF_2026-04-29_INFIN_MAP_LINEAR_V01_R10 | PONLY | 0.5 | 4.537 | 0.04694 | 0.03918 | 0.1166 | 0.00107 | 0.0006321 |
