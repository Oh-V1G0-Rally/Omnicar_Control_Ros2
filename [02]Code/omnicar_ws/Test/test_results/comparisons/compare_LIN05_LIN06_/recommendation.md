# Common-segment path-following comparison

All metrics are computed only on the common trajectory segment reached by every selected test.
The reference is the same geometric path for every controller, interpolated at the recorded path coordinate.

## Metrics

| test_id | controller | desired_speed | common_segment_m | rmse_contour_m | mae_contour_m | max_contour_m | rmse_lag_m | mae_lag_m |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| PF_2026-04-29_INFIN_MAP_LINEAR_V01_R05 | PONLY | 0.2 | 4.548 | 0.0115 | 0.008596 | 0.03667 | 0.000343 | 0.0001839 |
| PF_2026-04-29_INFIN_MAP_LINEAR_V01_R06 | PONLY | 0.3 | 4.548 | 0.02589 | 0.02104 | 0.06458 | 0.0009785 | 0.000501 |
