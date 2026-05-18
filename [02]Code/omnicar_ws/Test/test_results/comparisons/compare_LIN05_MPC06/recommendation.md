# Common-segment path-following comparison

All metrics are computed only on the common trajectory segment reached by every selected test.
The reference is the same geometric path for every controller, interpolated at the recorded path coordinate.

## Metrics

| test_id | controller | desired_speed | common_segment_m | rmse_contour_m | mae_contour_m | max_contour_m | rmse_lag_m | mae_lag_m |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| PF_2026-04-29_INFIN_MAP_LINEAR_V01_R10 | PONLY | 0.5 | 4.534 | 0.04691 | 0.03912 | 0.1166 | 0.001071 | 0.0006332 |
| PF_2026-05-11_INFIN_MAP_MPCC_V01_R06 | MPCC |  | 4.534 | 0.03298 | 0.02566 | 0.07047 | 0.1105 | 0.1039 |
