# Simulation results report

Generated from actual simulation samples.

## Settings

| Parameter | Value |
|---|---:|
| Time step | 0.005 s |
| Duration | 12 s |
| Nominal target | 1.8 m/s |
| Torque per payload box | 0.06 N m |
| PWM carrier | 20 Hz |
| Random seed | 7 |
| Actuator range | [0, 1] |
| Bang-Bang deadband | 0.12 m/s |
| Inertia | 0.06 kg m² |
| Torque coefficient | 1.1 N m/unit |
| Viscous friction | 0.04 N m s/rad |
| Roller radius | 0.08 m |

## Metrics by scenario

### baseline

Constant nominal target with two active payload boxes.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.6683 | 2.1482 | 0.22232 | 0.42302 | 1.8 | -0.095797 | 2.48 | n/a | 6.6696 | 1.9201 | n/a | n/a | 0.047075 | 0.16606 | 11.64 | 0.97001 | 0 | 5 | 0 | 0 | 0 | 0.33278 | 0.017228 |
| pwm | 2.7197 | 2.1476 | 0.22658 | 0.42295 | 1.8 | 0.060297 | 2.48 | n/a | 0 | 1.7431 | n/a | n/a | 0.0021267 | 0.0073441 | 11.075 | 0.88844 | 0.0011697 | 0 | 2.075 | 0.17292 | 17.292 | 19.967 | 0.0023618 |
| pid | 2.1504 | 2.1023 | 0.17913 | 0.41847 | 1.8 | 0.00013661 | 2.485 | 3.575 | 0.89324 | 1.8161 | n/a | n/a | 8.8264e-05 | 0.00026324 | 10.859 | 0.95075 | 6.5303e-05 | 0 | 0.83 | 0.069167 | 6.9167 | 0.33278 | 5.4471e-05 |

Graph: `plots/baseline_comparison.png`

### load_step

Increases from one to three active boxes at 40% of the run.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.2486 | 1.9344 | 0.18732 | 0.40141 | 1.8 | -0.03999 | 2.12 | n/a | 6.6903 | 1.9204 | 0 | n/a | 7.4957e-06 | 2.6442e-05 | 11.81 | 0.98417 | 0 | 3 | 0 | 0 | 0 | 0.33278 | 3.4677e-06 |
| pwm | 2.8395 | 2.0468 | 0.23659 | 0.41292 | 1.8 | 0.14891 | 2.12 | n/a | 1.2395 | 1.8223 | 0.055959 | n/a | 0.0027415 | 0.012116 | 11.1 | 0.89798 | 0.0015078 | 0 | 1.84 | 0.15333 | 15.333 | 2.995 | 0.0020909 |
| pid | 2.0289 | 1.9241 | 0.169 | 0.40035 | 1.8 | -0.00079766 | 2.14 | 6.91 | 1.5679 | 1.8282 | 0.056674 | 2.105 | 0.00024788 | 0.0010951 | 11.219 | 0.96627 | 0.00054043 | 0 | 0.565 | 0.047083 | 4.7083 | 0.33278 | 0.00072912 |

Graph: `plots/load_step_comparison.png`

### load_cycle

Increases from one to three boxes at 35%, then removes one at 70%.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.2871 | 1.9457 | 0.19054 | 0.40259 | 1.8 | -0.018345 | 2.12 | n/a | 6.6903 | 1.9204 | 0 | 0.695 | 0.071324 | 0.24464 | 11.63 | 0.96918 | 0.26314 | 5 | 0 | 0 | 0 | 0.33278 | 0.089169 |
| pwm | 2.7415 | 2.016 | 0.2284 | 0.40979 | 1.8 | 0.071779 | 2.12 | n/a | 0.11755 | 1.8021 | 0.052875 | n/a | 0.0089159 | 0.037016 | 11.06 | 0.89416 | 0.0049037 | 0 | 1.84 | 0.15333 | 15.333 | 0.33278 | 0.0039997 |
| pid | 2.063 | 1.9242 | 0.17185 | 0.40036 | 1.8 | -0.0093415 | 2.14 | 6.065 | 1.7929 | 1.8323 | 0.051351 | 1.865 | 0.0088095 | 0.027843 | 11.03 | 0.95817 | 0.0037165 | 0 | 0.565 | 0.047083 | 4.7083 | 0.33278 | 0.0020453 |

Graph: `plots/load_cycle_comparison.png`

### light_load

One active payload box.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.427 | 1.9618 | 0.20221 | 0.40425 | 1.8 | -0.036158 | 2.12 | n/a | 6.6903 | 1.9204 | n/a | n/a | 0.07366 | 0.24311 | 10.86 | 0.90504 | 0.36474 | 13 | 0 | 0 | 0 | 0.66556 | 0.086444 |
| pwm | 2.1273 | 1.9258 | 0.17721 | 0.40052 | 1.8 | -0.027478 | 2.12 | 3.56 | 1.8898 | 1.834 | n/a | n/a | 0.003099 | 0.013933 | 10.935 | 0.84743 | 0.0017045 | 0 | 1.84 | 0.15333 | 15.333 | 19.967 | 0.0026495 |
| pid | 1.9454 | 1.9205 | 0.16205 | 0.39997 | 1.8 | 0.00010286 | 2.14 | 3.015 | 1.5679 | 1.8282 | n/a | n/a | 0.00014182 | 0.00046781 | 9.8806 | 0.90599 | 3.0412e-05 | 0 | 0.565 | 0.047083 | 4.7083 | 0.33278 | 2.7584e-05 |

Graph: `plots/light_load_comparison.png`

### medium_load

Two active payload boxes.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.6683 | 2.1482 | 0.22232 | 0.42302 | 1.8 | -0.095797 | 2.48 | n/a | 6.6696 | 1.9201 | n/a | n/a | 0.047075 | 0.16606 | 11.64 | 0.97001 | 0 | 5 | 0 | 0 | 0 | 0.33278 | 0.017228 |
| pwm | 2.7197 | 2.1476 | 0.22658 | 0.42295 | 1.8 | 0.060297 | 2.48 | n/a | 0 | 1.7431 | n/a | n/a | 0.0021267 | 0.0073441 | 11.075 | 0.88844 | 0.0011697 | 0 | 2.075 | 0.17292 | 17.292 | 19.967 | 0.0023618 |
| pid | 2.1504 | 2.1023 | 0.17913 | 0.41847 | 1.8 | 0.00013661 | 2.485 | 3.575 | 0.89324 | 1.8161 | n/a | n/a | 8.8264e-05 | 0.00026324 | 10.859 | 0.95075 | 6.5303e-05 | 0 | 0.83 | 0.069167 | 6.9167 | 0.33278 | 5.4471e-05 |

Graph: `plots/medium_load_comparison.png`

### heavy_load

Three active payload boxes.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.6622 | 2.3419 | 0.22178 | 0.44168 | 1.8 | -0.039066 | 3.03 | n/a | 2.1884 | 1.8394 | n/a | n/a | 0.0006848 | 0.0024157 | 12 | 1 | 0 | 1 | 0 | 0 | 0 | 0.33278 | 0.00031681 |
| pwm | 3.6177 | 2.5139 | 0.30141 | 0.45762 | 1.8 | 0.1486 | 3.03 | n/a | 0 | 1.6574 | n/a | n/a | 0.0027756 | 0.011966 | 11.26 | 0.92372 | 0.0015266 | 0 | 2.4 | 0.2 | 20 | 2.995 | 0.0020908 |
| pid | 2.4857 | 2.3379 | 0.20706 | 0.4413 | 1.8 | 6.3948e-05 | 3.035 | 4.785 | 0.2224 | 1.804 | n/a | n/a | 0.00018454 | 0.00065585 | 11.776 | 0.99059 | 9.2403e-05 | 0 | 1.61 | 0.13417 | 13.417 | 0.33278 | 0.00018404 |

Graph: `plots/heavy_load_comparison.png`

### saturation

Requests 115% of the configured maximum belt speed.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 29.099 | 72.729 | 2.4248 | 2.4617 | 4.14 | 2.181 | n/a | n/a | 0 | 1.9594 | n/a | n/a | 0.00072946 | 0.0025732 | 12 | 1 | 0 | 1 | 0 | 0 | 0 | 0.33278 | 0.00033747 |
| pwm | 29.099 | 72.729 | 2.4248 | 2.4617 | 4.14 | 2.181 | n/a | n/a | 0 | 1.9594 | n/a | n/a | 0.00072946 | 0.0025732 | 12 | 1 | 0 | 0 | 12 | 1 | 100 | 0.33278 | 0.00033747 |
| pid | 29.099 | 72.729 | 2.4248 | 2.4617 | 4.14 | 2.181 | n/a | n/a | 0 | 1.9594 | n/a | n/a | 0.00072946 | 0.0025732 | 12 | 1 | 0 | 0 | 12 | 1 | 100 | 0.33278 | 0.00033747 |

Graph: `plots/saturation_comparison.png`

### changing_reference

Target steps through 55%, 100%, then 70% of nominal.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 1.9655 | 0.84864 | 0.16373 | 0.26588 | 0.99 | -0.02134 | 0.83 | 11.99 | 5.3491 | 1.8963 | n/a | n/a | 0.070355 | 0.24574 | 9.07 | 0.75552 | 0.47403 | 20 | 0 | 0 | 0 | 1.3311 | 0.095801 |
| pwm | 2.37 | 0.96691 | 0.19744 | 0.2838 | 0.99 | 0.057059 | 1.85 | n/a | 0 | 1.7248 | n/a | n/a | 0.025441 | 0.10684 | 8.645 | 0.68953 | 0.013993 | 0 | 1.175 | 0.097917 | 9.7917 | 0.33278 | 0.038928 |
| pid | 2.0449 | 0.93015 | 0.17035 | 0.27835 | 0.99 | 0.026276 | 1.545 | 11.915 | 0.89874 | 1.8162 | n/a | n/a | 0.036679 | 0.13461 | 7.15 | 0.75263 | 0.026723 | 0 | 0.48 | 0.04 | 4 | 0.33278 | 0.046988 |

Graph: `plots/changing_reference_comparison.png`

## Linear stability check

The reduced plant pole is -0.66667 rad/s (stable: True).
The ideal continuous PID linearization has poles -1.216+0j, -10.192+0j rad/s (stable: True). Continuous ideal PID linearization; saturation, PWM switching and derivative filter omitted.

## Frequency response

The plant and ideal unsaturated PID closed-loop Bode traces are in `plots/frequency_response.png`; samples are in `csv/frequency_response.csv`. Baseline speed FFT is in `plots/frequency_spectrum.png`; PWM carrier test is in `plots/pwm_frequency_sweep.png`.

## PWM carrier-frequency sweep

| Carrier (Hz) | Speed ripple RMS (m/s) | Speed ripple p-p (m/s) | Dominant ripple (Hz) |
|---:|---:|---:|---:|
| 5 | 0.00934854 | 0.0321468 | 4.9917 |
| 10 | 0.00448862 | 0.0202425 | 9.9834 |
| 20 | 0.00212675 | 0.00734414 | 19.967 |
| 40 | 0.00219962 | 0.00863976 | 11.647 |
Lowest measured peak-to-peak ripple in this sweep: 20 Hz.

## Interpretation

## baseline
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.2223, pwm=0.2266, pid=0.1791; lowest observed: pid (0.1791)
- RMSE: bang-bang=0.4230, pwm=0.4230, pid=0.4185; lowest observed: pid (0.4185)
- rise time s: bang-bang=2.480, pwm=2.480, pid=2.485; lowest observed: bang-bang (2.480)
- overshoot %: bang-bang=6.67, pwm=0.00, pid=0.89; lowest observed: pwm (0.00)
- settling s: bang-bang=n/a, pwm=n/a, pid=3.575; lowest observed: pid (3.575)
- load recovery s: no finite result
- effort: bang-bang=11.6400, pwm=11.0750, pid=10.8589; lowest observed: pid (10.8589)
- saturation %: bang-bang=0.00, pwm=17.29, pid=6.92; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.16606, pwm=0.00734, pid=0.00026; lowest observed: pid (0.00026)
- dominant ripple Hz: bang-bang=0.333, pwm=19.967, pid=0.333; lowest observed: bang-bang (0.333)

## load_step
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.1873, pwm=0.2366, pid=0.1690; lowest observed: pid (0.1690)
- RMSE: bang-bang=0.4014, pwm=0.4129, pid=0.4003; lowest observed: pid (0.4003)
- rise time s: bang-bang=2.120, pwm=2.120, pid=2.140; lowest observed: bang-bang (2.120)
- overshoot %: bang-bang=6.69, pwm=1.24, pid=1.57; lowest observed: pwm (1.24)
- settling s: bang-bang=n/a, pwm=n/a, pid=6.910; lowest observed: pid (6.910)
- load recovery s: bang-bang=n/a, pwm=n/a, pid=2.105; lowest observed: pid (2.105)
- effort: bang-bang=11.8100, pwm=11.1000, pid=11.2187; lowest observed: pwm (11.1000)
- saturation %: bang-bang=0.00, pwm=15.33, pid=4.71; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.00003, pwm=0.01212, pid=0.00110; lowest observed: bang-bang (0.00003)
- dominant ripple Hz: bang-bang=0.333, pwm=2.995, pid=0.333; lowest observed: bang-bang (0.333)

## load_cycle
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.1905, pwm=0.2284, pid=0.1718; lowest observed: pid (0.1718)
- RMSE: bang-bang=0.4026, pwm=0.4098, pid=0.4004; lowest observed: pid (0.4004)
- rise time s: bang-bang=2.120, pwm=2.120, pid=2.140; lowest observed: bang-bang (2.120)
- overshoot %: bang-bang=6.69, pwm=0.12, pid=1.79; lowest observed: pwm (0.12)
- settling s: bang-bang=n/a, pwm=n/a, pid=6.065; lowest observed: pid (6.065)
- load recovery s: bang-bang=0.695, pwm=n/a, pid=1.865; lowest observed: bang-bang (0.695)
- effort: bang-bang=11.6300, pwm=11.0600, pid=11.0295; lowest observed: pid (11.0295)
- saturation %: bang-bang=0.00, pwm=15.33, pid=4.71; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.24464, pwm=0.03702, pid=0.02784; lowest observed: pid (0.02784)
- dominant ripple Hz: bang-bang=0.333, pwm=0.333, pid=0.333; lowest observed: bang-bang (0.333)

## light_load
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.2022, pwm=0.1772, pid=0.1620; lowest observed: pid (0.1620)
- RMSE: bang-bang=0.4043, pwm=0.4005, pid=0.4000; lowest observed: pid (0.4000)
- rise time s: bang-bang=2.120, pwm=2.120, pid=2.140; lowest observed: bang-bang (2.120)
- overshoot %: bang-bang=6.69, pwm=1.89, pid=1.57; lowest observed: pid (1.57)
- settling s: bang-bang=n/a, pwm=3.560, pid=3.015; lowest observed: pid (3.015)
- load recovery s: no finite result
- effort: bang-bang=10.8600, pwm=10.9350, pid=9.8806; lowest observed: pid (9.8806)
- saturation %: bang-bang=0.00, pwm=15.33, pid=4.71; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.24311, pwm=0.01393, pid=0.00047; lowest observed: pid (0.00047)
- dominant ripple Hz: bang-bang=0.666, pwm=19.967, pid=0.333; lowest observed: pid (0.333)

## medium_load
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.2223, pwm=0.2266, pid=0.1791; lowest observed: pid (0.1791)
- RMSE: bang-bang=0.4230, pwm=0.4230, pid=0.4185; lowest observed: pid (0.4185)
- rise time s: bang-bang=2.480, pwm=2.480, pid=2.485; lowest observed: bang-bang (2.480)
- overshoot %: bang-bang=6.67, pwm=0.00, pid=0.89; lowest observed: pwm (0.00)
- settling s: bang-bang=n/a, pwm=n/a, pid=3.575; lowest observed: pid (3.575)
- load recovery s: no finite result
- effort: bang-bang=11.6400, pwm=11.0750, pid=10.8589; lowest observed: pid (10.8589)
- saturation %: bang-bang=0.00, pwm=17.29, pid=6.92; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.16606, pwm=0.00734, pid=0.00026; lowest observed: pid (0.00026)
- dominant ripple Hz: bang-bang=0.333, pwm=19.967, pid=0.333; lowest observed: bang-bang (0.333)

## heavy_load
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.2218, pwm=0.3014, pid=0.2071; lowest observed: pid (0.2071)
- RMSE: bang-bang=0.4417, pwm=0.4576, pid=0.4413; lowest observed: pid (0.4413)
- rise time s: bang-bang=3.030, pwm=3.030, pid=3.035; lowest observed: bang-bang (3.030)
- overshoot %: bang-bang=2.19, pwm=0.00, pid=0.22; lowest observed: pwm (0.00)
- settling s: bang-bang=n/a, pwm=n/a, pid=4.785; lowest observed: pid (4.785)
- load recovery s: no finite result
- effort: bang-bang=12.0000, pwm=11.2600, pid=11.7762; lowest observed: pwm (11.2600)
- saturation %: bang-bang=0.00, pwm=20.00, pid=13.42; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.00242, pwm=0.01197, pid=0.00066; lowest observed: pid (0.00066)
- dominant ripple Hz: bang-bang=0.333, pwm=2.995, pid=0.333; lowest observed: bang-bang (0.333)

## saturation
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=2.4248, pwm=2.4248, pid=2.4248; lowest observed: bang-bang (2.4248)
- RMSE: bang-bang=2.4617, pwm=2.4617, pid=2.4617; lowest observed: bang-bang (2.4617)
- rise time s: no finite result
- overshoot %: bang-bang=0.00, pwm=0.00, pid=0.00; lowest observed: bang-bang (0.00)
- settling s: no finite result
- load recovery s: no finite result
- effort: bang-bang=12.0000, pwm=12.0000, pid=12.0000; lowest observed: bang-bang (12.0000)
- saturation %: bang-bang=0.00, pwm=100.00, pid=100.00; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.00257, pwm=0.00257, pid=0.00257; lowest observed: bang-bang (0.00257)
- dominant ripple Hz: bang-bang=0.333, pwm=0.333, pid=0.333; lowest observed: bang-bang (0.333)

## changing_reference
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.1637, pwm=0.1974, pid=0.1703; lowest observed: bang-bang (0.1637)
- RMSE: bang-bang=0.2659, pwm=0.2838, pid=0.2784; lowest observed: bang-bang (0.2659)
- rise time s: bang-bang=0.830, pwm=1.850, pid=1.545; lowest observed: bang-bang (0.830)
- overshoot %: bang-bang=5.35, pwm=0.00, pid=0.90; lowest observed: pwm (0.00)
- settling s: bang-bang=11.990, pwm=n/a, pid=11.915; lowest observed: pid (11.915)
- load recovery s: no finite result
- effort: bang-bang=9.0700, pwm=8.6450, pid=7.1500; lowest observed: pid (7.1500)
- saturation %: bang-bang=0.00, pwm=9.79, pid=4.00; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.24574, pwm=0.10684, pid=0.13461; lowest observed: pwm (0.10684)
- dominant ripple Hz: bang-bang=1.331, pwm=0.333, pid=0.333; lowest observed: pwm (0.333)


## Limits

The plant is reduced-order. PyBullet is a synchronized visualization, not coupled co-simulation. Frequency response describes the linearized plant and is not a nonlinear closed-loop stability proof.