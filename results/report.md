# Simulation results report

Generated from actual simulation samples.

## Settings

| Parameter | Value |
|---|---:|
| Time step | 0.005 s |
| Duration | 12 s |
| Nominal target | 1.8 m/s |
| Load-step torque | 0.16 N m |
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

Constant nominal target; nominal load.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.257 | 1.8116 | 0.18803 | 0.38846 | 1.8 | -0.0099889 | 1.87 | n/a | 6.7021 | 1.9206 | n/a | n/a | 0.069967 | 0.24572 | 10.36 | 0.86339 | 0.37605 | 17 | 0 | 0 | 0 | 0.99834 | 0.082697 |
| pwm | 1.9697 | 1.7748 | 0.16409 | 0.38449 | 1.8 | -0.029894 | 1.87 | 11.945 | 2.1086 | 1.838 | n/a | n/a | 0.0037071 | 0.017091 | 10.405 | 0.83858 | 0.0020389 | 0 | 1.655 | 0.13792 | 13.792 | 19.967 | 0.0037859 |
| pid | 1.8008 | 1.7737 | 0.15 | 0.38437 | 1.8 | 7.3326e-05 | 1.915 | 4.435 | 2.2342 | 1.8402 | n/a | n/a | 0.00015895 | 0.00053818 | 8.9078 | 0.85892 | 1.545e-05 | 0 | 0.425 | 0.035417 | 3.5417 | 0.33278 | 5.8711e-05 |

Graph: `plots/baseline_comparison.png`

### load_step

Adds configured load torque at 40% of the run.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.2614 | 1.8065 | 0.1884 | 0.38792 | 1.8 | -0.078833 | 1.87 | n/a | 6.7002 | 1.9206 | 0 | n/a | 0.00085564 | 0.0030184 | 11.59 | 0.96585 | 0 | 5 | 0 | 0 | 0 | 0.33278 | 0.00039584 |
| pwm | 2.5182 | 1.8601 | 0.20983 | 0.39364 | 1.8 | 0.13786 | 1.87 | n/a | 2.1023 | 1.8378 | 0.047672 | n/a | 0.0026149 | 0.011965 | 10.9 | 0.88288 | 0.0014382 | 0 | 1.655 | 0.13792 | 13.792 | 19.967 | 0.0023618 |
| pid | 1.9353 | 1.7817 | 0.16121 | 0.38525 | 1.8 | -0.0011274 | 1.915 | 7.35 | 2.2342 | 1.8402 | 0.076935 | 2.545 | 0.0003331 | 0.0014506 | 10.62 | 0.93929 | 0.00075346 | 0 | 0.425 | 0.035417 | 3.5417 | 0.33278 | 0.0010037 |

Graph: `plots/load_step_comparison.png`

### load_cycle

Load rises to 80% of limit at 35%, then drops to 15% at 70%.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.1782 | 1.7999 | 0.18147 | 0.38721 | 1.8 | 0.0039625 | 1.87 | n/a | 6.6989 | 1.9206 | 0.053376 | 0.765 | 0.067991 | 0.24558 | 11.405 | 0.95044 | 0.29121 | 7 | 0 | 0 | 0 | 0.66556 | 0.087961 |
| pwm | 2.4291 | 1.8561 | 0.20235 | 0.39321 | 1.8 | -0.027914 | 1.87 | 9.37 | 2.1023 | 1.8378 | 0.092079 | 5.17 | 0.012796 | 0.055612 | 11.1 | 0.87581 | 0.0070376 | 0 | 1.655 | 0.13792 | 13.792 | 0.33278 | 0.019189 |
| pid | 2.2374 | 1.8104 | 0.18637 | 0.38834 | 1.8 | -0.023834 | 1.915 | 10.875 | 3.0626 | 1.8551 | 0.10891 | 0.01 | 0.0145 | 0.043168 | 10.577 | 0.93678 | 0.011652 | 0 | 3.15 | 0.2625 | 26.25 | 0.33278 | 0.010155 |

Graph: `plots/load_cycle_comparison.png`

### light_load

Constant 17% of configured maximum load torque.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.3804 | 1.935 | 0.1983 | 0.40148 | 1.8 | -0.0030009 | 2.08 | n/a | 6.6991 | 1.9206 | n/a | n/a | 0.066645 | 0.24132 | 10.86 | 0.90504 | 0.3608 | 13 | 0 | 0 | 0 | 0.66556 | 0.088515 |
| pwm | 2.1001 | 1.9011 | 0.17495 | 0.39794 | 1.8 | -0.027638 | 2.08 | 3.365 | 1.9177 | 1.8345 | n/a | n/a | 0.003178 | 0.01433 | 10.86 | 0.84561 | 0.0017479 | 0 | 1.805 | 0.15042 | 15.042 | 19.967 | 0.0028159 |
| pid | 1.9208 | 1.8966 | 0.16 | 0.39747 | 1.8 | 9.8057e-05 | 2.105 | 2.955 | 1.6631 | 1.8299 | n/a | n/a | 0.00014545 | 0.00048244 | 9.733 | 0.89902 | 2.695e-05 | 0 | 0.54 | 0.045 | 4.5 | 0.33278 | 3.2345e-05 |

Graph: `plots/light_load_comparison.png`

### medium_load

Constant 53% of configured maximum load torque.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 2.7924 | 2.2787 | 0.23264 | 0.43568 | 1.8 | -0.081045 | 2.8 | n/a | 4.5209 | 1.8814 | n/a | n/a | 0.00070043 | 0.0024708 | 12 | 1 | 0 | 1 | 0 | 0 | 0 | 0.33278 | 0.00032404 |
| pwm | 3.462 | 2.4103 | 0.28844 | 0.44809 | 1.8 | 0.13807 | 2.8 | n/a | 0 | 1.6653 | n/a | n/a | 0.0021142 | 0.0067579 | 11.11 | 0.9188 | 0.0011628 | 0 | 2.275 | 0.18958 | 18.958 | 19.967 | 0.0023618 |
| pid | 2.3427 | 2.2472 | 0.19515 | 0.43265 | 1.8 | 0.00013636 | 2.805 | 4.2 | 0.45801 | 1.8082 | n/a | n/a | 5.1211e-05 | 0.0002261 | 11.472 | 0.97762 | 9.6387e-05 | 0 | 1.19 | 0.099167 | 9.9167 | 0.33278 | 0.00013502 |

Graph: `plots/medium_load_comparison.png`

### heavy_load

Constant 90% of configured maximum load torque.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 4.1692 | 3.0023 | 0.34735 | 0.5001 | 1.8 | 0.14084 | 5.405 | n/a | 0 | 1.6595 | n/a | n/a | 0.00061781 | 0.0021794 | 12 | 1 | 0 | 1 | 0 | 0 | 0 | 0.33278 | 0.00028581 |
| pwm | 4.2068 | 3.0134 | 0.35049 | 0.50102 | 1.8 | 0.15213 | 5.405 | n/a | 0 | 1.6515 | n/a | n/a | 0.0021988 | 0.0074914 | 11.975 | 0.93827 | 0.0012093 | 0 | 3.245 | 0.27042 | 27.042 | 0.99834 | 0.0022117 |
| pid | 4.1713 | 3.0044 | 0.34752 | 0.50027 | 1.8 | 0.14085 | 5.41 | n/a | 0 | 1.6594 | n/a | n/a | 0.00061999 | 0.0021871 | 11.998 | 0.99992 | 1.849e-07 | 0 | 9.98 | 0.83167 | 83.167 | 0.33278 | 0.00028681 |

Graph: `plots/heavy_load_comparison.png`

### saturation

Requests 115% of the configured maximum belt speed.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 26.579 | 61.599 | 2.2148 | 2.2655 | 4.14 | 1.9411 | n/a | n/a | 0 | 2.1993 | n/a | n/a | 0.00081878 | 0.0028883 | 12 | 1 | 0 | 1 | 0 | 0 | 0 | 0.33278 | 0.00037879 |
| pwm | 26.579 | 61.599 | 2.2148 | 2.2655 | 4.14 | 1.9411 | n/a | n/a | 0 | 2.1993 | n/a | n/a | 0.00081878 | 0.0028883 | 12 | 1 | 0 | 0 | 12 | 1 | 100 | 0.33278 | 0.00037879 |
| pid | 26.579 | 61.599 | 2.2148 | 2.2655 | 4.14 | 1.9411 | n/a | n/a | 0 | 2.1993 | n/a | n/a | 0.00081878 | 0.0028883 | 12 | 1 | 0 | 0 | 12 | 1 | 100 | 0.33278 | 0.00037879 |

Graph: `plots/saturation_comparison.png`

### changing_reference

Target steps through 55%, 100%, then 70% of nominal.

| Controller | IAE_m_s_s | ISE_m2_s | MAE_m_s | RMSE_m_s | maximum_absolute_error_m_s | steady_state_error_m_s | rise_time_s | settling_time_s | overshoot_percent | peak_speed_m_s | speed_drop_on_load_m_s | recovery_time_after_load_s | speed_ripple_rms_m_s | speed_ripple_peak_to_peak_m_s | control_effort | mean_pwm_duty | pwm_duty_ripple_rms | bang_bang_switch_count | saturation_time_s | saturation_fraction | saturation_percent | dominant_ripple_frequency_hz | dominant_ripple_amplitude_m_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bang-bang | 1.6966 | 0.67014 | 0.14133 | 0.23627 | 0.99 | 0.0023204 | 0.71 | 11.99 | 6.6911 | 1.9204 | n/a | n/a | 0.069698 | 0.24359 | 7.855 | 0.65473 | 0.49718 | 25 | 0 | 0 | 0 | 1.3311 | 0.087632 |
| pwm | 1.827 | 0.68138 | 0.15221 | 0.23824 | 0.99 | -0.080316 | 1.21 | n/a | 2.0996 | 1.8378 | n/a | n/a | 0.01581 | 0.068965 | 8.125 | 0.62659 | 0.0086958 | 0 | 0.67 | 0.055833 | 5.5833 | 0.33278 | 0.022499 |
| pid | 1.8349 | 0.79517 | 0.15286 | 0.25736 | 0.99 | 0.027459 | 1.345 | 11.98 | 2.0571 | 1.837 | n/a | n/a | 0.036429 | 0.13422 | 5.6124 | 0.65767 | 0.027379 | 0 | 0.07 | 0.0058333 | 0.58333 | 0.33278 | 0.047701 |

Graph: `plots/changing_reference_comparison.png`

## Linear stability check

The reduced plant pole is -0.66667 rad/s (stable: True).
The ideal continuous PID linearization has poles -1.216+0j, -10.192+0j rad/s (stable: True). Continuous ideal PID linearization; saturation, PWM switching and derivative filter omitted.

## Frequency response

The plant and ideal unsaturated PID closed-loop Bode traces are in `plots/frequency_response.png`; samples are in `csv/frequency_response.csv`. Baseline speed FFT is in `plots/frequency_spectrum.png`; PWM carrier test is in `plots/pwm_frequency_sweep.png`.

## PWM carrier-frequency sweep

| Carrier (Hz) | Speed ripple RMS (m/s) | Speed ripple p-p (m/s) | Dominant ripple (Hz) |
|---:|---:|---:|---:|
| 5 | 0.0125973 | 0.0493167 | 4.9917 |
| 10 | 0.00641552 | 0.0264152 | 9.9834 |
| 20 | 0.00370715 | 0.0170906 | 19.967 |
| 40 | 0.00271534 | 0.0110183 | 6.6556 |
Lowest measured peak-to-peak ripple in this sweep: 40 Hz.

## Interpretation

## baseline
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.1880, pwm=0.1641, pid=0.1500; lowest observed: pid (0.1500)
- RMSE: bang-bang=0.3885, pwm=0.3845, pid=0.3844; lowest observed: pid (0.3844)
- rise time s: bang-bang=1.870, pwm=1.870, pid=1.915; lowest observed: bang-bang (1.870)
- overshoot %: bang-bang=6.70, pwm=2.11, pid=2.23; lowest observed: pwm (2.11)
- settling s: bang-bang=n/a, pwm=11.945, pid=4.435; lowest observed: pid (4.435)
- load recovery s: no finite result
- effort: bang-bang=10.3600, pwm=10.4050, pid=8.9078; lowest observed: pid (8.9078)
- saturation %: bang-bang=0.00, pwm=13.79, pid=3.54; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.24572, pwm=0.01709, pid=0.00054; lowest observed: pid (0.00054)
- dominant ripple Hz: bang-bang=0.998, pwm=19.967, pid=0.333; lowest observed: pid (0.333)

## load_step
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.1884, pwm=0.2098, pid=0.1612; lowest observed: pid (0.1612)
- RMSE: bang-bang=0.3879, pwm=0.3936, pid=0.3852; lowest observed: pid (0.3852)
- rise time s: bang-bang=1.870, pwm=1.870, pid=1.915; lowest observed: bang-bang (1.870)
- overshoot %: bang-bang=6.70, pwm=2.10, pid=2.23; lowest observed: pwm (2.10)
- settling s: bang-bang=n/a, pwm=n/a, pid=7.350; lowest observed: pid (7.350)
- load recovery s: bang-bang=n/a, pwm=n/a, pid=2.545; lowest observed: pid (2.545)
- effort: bang-bang=11.5900, pwm=10.9000, pid=10.6205; lowest observed: pid (10.6205)
- saturation %: bang-bang=0.00, pwm=13.79, pid=3.54; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.00302, pwm=0.01196, pid=0.00145; lowest observed: pid (0.00145)
- dominant ripple Hz: bang-bang=0.333, pwm=19.967, pid=0.333; lowest observed: bang-bang (0.333)

## load_cycle
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.1815, pwm=0.2023, pid=0.1864; lowest observed: bang-bang (0.1815)
- RMSE: bang-bang=0.3872, pwm=0.3932, pid=0.3883; lowest observed: bang-bang (0.3872)
- rise time s: bang-bang=1.870, pwm=1.870, pid=1.915; lowest observed: bang-bang (1.870)
- overshoot %: bang-bang=6.70, pwm=2.10, pid=3.06; lowest observed: pwm (2.10)
- settling s: bang-bang=n/a, pwm=9.370, pid=10.875; lowest observed: pwm (9.370)
- load recovery s: bang-bang=0.765, pwm=5.170, pid=0.010; lowest observed: pid (0.010)
- effort: bang-bang=11.4050, pwm=11.1000, pid=10.5768; lowest observed: pid (10.5768)
- saturation %: bang-bang=0.00, pwm=13.79, pid=26.25; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.24558, pwm=0.05561, pid=0.04317; lowest observed: pid (0.04317)
- dominant ripple Hz: bang-bang=0.666, pwm=0.333, pid=0.333; lowest observed: pwm (0.333)

## light_load
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.1983, pwm=0.1749, pid=0.1600; lowest observed: pid (0.1600)
- RMSE: bang-bang=0.4015, pwm=0.3979, pid=0.3975; lowest observed: pid (0.3975)
- rise time s: bang-bang=2.080, pwm=2.080, pid=2.105; lowest observed: bang-bang (2.080)
- overshoot %: bang-bang=6.70, pwm=1.92, pid=1.66; lowest observed: pid (1.66)
- settling s: bang-bang=n/a, pwm=3.365, pid=2.955; lowest observed: pid (2.955)
- load recovery s: no finite result
- effort: bang-bang=10.8600, pwm=10.8600, pid=9.7330; lowest observed: pid (9.7330)
- saturation %: bang-bang=0.00, pwm=15.04, pid=4.50; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.24132, pwm=0.01433, pid=0.00048; lowest observed: pid (0.00048)
- dominant ripple Hz: bang-bang=0.666, pwm=19.967, pid=0.333; lowest observed: pid (0.333)

## medium_load
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.2326, pwm=0.2884, pid=0.1951; lowest observed: pid (0.1951)
- RMSE: bang-bang=0.4357, pwm=0.4481, pid=0.4326; lowest observed: pid (0.4326)
- rise time s: bang-bang=2.800, pwm=2.800, pid=2.805; lowest observed: bang-bang (2.800)
- overshoot %: bang-bang=4.52, pwm=0.00, pid=0.46; lowest observed: pwm (0.00)
- settling s: bang-bang=n/a, pwm=n/a, pid=4.200; lowest observed: pid (4.200)
- load recovery s: no finite result
- effort: bang-bang=12.0000, pwm=11.1100, pid=11.4724; lowest observed: pwm (11.1100)
- saturation %: bang-bang=0.00, pwm=18.96, pid=9.92; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.00247, pwm=0.00676, pid=0.00023; lowest observed: pid (0.00023)
- dominant ripple Hz: bang-bang=0.333, pwm=19.967, pid=0.333; lowest observed: bang-bang (0.333)

## heavy_load
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.3473, pwm=0.3505, pid=0.3475; lowest observed: bang-bang (0.3473)
- RMSE: bang-bang=0.5001, pwm=0.5010, pid=0.5003; lowest observed: bang-bang (0.5001)
- rise time s: bang-bang=5.405, pwm=5.405, pid=5.410; lowest observed: bang-bang (5.405)
- overshoot %: bang-bang=0.00, pwm=0.00, pid=0.00; lowest observed: bang-bang (0.00)
- settling s: no finite result
- load recovery s: no finite result
- effort: bang-bang=12.0000, pwm=11.9750, pid=11.9981; lowest observed: pwm (11.9750)
- saturation %: bang-bang=0.00, pwm=27.04, pid=83.17; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.00218, pwm=0.00749, pid=0.00219; lowest observed: bang-bang (0.00218)
- dominant ripple Hz: bang-bang=0.333, pwm=0.998, pid=0.333; lowest observed: bang-bang (0.333)

## saturation
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=2.2148, pwm=2.2148, pid=2.2148; lowest observed: bang-bang (2.2148)
- RMSE: bang-bang=2.2655, pwm=2.2655, pid=2.2655; lowest observed: bang-bang (2.2655)
- rise time s: no finite result
- overshoot %: bang-bang=0.00, pwm=0.00, pid=0.00; lowest observed: bang-bang (0.00)
- settling s: no finite result
- load recovery s: no finite result
- effort: bang-bang=12.0000, pwm=12.0000, pid=12.0000; lowest observed: bang-bang (12.0000)
- saturation %: bang-bang=0.00, pwm=100.00, pid=100.00; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.00289, pwm=0.00289, pid=0.00289; lowest observed: bang-bang (0.00289)
- dominant ripple Hz: bang-bang=0.333, pwm=0.333, pid=0.333; lowest observed: bang-bang (0.333)

## changing_reference
Evidence-based comparison (computed from these run files):
- MAE: bang-bang=0.1413, pwm=0.1522, pid=0.1529; lowest observed: bang-bang (0.1413)
- RMSE: bang-bang=0.2363, pwm=0.2382, pid=0.2574; lowest observed: bang-bang (0.2363)
- rise time s: bang-bang=0.710, pwm=1.210, pid=1.345; lowest observed: bang-bang (0.710)
- overshoot %: bang-bang=6.69, pwm=2.10, pid=2.06; lowest observed: pid (2.06)
- settling s: bang-bang=11.990, pwm=n/a, pid=11.980; lowest observed: pid (11.980)
- load recovery s: no finite result
- effort: bang-bang=7.8550, pwm=8.1250, pid=5.6124; lowest observed: pid (5.6124)
- saturation %: bang-bang=0.00, pwm=5.58, pid=0.58; lowest observed: bang-bang (0.00)
- speed ripple p-p m/s: bang-bang=0.24359, pwm=0.06897, pid=0.13422; lowest observed: pwm (0.06897)
- dominant ripple Hz: bang-bang=1.331, pwm=0.333, pid=0.333; lowest observed: pwm (0.333)


## Limits

The plant is reduced-order. PyBullet is a synchronized visualization, not coupled co-simulation. Frequency response describes the linearized plant and is not a nonlinear closed-loop stability proof.