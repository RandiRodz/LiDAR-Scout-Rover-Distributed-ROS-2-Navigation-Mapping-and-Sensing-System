# CPU Comparison Report

CPU usage was computed from `sar -u` logs using:

- `CPU_active_percent = 100 - %idle`

## Summary Table

| Platform | Run | Avg CPU (%) | Peak CPU (%) | Duration (s) | Samples |
|---|---|---:|---:|---:|---:|
| NUC | Version 1 | 33.53 | 40.32 | 329.0 | 330 |
| NUC | Version 2 | 32.64 | 40.51 | 392.0 | 393 |
| NUC | Version 3 | 52.25 | 58.00 | 340.0 | 341 |
| NUC | Point-LIO Mapping + Camera | 20.64 | 31.61 | 377.0 | 378 |
| PI5 | Version 1 | 20.49 | 48.99 | 322.0 | 323 |
| PI5 | Version 2 | 20.38 | 52.02 | 322.0 | 323 |
| PI5 | Version 3 | 21.56 | 52.43 | 322.0 | 323 |
| PI5 | Point-LIO Mapping + Camera | 25.71 | 63.52 | 373.0 | 374 |

## Generated Figures

- `nuc_cpu_all_comparison.png`
- `nuc_cpu_v1_vs_v2.png`
- `nuc_cpu_v2_vs_v3.png`
- `nuc_cpu_v1_vs_v2_vs_v3.png`
- `pi5_cpu_all_comparison.png`
- `pi5_cpu_v1_vs_v2.png`
- `pi5_cpu_v2_vs_v3.png`
- `pi5_cpu_v1_vs_v2_vs_v3.png`

## Audit Files

- `cpu_summary.csv` contains the averages and peaks used in the bar charts.
- `per_run_csv/*.csv` contains the per-second CPU values used in the time-series plots.
- `*_plot_values.csv` contains the exact averages shown in each comparison figure.
