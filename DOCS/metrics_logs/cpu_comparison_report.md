# CPU Comparison Report

CPU usage was computed from `sar -u` logs using:

- `CPU_active_percent = 100 - %idle`
## Compute Platform

The rover used a distributed compute architecture with onboard and workstation-side processing.

### Onboard compute
- **Device:** Raspberry Pi 5
- **RAM:** 8 GB
- **Operating System:** Ubuntu
- **Kernel:** Linux 6.8.0-1048-raspi
- **Architecture:** `aarch64`
- **Logical CPUs:** 4

### Workstation compute
- **Device:** Intel NUC11PAHi7
- **CPU:** 11th Gen Intel Core i7-1165G7
- **RAM:** 24.0 GiB
- **Operating System:** Ubuntu
- **Kernel:** Linux 6.8.0-111-generic
- **Architecture:** `x86_64`
- **Logical CPUs:** 8
  
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
