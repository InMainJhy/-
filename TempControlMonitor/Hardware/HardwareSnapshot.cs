using System;
using System.Collections.Generic;

namespace TempControlMonitor.Hardware;

public sealed class HardwareSnapshot
{
    public required DateTimeOffset CapturedAt { get; init; }

    public double? CpuTemperatureC { get; init; }
    public double? CpuLoadPercent { get; init; }
    public double? CpuClockMHz { get; init; }

    public double? GpuTemperatureC { get; init; }
    public double? GpuLoadPercent { get; init; }
    public double? GpuMemoryUsedMB { get; init; }
    public double? GpuPowerW { get; init; }
    public double? GpuFanRpm { get; init; }

    public double? CpuFanRpm { get; init; }

    public List<string> Notes { get; init; } = new();
}
