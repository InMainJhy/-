using System;
using System.Collections.Generic;
using System.Linq;
using LibreHardwareMonitor.Hardware;

namespace TempControlMonitor.Hardware;

public sealed class HardwareMonitorService : IDisposable
{
    private readonly Computer _computer;
    private readonly object _gate = new();
    private bool _opened;

    public HardwareMonitorService()
    {
        _computer = new Computer
        {
            IsCpuEnabled = true,
            IsGpuEnabled = true,
            IsMotherboardEnabled = false,
            IsControllerEnabled = false
        };
    }

    public void Open()
    {
        lock (_gate)
        {
            if (_opened)
            {
                return;
            }

            _computer.Open();
            _opened = true;
        }
    }

    public HardwareSnapshot ReadSnapshot()
    {
        lock (_gate)
        {
            if (!_opened)
            {
                Open();
            }

            _computer.Accept(new UpdateVisitor());

            var cpu = _computer.Hardware.Where(h => h.HardwareType == HardwareType.Cpu).ToList();
            var gpus = _computer.Hardware.Where(h =>
                    h.HardwareType == HardwareType.GpuNvidia ||
                    h.HardwareType == HardwareType.GpuAmd ||
                    h.HardwareType == HardwareType.GpuIntel)
                .ToList();

            var cpuSensors = cpu.SelectMany(EnumerateSensors).ToList();
            var gpu = PickPrimaryGpu(gpus);
            var gpuSensors = gpu is null ? new List<ISensor>() : EnumerateSensors(gpu).ToList();
            var allSensors = _computer.Hardware.SelectMany(EnumerateSensors).ToList();

            var snapshot = new HardwareSnapshot
            {
                CapturedAt = DateTimeOffset.Now,
                CpuTemperatureC = PickCpuTemperatureC(cpuSensors, allSensors),
                CpuLoadPercent = PickCpuLoadPercent(cpuSensors),
                CpuClockMHz = PickCpuClockMHz(cpuSensors),
                GpuTemperatureC = PickTemperatureC(gpuSensors),
                GpuLoadPercent = PickGpuLoadPercent(gpuSensors),
                GpuMemoryUsedMB = PickGpuMemoryUsedMB(gpuSensors),
                GpuPowerW = PickGpuPowerW(gpuSensors),
                GpuFanRpm = PickFanRpm(gpuSensors),
                CpuFanRpm = PickCpuFanRpm(allSensors)
            };

            if (!snapshot.CpuTemperatureC.HasValue)
            {
                snapshot.Notes.Add("CPU温度不可用");
            }

            if (!snapshot.GpuTemperatureC.HasValue && gpu is not null)
            {
                snapshot.Notes.Add("GPU温度不可用");
            }

            if (gpu is null)
            {
                snapshot.Notes.Add("未检测到GPU");
            }

            return snapshot;
        }
    }

    public void Dispose()
    {
        lock (_gate)
        {
            if (_opened)
            {
                _computer.Close();
                _opened = false;
            }
        }
    }

    private static IHardware? PickPrimaryGpu(List<IHardware> gpus)
    {
        if (gpus.Count == 0)
        {
            return null;
        }

        return gpus
            .OrderByDescending(g => EnumerateSensors(g).Count(s => s.Value.HasValue))
            .FirstOrDefault();
    }

    private static IEnumerable<ISensor> EnumerateSensors(IHardware hardware)
    {
        foreach (var sensor in hardware.Sensors)
        {
            yield return sensor;
        }

        foreach (var sub in hardware.SubHardware)
        {
            foreach (var sensor in EnumerateSensors(sub))
            {
                yield return sensor;
            }
        }
    }

    private static double? PickTemperatureC(List<ISensor> sensors)
    {
        var package = sensors
            .Where(s => s.SensorType == SensorType.Temperature && s.Value.HasValue)
            .OrderByDescending(s => s.Name.Contains("Package", StringComparison.OrdinalIgnoreCase))
            .ThenByDescending(s => s.Name.Contains("Core", StringComparison.OrdinalIgnoreCase))
            .FirstOrDefault();

        if (package?.Value is not null)
        {
            return package.Value.Value;
        }

        var max = sensors
            .Where(s => s.SensorType == SensorType.Temperature && s.Value.HasValue)
            .Select(s => (double?)s.Value!.Value)
            .Max();

        return max;
    }

    private static double? PickCpuTemperatureC(List<ISensor> cpuSensors, List<ISensor> allSensors)
    {
        var direct = PickTemperatureC(cpuSensors);
        if (direct.HasValue)
        {
            return direct;
        }

        var temps = allSensors
            .Where(s => s.SensorType == SensorType.Temperature && s.Value.HasValue)
            .ToList();

        var preferred = temps
            .Where(s =>
                ContainsAny(s.Name, "CPU", "Tctl", "Tdie", "Package", "Core Max", "Processor"))
            .OrderByDescending(s => ContainsAny(s.Name, "Tctl", "Tdie"))
            .ThenByDescending(s => s.Name.Contains("Package", StringComparison.OrdinalIgnoreCase))
            .ThenByDescending(s => s.Name.Contains("CPU", StringComparison.OrdinalIgnoreCase))
            .ThenByDescending(s => s.Name.Contains("Core", StringComparison.OrdinalIgnoreCase))
            .FirstOrDefault();

        if (preferred?.Value is not null)
        {
            return preferred.Value.Value;
        }

        return temps.Select(s => (double?)s.Value!.Value).Max();
    }

    private static bool ContainsAny(string? text, params string[] tokens)
    {
        if (string.IsNullOrWhiteSpace(text))
        {
            return false;
        }

        foreach (var t in tokens)
        {
            if (text.Contains(t, StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }
        }

        return false;
    }

    private static double? PickCpuLoadPercent(List<ISensor> sensors)
    {
        var total = sensors.FirstOrDefault(s =>
            s.SensorType == SensorType.Load &&
            s.Value.HasValue &&
            (s.Name.Equals("CPU Total", StringComparison.OrdinalIgnoreCase) ||
             s.Name.Contains("Total", StringComparison.OrdinalIgnoreCase)));

        if (total?.Value is not null)
        {
            return total.Value.Value;
        }

        var max = sensors
            .Where(s => s.SensorType == SensorType.Load && s.Value.HasValue)
            .Select(s => (double?)s.Value!.Value)
            .Max();

        return max;
    }

    private static double? PickCpuClockMHz(List<ISensor> sensors)
    {
        var coreClocks = sensors.Where(s =>
            s.SensorType == SensorType.Clock &&
            s.Value.HasValue &&
            s.Name.Contains("Core", StringComparison.OrdinalIgnoreCase));

        var values = coreClocks.Select(s => (double)s.Value!.Value).ToList();
        if (values.Count == 0)
        {
            return null;
        }

        return values.Average();
    }

    private static double? PickGpuLoadPercent(List<ISensor> sensors)
    {
        var core = sensors.FirstOrDefault(s =>
            s.SensorType == SensorType.Load &&
            s.Value.HasValue &&
            (s.Name.Contains("GPU Core", StringComparison.OrdinalIgnoreCase) ||
             s.Name.Contains("Core", StringComparison.OrdinalIgnoreCase)));

        if (core?.Value is not null)
        {
            return core.Value.Value;
        }

        var max = sensors
            .Where(s => s.SensorType == SensorType.Load && s.Value.HasValue)
            .Select(s => (double?)s.Value!.Value)
            .Max();

        return max;
    }

    private static double? PickGpuMemoryUsedMB(List<ISensor> sensors)
    {
        var mem = sensors.FirstOrDefault(s =>
            (s.SensorType == SensorType.SmallData || s.SensorType == SensorType.Data) &&
            s.Value.HasValue &&
            s.Name.Contains("Memory Used", StringComparison.OrdinalIgnoreCase));

        if (mem?.Value is not null)
        {
            return mem.Value.Value;
        }

        return null;
    }

    private static double? PickGpuPowerW(List<ISensor> sensors)
    {
        var power = sensors.FirstOrDefault(s =>
            s.SensorType == SensorType.Power &&
            s.Value.HasValue &&
            (s.Name.Contains("Package", StringComparison.OrdinalIgnoreCase) ||
             s.Name.Contains("GPU", StringComparison.OrdinalIgnoreCase)));

        if (power?.Value is not null)
        {
            return power.Value.Value;
        }

        return sensors
            .Where(s => s.SensorType == SensorType.Power && s.Value.HasValue)
            .Select(s => (double?)s.Value!.Value)
            .Max();
    }

    private static double? PickFanRpm(List<ISensor> sensors)
    {
        var fan = sensors.FirstOrDefault(s =>
            s.SensorType == SensorType.Fan &&
            s.Value.HasValue);

        if (fan?.Value is not null)
        {
            return fan.Value.Value;
        }

        return null;
    }

    private static double? PickCpuFanRpm(List<ISensor> sensors)
    {
        var cpuFan = sensors.FirstOrDefault(s =>
            s.SensorType == SensorType.Fan &&
            s.Value.HasValue &&
            (s.Name.Contains("CPU", StringComparison.OrdinalIgnoreCase) ||
             s.Name.Contains("Processor", StringComparison.OrdinalIgnoreCase)));

        if (cpuFan?.Value is not null)
        {
            return cpuFan.Value.Value;
        }

        return null;
    }

    private sealed class UpdateVisitor : IVisitor
    {
        public void VisitComputer(IComputer computer)
        {
            computer.Traverse(this);
        }

        public void VisitHardware(IHardware hardware)
        {
            hardware.Update();
            foreach (var sub in hardware.SubHardware)
            {
                sub.Accept(this);
            }
        }

        public void VisitSensor(ISensor sensor)
        {
        }

        public void VisitParameter(IParameter parameter)
        {
        }
    }
}
