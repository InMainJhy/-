using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace TempControlMonitor.Monitoring;

// 使用 Windows 性能计数器读取 CPU 使用率：
// - _Total：总占用率
// - 0..N-1：每核心占用率
// 这是和“任务管理器”相同的数据来源之一，通常比硬件传感器里的 Load 更贴近系统实际占用率。
public sealed class CpuUsageReader : IDisposable
{
    private readonly PerformanceCounter _totalCounter;
    private readonly List<PerformanceCounter> _coreCounters;
    private bool _warmedUp;

    public CpuUsageReader(bool enablePerCore)
    {
        _totalCounter = new PerformanceCounter("Processor", "% Processor Time", "_Total", readOnly: true);
        _coreCounters = new List<PerformanceCounter>();

        if (enablePerCore)
        {
            var coreCount = Math.Max(1, Environment.ProcessorCount);
            for (var i = 0; i < coreCount; i++)
            {
                _coreCounters.Add(new PerformanceCounter("Processor", "% Processor Time", i.ToString(), readOnly: true));
            }
        }
    }

    // 第一次读取往往会返回 0，需要先“预热”一次再开始取值。
    public void WarmUp()
    {
        _ = _totalCounter.NextValue();
        foreach (var c in _coreCounters)
        {
            _ = c.NextValue();
        }

        _warmedUp = true;
    }

    public CpuUsageSample Read()
    {
        if (!_warmedUp)
        {
            WarmUp();
        }

        var total = ClampPercent(_totalCounter.NextValue());

        float[]? cores = null;
        if (_coreCounters.Count > 0)
        {
            cores = new float[_coreCounters.Count];
            for (var i = 0; i < _coreCounters.Count; i++)
            {
                cores[i] = ClampPercent(_coreCounters[i].NextValue());
            }
        }

        return new CpuUsageSample(total, cores);
    }

    public void Dispose()
    {
        _totalCounter.Dispose();
        foreach (var c in _coreCounters)
        {
            c.Dispose();
        }
    }

    private static float ClampPercent(float v)
    {
        if (float.IsNaN(v) || float.IsInfinity(v))
        {
            return 0f;
        }

        if (v < 0f) return 0f;
        if (v > 100f) return 100f;
        return v;
    }
}

public readonly record struct CpuUsageSample(float TotalPercent, float[]? PerCorePercent);

