using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace TempControlMonitor.Monitoring;

// GPU 使用率优先使用 Windows 的 "GPU Engine" 性能计数器：
// - 数据来源与任务管理器的 GPU 图表一致，通常比硬件库里的 Load 更接近“占用率”概念。
// - 兼容 NVIDIA/AMD/Intel，前提是系统支持该计数器（Win10+ 一般可用）。
// - 取所有 3D 引擎实例的 Utilization Percentage 求和，并限制到 0..100。
public sealed class GpuUsageReader : IDisposable
{
    private readonly List<PerformanceCounter> _counters = new();
    private bool _available;
    private bool _warmedUp;

    public bool IsAvailable => _available;

    public GpuUsageReader()
    {
        try
        {
            var category = new PerformanceCounterCategory("GPU Engine");
            var instances = category.GetInstanceNames();

            foreach (var inst in instances)
            {
                // 只统计 3D 引擎更贴近“核心占用率”（类似任务管理器的 3D）
                if (!inst.Contains("engtype_3D", StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }

                _counters.Add(new PerformanceCounter("GPU Engine", "Utilization Percentage", inst, readOnly: true));
            }

            _available = _counters.Count > 0;
        }
        catch
        {
            _available = false;
        }
    }

    public void WarmUp()
    {
        if (!_available)
        {
            return;
        }

        foreach (var c in _counters)
        {
            _ = c.NextValue();
        }

        _warmedUp = true;
    }

    public float ReadTotal3DPercent()
    {
        if (!_available)
        {
            return 0f;
        }

        if (!_warmedUp)
        {
            WarmUp();
        }

        var sum = 0f;
        foreach (var c in _counters)
        {
            var v = c.NextValue();
            if (float.IsNaN(v) || float.IsInfinity(v) || v < 0)
            {
                continue;
            }

            sum += v;
        }

        if (sum < 0f) return 0f;
        if (sum > 100f) return 100f;
        return sum;
    }

    public void Dispose()
    {
        foreach (var c in _counters)
        {
            c.Dispose();
        }
    }
}

