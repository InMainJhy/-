using System;
using System.Diagnostics;
using System.Threading;
using TempControlMonitor.Hardware;

namespace TempControlMonitor.Monitoring;

// 后台采集线程：
// - 严格按采样间隔循环（默认 100ms，可调 50/100/200）
// - 每次采集都会强制刷新硬件传感器（hardware.Update）
// - 采集结束后通过事件把数据推给 UI（UI 线程只做显示，不做采集）
public sealed class MonitorWorker : IDisposable
{
    private readonly HardwareMonitorService _hardware;
    private Thread? _thread;
    private volatile bool _running;
    private int _intervalMs;

    private CpuUsageReader? _cpuUsage;
    private GpuUsageReader? _gpuUsage;

    public event Action<MonitorSample>? Sampled;
    public event Action<string>? Error;

    public bool EnablePerCoreCpu { get; set; }

    public MonitorWorker(HardwareMonitorService hardware)
    {
        _hardware = hardware;
        _intervalMs = 100;
    }

    public void SetIntervalMs(int intervalMs)
    {
        _intervalMs = Math.Clamp(intervalMs, 20, 2000);
    }

    public void Start()
    {
        if (_running)
        {
            return;
        }

        _running = true;

        _cpuUsage?.Dispose();
        _gpuUsage?.Dispose();
        _cpuUsage = new CpuUsageReader(enablePerCore: EnablePerCoreCpu);
        _gpuUsage = new GpuUsageReader();

        _thread = new Thread(Loop)
        {
            IsBackground = true,
            Name = "MonitorWorker"
        };
        _thread.Start();
    }

    public void Stop()
    {
        _running = false;
    }

    public void Dispose()
    {
        Stop();
        _thread = null;
        _cpuUsage?.Dispose();
        _gpuUsage?.Dispose();
        _cpuUsage = null;
        _gpuUsage = null;
    }

    private void Loop()
    {
        var sw = Stopwatch.StartNew();
        var nextTick = sw.ElapsedMilliseconds;

        try
        {
            _hardware.Open();
            _cpuUsage?.WarmUp();
            _gpuUsage?.WarmUp();
        }
        catch (Exception ex)
        {
            Error?.Invoke($"初始化失败：{ex.Message}");
        }

        while (_running)
        {
            nextTick += _intervalMs;

            var sampleStopwatch = Stopwatch.StartNew();

            HardwareSnapshot snapshot;
            try
            {
                snapshot = _hardware.ReadSnapshot();
            }
            catch (Exception ex)
            {
                Error?.Invoke($"传感器读取失败：{ex.Message}");
                SleepUntil(sw, nextTick);
                continue;
            }

            var cpu = _cpuUsage?.Read();
            var gpuLoad = _gpuUsage?.IsAvailable == true ? _gpuUsage.ReadTotal3DPercent() : (float?)null;

            var sample = new MonitorSample(
                CapturedAt: DateTimeOffset.Now,
                CpuTempC: snapshot.CpuTemperatureC,
                CpuTotalPercent: cpu?.TotalPercent,
                CpuPerCorePercent: cpu?.PerCorePercent,
                GpuTempC: snapshot.GpuTemperatureC,
                GpuCorePercent: gpuLoad,
                SampleDurationMs: sampleStopwatch.ElapsedMilliseconds
            );

            Sampled?.Invoke(sample);

            SleepUntil(sw, nextTick);
        }
    }

    private static void SleepUntil(Stopwatch sw, long targetMs)
    {
        var remain = targetMs - sw.ElapsedMilliseconds;
        if (remain <= 0)
        {
            return;
        }

        // 50ms 这种较高频率下，Thread.Sleep 会有一定抖动；这里用“先 Sleep 再短暂自旋”提高节拍精度。
        if (remain > 2)
        {
            Thread.Sleep((int)(remain - 1));
        }

        while (sw.ElapsedMilliseconds < targetMs)
        {
            Thread.SpinWait(50);
        }
    }
}

public sealed record MonitorSample(
    DateTimeOffset CapturedAt,
    double? CpuTempC,
    float? CpuTotalPercent,
    float[]? CpuPerCorePercent,
    double? GpuTempC,
    float? GpuCorePercent,
    long SampleDurationMs
);

