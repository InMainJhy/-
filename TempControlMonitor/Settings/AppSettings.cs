using System;
using System.IO;
using System.Drawing;

namespace TempControlMonitor.Settings;

public sealed class AppSettings
{
    public int PollIntervalMs { get; set; } = 1000;
    public bool AlwaysOnTop { get; set; } = true;
    public double Opacity { get; set; } = 0.85;
    public float FontSize { get; set; } = 14f;
    public int PaddingPx { get; set; } = 10;
    public bool StartVisible { get; set; } = true;
    public int? WindowX { get; set; }
    public int? WindowY { get; set; }
    public bool ShowCpu { get; set; } = true;
    public bool ShowGpu { get; set; } = true;
    public bool ShowGpuVram { get; set; } = false;
    public bool ShowGpuPower { get; set; } = false;
    public bool ShowGpuFan { get; set; } = false;
    public int TextColorArgb { get; set; } = Color.White.ToArgb();

    public static string DefaultPath()
    {
        return Path.Combine(AppContext.BaseDirectory, "settings.json");
    }
}
