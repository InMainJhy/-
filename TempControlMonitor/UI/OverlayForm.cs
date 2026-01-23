using System;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using TempControlMonitor.Hardware;
using TempControlMonitor.Settings;

namespace TempControlMonitor.UI;

public sealed class OverlayForm : Form
{
    private static readonly Color TransparencyColor = Color.Magenta;

    private readonly HardwareMonitorService _monitor;
    private readonly SettingsStore _settingsStore;
    private readonly AppSettings _settings;
    private readonly OverlayTextView _view;
    private readonly System.Windows.Forms.Timer _timer;
    private readonly NotifyIcon _notifyIcon;
    private readonly ContextMenuStrip _menu;

    private bool _exiting;
    private bool _paused;

    public OverlayForm(HardwareMonitorService monitor, SettingsStore settingsStore, AppSettings settings)
    {
        _monitor = monitor;
        _settingsStore = settingsStore;
        _settings = settings;

        StartPosition = FormStartPosition.Manual;
        if (_settings.WindowX.HasValue && _settings.WindowY.HasValue)
        {
            Location = new Point(_settings.WindowX.Value, _settings.WindowY.Value);
        }
        TopMost = _settings.AlwaysOnTop;
        Opacity = 1.0;
        FormBorderStyle = FormBorderStyle.None;
        ShowInTaskbar = false;
        BackColor = TransparencyColor;
        TransparencyKey = TransparencyColor;

        _view = new OverlayTextView
        {
            BackColor = TransparencyColor,
            Font = new Font("Segoe UI", _settings.FontSize, FontStyle.Regular),
            PaddingPx = _settings.PaddingPx,
            Text = "初始化中..."
        };
        _view.TextColor = Color.FromArgb(_settings.TextColorArgb);

        Controls.Add(_view);

        _menu = BuildMenu();
        ContextMenuStrip = _menu;

        _notifyIcon = new NotifyIcon
        {
            Icon = SystemIcons.Application,
            Visible = true,
            Text = "温控监控",
            ContextMenuStrip = _menu
        };
        _notifyIcon.DoubleClick += (_, _) => ToggleVisible();

        _timer = new System.Windows.Forms.Timer();
        _timer.Tick += (_, _) => RefreshSnapshot();
        ApplyPollInterval();

        MouseDown += (_, e) =>
        {
            if (e.Button == MouseButtons.Left)
            {
                BeginDrag();
            }
        };
        MouseUp += (_, e) =>
        {
            if (e.Button == MouseButtons.Left)
            {
                SaveWindowLocation();
            }
        };
        _view.MouseDown += (_, e) =>
        {
            if (e.Button == MouseButtons.Left)
            {
                BeginDrag();
            }
        };
        _view.MouseUp += (_, e) =>
        {
            if (e.Button == MouseButtons.Left)
            {
                SaveWindowLocation();
            }
        };

        Load += (_, _) =>
        {
            if (_settings.StartVisible)
            {
                Show();
            }
            else
            {
                Hide();
            }

            RefreshSnapshot();
            UpdatePollingState();
        };

        VisibleChanged += (_, _) => UpdatePollingState();
    }

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        if (!_exiting)
        {
            e.Cancel = true;
            Hide();
            return;
        }

        base.OnFormClosing(e);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _timer.Dispose();
            _notifyIcon.Dispose();
            _menu.Dispose();
            _view.Dispose();
        }

        base.Dispose(disposing);
    }

    private void RefreshSnapshot()
    {
        if (_paused)
        {
            return;
        }

        HardwareSnapshot snap;
        try
        {
            snap = _monitor.ReadSnapshot();
        }
        catch (Exception ex)
        {
            _view.Text = $"读取失败\n{ex.GetType().Name}: {ex.Message}";
            FitToContent();
            return;
        }

        _view.Text = SnapshotFormatter.Format(snap, _settings);
        FitToContent();
    }

    private void ToggleVisible()
    {
        if (Visible)
        {
            Hide();
        }
        else
        {
            Show();
            Activate();
        }
    }

    private void UpdatePollingState()
    {
        if (!Visible || _paused)
        {
            _timer.Stop();
            return;
        }

        if (!_timer.Enabled)
        {
            _timer.Start();
        }
    }

    private void ApplyPollInterval()
    {
        var interval = Math.Clamp(_settings.PollIntervalMs, 200, 10_000);
        _timer.Interval = interval;
        _settings.PollIntervalMs = interval;
        SaveSettings();
    }

    private void SaveSettings()
    {
        _settingsStore.Save(_settings);
    }

    private void SaveWindowLocation()
    {
        _settings.WindowX = Left;
        _settings.WindowY = Top;
        SaveSettings();
    }

    private void FitToContent()
    {
        var size = _view.MeasurePreferredSize();
        _view.Size = size;
        ClientSize = size;
    }

    private ContextMenuStrip BuildMenu()
    {
        var menu = new ContextMenuStrip();

        var showHide = new ToolStripMenuItem("显示/隐藏");
        showHide.Click += (_, _) => ToggleVisible();
        menu.Items.Add(showHide);

        var pause = new ToolStripMenuItem("暂停采集") { Checked = _paused, CheckOnClick = true };
        pause.CheckedChanged += (_, _) =>
        {
            _paused = pause.Checked;
            pause.Text = _paused ? "继续采集" : "暂停采集";
            if (_paused)
            {
                _timer.Stop();
                _view.Text = "已暂停";
                FitToContent();
            }
            else
            {
                RefreshSnapshot();
                UpdatePollingState();
            }
        };
        pause.Text = _paused ? "继续采集" : "暂停采集";
        menu.Items.Add(pause);

        var showCpu = new ToolStripMenuItem("显示CPU") { Checked = _settings.ShowCpu, CheckOnClick = true };
        showCpu.CheckedChanged += (_, _) =>
        {
            _settings.ShowCpu = showCpu.Checked;
            SaveSettings();
            RefreshSnapshot();
        };
        menu.Items.Add(showCpu);

        var showGpu = new ToolStripMenuItem("显示GPU") { Checked = _settings.ShowGpu, CheckOnClick = true };
        showGpu.CheckedChanged += (_, _) =>
        {
            _settings.ShowGpu = showGpu.Checked;
            SaveSettings();
            RefreshSnapshot();
        };
        menu.Items.Add(showGpu);

        var gpuDetails = new ToolStripMenuItem("GPU明细");

        var gpuVram = new ToolStripMenuItem("显存(VRAM)") { Checked = _settings.ShowGpuVram, CheckOnClick = true };
        gpuVram.CheckedChanged += (_, _) =>
        {
            _settings.ShowGpuVram = gpuVram.Checked;
            SaveSettings();
            RefreshSnapshot();
        };
        gpuDetails.DropDownItems.Add(gpuVram);

        var gpuPower = new ToolStripMenuItem("功耗(PWR)") { Checked = _settings.ShowGpuPower, CheckOnClick = true };
        gpuPower.CheckedChanged += (_, _) =>
        {
            _settings.ShowGpuPower = gpuPower.Checked;
            SaveSettings();
            RefreshSnapshot();
        };
        gpuDetails.DropDownItems.Add(gpuPower);

        var gpuFan = new ToolStripMenuItem("风扇(FAN)") { Checked = _settings.ShowGpuFan, CheckOnClick = true };
        gpuFan.CheckedChanged += (_, _) =>
        {
            _settings.ShowGpuFan = gpuFan.Checked;
            SaveSettings();
            RefreshSnapshot();
        };
        gpuDetails.DropDownItems.Add(gpuFan);

        menu.Items.Add(gpuDetails);

        var topMost = new ToolStripMenuItem("置顶显示") { Checked = _settings.AlwaysOnTop, CheckOnClick = true };
        topMost.CheckedChanged += (_, _) =>
        {
            _settings.AlwaysOnTop = topMost.Checked;
            TopMost = _settings.AlwaysOnTop;
            SaveSettings();
        };
        menu.Items.Add(topMost);

        var textColor = new ToolStripMenuItem("文字颜色...");
        textColor.Click += (_, _) =>
        {
            using var dialog = new ColorDialog
            {
                FullOpen = true,
                Color = _view.TextColor
            };
            if (dialog.ShowDialog(this) == DialogResult.OK)
            {
                _settings.TextColorArgb = dialog.Color.ToArgb();
                _view.TextColor = dialog.Color;
                SaveSettings();
                _view.Invalidate();
            }
        };
        menu.Items.Add(textColor);

        var fontMenu = new ToolStripMenuItem("字体大小");
        AddFontItem(fontMenu, "12", 12f);
        AddFontItem(fontMenu, "14", 14f);
        AddFontItem(fontMenu, "16", 16f);
        AddFontItem(fontMenu, "18", 18f);
        AddFontItem(fontMenu, "20", 20f);
        menu.Items.Add(fontMenu);

        var intervalMenu = new ToolStripMenuItem("刷新频率");
        AddIntervalItem(intervalMenu, "200ms", 200);
        AddIntervalItem(intervalMenu, "500ms", 500);
        AddIntervalItem(intervalMenu, "1000ms", 1000);
        AddIntervalItem(intervalMenu, "2000ms", 2000);
        AddIntervalItem(intervalMenu, "5000ms", 5000);
        menu.Items.Add(intervalMenu);

        menu.Items.Add(new ToolStripSeparator());

        var exit = new ToolStripMenuItem("退出");
        exit.Click += (_, _) =>
        {
            _exiting = true;
            Close();
            Application.Exit();
        };
        menu.Items.Add(exit);

        return menu;
    }

    private void AddFontItem(ToolStripMenuItem parent, string label, float fontSize)
    {
        var item = new ToolStripMenuItem(label) { Checked = Math.Abs(_settings.FontSize - fontSize) < 0.1 };
        item.Click += (_, _) =>
        {
            _settings.FontSize = fontSize;
            _view.Font = new Font(_view.Font.FontFamily, _settings.FontSize, FontStyle.Regular);
            SaveSettings();
            FitToContent();
            foreach (ToolStripMenuItem sibling in parent.DropDownItems)
            {
                sibling.Checked = ReferenceEquals(sibling, item);
            }
        };
        parent.DropDownItems.Add(item);
    }

    private void AddIntervalItem(ToolStripMenuItem parent, string label, int intervalMs)
    {
        var item = new ToolStripMenuItem(label) { Checked = _settings.PollIntervalMs == intervalMs };
        item.Click += (_, _) =>
        {
            _settings.PollIntervalMs = intervalMs;
            ApplyPollInterval();
            foreach (ToolStripMenuItem sibling in parent.DropDownItems)
            {
                sibling.Checked = ReferenceEquals(sibling, item);
            }
        };
        parent.DropDownItems.Add(item);
    }

    private void BeginDrag()
    {
        ReleaseCapture();
        SendMessage(Handle, WM_NCLBUTTONDOWN, HTCAPTION, 0);
    }

    private const int WM_NCLBUTTONDOWN = 0xA1;
    private const int HTCAPTION = 0x2;

    [DllImport("user32.dll")]
    private static extern bool ReleaseCapture();

    [DllImport("user32.dll")]
    private static extern IntPtr SendMessage(IntPtr hWnd, int msg, int wParam, int lParam);

    private static class SnapshotFormatter
    {
        public static string Format(HardwareSnapshot s, AppSettings settings)
        {
            var cpuTemp = FormatNumber(s.CpuTemperatureC, "°C");
            var cpuLoad = FormatNumber(s.CpuLoadPercent, "%");

            var gpuTemp = FormatNumber(s.GpuTemperatureC, "°C");
            var gpuLoad = FormatNumber(s.GpuLoadPercent, "%");
            var gpuMem = s.GpuMemoryUsedMB.HasValue ? $"{s.GpuMemoryUsedMB.Value:0}MB" : "-";
            var gpuPower = FormatNumber(s.GpuPowerW, "W");
            var gpuFan = FormatNumber(s.GpuFanRpm, "RPM");

            var lines = new System.Collections.Generic.List<string>(2);
            if (settings.ShowCpu)
            {
                lines.Add($"CPU  {cpuTemp}  {cpuLoad}");
            }
            if (settings.ShowGpu)
            {
                var segments = new System.Collections.Generic.List<string>
                {
                    $"GPU  {gpuTemp}  {gpuLoad}"
                };
                if (settings.ShowGpuVram)
                {
                    segments.Add($"VRAM {gpuMem}");
                }
                if (settings.ShowGpuPower)
                {
                    segments.Add($"PWR {gpuPower}");
                }
                if (settings.ShowGpuFan)
                {
                    segments.Add($"FAN {gpuFan}");
                }
                lines.Add(string.Join("  ", segments));
            }
            if (lines.Count == 0)
            {
                lines.Add("已隐藏CPU/GPU显示");
            }

            return string.Join(Environment.NewLine, lines);
        }

        private static string FormatNumber(double? value, string suffix)
        {
            if (!value.HasValue || double.IsNaN(value.Value) || double.IsInfinity(value.Value))
            {
                return "-";
            }

            return $"{value.Value:0.#}{suffix}";
        }
    }
}

