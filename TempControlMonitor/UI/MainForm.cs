using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using TempControlMonitor.Hardware;
using TempControlMonitor.Monitoring;

namespace TempControlMonitor.UI;

// 主界面（按你的要求：Label + ComboBox + Start/Stop）
// - 数值格式：温度 0.1℃，占用率 0.1%
// - 采样频率：50/100/200ms
public sealed class MainForm : Form
{
    private readonly HardwareMonitorService _hardware;
    private readonly MonitorWorker _worker;

    private readonly Label _cpuTempLabel;
    private readonly Label _cpuUsageLabel;
    private readonly Label _gpuTempLabel;
    private readonly Label _gpuUsageLabel;
    private readonly Label _statusLabel;
    private readonly Label _perCoreLabel;

    private readonly ComboBox _intervalComboBox;
    private readonly CheckBox _perCoreCheckBox;
    private readonly Button _startButton;
    private readonly Button _stopButton;

    public MainForm()
    {
        Text = ".NET 10 高精度CPU/GPU监控";
        StartPosition = FormStartPosition.CenterScreen;
        FormBorderStyle = FormBorderStyle.FixedDialog;
        MaximizeBox = false;
        MinimizeBox = true;
        ClientSize = new Size(480, 260);

        BackColor = Color.FromArgb(20, 20, 20);
        ForeColor = Color.White;
        Font = new Font("Segoe UI", 10f, FontStyle.Regular);

        _hardware = new HardwareMonitorService();
        _worker = new MonitorWorker(_hardware);
        _worker.Sampled += OnSampledFromWorkerThread;
        _worker.Error += OnErrorFromWorkerThread;

        var title = new Label
        {
            AutoSize = true,
            Text = "实时监控（温度/占用率）",
            Font = new Font("Segoe UI", 12f, FontStyle.Bold),
            Location = new Point(16, 14)
        };
        Controls.Add(title);

        _cpuTempLabel = CreateValueLabel("CPU温度：-.-℃", 16, 56);
        _cpuUsageLabel = CreateValueLabel("CPU占用率：-.-%", 16, 84);
        _gpuTempLabel = CreateValueLabel("GPU温度：-.-℃", 16, 120);
        _gpuUsageLabel = CreateValueLabel("GPU占用率：-.-%", 16, 148);
        Controls.AddRange(new Control[] { _cpuTempLabel, _cpuUsageLabel, _gpuTempLabel, _gpuUsageLabel });

        _perCoreLabel = new Label
        {
            AutoSize = false,
            Text = "",
            Location = new Point(16, 206),
            Size = new Size(448, 22),
            ForeColor = Color.Gainsboro,
            Visible = false
        };
        Controls.Add(_perCoreLabel);

        _perCoreCheckBox = new CheckBox
        {
            AutoSize = true,
            Text = "显示每核心占用率（可选）",
            Location = new Point(16, 182),
            ForeColor = Color.White
        };
        _perCoreCheckBox.CheckedChanged += (_, _) =>
        {
            _perCoreLabel.Visible = _perCoreCheckBox.Checked;
            _worker.EnablePerCoreCpu = _perCoreCheckBox.Checked;
        };
        Controls.Add(_perCoreCheckBox);

        var intervalLabel = new Label
        {
            AutoSize = true,
            Text = "采样间隔：",
            Location = new Point(280, 58),
            ForeColor = Color.White
        };
        Controls.Add(intervalLabel);

        _intervalComboBox = new ComboBox
        {
            DropDownStyle = ComboBoxStyle.DropDownList,
            Location = new Point(352, 54),
            Size = new Size(112, 26)
        };
        _intervalComboBox.Items.AddRange(new object[] { "50ms", "100ms", "200ms" });
        _intervalComboBox.SelectedIndex = 1; // 默认 100ms
        _intervalComboBox.SelectedIndexChanged += (_, _) => ApplyIntervalToWorker();
        Controls.Add(_intervalComboBox);

        _startButton = new Button
        {
            Text = "开始监控",
            Location = new Point(280, 92),
            Size = new Size(88, 30)
        };
        _startButton.Click += (_, _) =>
        {
            ApplyIntervalToWorker();
            _worker.Start();
            UpdateUiState(isRunning: true);
        };
        Controls.Add(_startButton);

        _stopButton = new Button
        {
            Text = "停止监控",
            Location = new Point(376, 92),
            Size = new Size(88, 30),
            Enabled = false
        };
        _stopButton.Click += (_, _) =>
        {
            _worker.Stop();
            UpdateUiState(isRunning: false);
        };
        Controls.Add(_stopButton);

        _statusLabel = new Label
        {
            AutoSize = false,
            Text = "状态：未开始",
            Location = new Point(280, 132),
            Size = new Size(184, 60),
            ForeColor = Color.Silver
        };
        Controls.Add(_statusLabel);

        FormClosing += (_, _) =>
        {
            _worker.Stop();
            _worker.Dispose();
            _hardware.Dispose();
        };
    }

    private static Label CreateValueLabel(string text, int x, int y)
    {
        return new Label
        {
            AutoSize = true,
            Text = text,
            Location = new Point(x, y),
            ForeColor = Color.White
        };
    }

    private void ApplyIntervalToWorker()
    {
        var ms = _intervalComboBox.SelectedIndex switch
        {
            0 => 50,
            1 => 100,
            2 => 200,
            _ => 100
        };
        _worker.SetIntervalMs(ms);
    }

    private void UpdateUiState(bool isRunning)
    {
        _startButton.Enabled = !isRunning;
        _stopButton.Enabled = isRunning;
        _intervalComboBox.Enabled = !isRunning;
        _perCoreCheckBox.Enabled = !isRunning;
        _statusLabel.Text = isRunning ? "状态：监控中..." : "状态：已停止";
    }

    private void OnSampledFromWorkerThread(MonitorSample sample)
    {
        if (IsDisposed)
        {
            return;
        }

        BeginInvoke(new Action(() =>
        {
            _cpuTempLabel.Text = $"CPU温度：{FormatTemp(sample.CpuTempC)}";
            _cpuUsageLabel.Text = $"CPU占用率：{FormatPercent(sample.CpuTotalPercent)}";

            var gpuTempText = FormatTemp(sample.GpuTempC);
            var gpuUsageText = FormatPercent(sample.GpuCorePercent);
            if (gpuTempText == "-.-℃" && gpuUsageText == "-.-%")
            {
                _gpuTempLabel.Text = "GPU温度：未检测到GPU";
                _gpuUsageLabel.Text = "GPU占用率：未检测到GPU";
            }
            else
            {
                _gpuTempLabel.Text = $"GPU温度：{gpuTempText}";
                _gpuUsageLabel.Text = $"GPU占用率：{gpuUsageText}";
            }

            if (_perCoreCheckBox.Checked && sample.CpuPerCorePercent is { Length: > 0 })
            {
                // 为了界面简洁：只展示前 8 个核心（其余省略），你要全量也可以改成 ListView
                var show = sample.CpuPerCorePercent.Take(8).Select(v => $"{v:0.0}%").ToArray();
                _perCoreLabel.Text = $"核心：{string.Join("  ", show)}";
            }

            _statusLabel.Text = $"状态：监控中...\n采样耗时：{sample.SampleDurationMs}ms";
        }));
    }

    private void OnErrorFromWorkerThread(string message)
    {
        if (IsDisposed)
        {
            return;
        }

        BeginInvoke(new Action(() =>
        {
            _statusLabel.Text = $"状态：{message}";
        }));
    }

    private static string FormatTemp(double? c)
    {
        if (!c.HasValue || double.IsNaN(c.Value) || double.IsInfinity(c.Value))
        {
            return "-.-℃";
        }

        return $"{c.Value:0.0}℃";
    }

    private static string FormatPercent(float? p)
    {
        if (!p.HasValue || float.IsNaN(p.Value) || float.IsInfinity(p.Value))
        {
            return "-.-%";
        }

        var v = Math.Clamp(p.Value, 0f, 100f);
        return $"{v:0.0}%";
    }
}

