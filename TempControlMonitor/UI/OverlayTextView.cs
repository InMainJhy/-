using System;
using System.ComponentModel;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace TempControlMonitor.UI;

public sealed class OverlayTextView : Control
{
    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public Color TextColor { get; set; } = Color.White;

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public int PaddingPx { get; set; } = 10;

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public int LineSpacingPx { get; set; } = 2;

    public OverlayTextView()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.UserPaint |
                 ControlStyles.ResizeRedraw, true);
        TabStop = false;
    }

    public Size MeasurePreferredSize()
    {
        var text = Text ?? string.Empty;
        var lines = text.Split(new[] { "\r\n", "\n" }, StringSplitOptions.None);

        using var g = CreateGraphics();
        var flags = TextFormatFlags.NoPadding | TextFormatFlags.NoPrefix;

        var maxWidth = 0;
        var totalHeight = 0;
        for (var i = 0; i < lines.Length; i++)
        {
            var line = lines[i].Length == 0 ? " " : lines[i];
            var size = TextRenderer.MeasureText(g, line, Font, new Size(int.MaxValue, int.MaxValue), flags);
            maxWidth = Math.Max(maxWidth, size.Width);
            totalHeight += size.Height;
            if (i != lines.Length - 1)
            {
                totalHeight += LineSpacingPx;
            }
        }

        maxWidth += PaddingPx * 2;
        totalHeight += PaddingPx * 2;

        return new Size(Math.Max(1, maxWidth), Math.Max(1, totalHeight));
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
        e.Graphics.Clear(BackColor);

        var bounds = ClientRectangle;
        if (bounds.Width <= 0 || bounds.Height <= 0)
        {
            return;
        }

        var text = Text ?? string.Empty;
        var lines = text.Split(new[] { "\r\n", "\n" }, StringSplitOptions.None);
        var flags = TextFormatFlags.NoPadding | TextFormatFlags.NoPrefix | TextFormatFlags.SingleLine;

        var x = PaddingPx;
        var y = PaddingPx;

        for (var i = 0; i < lines.Length; i++)
        {
            var line = lines[i];
            if (line.Length == 0)
            {
                line = " ";
            }

            var lineSize = TextRenderer.MeasureText(e.Graphics, line, Font, new Size(int.MaxValue, int.MaxValue), flags);
            var shadow = Color.FromArgb(220, 0, 0, 0);
            TextRenderer.DrawText(e.Graphics, line, Font, new Point(x + 1, y + 1), shadow, flags);
            TextRenderer.DrawText(e.Graphics, line, Font, new Point(x + 2, y + 2), shadow, flags);
            TextRenderer.DrawText(e.Graphics, line, Font, new Point(x, y), TextColor, flags);

            y += lineSize.Height + LineSpacingPx;
        }
    }
}

