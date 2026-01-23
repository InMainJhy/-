using System;
using System.Windows.Forms;
using TempControlMonitor.UI;

namespace TempControlMonitor;

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        ApplicationConfiguration.Initialize();
        Application.Run(new MainForm());
    }
}
