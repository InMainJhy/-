# .NET 10 高精度 CPU/GPU 监控工具（WinForms / Windows）

一个基于 **.NET 10 WinForms** 的监控工具，重点解决“读数接近任务管理器/奥创/专业监控工具”的一致性问题：

- CPU：总温度、总占用率、每核心占用率（可选）
- GPU：核心温度、核心占用率
- 精度：温度 0.1℃，占用率 0.1%
- 采样频率：默认 100ms，可选 50/100/200ms

## 技术路线（准确 + 低占用）

- 界面：WinForms（Windows 原生桌面 UI，依赖少）
- 温度：LibreHardwareMonitorLib（读取硬件温度传感器）
- 占用率：
  - CPU：Windows 性能计数器 `Processor\\% Processor Time`（与任务管理器逻辑更接近）
  - GPU：Windows 性能计数器 `GPU Engine\\Utilization Percentage`（与任务管理器 GPU 图表更接近）
- 采集线程：后台线程采样，UI 只负责显示，避免卡顿
- 只启用 CPU/GPU：减少无关硬件扫描带来的耗时

> 温度属于主板/EC/驱动相关传感器，部分机器可能读不到或命名不同。本项目对 CPU 温度做了“择优匹配 + 回退”以提高命中率。

## 运行与构建

本仓库提供完整源码工程；需要安装 .NET 10 SDK 后编译。

```bash
dotnet restore
dotnet build -c Release
dotnet run --project TempControlMonitor
```

发布为“独立部署 / Self-contained”（win-x64）：

```bash
dotnet publish TempControlMonitor -c Release -p:PublishProfile=win-x64-selfcontained
```

发布产物位置：

- `TempControlMonitor\bin\Release\net10.0-windows\win-x64\publish\TempControlMonitor.exe`

## 管理员权限（必须）

硬件温度读取在部分机器上需要管理员权限。本项目已内置清单文件并默认请求管理员：

- 清单文件：`TempControlMonitor/app.manifest`
- 关键项：`requestedExecutionLevel level="requireAdministrator"`

如果你要手动修改：
1. 右键项目 → 属性 → 应用程序
2. 选择/指定清单文件（或把清单内容改成 `requireAdministrator`）

## 代码入口与核心位置（新手找代码用）

- 程序入口：`TempControlMonitor/Program.cs`
- 主界面：`TempControlMonitor/UI/MainForm.cs`
- 后台采集线程：`TempControlMonitor/Monitoring/MonitorWorker.cs`
- CPU 占用率（总/每核心）：`TempControlMonitor/Monitoring/CpuUsageReader.cs`
- GPU 占用率（3D 引擎汇总）：`TempControlMonitor/Monitoring/GpuUsageReader.cs`
- 温度传感器读取：`TempControlMonitor/Hardware/HardwareMonitorService.cs`

## 依赖说明（两种方式）

### 方式 A（推荐）：使用 LibreHardwareMonitor（当前项目默认）
- 优点：维护更活跃，兼容性更好
- 用法：项目已经通过 NuGet 引用了 `LibreHardwareMonitorLib`，不需要你手动下载 DLL，也不需要把 DLL 放到 exe 同目录

### 方式 B：使用 OpenHardwareMonitorLib.dll（你指定的方案）
> 注意：OpenHardwareMonitor 已较久未维护，部分新硬件可能不如 LibreHardwareMonitor。

1. 下载 `OpenHardwareMonitor` 源码/Release（一般是一个压缩包）
2. 找到 `OpenHardwareMonitorLib.dll`
3. 把 DLL 复制到你的项目目录（例如放到 `TempControlMonitor/lib/`）
4. Visual Studio 中添加引用：
   - 右键“依赖项/引用” → “添加引用” → “浏览” → 选择 `OpenHardwareMonitorLib.dll`
5. 发布时把 `OpenHardwareMonitorLib.dll` 一起打包（确保与最终 `exe` 同目录）

## 用 Visual Studio 2022 发布为独立部署（win-x64）的步骤

1. 打开 `TempControlMonitor.sln`
2. 右键项目 `TempControlMonitor` → “发布”
3. 选择“文件夹”发布
4. 目标运行时选择 `win-x64`
5. 部署模式选择“独立部署（Self-contained）”
6. （可选）勾选“生成单个文件”
7. 点击“发布”，输出目录里会生成可运行包

## 用 Inno Setup 制作安装包（包含 exe / dll）

1. 先按上面步骤发布到某个目录（例如 `publish\`）
2. 安装 Inno Setup（官网下载安装即可）
3. 新建脚本（Wizard 向导）
4. 选择主程序：`publish\TempControlMonitor.exe`
5. 选择要一起打包的文件：
   - 若你使用 OpenHardwareMonitorLib.dll：把 `OpenHardwareMonitorLib.dll` 也加入
   - 若你有配置文件/默认设置：也一起加入（例如 `settings.json`，如果你需要）
6. 生成安装包并测试安装/卸载

## 测试与校验（如何验证“精度”）

建议对比：
- 任务管理器（CPU/GPU 占用率）
- HWMonitor / HWiNFO / 奥创（温度）

校验方法：
1. 把采样间隔设为 100ms
2. 打开任务管理器固定在屏幕上对照
3. 同时跑一个 CPU 压力测试（或游戏）观察：
   - CPU 占用率是否接近任务管理器（允许存在 1~3% 的刷新差异）
   - GPU 占用率是否接近任务管理器的 GPU 3D（允许存在小幅差异）
   - 温度是否在合理范围内波动（传感器来源不同会有轻微差异）

## 常见问题（排错）

- 读不到 GPU：可能是硬件库不支持/驱动限制/独显未唤醒。软件会显示 `-.-` 或提示未检测到 GPU。
- CPU 温度为 `-.-℃`：某些机型温度传感器名称不同。可以继续增强匹配逻辑或增加“传感器列表导出”功能来定位传感器名称。
- 采样卡顿：把采样间隔调到 200ms；并确保只启用 CPU/GPU（本项目已默认只启用这两类）。
- 发布后双击闪退：检查是否被杀毒拦截；若使用外部 DLL（OpenHardwareMonitorLib.dll），确认 DLL 与 exe 同目录。
