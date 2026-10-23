using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using Microsoft.Win32;

namespace VcmHost;

public partial class MainWindow : Window
{
    private const int WindowSamples = 160;
    private readonly SerialLink _link = new();
    private readonly DispatcherTimer _poll = new();
    private readonly Queue<double> _iref = new();
    private readonly Queue<double> _hat = new();
    private readonly Queue<double> _avg = new();
    private readonly List<double> _capRef = new();
    private readonly List<double> _capIfb = new();
    private readonly ConcurrentQueue<(int Gen, string Line)> _capLines = new();
    private volatile int _capAccept;
    private int _capSid;
    private int _capStopSid;
    private bool _capNeedArm;
    private int _capArmTries;
    private DateTime _capArmDeadline;
    private string _capStartCmd = "";
    private bool _capLive;
    private double _plotHz = 1000;
    private double _areaL;
    private double _areaT;
    private double _areaW;
    private double _areaH;
    private static readonly Brush AxisBrush;
    private static readonly Brush GridBrush;

    static MainWindow()
    {
        AxisBrush = new SolidColorBrush(Color.FromRgb(0x8B, 0x95, 0xA5));
        GridBrush = new SolidColorBrush(Color.FromRgb(0x32, 0x38, 0x43));
        AxisBrush.Freeze();
        GridBrush.Freeze();
    }
    private bool _viewOn;
    private bool _plotFull;
    private bool _dragZoom;
    private bool _dragPan;
    private double _vx0;
    private double _vx1 = 1;
    private double _vy0;
    private double _vy1 = 1;
    private double _drawnY0 = -0.2;
    private double _drawnY1 = 0.2;
    private Point _dragStart;
    private double _panVx0;
    private double _panVx1 = 1;
    private double _panVy0;
    private double _panVy1 = 1;
    private readonly List<string> _csv = new();
    private readonly object _logGate = new();
    private bool _syncingIref;
    private bool _syncingObs;
    private bool _syncingMode;
    private DateTime _lastPoll = DateTime.MinValue;
    private bool _awaitingTl;
    private DateTime _nvQuietUntil = DateTime.MinValue;
    private bool _syncBaud;
    private int _pendingBaud;
    private string? _portName;
    private bool _nvPending;

    public MainWindow()
    {
        InitializeComponent();
        _link.LineReceived += OnLine;
        _link.Error += msg => Dispatcher.BeginInvoke(() => AppendLog("ERR " + msg));
        _poll.Interval = TimeSpan.FromMilliseconds(70);
        _poll.Tick += Poll_Tick;
        _csv.Add("t_ms,iref,ihat,iavg,ip,im,mod,err,state,fault");
    }

    private void Window_Loaded(object sender, RoutedEventArgs e)
    {
        RefreshPorts_Click(sender, e);
    }

    private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
    {
        _poll.Stop();
        _link.Dispose();
    }

    private void RefreshPorts_Click(object sender, RoutedEventArgs e)
    {
        var ports = SerialLink.ListPorts();
        var keep = PortBox.SelectedItem as string;
        PortBox.ItemsSource = ports;
        if (keep != null && ports.Contains(keep))
        {
            PortBox.SelectedItem = keep;
        }
        else if (ports.Length > 0)
        {
            PortBox.SelectedIndex = 0;
        }
    }

    private int SelectedBaud()
    {
        if (BaudBox?.SelectedItem is ComboBoxItem item &&
            int.TryParse(item.Content?.ToString(), NumberStyles.Integer, CultureInfo.InvariantCulture, out var baud))
        {
            return baud;
        }

        return 115200;
    }

    private void SelectBaud(int baud)
    {
        _syncBaud = true;
        foreach (var obj in BaudBox.Items)
        {
            if (obj is ComboBoxItem item &&
                item.Content?.ToString() == baud.ToString(CultureInfo.InvariantCulture))
            {
                BaudBox.SelectedItem = item;
                break;
            }
        }

        _syncBaud = false;
    }

    private void BaudBox_Changed(object sender, SelectionChangedEventArgs e)
    {
        if (_syncBaud || BaudBox == null || !_link.IsOpen)
        {
            return;
        }

        var baud = SelectedBaud();
        if (baud == _link.Baud)
        {
            return;
        }

        _pendingBaud = baud;
        _link.Send($"$Q {baud}\n");
        AppendLog($"请求波特率 {baud}");
    }

    private void ApplyBaudAck(string line)
    {
        var text = line.Length > 5 ? line[5..].Trim() : "";
        if (!int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var baud) ||
            string.IsNullOrEmpty(_portName))
        {
            return;
        }

        _pendingBaud = 0;
        try
        {
            _poll.Stop();
            _link.Close();
            _link.Open(_portName, baud);
            ConnectBtn.Content = "断开";
            AppendLog($"波特率 {baud}");
            _link.Send("$I\n");
            _poll.Start();
        }
        catch (Exception ex)
        {
            AppendLog("按新波特率打开失败 " + ex.Message);
        }
    }

    private void Connect_Click(object sender, RoutedEventArgs e)
    {
        if (_link.IsOpen)
        {
            _poll.Stop();
            _link.Close();
            ConnectBtn.Content = "连接";
            LinkText.Text = "● 断开";
            LinkText.Foreground = (Brush)FindResource("DangerBrush");
            IdText.Text = "未连接";
            AppendLog("断开");
            return;
        }

        if (PortBox.SelectedItem is not string port)
        {
            MessageBox.Show("请选择串口。", "VCM电流环调试");
            return;
        }

        try
        {
            _link.Open(port, SelectedBaud());
            _portName = port;
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "打开串口失败");
            return;
        }

        ConnectBtn.Content = "断开";
        LinkText.Text = "● " + port;
        LinkText.Foreground = (Brush)FindResource("OkBrush");
        _nvPending = false;
        _nvQuietUntil = DateTime.MinValue;
        NvText.Text = "Flash：未读";
        AppendLog("连接 " + port);
        _ = Dispatcher.BeginInvoke(async () =>
        {
            await Task.Delay(200);
            if (_link.IsOpen)
            {
                _link.Send("$I\n");
                _link.Send("$D\n");
            }
        });
        ApplyPollInterval();
        _poll.Start();
    }

    private void ApplyPollInterval()
    {
        if (!double.TryParse(PollHzBox.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out var hz))
        {
            hz = 15;
        }

        hz = Math.Clamp(hz, 2, 40);
        _poll.Interval = TimeSpan.FromMilliseconds(1000.0 / hz);
    }

    private void Poll_Tick(object? sender, EventArgs e)
    {
        if (!_link.IsOpen)
        {
            return;
        }

        DrainCapQueue();
        if (_capNeedArm && _capLive && DateTime.UtcNow >= _capArmDeadline)
        {
            if (_capArmTries < 1 && _capStartCmd.Length > 0)
            {
                _capArmTries++;
                _capArmDeadline = DateTime.UtcNow.AddMilliseconds(500);
                _link.SendExclusive(_capStartCmd);
            }
            else
            {
                _capNeedArm = false;
                _capLive = false;
                CapStatus.Text = "没有开始，请再点一次开始采样";
            }
        }

        if (_capLive)
        {
            if (_poll.Interval.TotalMilliseconds > 40)
            {
                _poll.Interval = TimeSpan.FromMilliseconds(33);
            }

            return;
        }

        if (_nvPending && DateTime.UtcNow >= _nvQuietUntil)
        {
            _nvPending = false;
            NvText.Text = "Flash：超时无应答。请用 CubeIDE 烧录当前固件并复位后再保存。";
            AppendLog("NV TIMEOUT");
        }

        if (PollBox.IsChecked != true || _capLive)
        {
            return;
        }

        ApplyPollInterval();
        if (_awaitingTl && (DateTime.UtcNow - _lastPoll).TotalMilliseconds < 250)
        {
            return;
        }

        if (DateTime.UtcNow < _nvQuietUntil)
        {
            return;
        }

        _awaitingTl = true;
        _lastPoll = DateTime.UtcNow;
        _link.Send("$P\n");
    }

    private void OnLine(string line)
    {
        if (line.StartsWith("CAP d=", StringComparison.Ordinal))
        {
            var sid = _capAccept;
            if (sid == 0)
            {
                return;
            }

            _capLines.Enqueue((sid, line));
            return;
        }

        Dispatcher.BeginInvoke(() => HandleLine(line));
    }

    private void DrainCapQueue()
    {
        var any = false;
        var sid = _capAccept;
        while (_capLines.TryDequeue(out var item))
        {
            if (sid == 0 || item.Gen != sid)
            {
                continue;
            }

            IngestCap(item.Line);
            any = true;
        }

        if (!any)
        {
            return;
        }

        _capNeedArm = false;

        CapStatus.Text = _capLive
            ? $"采样中  {_capIfb.Count} 点"
            : $"已收到 {_capIfb.Count} 点";
        Redraw();
    }

    private void HandleLine(string line)
    {
        if (line.StartsWith("CAP d=", StringComparison.Ordinal))
        {
            IngestCap(line);
            return;
        }

        if (line.StartsWith("BAUD ", StringComparison.Ordinal))
        {
            ApplyBaudAck(line);
            return;
        }

        AppendLog(line);
        if (_pendingBaud != 0 && line.StartsWith("ERR", StringComparison.Ordinal))
        {
            var failed = _pendingBaud;
            _pendingBaud = 0;
            SelectBaud(_link.Baud);
            AppendLog($"波特率 {failed} 未生效，仍为 {_link.Baud}。需烧录 fw=1.4.43。");
        }
        if (line.StartsWith("CAP ARM", StringComparison.Ordinal))
        {
            var arm = Protocol.ParseFields(line);
            var sid = arm.TryGetValue("id", out var armId) ? (int)armId : 0;
            if (_capAccept == 0 || (sid != 0 && sid != _capAccept))
            {
                return;
            }

            PlotResetView();
            _capLive = true;
            if (arm.TryGetValue("hz", out var armHz))
            {
                _plotHz = armHz;
                if (CapHzPick != null && CapSingle.IsChecked == true)
                {
                    CapHzPick.Text = armHz.ToString(CultureInfo.InvariantCulture);
                }
            }
            CapStatus.Text = line;
            Redraw();
            return;
        }

        if (line.StartsWith("CAP DONE", StringComparison.Ordinal))
        {
            var done = Protocol.ParseFields(line);
            var sid = done.TryGetValue("id", out var doneId) ? (int)doneId : 0;
            if (_capAccept == 0 || (sid != 0 && sid != _capAccept))
            {
                return;
            }

            _capNeedArm = false;
            _capLive = false;
            DrainCapQueue();
            CapStatus.Text = line;
            Redraw();
            return;
        }

        if (line.StartsWith("CAP STOP", StringComparison.Ordinal))
        {
            var stop = Protocol.ParseFields(line);
            var sid = stop.TryGetValue("id", out var stopId) ? (int)stopId : 0;
            if (_capAccept != 0 || (sid != 0 && sid != _capStopSid))
            {
                return;
            }

            _capLive = false;
            DrainCapQueue();
            CapStatus.Text = "已停止";
            Redraw();
            return;
        }
        if (line.StartsWith("TL ", StringComparison.Ordinal))
        {
            _awaitingTl = false;
            ApplyTelemetry(Protocol.ParseFields(line));
        }
        else if (line.StartsWith("DT ns=", StringComparison.Ordinal))
        {
            var dt = Protocol.ParseFields(line);
            if (dt.ContainsKey("ns"))
            {
                DtBox.Text = Protocol.Int(dt, "ns").ToString(CultureInfo.InvariantCulture);
            }
        }
        else if (line.StartsWith("PR ", StringComparison.Ordinal) ||
                 line.StartsWith("ID ", StringComparison.Ordinal))
        {
            ApplyDump(Protocol.ParseFields(line), line);
        }
        else if (line.StartsWith("NV ", StringComparison.Ordinal))
        {
            NvText.Text = "Flash：" + line;
            if (line.StartsWith("NV SAVE", StringComparison.Ordinal) ||
                line.StartsWith("NV FACTORY", StringComparison.Ordinal) ||
                line.StartsWith("NV LOAD", StringComparison.Ordinal) ||
                line.StartsWith("NV ERR", StringComparison.Ordinal))
            {
                _nvPending = false;
                _nvQuietUntil = DateTime.UtcNow;
                if (!line.StartsWith("NV ERR", StringComparison.Ordinal))
                {
                    _link.Send("$D\n");
                }
            }
            else if (line.StartsWith("NV BUSY", StringComparison.Ordinal))
            {
                _nvQuietUntil = DateTime.UtcNow.AddSeconds(40);
            }
        }
        else if (line.StartsWith("ERR", StringComparison.Ordinal) && _nvPending)
        {
            _nvPending = false;
            _nvQuietUntil = DateTime.UtcNow;
            NvText.Text = "Flash：命令被拒绝 " + line;
        }
    }

    private void ApplyDump(Dictionary<string, long> m, string line)
    {
        if (line.StartsWith("ID ", StringComparison.Ordinal))
        {
            IdText.Text = line.Length > 3 ? line[3..] : line;
        }

        if (m.ContainsKey("kp_e3"))
        {
            KpBox.Text = Protocol.Milli(m, "kp_e3").ToString("0.###", CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("ki_e3"))
        {
            KiBox.Text = Protocol.Milli(m, "ki_e3").ToString("0.###", CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("obs_e3"))
        {
            _syncingObs = true;
            var a = Protocol.Milli(m, "obs_e3");
            ObsBox.Text = a.ToString("0.###", CultureInfo.InvariantCulture);
            ObsHzBox.Text = Protocol.AlphaToHz(a).ToString("0", CultureInfo.InvariantCulture);
            _syncingObs = false;
        }

        if (m.ContainsKey("lpf_e3"))
        {
            LpfBox.Text = Protocol.Milli(m, "lpf_e3").ToString("0.###", CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("ff_e6"))
        {
            FfBox.Text = Protocol.Micro(m, "ff_e6").ToString("0.#####", CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("vbus_e3"))
        {
            VbusBox.Text = Protocol.Milli(m, "vbus_e3").ToString("0.###", CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("r_e3"))
        {
            RcoilBox.Text = Protocol.Milli(m, "r_e3").ToString("0.###", CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("l_e6"))
        {
            var mh = Protocol.Micro(m, "l_e6") * 1000.0;
            LcoilBox.Text = mh.ToString("0.###", CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("fdbk"))
        {
            var obs = Protocol.Int(m, "fdbk") != 0;
            FdbkObs.IsChecked = obs;
            FdbkAvg.IsChecked = !obs;
        }

        if (m.ContainsKey("dt_ns"))
        {
            DtBox.Text = Protocol.Int(m, "dt_ns").ToString(CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("n0"))
        {
            Notch0Box.Text = Protocol.Int(m, "n0").ToString(CultureInfo.InvariantCulture);
            Notch1Box.Text = Protocol.Int(m, "n1").ToString(CultureInfo.InvariantCulture);
            Notch2Box.Text = Protocol.Int(m, "n2").ToString(CultureInfo.InvariantCulture);
            Notch3Box.Text = Protocol.Int(m, "n3").ToString(CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("gz"))
        {
            var two = Protocol.Int(m, "gz") != 0;
            GzOn.IsChecked = two;
            GzOff.IsChecked = !two;
        }

        if (m.ContainsKey("gs_e3"))
        {
            GsBox.Text = Protocol.Milli(m, "gs_e3").ToString("0.###", CultureInfo.InvariantCulture);
        }

        if (m.ContainsKey("mode"))
        {
            ShowPwmMode(Protocol.Int(m, "mode") != 0);
        }

        if (m.ContainsKey("nv"))
        {
            NvText.Text = Protocol.Int(m, "nv") != 0
                ? "Flash：有有效参数（上电已加载）"
                : "Flash：空 / CRC 无效（用编译默认）";
        }
    }

    private void ApplyTelemetry(Dictionary<string, long> m)
    {
        var iref = Protocol.Milli(m, "iref_ma");
        var hat = Protocol.Milli(m, "ihat_ma");
        var avg = Protocol.Milli(m, "iavg_ma");
        var ip = Protocol.Milli(m, "ip_ma");
        var im = Protocol.Milli(m, "im_ma");
        var mod = Protocol.Micro(m, "mod_e6");
        var err = hat - iref;
        var state = Protocol.Int(m, "state");
        var fault = Protocol.Int(m, "fault");

        StateText.Text = Protocol.StateName(state);
        StateText.Foreground = state == 4
            ? (Brush)FindResource("DangerBrush")
            : (Brush)FindResource("HatBrush");
        IrefText.Text = FmtA(iref);
        HatText.Text = FmtA(hat);
        AvgText.Text = FmtA(avg);
        ErrText.Text = FmtA(err);
        ModText.Text = double.IsNaN(mod) ? "—" : mod.ToString("0.0000", CultureInfo.InvariantCulture);
        if (fault != 0)
        {
            ModText.Text += "  F=" + fault;
            ModText.Foreground = (Brush)FindResource("DangerBrush");
        }
        else
        {
            ModText.Foreground = (Brush)FindResource("TextBrush");
        }

        FlagText.Text =
            $"ov={Protocol.Int(m, "ov")}  hold={Protocol.Int(m, "hold")}  armed={Protocol.Int(m, "armed")}  pair={Protocol.Int(m, "pair")}  fdbk={Protocol.Int(m, "fdbk")}  增益={(Protocol.Int(m, "gz") == 0 ? "单段" : Protocol.Int(m, "gzh") != 0 ? "静止" : "运动")}  ovr={Protocol.Int(m, "ovr1")}/{Protocol.Int(m, "ovr2")}";

        if (m.ContainsKey("mode"))
        {
            ShowPwmMode(Protocol.Int(m, "mode") != 0);
        }

        Push(_iref, iref);
        Push(_hat, hat);
        Push(_avg, avg);
        Redraw();

        var t = Environment.TickCount64;
        _csv.Add(string.Create(CultureInfo.InvariantCulture,
            $"{t},{iref:F4},{hat:F4},{avg:F4},{ip:F4},{im:F4},{mod:F6},{err:F4},{state},{fault}"));
        if (_csv.Count > 20000)
        {
            _csv.RemoveRange(1, 4000);
        }
    }

    private static string FmtA(double a) =>
        double.IsNaN(a) ? "—" : a.ToString("0.000", CultureInfo.InvariantCulture);

    private static void Push(Queue<double> q, double v)
    {
        if (double.IsNaN(v))
        {
            return;
        }

        q.Enqueue(v);
        while (q.Count > WindowSamples)
        {
            q.Dequeue();
        }
    }

    private void Plot_SizeChanged(object sender, SizeChangedEventArgs e) => Redraw();

    private bool PlotFrame()
    {
        const double padL = 52;
        const double padR = 12;
        const double padT = 4;
        const double padB = 22;
        _areaL = padL;
        _areaT = padT;
        _areaW = Plot.ActualWidth - padL - padR;
        _areaH = Plot.ActualHeight - padT - padB;
        return _areaW >= 8 && _areaH >= 8;
    }

    private void Redraw()
    {
        if (!PlotFrame())
        {
            return;
        }

        ZeroLine.X1 = _areaL;
        ZeroLine.X2 = _areaL + _areaW;
        double ymin;
        double ymax;
        if (_viewOn)
        {
            ymin = _vy0;
            ymax = _vy1;
        }
        else
        {
            ymin = -0.2;
            ymax = 0.2;
            if (_capIfb.Count > 0)
            {
                foreach (var v in _capRef)
                {
                    ymin = Math.Min(ymin, v);
                    ymax = Math.Max(ymax, v);
                }
                foreach (var v in _capIfb)
                {
                    ymin = Math.Min(ymin, v);
                    ymax = Math.Max(ymax, v);
                }
            }
            else
            {
                foreach (var q in new[] { _iref, _hat, _avg })
                {
                    foreach (var v in q)
                    {
                        ymin = Math.Min(ymin, v);
                        ymax = Math.Max(ymax, v);
                    }
                }
            }

            var pad = Math.Max(0.05, 0.08 * (ymax - ymin));
            ymin -= pad;
            ymax += pad;
        }

        if (ymax <= ymin)
        {
            ymax = ymin + 0.1;
        }

        _drawnY0 = ymin;
        _drawnY1 = ymax;
        ZeroLine.Y1 = ZeroLine.Y2 = Y(0, ymin, ymax);
        var samples = _capIfb.Count > 0
            ? _capIfb.Count
            : Math.Max(_iref.Count, Math.Max(_hat.Count, _avg.Count));
        DrawScale(ymin, ymax, samples);
        if (_capIfb.Count > 0)
        {
            LineHat.Points = new PointCollection();
            LineIref.Points = ToPointsList(_capRef, ymin, ymax);
            LineAvg.Points = ToPointsList(_capIfb, ymin, ymax);
        }
        else
        {
            LineIref.Points = ToPoints(_iref, ymin, ymax);
            LineHat.Points = ToPoints(_hat, ymin, ymax);
            LineAvg.Points = ToPoints(_avg, ymin, ymax);
        }
    }

    private double Y(double v, double ymin, double ymax) =>
        _areaT + _areaH - ((v - ymin) / (ymax - ymin) * _areaH);

    private double PlotX(int index, int last)
    {
        var frac = last <= 0 ? 0 : index / (double)last;
        var x0 = _viewOn ? _vx0 : 0;
        var x1 = _viewOn ? _vx1 : 1;
        return _areaL + (frac - x0) / Math.Max(1e-9, x1 - x0) * _areaW;
    }

    private double ViewHz(int samples)
    {
        if (samples > 1 && _capIfb.Count > 0 && _plotHz >= 1)
        {
            return _plotHz;
        }

        var sec = _poll.Interval.TotalSeconds;
        return sec < 1e-4 ? 15 : 1.0 / sec;
    }

    private void DrawScale(double ymin, double ymax, int samples)
    {
        PlotAxis.Children.Clear();
        AddLine(_areaL, _areaT, _areaL + _areaW, _areaT, AxisBrush);
        AddLine(_areaL, _areaT, _areaL, _areaT + _areaH, AxisBrush);
        AddLine(_areaL, _areaT + _areaH, _areaL + _areaW, _areaT + _areaH, AxisBrush);

        var yStep = NiceStep(ymax - ymin);
        var yTick = Math.Ceiling((ymin - yStep * 1e-6) / yStep) * yStep;
        double lastY = double.NaN;
        for (var v = yTick; v <= ymax + yStep * 1e-6; v += yStep)
        {
            var y = Y(v, ymin, ymax);
            if (y < _areaT - 0.5 || y > _areaT + _areaH + 0.5)
            {
                continue;
            }

            if (Math.Abs(v) > yStep * 0.01)
            {
                AddLine(_areaL, y, _areaL + _areaW, y, GridBrush);
            }

            AddLine(_areaL - 4, y, _areaL, y, AxisBrush);
            if (!double.IsNaN(lastY) && Math.Abs(lastY - y) < 13)
            {
                continue;
            }

            lastY = y;
            var shown = Math.Abs(v) < yStep * 0.001 ? 0 : v;
            AddText(FmtTick(shown, yStep), 0, y - 7, 46, TextAlignment.Right);
        }

        var n = samples < 2 ? WindowSamples : samples;
        var hz = ViewHz(n);
        var xVis0 = _viewOn ? _vx0 : 0;
        var xVis1 = _viewOn ? _vx1 : 1;
        var t0 = xVis0 * (n - 1) / hz;
        var t1 = xVis1 * (n - 1) / hz;
        if (t1 < t0)
        {
            (t0, t1) = (t1, t0);
        }

        var span = Math.Max(t1 - t0, 1e-9);
        var (mul, unit) = TimeUnit(span);
        var xStep = NiceStep(span);
        var xt = Math.Ceiling((t0 - xStep * 1e-6) / xStep) * xStep;
        double lastX = double.NegativeInfinity;
        for (; xt <= t1 + xStep * 1e-6; xt += xStep)
        {
            var x = _areaL + (xt - t0) / span * _areaW;
            if (x < _areaL - 0.5 || x > _areaL + _areaW + 0.5)
            {
                continue;
            }

            AddLine(x, _areaT, x, _areaT + _areaH, GridBrush);
            AddLine(x, _areaT + _areaH, x, _areaT + _areaH + 4, AxisBrush);
            if (x - lastX < 40 || x < _areaL + 18 || x > _areaL + _areaW - 36)
            {
                continue;
            }

            lastX = x;
            AddText((xt * mul).ToString("0.##", CultureInfo.InvariantCulture), x - 20, _areaT + _areaH + 4, 40, TextAlignment.Center);
        }

        AddText(unit, _areaL + _areaW - 28, _areaT + _areaH + 4, 32, TextAlignment.Left);
    }

    private void AddLine(double x1, double y1, double x2, double y2, Brush stroke)
    {
        PlotAxis.Children.Add(new Line
        {
            X1 = x1,
            Y1 = y1,
            X2 = x2,
            Y2 = y2,
            Stroke = stroke,
            StrokeThickness = 1,
            IsHitTestVisible = false,
        });
    }

    private void AddText(string text, double x, double y, double width, TextAlignment align)
    {
        var block = new TextBlock
        {
            Text = text,
            FontSize = 11,
            Foreground = AxisBrush,
            Width = width,
            TextAlignment = align,
            IsHitTestVisible = false,
        };
        Canvas.SetLeft(block, x);
        Canvas.SetTop(block, Math.Max(0, y));
        PlotAxis.Children.Add(block);
    }

    private static double NiceStep(double span)
    {
        if (span <= 0 || double.IsNaN(span) || double.IsInfinity(span))
        {
            return 1;
        }

        var raw = span / 5.0;
        var pow = Math.Pow(10, Math.Floor(Math.Log10(raw)));
        if (pow <= 0 || double.IsNaN(pow) || double.IsInfinity(pow))
        {
            return raw;
        }

        var m = raw / pow;
        var nice = m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10;
        return nice * pow;
    }

    private static (double Mul, string Unit) TimeUnit(double span)
    {
        if (span >= 1)
        {
            return (1, "s");
        }

        if (span >= 1e-3)
        {
            return (1e3, "ms");
        }

        return (1e6, "µs");
    }

    private static string FmtTick(double v, double step)
    {
        var dec = step >= 1 ? 0 : step >= 0.1 ? 1 : step >= 0.01 ? 2 : 3;
        return v.ToString("F" + dec, CultureInfo.InvariantCulture);
    }

    private void PlotResetView()
    {
        _viewOn = false;
        _vx0 = 0;
        _vx1 = 1;
    }

    private void EnsurePlotView()
    {
        if (_viewOn)
        {
            return;
        }

        _vx0 = 0;
        _vx1 = 1;
        _vy0 = _drawnY0;
        _vy1 = Math.Max(_drawnY1, _drawnY0 + 1e-3);
        _viewOn = true;
    }

    private void ClampPlotX()
    {
        var span = _vx1 - _vx0;
        if (span >= 1)
        {
            _vx0 = 0;
            _vx1 = 1;
            return;
        }

        if (_vx0 < 0)
        {
            _vx1 -= _vx0;
            _vx0 = 0;
        }

        if (_vx1 > 1)
        {
            _vx0 -= _vx1 - 1;
            _vx1 = 1;
        }
    }

    private void PlotReset_Click(object sender, RoutedEventArgs e)
    {
        PlotResetView();
        Redraw();
    }

    private void PlotFull_Click(object sender, RoutedEventArgs e)
    {
        if (!_plotFull)
        {
            PlotHost.Children.Remove(PlotCard);
            PlotOverlay.Children.Add(PlotCard);
            PlotCard.ClearValue(Grid.RowProperty);
            PlotCard.Height = double.NaN;
            PlotCard.MaxHeight = double.PositiveInfinity;
            PlotCard.Margin = new Thickness(16);
            PlotOverlay.Visibility = Visibility.Visible;
            PlotFullBtn.Content = "退出全屏";
            _plotFull = true;
        }
        else
        {
            PlotOverlay.Children.Remove(PlotCard);
            PlotHost.Children.Add(PlotCard);
            Grid.SetRow(PlotCard, 0);
            PlotCard.Height = double.NaN;
            PlotCard.ClearValue(MaxHeightProperty);
            PlotCard.Margin = new Thickness(0);
            PlotOverlay.Visibility = Visibility.Collapsed;
            PlotFullBtn.Content = "全屏";
            _plotFull = false;
        }

        Dispatcher.BeginInvoke(Redraw);
    }

    private void Window_PreviewKeyDown(object sender, System.Windows.Input.KeyEventArgs e)
    {
        if (e.Key == System.Windows.Input.Key.Escape && _plotFull)
        {
            PlotFull_Click(sender, e);
            e.Handled = true;
        }
    }

    private void Plot_Down(object sender, System.Windows.Input.MouseButtonEventArgs e)
    {
        if (e.ClickCount == 2)
        {
            PlotReset_Click(sender, e);
            e.Handled = true;
            return;
        }

        _dragZoom = true;
        _dragStart = e.GetPosition(Plot);
        Plot.CaptureMouse();
        Canvas.SetLeft(ZoomRect, _dragStart.X);
        Canvas.SetTop(ZoomRect, _dragStart.Y);
        ZoomRect.Width = 0;
        ZoomRect.Height = 0;
        ZoomRect.Visibility = Visibility.Visible;
        e.Handled = true;
    }

    private void Plot_Move(object sender, System.Windows.Input.MouseEventArgs e)
    {
        var p = e.GetPosition(Plot);
        if (_dragZoom)
        {
            var x = Math.Min(p.X, _dragStart.X);
            var y = Math.Min(p.Y, _dragStart.Y);
            Canvas.SetLeft(ZoomRect, x);
            Canvas.SetTop(ZoomRect, y);
            ZoomRect.Width = Math.Abs(p.X - _dragStart.X);
            ZoomRect.Height = Math.Abs(p.Y - _dragStart.Y);
        }
        else if (_dragPan)
        {
            if (!PlotFrame())
            {
                return;
            }

            var dx = (p.X - _dragStart.X) / _areaW * (_panVx1 - _panVx0);
            var dy = (p.Y - _dragStart.Y) / _areaH * (_panVy1 - _panVy0);
            _vx0 = _panVx0 - dx;
            _vx1 = _panVx1 - dx;
            _vy0 = _panVy0 + dy;
            _vy1 = _panVy1 + dy;
            ClampPlotX();
            Redraw();
        }
    }

    private void Plot_Up(object sender, System.Windows.Input.MouseButtonEventArgs e)
    {
        if (!_dragZoom)
        {
            return;
        }

        _dragZoom = false;
        Plot.ReleaseMouseCapture();
        ZoomRect.Visibility = Visibility.Collapsed;
        var p = e.GetPosition(Plot);
        var x0 = Math.Min(p.X, _dragStart.X);
        var x1 = Math.Max(p.X, _dragStart.X);
        var y0 = Math.Min(p.Y, _dragStart.Y);
        var y1 = Math.Max(p.Y, _dragStart.Y);
        if (x1 - x0 < 6 || y1 - y0 < 6)
        {
            return;
        }

        if (!PlotFrame())
        {
            return;
        }

        x0 = Math.Clamp(x0, _areaL, _areaL + _areaW);
        x1 = Math.Clamp(x1, _areaL, _areaL + _areaW);
        y0 = Math.Clamp(y0, _areaT, _areaT + _areaH);
        y1 = Math.Clamp(y1, _areaT, _areaT + _areaH);
        EnsurePlotView();
        var fx0 = (x0 - _areaL) / _areaW;
        var fx1 = (x1 - _areaL) / _areaW;
        var fy0 = (y0 - _areaT) / _areaH;
        var fy1 = (y1 - _areaT) / _areaH;
        var nx0 = _vx0 + fx0 * (_vx1 - _vx0);
        var nx1 = _vx0 + fx1 * (_vx1 - _vx0);
        var nyTop = _vy1 - fy0 * (_vy1 - _vy0);
        var nyBot = _vy1 - fy1 * (_vy1 - _vy0);
        if (nx1 - nx0 < 1e-4 || nyTop - nyBot < 1e-6)
        {
            return;
        }

        _vx0 = Math.Clamp(nx0, 0, 1);
        _vx1 = Math.Clamp(nx1, 0, 1);
        _vy0 = nyBot;
        _vy1 = nyTop;
        _viewOn = true;
        Redraw();
    }

    private void Plot_PanDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
    {
        EnsurePlotView();
        _dragPan = true;
        _dragStart = e.GetPosition(Plot);
        _panVx0 = _vx0;
        _panVx1 = _vx1;
        _panVy0 = _vy0;
        _panVy1 = _vy1;
        Plot.CaptureMouse();
        e.Handled = true;
    }

    private void Plot_PanUp(object sender, System.Windows.Input.MouseButtonEventArgs e)
    {
        _dragPan = false;
        Plot.ReleaseMouseCapture();
        e.Handled = true;
    }

    private void Plot_Wheel(object sender, System.Windows.Input.MouseWheelEventArgs e)
    {
        if (!PlotFrame())
        {
            return;
        }

        var p = e.GetPosition(Plot);
        EnsurePlotView();
        var factor = e.Delta > 0 ? 0.8 : 1.25;
        var fx = Math.Clamp((p.X - _areaL) / _areaW, 0, 1);
        var fy = Math.Clamp((p.Y - _areaT) / _areaH, 0, 1);
        var x = _vx0 + fx * (_vx1 - _vx0);
        var y = _vy1 - fy * (_vy1 - _vy0);
        _vx0 = x - (x - _vx0) * factor;
        _vx1 = x + (_vx1 - x) * factor;
        _vy0 = y - (y - _vy0) * factor;
        _vy1 = y + (_vy1 - y) * factor;
        if (_vx1 - _vx0 > 1)
        {
            _vx0 = 0;
            _vx1 = 1;
        }

        ClampPlotX();
        if (_vy1 - _vy0 < 1e-4)
        {
            _vy1 = _vy0 + 1e-4;
        }

        _viewOn = true;
        Redraw();
        e.Handled = true;
    }

    private void CapStart_Click(object sender, RoutedEventArgs e)
    {
        if (!_link.IsOpen)
        {
            AppendLog("未连接");
            return;
        }

        if (!int.TryParse(CapHzPick.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var hz))
        {
            hz = 1000;
        }

        if (!int.TryParse(CapNBox.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var n))
        {
            n = 4000;
        }

        var mode = CapCont.IsChecked == true ? 1 : 0;
        if (mode != 0)
        {
            hz = SnapControlHz(ContinuousMaxHz(_link.Baud), ContinuousMaxHz(_link.Baud));
        }
        else
        {
            hz = Math.Clamp(hz, 1, 50000);
        }
        n = Math.Clamp(n, 1, 16384);
        _plotHz = hz;
        CapNBox.Text = n.ToString(CultureInfo.InvariantCulture);
        _capSid++;
        if (_capSid == 0)
        {
            _capSid = 1;
        }

        _capAccept = _capSid;
        _capStopSid = 0;
        _capNeedArm = true;
        _capArmTries = 0;
        _capArmDeadline = DateTime.UtcNow.AddMilliseconds(450);
        _capStartCmd = $"$U id={_capSid} hz={hz} mode={mode} n={n}\n";
        while (_capLines.TryDequeue(out _))
        {
        }

        _capRef.Clear();
        _capIfb.Clear();
        PlotResetView();
        _capLive = true;
        CapStatus.Text = "等待 CAP ARM";
        _link.SendExclusive(_capStartCmd);
    }

    private void CapStop_Click(object sender, RoutedEventArgs e)
    {
        var sid = _capAccept;
        _capStopSid = sid;
        _capAccept = 0;
        _capNeedArm = false;
        _capLive = false;
        while (_capLines.TryDequeue(out _))
        {
        }

        CapStatus.Text = "已停止";
        if (_link.IsOpen)
        {
            _link.SendExclusive(sid == 0 ? "$U stop\n" : $"$U stop id={sid}\n");
        }
    }

    private void CapMode_Changed(object sender, RoutedEventArgs e)
    {
        if (CapCont == null || CapHzPick == null)
        {
            return;
        }

        var continuous = CapCont.IsChecked == true;
        CapHzPick.IsEnabled = !continuous;
        if (CapNBox != null)
        {
            CapNBox.IsEnabled = !continuous;
        }

        if (continuous && CapStatus != null)
        {
            var hz = SnapControlHz(ContinuousMaxHz(_link.Baud), ContinuousMaxHz(_link.Baud));
            CapStatus.Text = $"连续采集 {hz} Hz，频率由当前波特率决定。";
        }
        else if (CapStatus != null)
        {
            CapStatus.Text = "单次采集：选择采样率和点数后点开始。";
        }
    }

    /* Same budget as the firmware: 8N1, 8 pairs per line, worst-case "-32767". */
    private static int ContinuousMaxHz(int baud)
    {
        if (baud < 10)
        {
            baud = 10;
        }

        var lines = Math.Max(1, (baud / 10) / 119);
        return Math.Clamp(lines * 8, 1, 50000);
    }

    private static int SnapControlHz(int hz, int cap)
    {
        hz = Math.Clamp(hz, 1, 50000);
        var div = Math.Clamp((50000 + hz / 2) / hz, 1, 50000);
        var actual = 50000 / div;
        while (actual > cap && div < 50000)
        {
            div++;
            actual = 50000 / div;
        }

        return Math.Max(1, actual);
    }

    private void IngestCap(string line)
    {
        var payload = line.Length > 6 ? line[6..] : "";
        var nums = new List<int>();
        foreach (var part in payload.Split(',', StringSplitOptions.RemoveEmptyEntries))
        {
            if (int.TryParse(part, NumberStyles.Integer, CultureInfo.InvariantCulture, out var ma))
            {
                nums.Add(ma);
            }
        }

        for (var i = 0; i + 1 < nums.Count; i += 2)
        {
            _capRef.Add(nums[i] / 1000.0);
            _capIfb.Add(nums[i + 1] / 1000.0);
        }

        if (_capIfb.Count > 16384)
        {
            var drop = _capIfb.Count - 16384;
            _capRef.RemoveRange(0, drop);
            _capIfb.RemoveRange(0, drop);
        }
    }

    private PointCollection ToPointsList(List<double> src, double ymin, double ymax)
    {
        var pts = new PointCollection();
        if (src.Count == 0)
        {
            return pts;
        }

        var last = src.Count - 1;
        var x0 = _viewOn ? _vx0 : 0;
        var x1 = _viewOn ? _vx1 : 1;
        var i0 = (int)Math.Clamp(Math.Floor(x0 * last), 0, last);
        var i1 = (int)Math.Clamp(Math.Ceiling(x1 * last), 0, last);
        if (i0 > 0)
        {
            i0--;
        }

        if (i1 < last)
        {
            i1++;
        }

        var stride = Math.Max(1, (i1 - i0) / 2000);
        for (var i = i0; i <= i1; i += stride)
        {
            pts.Add(new Point(PlotX(i, last), Y(src[i], ymin, ymax)));
        }

        return pts;
    }

    private PointCollection ToPoints(Queue<double> q, double ymin, double ymax)
    {
        var pts = new PointCollection();
        var arr = q.ToArray();
        if (arr.Length == 0)
        {
            return pts;
        }

        var last = arr.Length - 1;
        var x0 = _viewOn ? _vx0 : 0;
        var x1 = _viewOn ? _vx1 : 1;
        var i0 = (int)Math.Clamp(Math.Floor(x0 * last), 0, last);
        var i1 = (int)Math.Clamp(Math.Ceiling(x1 * last), 0, last);
        for (var i = i0; i <= i1; i++)
        {
            pts.Add(new Point(PlotX(i, last), Y(arr[i], ymin, ymax)));
        }

        return pts;
    }

    private void IrefSlider_Changed(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (_syncingIref)
        {
            return;
        }

        IrefCmdBox.Text = e.NewValue.ToString("0.000", CultureInfo.InvariantCulture);
    }

    private void ApplyIref_Click(object sender, RoutedEventArgs e)
    {
        if (!TryParseBox(IrefCmdBox.Text, out var a))
        {
            return;
        }

        a = Math.Clamp(a, -Protocol.IMaxA, Protocol.IMaxA);
        _syncingIref = true;
        IrefSlider.Value = a;
        IrefCmdBox.Text = a.ToString("0.000", CultureInfo.InvariantCulture);
        _syncingIref = false;
        _link.Send(Protocol.IrefLine(a));
    }

    private void Analog_Click(object sender, RoutedEventArgs e) => _link.Send("$A\n");
    private void Force_Click(object sender, RoutedEventArgs e) => _link.Send("$F\n");
    private void Zero_Click(object sender, RoutedEventArgs e) => _link.Send("$Z\n");
    private void ClearFault_Click(object sender, RoutedEventArgs e) => _link.Send("$C\n");

    private void ShowPwmMode(bool tri)
    {
        _syncingMode = true;
        if (tri)
        {
            ModeTri.IsChecked = true;
        }
        else
        {
            ModeBi.IsChecked = true;
        }
        _syncingMode = false;
    }

    private void ModeTri_Click(object sender, RoutedEventArgs e)
    {
        if (_syncingMode)
        {
            return;
        }

        _link.Send("$T\n");
    }

    private void ModeBi_Click(object sender, RoutedEventArgs e)
    {
        if (_syncingMode)
        {
            return;
        }

        _link.Send("$B\n");
    }
    private void ReadParams_Click(object sender, RoutedEventArgs e) => _link.Send("$D\n");

    private void SaveNv_Click(object sender, RoutedEventArgs e)
    {
        _nvPending = true;
        _nvQuietUntil = DateTime.UtcNow.AddSeconds(40);
        NvText.Text = "Flash：正在保存…";
        AppendLog("保存到 Flash");
        _link.SendExclusive("$W\n");
    }

    private void LoadNv_Click(object sender, RoutedEventArgs e) => _link.Send("$L\n");

    private void FactoryNv_Click(object sender, RoutedEventArgs e)
    {
        if (MessageBox.Show("擦除 Flash 参数并恢复 vcm_config.h 默认值？", "恢复出厂",
                MessageBoxButton.OKCancel, MessageBoxImage.Warning) != MessageBoxResult.OK)
        {
            return;
        }

        _nvPending = true;
        _nvQuietUntil = DateTime.UtcNow.AddSeconds(40);
        NvText.Text = "Flash：正在恢复出厂…";
        AppendLog("恢复出厂");
        _link.SendExclusive("$E\n");
    }

    private void ApplyParams_Click(object sender, RoutedEventArgs e)
    {
        if (!TryReadUiParams(out var p, out var err))
        {
            MessageBox.Show(err, "VCM电流环调试");
            return;
        }

        ApplyUiParams(p);
        if (_link.IsOpen)
        {
            SendParams(p);
        }
        else
        {
            MessageBox.Show("串口未连接：已更新界面，未写入板子。", "VCM电流环调试");
        }
    }

    private void ExportParams_Click(object sender, RoutedEventArgs e)
    {
        if (!TryReadUiParams(out var p, out var err))
        {
            MessageBox.Show(err, "导出参数");
            return;
        }

        var dlg = new SaveFileDialog
        {
            Filter = "VCM 参数 (*.vcmparams)|*.vcmparams|JSON (*.json)|*.json",
            FileName = "vcm_loop_params.vcmparams",
            DefaultExt = ".vcmparams",
        };
        if (dlg.ShowDialog() != true)
        {
            return;
        }

        p.Note = "Exported " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture);
        File.WriteAllText(dlg.FileName, ParamFile.ToJson(p), new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));
        AppendLog("导出参数 " + dlg.FileName);
    }

    private void ImportParams_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Filter = "VCM 参数 (*.vcmparams;*.json)|*.vcmparams;*.json|所有文件|*.*",
        };
        if (dlg.ShowDialog() != true)
        {
            return;
        }

        LoopParams p;
        try
        {
            p = ParamFile.FromJson(File.ReadAllText(dlg.FileName));
        }
        catch (Exception ex)
        {
            MessageBox.Show("导入失败：\n" + ex.Message, "导入参数");
            return;
        }

        ApplyUiParams(p);
        AppendLog("导入参数 " + dlg.FileName);

        if (!_link.IsOpen)
        {
            MessageBox.Show("已载入到界面。串口未连接，未写入板子。", "导入参数");
            return;
        }

        var send = MessageBox.Show(
            "参数已载入界面。是否立即写入板子？\n掉电保持仍需再按「保存到 Flash」。",
            "导入参数",
            MessageBoxButton.YesNo,
            MessageBoxImage.Question);
        if (send == MessageBoxResult.Yes)
        {
            SendParams(p);
        }
    }

    private static int ClampNotch(double hz)
    {
        var v = (int)Math.Round(hz);
        if (v <= 0)
        {
            return 0;
        }
        return Math.Clamp(v, 50, 10000);
    }

    private bool TryReadUiParams(out LoopParams p, out string err)
    {
        p = new LoopParams();
        err = "参数无法解析。";
        if (!TryParseBox(KpBox.Text, out var kp) ||
            !TryParseBox(KiBox.Text, out var ki) ||
            !TryParseBox(ObsBox.Text, out var obs) ||
            !TryParseBox(LpfBox.Text, out var lpf) ||
            !TryParseBox(FfBox.Text, out var ff) ||
            !TryParseBox(VbusBox.Text, out var vbus) ||
            !TryParseBox(RcoilBox.Text, out var rOhm) ||
            !TryParseBox(LcoilBox.Text, out var lMh) ||
            !TryParseBox(DtBox.Text, out var dtNs) ||
            !TryParseBox(Notch0Box.Text, out var n0) ||
            !TryParseBox(Notch1Box.Text, out var n1) ||
            !TryParseBox(Notch2Box.Text, out var n2) ||
            !TryParseBox(Notch3Box.Text, out var n3) ||
            !TryParseBox(GsBox.Text, out var gs))
        {
            return false;
        }

        p.Kp = kp;
        p.Ki = ki;
        p.Obs = obs;
        p.Lpf = lpf;
        p.Ff = ff;
        p.Vbus = Math.Clamp(vbus, Protocol.VbusMin, Protocol.VbusMax);
        p.R = rOhm;
        p.LMh = lMh;
        p.Fdbk = FdbkObs.IsChecked == true ? 1 : 0;
        p.DtNs = (int)Math.Clamp(Math.Round(dtNs), 0, 1000);
        p.N0 = ClampNotch(n0);
        p.N1 = ClampNotch(n1);
        p.N2 = ClampNotch(n2);
        p.N3 = ClampNotch(n3);
        p.Gz = GzOn.IsChecked == true ? 1 : 0;
        p.Gs = Math.Clamp(gs, 0.05, 1.0);
        err = "";
        return true;
    }

    private void ApplyUiParams(LoopParams p)
    {
        _syncingObs = true;
        KpBox.Text = ParamFile.Fmt(p.Kp, "0.###");
        KiBox.Text = ParamFile.Fmt(p.Ki, "0.###");
        ObsBox.Text = ParamFile.Fmt(p.Obs, "0.###");
        ObsHzBox.Text = Protocol.AlphaToHz(p.Obs).ToString("0", CultureInfo.InvariantCulture);
        LpfBox.Text = ParamFile.Fmt(p.Lpf, "0.###");
        FfBox.Text = ParamFile.Fmt(p.Ff, "0.#####");
        VbusBox.Text = ParamFile.Fmt(Math.Clamp(p.Vbus, Protocol.VbusMin, Protocol.VbusMax), "0.###");
        RcoilBox.Text = ParamFile.Fmt(p.R, "0.###");
        LcoilBox.Text = ParamFile.Fmt(p.LMh, "0.###");
        DtBox.Text = Math.Clamp(p.DtNs, 0, 1000).ToString(CultureInfo.InvariantCulture);
        Notch0Box.Text = ClampNotch(p.N0).ToString(CultureInfo.InvariantCulture);
        Notch1Box.Text = ClampNotch(p.N1).ToString(CultureInfo.InvariantCulture);
        Notch2Box.Text = ClampNotch(p.N2).ToString(CultureInfo.InvariantCulture);
        Notch3Box.Text = ClampNotch(p.N3).ToString(CultureInfo.InvariantCulture);
        GsBox.Text = ParamFile.Fmt(Math.Clamp(p.Gs <= 0 ? 0.35 : p.Gs, 0.05, 1.0), "0.###");
        var two = p.Gz != 0;
        GzOn.IsChecked = two;
        GzOff.IsChecked = !two;
        var obs = p.Fdbk != 0;
        FdbkObs.IsChecked = obs;
        FdbkAvg.IsChecked = !obs;
        _syncingObs = false;
    }

    private void MeasureDt_Click(object sender, RoutedEventArgs e)
    {
        _link.Send("$K\n");
        AppendLog("测定 t_d：机构请保持静止");
    }

    private void SendParams(LoopParams p)
    {
        var vbus = Math.Clamp(p.Vbus, Protocol.VbusMin, Protocol.VbusMax);
        _link.Send(Protocol.SetLine(p.Kp, p.Ki, p.Obs, p.Lpf, p.Ff, vbus, p.R, p.LMh * 0.001, p.Fdbk, p.DtNs, p.N0, p.N1, p.N2, p.N3, p.Gz, p.Gs));
        AppendLog("写入参数");
    }

    private void ObsAlpha_Changed(object sender, RoutedEventArgs e)
    {
        if (_syncingObs || !TryParseBox(ObsBox.Text, out var a))
        {
            return;
        }

        _syncingObs = true;
        ObsHzBox.Text = Protocol.AlphaToHz(a).ToString("0", CultureInfo.InvariantCulture);
        _syncingObs = false;
    }

    private void ObsHz_Changed(object sender, RoutedEventArgs e)
    {
        if (_syncingObs || !TryParseBox(ObsHzBox.Text, out var hz))
        {
            return;
        }

        _syncingObs = true;
        var a = Protocol.HzToAlpha(hz);
        ObsBox.Text = a.ToString("0.###", CultureInfo.InvariantCulture);
        _syncingObs = false;
    }

    private void ClearLog_Click(object sender, RoutedEventArgs e) => LogBox.Clear();

    private void ExportCsv_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new SaveFileDialog
        {
            Filter = "CSV|*.csv",
            FileName = "vcm_telem.csv"
        };
        if (dlg.ShowDialog() != true)
        {
            return;
        }

        File.WriteAllLines(dlg.FileName, _csv, Encoding.UTF8);
    }

    private static bool TryParseBox(string s, out double v) =>
        double.TryParse(s.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out v) ||
        double.TryParse(s.Trim(), NumberStyles.Float, CultureInfo.CurrentCulture, out v);

    private void AppendLog(string line)
    {
        lock (_logGate)
        {
            if (LogBox.Text.Length > 24000)
            {
                LogBox.Text = LogBox.Text[^8000..];
            }

            LogBox.AppendText(line + Environment.NewLine);
            LogBox.ScrollToEnd();
        }
    }
}
