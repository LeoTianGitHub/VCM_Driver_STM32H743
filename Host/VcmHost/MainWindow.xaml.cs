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
    private readonly List<string> _csv = new();
    private readonly object _logGate = new();
    private bool _syncingIref;
    private bool _syncingObs;
    private DateTime _lastPoll = DateTime.MinValue;
    private bool _awaitingTl;
    private DateTime _nvQuietUntil = DateTime.MinValue;
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
            MessageBox.Show("请选择串口。", "VCM Host");
            return;
        }

        try
        {
            _link.Open(port);
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
        NvText.Text = "Flash：未读（nv=）";
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

        if (_nvPending && DateTime.UtcNow >= _nvQuietUntil)
        {
            _nvPending = false;
            NvText.Text = "Flash：超时无应答。请用 CubeIDE 烧录当前固件并复位后再保存。";
            AppendLog("NV TIMEOUT");
        }

        if (PollBox.IsChecked != true)
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
        Dispatcher.BeginInvoke(() => HandleLine(line));
    }

    private void HandleLine(string line)
    {
        AppendLog(line);
        if (line.StartsWith("TL ", StringComparison.Ordinal))
        {
            _awaitingTl = false;
            ApplyTelemetry(Protocol.ParseFields(line));
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
                _nvQuietUntil = DateTime.UtcNow.AddSeconds(8);
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
            $"ov={Protocol.Int(m, "ov")}  hold={Protocol.Int(m, "hold")}  armed={Protocol.Int(m, "armed")}  pair={Protocol.Int(m, "pair")}  ovr={Protocol.Int(m, "ovr1")}/{Protocol.Int(m, "ovr2")}";
        SampText.Text =
            $"ip = {FmtA(ip)} A\nim = {FmtA(im)} A\nADC IFB = {Protocol.Int(m, "adc_ifb")}\nADC IREF = {Protocol.Int(m, "adc_iref")}\nticks = {Protocol.Int(m, "ticks")}";

        var mode = Protocol.Int(m, "mode");
        ModeTri.IsChecked = mode != 0;
        ModeBi.IsChecked = mode == 0;

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

    private void Redraw()
    {
        var w = Plot.ActualWidth;
        var h = Plot.ActualHeight;
        if (w < 8 || h < 8)
        {
            return;
        }

        ZeroLine.X1 = 0;
        ZeroLine.X2 = w;
        var ymin = -0.2;
        var ymax = 0.2;
        foreach (var q in new[] { _iref, _hat, _avg })
        {
            foreach (var v in q)
            {
                ymin = Math.Min(ymin, v);
                ymax = Math.Max(ymax, v);
            }
        }

        var pad = Math.Max(0.05, 0.08 * (ymax - ymin));
        ymin -= pad;
        ymax += pad;
        ZeroLine.Y1 = ZeroLine.Y2 = Y(0, ymin, ymax, h);
        LineIref.Points = ToPoints(_iref, w, h, ymin, ymax);
        LineHat.Points = ToPoints(_hat, w, h, ymin, ymax);
        LineAvg.Points = ToPoints(_avg, w, h, ymin, ymax);
    }

    private static double Y(double v, double ymin, double ymax, double h) =>
        h - ((v - ymin) / (ymax - ymin) * h);

    private static PointCollection ToPoints(Queue<double> q, double w, double h, double ymin, double ymax)
    {
        var pts = new PointCollection();
        var arr = q.ToArray();
        if (arr.Length == 0)
        {
            return pts;
        }

        var n = Math.Max(arr.Length - 1, 1);
        for (var i = 0; i < arr.Length; i++)
        {
            var x = i / (double)n * w;
            pts.Add(new Point(x, Y(arr[i], ymin, ymax, h)));
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
    private void ModeTri_Click(object sender, RoutedEventArgs e) => _link.Send("$T\n");
    private void ModeBi_Click(object sender, RoutedEventArgs e) => _link.Send("$B\n");
    private void ReadParams_Click(object sender, RoutedEventArgs e) => _link.Send("$D\n");

    private void SaveNv_Click(object sender, RoutedEventArgs e)
    {
        _nvPending = true;
        _nvQuietUntil = DateTime.UtcNow.AddSeconds(15);
        NvText.Text = "Flash：正在擦写 Sector3…";
        AppendLog("发送 $W");
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
        _nvQuietUntil = DateTime.UtcNow.AddSeconds(15);
        NvText.Text = "Flash：正在擦除 Sector3…";
        AppendLog("发送 $E");
        _link.SendExclusive("$E\n");
    }

    private void ApplyParams_Click(object sender, RoutedEventArgs e)
    {
        if (!TryParseBox(KpBox.Text, out var kp) ||
            !TryParseBox(KiBox.Text, out var ki) ||
            !TryParseBox(ObsBox.Text, out var obs) ||
            !TryParseBox(LpfBox.Text, out var lpf) ||
            !TryParseBox(FfBox.Text, out var ff) ||
            !TryParseBox(VbusBox.Text, out var vbus) ||
            !TryParseBox(RcoilBox.Text, out var rOhm) ||
            !TryParseBox(LcoilBox.Text, out var lMh))
        {
            MessageBox.Show("参数无法解析。", "VCM Host");
            return;
        }

        vbus = Math.Clamp(vbus, Protocol.VbusMin, Protocol.VbusMax);
        VbusBox.Text = vbus.ToString("0.###", CultureInfo.InvariantCulture);
        _link.Send(Protocol.SetLine(kp, ki, obs, lpf, ff, vbus, rOhm, lMh * 0.001));
    }

    private void Plant_Changed(object sender, RoutedEventArgs e)
    {
        if (!TryParseBox(VbusBox.Text, out var vbus) ||
            !TryParseBox(RcoilBox.Text, out var rOhm))
        {
            return;
        }

        vbus = Math.Clamp(vbus, Protocol.VbusMin, Protocol.VbusMax);
        var ff = (rOhm + 0.02) / (2.0 * vbus);
        FfBox.Text = ff.ToString("0.#####", CultureInfo.InvariantCulture);
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
