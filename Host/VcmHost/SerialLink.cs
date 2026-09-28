using System.Collections.Concurrent;
using System.IO.Ports;
using System.Text;

namespace VcmHost;

internal sealed class SerialLink : IDisposable
{
    private readonly object _gate = new();
    private readonly ConcurrentQueue<string> _tx = new();
    private readonly StringBuilder _acc = new();
    private SerialPort? _port;
    private CancellationTokenSource? _cts;
    private Task? _io;

    public bool IsOpen
    {
        get
        {
            lock (_gate)
            {
                return _port is { IsOpen: true };
            }
        }
    }

    public event Action<string>? LineReceived;
    public event Action<string>? Error;

    public static string[] ListPorts()
    {
        var names = SerialPort.GetPortNames();
        Array.Sort(names, StringComparer.OrdinalIgnoreCase);
        return names;
    }

    public void Open(string portName, int baud = 115200)
    {
        Close();
        var p = new SerialPort(portName, baud, Parity.None, 8, StopBits.One)
        {
            Handshake = Handshake.None,
            /* DTR/RTS often reset USB-UART STM32 into the IAP window. */
            DtrEnable = false,
            RtsEnable = false,
            NewLine = "\n",
            Encoding = Encoding.ASCII,
            ReadTimeout = 30,
            WriteTimeout = 500,
        };
        p.Open();
        lock (_gate)
        {
            _port = p;
            _cts = new CancellationTokenSource();
            _io = Task.Run(() => IoLoop(_cts.Token));
        }
    }

    public void Send(string text)
    {
        if (string.IsNullOrEmpty(text))
        {
            return;
        }

        _tx.Enqueue(text);
    }

    /* Drop queued polls/commands so $W is not glued to a leftover '$'. */
    public void ClearTx()
    {
        while (_tx.TryDequeue(out _)) { }
    }

    public void SendExclusive(string text)
    {
        ClearTx();
        Send(text);
    }

    public void Close()
    {
        CancellationTokenSource? cts;
        Task? io;
        SerialPort? p;
        lock (_gate)
        {
            cts = _cts;
            io = _io;
            p = _port;
            _cts = null;
            _io = null;
            _port = null;
        }

        try { cts?.Cancel(); } catch { /* ignore */ }
        try { io?.Wait(400); } catch { /* ignore */ }
        try { p?.Close(); } catch { /* ignore */ }
        p?.Dispose();
        cts?.Dispose();
        _acc.Clear();
        while (_tx.TryDequeue(out _)) { }
    }

    public void Dispose() => Close();

    private void IoLoop(CancellationToken ct)
    {
        var buf = new byte[256];
        while (!ct.IsCancellationRequested)
        {
            SerialPort? p;
            lock (_gate)
            {
                p = _port;
            }

            if (p is not { IsOpen: true })
            {
                break;
            }

            try
            {
                while (_tx.TryDequeue(out var msg))
                {
                    p.Write(msg);
                }

                int n;
                try
                {
                    n = p.Read(buf, 0, buf.Length);
                }
                catch (TimeoutException)
                {
                    continue;
                }

                if (n <= 0)
                {
                    continue;
                }

                for (var i = 0; i < n; i++)
                {
                    var c = (char)buf[i];
                    if (c == '\n' || c == '\r')
                    {
                        if (_acc.Length > 0)
                        {
                            var line = _acc.ToString();
                            _acc.Clear();
                            LineReceived?.Invoke(line);
                        }
                    }
                    else if (_acc.Length < 512)
                    {
                        _acc.Append(c);
                    }
                    else
                    {
                        _acc.Clear();
                    }
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                Error?.Invoke(ex.Message);
                break;
            }
        }
    }
}
