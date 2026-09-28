using System.Globalization;

namespace VcmHost;

internal static class Protocol
{
    public const int PwmHz = 50000;
    public const double IMaxA = 10.0;
    public const double VbusMin = 24.0;
    public const double VbusMax = 48.0;

    public static Dictionary<string, long> ParseFields(string line)
    {
        var map = new Dictionary<string, long>(StringComparer.Ordinal);
        if (string.IsNullOrWhiteSpace(line))
        {
            return map;
        }

        foreach (var tok in line.Split(' ', StringSplitOptions.RemoveEmptyEntries))
        {
            var eq = tok.IndexOf('=');
            if (eq <= 0 || eq == tok.Length - 1)
            {
                continue;
            }

            var key = tok[..eq];
            if (long.TryParse(tok[(eq + 1)..], NumberStyles.Integer, CultureInfo.InvariantCulture, out var v))
            {
                map[key] = v;
            }
        }

        return map;
    }

    public static double Milli(Dictionary<string, long> m, string key) =>
        m.TryGetValue(key, out var v) ? v / 1000.0 : double.NaN;

    public static double Micro(Dictionary<string, long> m, string key) =>
        m.TryGetValue(key, out var v) ? v / 1_000_000.0 : double.NaN;

    public static int Int(Dictionary<string, long> m, string key, int fallback = 0) =>
        m.TryGetValue(key, out var v) ? (int)v : fallback;

    public static double AlphaToHz(double a)
    {
        a = Math.Clamp(a, 0.0, 0.999);
        if (a <= 0.0)
        {
            return 0.0;
        }

        return -Math.Log(1.0 - a) * PwmHz / (2.0 * Math.PI);
    }

    public static double HzToAlpha(double hz)
    {
        hz = Math.Clamp(hz, 0.0, PwmHz * 0.4);
        return 1.0 - Math.Exp(-2.0 * Math.PI * hz / PwmHz);
    }

    public static string SetLine(double kp, double ki, double obs, double lpf, double ff,
        double vbus, double rOhm, double lHenry)
    {
        var kpE3 = (int)Math.Round(kp * 1000.0);
        var kiE3 = (int)Math.Round(ki * 1000.0);
        var obsE3 = (int)Math.Round(obs * 1000.0);
        var lpfE3 = (int)Math.Round(lpf * 1000.0);
        var ffE6 = (int)Math.Round(ff * 1_000_000.0);
        var vbusE3 = (int)Math.Round(vbus * 1000.0);
        var rE3 = (int)Math.Round(rOhm * 1000.0);
        var lE6 = (int)Math.Round(lHenry * 1_000_000.0);
        return $"$S kp={kpE3} ki={kiE3} obs={obsE3} lpf={lpfE3} ff={ffE6} vbus={vbusE3} r={rE3} l={lE6}\n";
    }

    public static string IrefLine(double amps)
    {
        var ma = (int)Math.Round(Math.Clamp(amps, -IMaxA, IMaxA) * 1000.0);
        return $"$R {ma}\n";
    }

    public static string StateName(int s) => s switch
    {
        0 => "INIT",
        1 => "IDLE",
        2 => "CALIB",
        3 => "RUN",
        4 => "FAULT",
        _ => $"?{s}"
    };
}
