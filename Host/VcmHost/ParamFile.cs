using System.Globalization;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace VcmHost;

internal sealed class LoopParams
{
    [JsonPropertyName("format")]
    public string Format { get; set; } = "vcm-host-params";

    [JsonPropertyName("version")]
    public int Version { get; set; } = 1;

    [JsonPropertyName("kp")]
    public double Kp { get; set; }

    [JsonPropertyName("ki")]
    public double Ki { get; set; }

    [JsonPropertyName("fdbk")]
    public int Fdbk { get; set; } = 1;

    [JsonPropertyName("obs")]
    public double Obs { get; set; }

    [JsonPropertyName("lpf")]
    public double Lpf { get; set; }

    [JsonPropertyName("ff")]
    public double Ff { get; set; }

    [JsonPropertyName("vbus")]
    public double Vbus { get; set; }

    [JsonPropertyName("r")]
    public double R { get; set; }

    [JsonPropertyName("l_mh")]
    public double LMh { get; set; }

    [JsonPropertyName("dt_ns")]
    public int DtNs { get; set; } = 200;

    [JsonPropertyName("note")]
    public string? Note { get; set; }
}

internal static class ParamFile
{
    public const string FormatId = "vcm-host-params";
    public const int FormatVersion = 1;

    private static readonly JsonSerializerOptions JsonOpts = new()
    {
        WriteIndented = true,
        NumberHandling = JsonNumberHandling.AllowReadingFromString,
        PropertyNameCaseInsensitive = true,
    };

    public static string ToJson(LoopParams p)
    {
        p.Format = FormatId;
        p.Version = FormatVersion;
        return JsonSerializer.Serialize(p, JsonOpts) + Environment.NewLine;
    }

    public static LoopParams FromJson(string text)
    {
        var p = JsonSerializer.Deserialize<LoopParams>(text, JsonOpts)
                ?? throw new InvalidDataException("空文件。");
        if (!string.IsNullOrWhiteSpace(p.Format) &&
            !string.Equals(p.Format, FormatId, StringComparison.OrdinalIgnoreCase))
        {
            throw new InvalidDataException("不是 VCM 环路参数文件（format=" + p.Format + "）。");
        }

        if (p.Version > FormatVersion)
        {
            throw new InvalidDataException("参数文件版本过高：" + p.Version);
        }

        return p;
    }

    public static string Fmt(double v, string fmt) =>
        v.ToString(fmt, CultureInfo.InvariantCulture);
}
