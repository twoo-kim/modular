#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# --- 共通ユーティリティ関数 ---
def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0

def unwrap_deg(a):
    return np.rad2deg(np.unwrap(np.deg2rad(a)))

def infer_time_seconds(t: np.ndarray) -> tuple[np.ndarray, str]:
    t = np.asarray(t, dtype=float)
    mx = np.nanmax(t)
    if mx > 1e9: return t * 1e-6, "Time [s] (auto: us->s)"
    if mx > 1e6: return t * 1e-3, "Time [s] (auto: ms->s)"
    return t, "Time [s]"

def moving_average(y: np.ndarray, win: int) -> np.ndarray:
    if win <= 1: return y
    win = int(win)
    k = np.ones(win, dtype=float) / win
    y_pad = np.r_[y[0] * np.ones(win - 1), y]
    return np.convolve(y_pad, k, mode="valid")

def find_active_segments(t: np.ndarray, active: np.ndarray, min_len_s: float):
    t = np.asarray(t, float)
    active = np.asarray(active, bool)
    segs = []
    n = len(active)
    i = 0
    while i < n:
        if not active[i]:
            i += 1
            continue
        j = i
        while j < n and active[j]:
            j += 1
        dur = t[j-1] - t[i] if j-1 >= i else 0.0
        if dur >= min_len_s:
            segs.append((i, j-1))
        i = j
    return segs

def butter2_lowpass_biquad(fc_hz: float, fs_hz: float):
    Q = 1.0 / np.sqrt(2.0)
    w0 = 2.0 * np.pi * fc_hz / fs_hz
    cosw0 = np.cos(w0)
    alpha = np.sin(w0) / (2.0 * Q)
    b0, b1, b2 = (1 - cosw0) / 2, 1 - cosw0, (1 - cosw0) / 2
    a0, a1, a2 = 1 + alpha, -2 * cosw0, 1 - alpha
    return np.array([b0/a0, b1/a0, b2/a0]), np.array([1.0, a1/a0, a2/a0])

def lfilter_biquad(x, b, a):
    y = np.empty_like(x)
    x1 = x2 = y1 = y2 = 0.0
    for i, x0 in enumerate(x):
        y0 = b[0]*x0 + b[1]*x1 + b[2]*x2 - a[1]*y1 - a[2]*y2
        y[i] = y0
        x2, x1, y2, y1 = x1, x0, y1, y0
    return y

def align_yaxis_zeros(ax1, ax2):
    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax2.get_ylim()
    p1 = -y1_min / (y1_max - y1_min)
    p2 = -y2_min / (y2_max - y2_min)
    if p1 < p2:
        new_min1 = -p2 * y1_max / (1 - p2)
        ax1.set_ylim(new_min1, y1_max)
    else:
        new_min2 = -p1 * y2_max / (1 - p1)
        ax2.set_ylim(new_min2, y2_max)

# --- メイン処理 ---
def main():
    ap = argparse.ArgumentParser(description="Integrated Angle & Sensor Data Processor")
    ap.add_argument("csv_coord", type=Path, help="Coordinate CSV")
    ap.add_argument("csv_sensor", type=Path, help="Sensor CSV")
    ap.add_argument("--ccw_positive", action="store_true", help="Counter-clockwise positive")
    ap.add_argument("--q0", type=float, default=-22.5, help="q offset (deg)")
    ap.add_argument("--target_start", type=float, default=3.0, help="2nd motion alignment time [s]")
    ap.add_argument("--wrap", action="store_true", help="Wrap q to [-180, 180)")
    ap.add_argument("--no_csv", action="store_true", help="Disable intermediate CSV saving")
    
    args = ap.parse_args()

    # --- 0. タイトル抽出 ---
    fname_stem = args.csv_sensor.stem
    parts = fname_stem.split('_')
    # _ で区切られた2番目と3番目を結合し、余計なドット(.)があれば削除
    if len(parts) >= 3:
        plot_title = f"{parts[1]}_{parts[2]}".replace(".", "")
    else:
        plot_title = fname_stem

    # --- 1. 座標データ処理 ---
    REQ = ["P1_x","P1_y","P2_x","P2_y","P3_x","P3_y"]
    try:
        df_c = pd.read_csv(args.csv_coord)
    except:
        df_c = pd.read_csv(args.csv_coord, sep="\t")

    t_col = "Time" if "Time" in df_c.columns else df_c.columns[0]
    t_raw = pd.to_numeric(df_c[t_col], errors="coerce").to_numpy(dtype=float)
    ok = np.isfinite(t_raw)
    for c in REQ:
        ok &= np.isfinite(pd.to_numeric(df_c[c], errors="coerce").to_numpy(dtype=float))
    
    df_c = df_c.loc[ok].reset_index(drop=True)
    t_c, _ = infer_time_seconds(t_raw[ok])
    t_c = t_c - t_c[0]

    P1 = df_c[["P1_x","P1_y"]].to_numpy(float)
    P2 = df_c[["P2_x","P2_y"]].to_numpy(float)
    P3 = df_c[["P3_x","P3_y"]].to_numpy(float)
    v12, v23 = P2 - P1, P3 - P2
    if args.ccw_positive:
        v12[:, 1] *= -1.0; v23[:, 1] *= -1.0

    theta12_u = unwrap_deg(np.rad2deg(np.arctan2(v12[:, 1], v12[:, 0])))
    theta23_u = unwrap_deg(np.rad2deg(np.arctan2(v23[:, 1], v23[:, 0])))
    q = (theta23_u - theta12_u) + args.q0
    q_plot = wrap_deg(q) if args.wrap else q

    # 時間軸補正
    dt = np.diff(t_c, prepend=t_c[0])
    activity = (np.abs(np.gradient(theta12_u)) + np.abs(np.gradient(q))) / np.where(dt > 0, dt, 1e-6)
    act_s = moving_average(np.nan_to_num(activity), max(1, int(0.20 * (1.0 / np.median(dt)))))
    thr = 0.20 * np.percentile(act_s, 95) if len(act_s) > 0 else 0
    segs = find_active_segments(t_c, act_s > thr, min_len_s=0.40)
    
    t2 = 0.0
    if len(segs) >= 2:
        e1 = segs[0][1]
        for (s_i, e_i) in segs[1:]:
            if t_c[s_i] - t_c[e1] >= 0.60:
                t2 = t_c[s_i]; break
    t_c_shifted = t_c - t2 + args.target_start

    # --- 2. センサデータ処理 ---
    df_s = pd.read_csv(args.csv_sensor).dropna(subset=["t_us", "Fz"]).reset_index(drop=True)
    t_s = (df_s["t_us"] - df_s["t_us"].iloc[0]) * 1e-6
    fz_raw = df_s["Fz"].to_numpy()
    state_s = df_s["state"].astype(str).str.strip().str.lower().to_numpy()

    mask0 = (state_s == "zero") | (state_s == "0")
    offset = np.nanmean(fz_raw[mask0]) if mask0.sum() > 5 else 0.0
    fz_off = fz_raw - offset

    fs_s = 1.0 / np.nanmedian(np.diff(t_s))
    try:
        from scipy.signal import butter, filtfilt
        b, a = butter(2, 10.0 / (fs_s / 2.0), btype="low")
        fz_lp = filtfilt(b, a, fz_off)
    except:
        bb, aa = butter2_lowpass_biquad(10.0, fs_s)
        fz_lp = lfilter_biquad(fz_off, bb, aa)

# --- 3. CSV保存 ---
    if not args.no_csv:
        # 1. 全期間の角度データ（位置合わせ後のバックアップ用）
        out_csv_angle_all = args.csv_coord.with_name(f"{plot_title}_angle_all.csv")
        df_angle_all = pd.DataFrame({"time_shifted": t_c_shifted, "theta12_deg": theta12_u, "q_deg": q_plot})
        df_angle_all.to_csv(out_csv_angle_all, index=False)
        
        # 2. 【修正】5-20s 角度データ（最終解析用）
        mask_ca = (t_c_shifted >= 5.0) & (t_c_shifted <= 20.0)
        df_angle_clip = df_angle_all[mask_ca].copy()
        out_csv_angle_clip = args.csv_coord.with_name(f"{plot_title}_angle.csv") # ← ここを変更
        df_angle_clip.to_csv(out_csv_angle_clip, index=False)
        
        # 3. 【修正】5-20s センサデータ（最終解析用）
        mask_cs = (t_s >= 5.0) & (t_s <= 20.0)
        df_sensor_clip = pd.DataFrame({"time_s": t_s[mask_cs], "fz_lp_N": fz_lp[mask_cs], "state": state_s[mask_cs]})
        out_csv_sensor_clip = args.csv_sensor.with_name(f"{plot_title}_force.csv") # ← ここを変更
        df_sensor_clip.to_csv(out_csv_sensor_clip, index=False)
        
        print(f"[INFO] CSVs saved: \n - {out_csv_angle_all}\n - {out_csv_angle_clip}\n - {out_csv_sensor_clip}")

    # --- 4. プロットとPDF保存 ---
    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax1.axhline(0, color='black', lw=1.0, alpha=0.3)
    ln1 = ax1.plot(t_c_shifted, theta12_u, label="active joint", color="tab:blue", lw=1.5)
    ln2 = ax1.plot(t_c_shifted, q_plot, label="passive joint", color="tab:cyan", lw=1.5)
    ax1.set_xlabel("Time [s]"); ax1.set_ylabel("Angle [deg]")
    ax1.set_xlim(0, 25); ax1.grid(True, linestyle='--', alpha=0.5)

    ax2 = ax1.twinx()
    ln3 = ax2.plot(t_s, fz_lp, label="Fz (10Hz LPF)", color="tab:red", lw=1.2, alpha=0.8)
    ax2.set_ylabel("Force Fz [N]")

    align_yaxis_zeros(ax1, ax2)
    lns = ln1 + ln2 + ln3
    ax1.legend(lns, [l.get_label() for l in lns], loc="upper right")
    plt.title(plot_title)
    fig.tight_layout()

    out_pdf = args.csv_sensor.with_name(f"{plot_title}.pdf")
    plt.savefig(out_pdf, format='pdf', dpi=300)
    print(f"[INFO] PDF saved: {out_pdf}")

    plt.show()

if __name__ == "__main__":
    main()    # --- 3. CSV保存 (修正箇所) ---
