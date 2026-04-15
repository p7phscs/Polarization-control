#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pola_control_unificado_stokes.py
================================

Versão unificada para Raspberry Pi + ADS1115 + MPC320, combinando:
- aquisição em thread dedicada para maior taxa de amostragem;
- controle em thread separada, usando as amostras mais recentes da thread de aquisição;
- parâmetros de Stokes S1 e S2 calculados em tempo real;
- salvamento no estilo do fluxo do pola_control_2 (timeseries + controller, com toggle opcional);
- seleção explícita dos pedais ativos na compensação;
- power_impactdet e power_impact_geral integrados ao mesmo arquivo.

Requisitos:
  pip install ADS1x15-ADC qmi numpy matplotlib
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
import math
import select
import termios
import tty
import threading
import statistics
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime

try:
    from tools.utils import get_port
except Exception:
    get_port = None

import ADS1x15
import numpy as np
#import matplotlib.pyplot as plt

DEFAULT_SAVE_DIR = "/home/f1234/Documents/pola_control/MEDIDAS"
DEFAULT_ADDRS = [0x48]
MPC320_VID = 1027
MPC320_PID = 64240
EPS = 1e-9
ADS_IO_DELAY_S = 0.0
USB_IO_DELAY_S = 0.002
SAFE_FALLBACK_VELOCITY = 100


# -----------------------------------------------------------------------------
# Utilitários gerais
# -----------------------------------------------------------------------------
def sanitize_place(place: str) -> str:
    return "".join(c if c.isalnum() or c in "-_" else "-" for c in place)


def _import_qmi():
    from qmi.core.context import QMI_Context
    from qmi.instruments.thorlabs.mpc320 import Thorlabs_Mpc320
    from qmi.instruments.thorlabs.apt_protocol import AptChannelJogDirection
    return QMI_Context, Thorlabs_Mpc320, AptChannelJogDirection


def build_default_mapping(num_ads: int):
    if num_ads <= 1:
        return [("h", 0, 0), ("v", 0, 1), ("d", 0, 2), ("a", 0, 3)]
    return [("h", 0, 0), ("v", 0, 1), ("d", 0, 2), ("a", 0, 3), ("l", 1, 0), ("r", 1, 1)]


def init_ads_modules(bus: int, addrs: list[int], gain_const: int, datarate_idx: int):
    ads_list = []
    for addr in addrs:
        ads = ADS1x15.ADS1115(bus, addr)
        ads.setGain(gain_const)
        ads.setDataRate(datarate_idx)
        ads_list.append(ads)
    f_list = [ads.toVoltage() for ads in ads_list]
    return ads_list, f_list


def safe_read_all(ads_list, f_list, mapping, io_delay_s: float = ADS_IO_DELAY_S):
    out = []
    for _, ads_i, ch in mapping:
        try:
            raw = ads_list[ads_i].readADC(ch)
            val = raw * f_list[ads_i]
            out.append(float(val))
        except Exception:
            out.append(float("nan"))
        if io_delay_s > 0:
            time.sleep(io_delay_s)
    return out


def safe_read_one(ads_list, f_list, mapping, det_key: str, io_delay_s: float = ADS_IO_DELAY_S):
    vals = safe_read_all(ads_list, f_list, mapping, io_delay_s=io_delay_s)
    vals_dict = dict(zip([name for name, _, _ in mapping], vals))
    return float(vals_dict.get(det_key, float("nan")))


def normalize_pair(x: float, y: float, eps_norm: float = 1e-12):
    s = float(x) + float(y)
    if not math.isfinite(s) or abs(s) <= eps_norm:
        return float("nan"), float("nan"), s
    return float(x) / s, float(y) / s, s


def compute_stokes_from_vals(vals: dict):
    h = float(vals.get("h", 0.0))
    v = float(vals.get("v", 0.0))
    d = float(vals.get("d", 0.0))
    a = float(vals.get("a", 0.0))

    h_norm, v_norm, hv_sum = normalize_pair(h, v)
    d_norm, a_norm, da_sum = normalize_pair(d, a)

    s1 = (h - v) / (hv_sum + EPS) if math.isfinite(hv_sum) else float("nan")
    s2 = (d - a) / (da_sum + EPS) if math.isfinite(da_sum) else float("nan")

    return {
        "h": h,
        "v": v,
        "d": d,
        "a": a,
        "h_norm": h_norm,
        "v_norm": v_norm,
        "d_norm": d_norm,
        "a_norm": a_norm,
        "hv_sum": hv_sum,
        "da_sum": da_sum,
        "s1": s1,
        "s2": s2,
    }


def minimize_to_control_basis(minimize_name: str) -> str:
    m = minimize_name.lower()
    table = {
        "h": "HV_VMAX",
        "v": "HV_HMAX",
        "d": "DA_AMAX",
        "a": "DA_DMAX",
    }
    if m not in table:
        raise ValueError(f"--minimize inválido: {minimize_name}")
    return table[m]


def compute_control_score(vals: dict, basis: str):
    st = compute_stokes_from_vals(vals)
    s1 = st["s1"]
    s2 = st["s2"]
    hv_sum = st["hv_sum"]
    da_sum = st["da_sum"]

    if basis == "HV_HMAX":
        return s1, hv_sum, "S1"
    if basis == "HV_VMAX":
        return -s1, hv_sum, "-S1"
    if basis == "DA_DMAX":
        return s2, da_sum, "S2"
    if basis == "DA_AMAX":
        return -s2, da_sum, "-S2"
    raise ValueError(f"Base de controle inválida: {basis}")


def signal_ok_from_vals(vals: dict, basis: str, signal_sum_min: float):
    _, den, _ = compute_control_score(vals, basis)
    return math.isfinite(den) and den >= signal_sum_min


def score_to_error(score: float) -> float:
    if not math.isfinite(score):
        return float("inf")
    return max(0.0, 1.0 - score)


def step_from_score(score: float, step_large: float, step_medium: float, step_small: float, step_fine: float):
    err = score_to_error(score)
    if err > 0.40:
        return step_large
    if err > 0.20:
        return step_medium
    if err > 0.08:
        return step_small
    return step_fine


def _mean_finite(values):
    vals = [float(x) for x in values if math.isfinite(x)]
    if not vals:
        return float("nan")
    return sum(vals) / len(vals)


# -----------------------------------------------------------------------------
# MPC helpers
# -----------------------------------------------------------------------------
def resolve_mpc_port(serial_port=None):
    if serial_port:
        return serial_port
    if get_port is not None:
        try:
            port = get_port(MPC320_VID, MPC320_PID, "MPC320")
            if port:
                return port
        except Exception:
            pass
    if os.path.exists("/dev/ttyUSB0"):
        return "/dev/ttyUSB0"
    raise RuntimeError("MPC320 não encontrado.")


def safe_set_polarisation_parameters(mpc, velocity: int):
    try:
        mpc.set_polarisation_parameters(
            velocity=int(velocity), home_pos=0,
            jog_step1=8, jog_step2=8, jog_step3=8,
        )
        time.sleep(USB_IO_DELAY_S)
        return int(velocity)
    except Exception:
        mpc.set_polarisation_parameters(
            velocity=int(SAFE_FALLBACK_VELOCITY), home_pos=0,
            jog_step1=8, jog_step2=8, jog_step3=8,
        )
        time.sleep(USB_IO_DELAY_S)
        return int(SAFE_FALLBACK_VELOCITY)


def mpc_open(serial_port: str | None = None, velocity: int = 150):
    QMI_Context, Thorlabs_Mpc320, AptChannelJogDirection = _import_qmi()
    port = resolve_mpc_port(serial_port)
    print(f"[MPC] conectando em: {port}")
    ctx = QMI_Context("mpc_context")
    ctx.start()
    mpc = Thorlabs_Mpc320(context=ctx, name="mpc", transport=port)
    mpc.open()
    time.sleep(USB_IO_DELAY_S)
    mpc.enable_channels([1, 2, 3])
    time.sleep(USB_IO_DELAY_S)
    velocity_applied = safe_set_polarisation_parameters(mpc, velocity)
    return ctx, mpc, AptChannelJogDirection, velocity_applied


def mpc_close(ctx, mpc):
    try:
        if mpc is not None:
            try:
                mpc.close()
            except Exception:
                pass
    finally:
        if ctx is not None:
            try:
                ctx.stop()
            except Exception:
                pass


def safe_move_abs(mpc, ch: int, pos_deg: float, bounds_min: float, bounds_max: float,
                  io_delay_s: float = USB_IO_DELAY_S):
    pos_deg = max(bounds_min, min(bounds_max, float(pos_deg)))
    try:
        mpc.move_absolute(ch, pos_deg)
        if io_delay_s > 0:
            time.sleep(io_delay_s)
        return pos_deg
    except Exception:
        if io_delay_s > 0:
            time.sleep(io_delay_s)
        return None


def home_pos_blind(mpc, io_delay_s: float = USB_IO_DELAY_S):
    for ch in (1, 2, 3):
        try:
            mpc.move_absolute(ch, 0.0)
        except Exception:
            pass
        if io_delay_s > 0:
            time.sleep(io_delay_s)


def move_def_blind(mpc, pos_deg: float = 80.0, io_delay_s: float = USB_IO_DELAY_S):
    for ch in (1, 2, 3):
        try:
            mpc.move_absolute(ch, float(pos_deg))
        except Exception:
            pass
        if io_delay_s > 0:
            time.sleep(io_delay_s)


# -----------------------------------------------------------------------------
# Estado compartilhado
# -----------------------------------------------------------------------------
@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    running: bool = True
    t_rel: float = 0.0
    sample_idx: int = 0
    sample_rate_hz: float = 0.0
    raw_vals: dict = field(default_factory=dict)
    stokes: dict = field(default_factory=dict)
    pos_cmd: dict = field(default_factory=lambda: {1: 90.0, 2: 90.0, 3: 90.0})
    active_paddles: list[int] = field(default_factory=lambda: [1, 2, 3])
    control_basis: str | None = None
    control_score: float = float("nan")
    score_label: str = "score"
    last_action: str = "idle"
    last_step: float = float("nan")
    last_control_t: float = float("nan")
    last_control_paddle: int = 0
    ts_logger: object = None
    ctrl_logger: object = None

    def snapshot(self):
        with self.lock:
            return {
                "t_rel": self.t_rel,
                "sample_idx": self.sample_idx,
                "sample_rate_hz": self.sample_rate_hz,
                "raw_vals": dict(self.raw_vals),
                "stokes": dict(self.stokes),
                "pos_cmd": dict(self.pos_cmd),
                "active_paddles": list(self.active_paddles),
                "control_basis": self.control_basis,
                "control_score": self.control_score,
                "score_label": self.score_label,
                "last_action": self.last_action,
                "last_step": self.last_step,
                "last_control_t": self.last_control_t,
                "last_control_paddle": self.last_control_paddle,
            }


# -----------------------------------------------------------------------------
# Loggers
# -----------------------------------------------------------------------------
class TimeSeriesLogger:
    def __init__(self, save_dir, place, flush_every=300):
        self.save_dir = save_dir
        self.place = place
        self.flush_every = max(1, int(flush_every))
        self.filepath = None
        self._file = None
        self._writer = None
        self._rows_since_flush = 0

    def start(self):
        os.makedirs(self.save_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        safe_place = sanitize_place(self.place)
        self.filepath = os.path.join(self.save_dir, f"serie_{safe_place}_{timestamp}.csv")
        self._file = open(self.filepath, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "t", "sample_idx", "fs_est_hz",
            "ten_h", "ten_v", "ten_d", "ten_a",
            "h_norm", "v_norm", "d_norm", "a_norm",
            "hv_sum", "da_sum", "S1", "S2",
            "control_basis", "score_label", "control_score",
            "cmd_ch1", "cmd_ch2", "cmd_ch3",
            "last_step_deg", "last_action", "last_control_paddle",
        ])
        self._file.flush()

    def write_row(self, row):
        if self._writer is None:
            return
        self._writer.writerow(row)
        self._rows_since_flush += 1
        if self._rows_since_flush >= self.flush_every:
            self._file.flush()
            self._rows_since_flush = 0

    def close(self):
        if self._file is not None:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None


class ControllerLogger:
    def __init__(self, save_dir, place, flush_every=300):
        self.save_dir = save_dir
        self.place = place
        self.flush_every = max(1, int(flush_every))
        self.filepath = None
        self._file = None
        self._writer = None
        self._rows_since_flush = 0

    def start(self):
        os.makedirs(self.save_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        safe_place = sanitize_place(self.place)
        self.filepath = os.path.join(self.save_dir, f"controller_{safe_place}_{timestamp}.csv")
        self._file = open(self.filepath, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "t", "sample_idx", "paddle", "action",
            "control_basis", "score_label", "score_before", "score_after", "step_deg",
            "S1_before", "S2_before", "S1_after", "S2_after",
            "cmd_ch1", "cmd_ch2", "cmd_ch3",
        ])
        self._file.flush()

    def write_row(self, row):
        if self._writer is None:
            return
        self._writer.writerow(row)
        self._rows_since_flush += 1
        if self._rows_since_flush >= self.flush_every:
            self._file.flush()
            self._rows_since_flush = 0

    def close(self):
        if self._file is not None:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None


class KeyPoller:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def poll(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None


# -----------------------------------------------------------------------------
# Aquisição e controle
# -----------------------------------------------------------------------------
def acquisition_thread_func(state: SharedState, ads_list, f_list, mapping, args, t0):
    last_t = None
    while state.running:
        now = time.perf_counter()
        t_rel = now - t0
        vals = safe_read_all(ads_list, f_list, mapping, io_delay_s=args.ads_io_delay_s)
        vals_dict = dict(zip([name for name, _, _ in mapping], vals))
        st = compute_stokes_from_vals(vals_dict)

        fs = float("nan")
        if last_t is not None:
            dt = now - last_t
            if dt > 0:
                fs = 1.0 / dt
        last_t = now

        with state.lock:
            state.t_rel = t_rel
            state.sample_idx += 1
            state.raw_vals = vals_dict
            state.stokes = st
            state.sample_rate_hz = fs
            control_basis = state.control_basis
            score_label = state.score_label
            control_score = state.control_score
            pos_cmd = dict(state.pos_cmd)
            last_step = state.last_step
            last_action = state.last_action
            last_control_paddle = state.last_control_paddle
            sample_idx = state.sample_idx

        if control_basis is not None:
            current_score, _, label = compute_control_score(vals_dict, control_basis)
            if not math.isfinite(control_score):
                control_score = current_score
            score_label = label

        with state.lock:
            ts_logger = state.ts_logger

        if ts_logger is not None:
            ts_logger.write_row([
                f"{t_rel:.6f}",
                sample_idx,
                "" if not math.isfinite(fs) else f"{fs:.6f}",
                f"{st['h']:.6f}", f"{st['v']:.6f}", f"{st['d']:.6f}", f"{st['a']:.6f}",
                "" if not math.isfinite(st['h_norm']) else f"{st['h_norm']:.6f}",
                "" if not math.isfinite(st['v_norm']) else f"{st['v_norm']:.6f}",
                "" if not math.isfinite(st['d_norm']) else f"{st['d_norm']:.6f}",
                "" if not math.isfinite(st['a_norm']) else f"{st['a_norm']:.6f}",
                "" if not math.isfinite(st['hv_sum']) else f"{st['hv_sum']:.6f}",
                "" if not math.isfinite(st['da_sum']) else f"{st['da_sum']:.6f}",
                "" if not math.isfinite(st['s1']) else f"{st['s1']:.6f}",
                "" if not math.isfinite(st['s2']) else f"{st['s2']:.6f}",
                control_basis or "",
                score_label or "",
                "" if not math.isfinite(control_score) else f"{control_score:.6f}",
                f"{pos_cmd[1]:.6f}", f"{pos_cmd[2]:.6f}", f"{pos_cmd[3]:.6f}",
                "" if not math.isfinite(last_step) else f"{last_step:.6f}",
                last_action,
                last_control_paddle,
            ])

        if args.period > 0:
            time.sleep(args.period)


def wait_for_new_samples(state: SharedState, last_sample_idx: int, min_new_samples: int,
                         timeout_s: float, poll_sleep_s: float = 0.0005):
    deadline = time.perf_counter() + max(0.0, timeout_s)
    while time.perf_counter() < deadline and state.running:
        with state.lock:
            current_idx = state.sample_idx
            vals = dict(state.raw_vals)
            st = dict(state.stokes)
            t_rel = state.t_rel
        if current_idx >= last_sample_idx + min_new_samples and vals:
            return current_idx, vals, st, t_rel
        time.sleep(poll_sleep_s)
    with state.lock:
        return state.sample_idx, dict(state.raw_vals), dict(state.stokes), state.t_rel


def average_shared_measurement(state: SharedState, start_idx: int, n_avg: int, timeout_s: float):
    scores = []
    s1_vals = []
    s2_vals = []
    last_vals = {}
    last_st = {}
    idx = start_idx

    for _ in range(max(1, int(n_avg))):
        idx, vals, st, _ = wait_for_new_samples(state, idx, 1, timeout_s)
        last_vals = vals
        last_st = st
        scores.append((vals, st))
        if math.isfinite(st.get("s1", float("nan"))):
            s1_vals.append(st["s1"])
        if math.isfinite(st.get("s2", float("nan"))):
            s2_vals.append(st["s2"])

    if not scores:
        return None

    out_vals = dict(last_vals)
    out_st = dict(last_st)
    out_st["s1"] = _mean_finite(s1_vals)
    out_st["s2"] = _mean_finite(s2_vals)
    if "h" in out_vals and "v" in out_vals and math.isfinite(out_st["s1"]):
        out_st["hv_sum"] = float(out_st.get("hv_sum", out_vals.get("h", 0.0) + out_vals.get("v", 0.0)))
    if "d" in out_vals and "a" in out_vals and math.isfinite(out_st["s2"]):
        out_st["da_sum"] = float(out_st.get("da_sum", out_vals.get("d", 0.0) + out_vals.get("a", 0.0)))
    return idx, out_vals, out_st


def control_thread_func(state: SharedState, mpc, args):
    active = list(state.active_paddles)
    if not active:
        return

    current_idx = 0
    while state.running:
        ch = active[current_idx % len(active)]
        current_idx += 1

        with state.lock:
            basis = state.control_basis
            pos_cmd = dict(state.pos_cmd)
            sample_idx = state.sample_idx
            raw_vals = dict(state.raw_vals)

        if not basis or not raw_vals:
            time.sleep(max(0.001, 1.0 / max(0.1, args.control_hz)))
            continue

        if not signal_ok_from_vals(raw_vals, basis, args.signal_sum_min):
            with state.lock:
                state.last_action = f"CH{ch}_safe_no_move"
                state.last_step = 0.0
                state.last_control_paddle = ch
                state.last_control_t = state.t_rel
            time.sleep(max(0.001, 1.0 / max(0.1, args.control_hz)))
            continue

        meas0 = average_shared_measurement(state, sample_idx, args.navg, args.measure_timeout_s)
        if meas0 is None:
            time.sleep(max(0.001, 1.0 / max(0.1, args.control_hz)))
            continue

        idx0, vals0, st0 = meas0
        score0, _, score_label = compute_control_score(vals0, basis)
        step_deg = step_from_score(score0, args.step_large_deg, args.step_medium_deg,
                                   args.step_small_deg, args.step_fine_deg)

        p0 = float(pos_cmd[ch])
        p_plus = max(args.bounds_min, min(args.bounds_max, p0 + step_deg))
        p_minus = max(args.bounds_min, min(args.bounds_max, p0 - step_deg))

        best_score = score0
        best_pos = p0
        best_action = f"CH{ch} center"
        best_st = dict(st0)

        for candidate_pos, label in ((p_plus, "+"), (p_minus, "-")):
            if abs(candidate_pos - p0) <= 1e-12:
                continue

            sent = safe_move_abs(mpc, ch, candidate_pos, args.bounds_min, args.bounds_max,
                                 io_delay_s=args.usb_io_delay_s)
            if sent is None:
                continue
            with state.lock:
                state.pos_cmd[ch] = sent

            if args.settle_s > 0:
                time.sleep(args.settle_s)

            meas = average_shared_measurement(state, idx0, args.navg, args.measure_timeout_s)
            if meas is None:
                continue
            idx_c, vals_c, st_c = meas
            idx0 = idx_c
            score_c, _, _ = compute_control_score(vals_c, basis)
            if math.isfinite(score_c) and (not math.isfinite(best_score) or score_c > best_score):
                best_score = score_c
                best_pos = candidate_pos
                best_action = f"CH{ch}{label}"
                best_st = dict(st_c)

        if abs(best_pos - state.pos_cmd[ch]) > 1e-12:
            sent = safe_move_abs(mpc, ch, best_pos, args.bounds_min, args.bounds_max,
                                 io_delay_s=args.usb_io_delay_s)
            if sent is not None:
                with state.lock:
                    state.pos_cmd[ch] = sent
                if args.settle_s > 0:
                    time.sleep(args.settle_s)

        with state.lock:
            pos_after = dict(state.pos_cmd)
            state.control_score = best_score
            state.score_label = score_label
            state.last_action = best_action
            state.last_step = step_deg
            state.last_control_paddle = ch
            state.last_control_t = state.t_rel
            t_rel = state.t_rel
            sample_idx_log = state.sample_idx

        with state.lock:
            ctrl_logger = state.ctrl_logger

        if ctrl_logger is not None:
            ctrl_logger.write_row([
                f"{t_rel:.6f}",
                sample_idx_log,
                ch,
                best_action,
                basis,
                score_label,
                "" if not math.isfinite(score0) else f"{score0:.6f}",
                "" if not math.isfinite(best_score) else f"{best_score:.6f}",
                f"{step_deg:.6f}",
                "" if not math.isfinite(st0.get('s1', float('nan'))) else f"{st0['s1']:.6f}",
                "" if not math.isfinite(st0.get('s2', float('nan'))) else f"{st0['s2']:.6f}",
                "" if not math.isfinite(best_st.get('s1', float('nan'))) else f"{best_st['s1']:.6f}",
                "" if not math.isfinite(best_st.get('s2', float('nan'))) else f"{best_st['s2']:.6f}",
                f"{pos_after[1]:.6f}", f"{pos_after[2]:.6f}", f"{pos_after[3]:.6f}",
            ])

        if args.control_sleep_s > 0:
            time.sleep(args.control_sleep_s)


# -----------------------------------------------------------------------------
# Power impact helpers
# -----------------------------------------------------------------------------
def _pad_min_csv_path(save_dir: str) -> str:
    return os.path.join(save_dir, "pad_min.csv")


def _save_pad_min_csv(save_dir: str, impacts: dict[int, float], pad_min: list[int]):
    os.makedirs(save_dir, exist_ok=True)
    p = _pad_min_csv_path(save_dir)
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["rank", "paddle", "impact"])
        for rank, pad in enumerate(pad_min, start=1):
            w.writerow([rank, pad, f"{impacts.get(pad, 0.0):.12g}"])
    return p


def _load_pad_min(save_dir: str):
    p = _pad_min_csv_path(save_dir)
    if not os.path.exists(p):
        return None
    try:
        pads = []
        with open(p, "r", encoding="utf-8") as f:
            r = csv.DictReader(f)
            for row in r:
                pads.append(int(row["paddle"]))
        return pads or None
    except Exception:
        return None


def power_impact_geral_4det(
    bus=1,
    addr=None,
    gain="4.096",
    datarate_idx=7,
    serial_mpc=None,
    mpc_velocity=150,
    step=8,
    dwell=0.8,
    cycles=1,
    back_sweep=True,
    unit="V",
    plot=True,
    place="place",
    show=True,
    save_dir=None,
    save_csv=False,
    ads_io_delay_s=0.0,
):
    if addr is None:
        addr = list(DEFAULT_ADDRS)

    gain_map = {
        "6.144": ADS1x15.ADS1115.PGA_6_144V,
        "4.096": ADS1x15.ADS1115.PGA_4_096V,
        "2.048": ADS1x15.ADS1115.PGA_2_048V,
        "1.024": ADS1x15.ADS1115.PGA_1_024V,
        "0.512": ADS1x15.ADS1115.PGA_0_512V,
        "0.256": ADS1x15.ADS1115.PGA_0_256V,
    }
    if gain not in gain_map:
        raise ValueError(f"Ganho inválido: {gain}")

    ads_list, f_list = init_ads_modules(bus, addr, gain_map[gain], datarate_idx)
    mapping = build_default_mapping(len(ads_list))
    ctx, mpc, _, velocity_applied = mpc_open(serial_mpc, velocity=mpc_velocity)

    pads = (1, 2, 3)
    channels = ["ten_h", "ten_v", "ten_d", "ten_a", "S1", "S2"]
    positions = {pad: [] for pad in pads}
    series = {ch: {pad: [] for pad in pads} for ch in channels}

    forward = list(range(0, 161, step))
    backward = list(range(160, -1, -step))

    def read_4_and_stokes():
        vals = safe_read_all(ads_list, f_list, mapping, io_delay_s=ads_io_delay_s)
        vals_dict = dict(zip([name for name, _, _ in mapping], vals))
        st = compute_stokes_from_vals(vals_dict)
        return st

    try:
        home_pos_blind(mpc)
        time.sleep(max(0.5, dwell))

        for pad in pads:
            for _ in range(cycles):
                for direction_name, scan in (("ida", forward), ("volta", backward if back_sweep else [])):
                    for pos in scan:
                        safe_move_abs(mpc, pad, pos, 0.0, 160.0)
                        time.sleep(dwell)
                        st = read_4_and_stokes()
                        positions[pad].append(pos)
                        series["ten_h"][pad].append(st["h"])
                        series["ten_v"][pad].append(st["v"])
                        series["ten_d"][pad].append(st["d"])
                        series["ten_a"][pad].append(st["a"])
                        series["S1"][pad].append(st["s1"])
                        series["S2"][pad].append(st["s2"])
                        print(
                            f"Pad {pad} | {direction_name:5s} | {pos:3d}° -> "
                            f"H:{st['h']:.6e} V:{st['v']:.6e} D:{st['d']:.6e} A:{st['a']:.6e} "
                            f"S1:{st['s1']:.6f} S2:{st['s2']:.6f}"
                        )

        impacts = {ch: {} for ch in channels}
        combined = {}
        for pad in pads:
            total = 0.0
            for ch in channels:
                vals = [x for x in series[ch][pad] if math.isfinite(x)]
                sd = statistics.stdev(vals) if len(vals) >= 2 else 0.0
                impacts[ch][pad] = sd
                if ch in ("ten_h", "ten_v", "ten_d", "ten_a"):
                    total += sd
            combined[pad] = total
        pad_order = [p for p, _ in sorted(combined.items(), key=lambda kv: kv[1], reverse=True)]

        maxima = {ch: {} for ch in channels}
        for pad in pads:
            x = np.array(positions[pad], dtype=float)
            for ch in channels:
                y = np.array(series[ch][pad], dtype=float)
                if x.size and np.isfinite(y).any():
                    idx = int(np.nanargmax(y))
                    maxima[ch][pad] = {"pos": float(x[idx]), "val": float(y[idx])}
                else:
                    maxima[ch][pad] = {"pos": np.nan, "val": np.nan}

        if plot:
            pass
            """
            plt.figure(figsize=(12, 7))
            cmap = plt.get_cmap("tab10")
            ls = {"ten_h": "solid", "ten_v": "dashdot", "ten_d": "dashed", "ten_a": "dotted", "S1": (0, (3, 1, 1, 1)), "S2": (0, (5, 2))}
            for pad in pads:
                x = np.array(positions[pad], dtype=float)
                for ch in channels:
                    y = np.array(series[ch][pad], dtype=float)
                    if ch.startswith("ten_"):
                        denom = np.nanmax(np.abs(y))
                        if not np.isfinite(denom) or denom == 0:
                            denom = 1.0
                        y_plot = y / denom
                    else:
                        y_plot = y
                    plt.plot(x, y_plot, label=f"Pad {pad} - {ch}", linestyle=ls[ch], color=cmap((pad - 1) % 10), alpha=0.8)
            plt.xlabel("Ângulo [°]")
            plt.ylabel(f"Leitura normalizada / Stokes ({unit})")
            plt.title(f"Power impact geral - {place} | velocity={velocity_applied}")
            plt.grid(True, linestyle=":", alpha=0.6)
            plt.legend(ncol=2, fontsize=8)
            plt.tight_layout()
            if show:
                plt.show()
            else:
                plt.close()
            """

        csv_path = None
        if save_csv and save_dir:
            os.makedirs(save_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
            safe_place = sanitize_place(place)
            csv_path = os.path.join(save_dir, f"power_impact_geral_{safe_place}_{timestamp}.csv")
            with open(csv_path, "w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(["pad", "pos", "ten_h", "ten_v", "ten_d", "ten_a", "S1", "S2"])
                for pad in pads:
                    for i, pos in enumerate(positions[pad]):
                        writer.writerow([
                            pad, pos,
                            f"{series['ten_h'][pad][i]:.6e}",
                            f"{series['ten_v'][pad][i]:.6e}",
                            f"{series['ten_d'][pad][i]:.6e}",
                            f"{series['ten_a'][pad][i]:.6e}",
                            f"{series['S1'][pad][i]:.6e}",
                            f"{series['S2'][pad][i]:.6e}",
                        ])
            print(f"Arquivo salvo em: {csv_path}")

        return {
            "positions": positions,
            "power_all": series,
            "impacts": {**impacts, "combined": combined},
            "pad_order": pad_order,
            "maxima": maxima,
            "csv_path": csv_path,
            "velocity_applied": velocity_applied,
        }
    finally:
        mpc_close(ctx, mpc)


# -----------------------------------------------------------------------------
# Comandos
# -----------------------------------------------------------------------------
def build_toggle_loggers(args, compensation_enabled: bool):
    ts_logger = TimeSeriesLogger(save_dir=args.save_dir, place=args.place, flush_every=args.flush_every)
    ts_logger.start()
    ctrl_logger = None
    if compensation_enabled:
        ctrl_logger = ControllerLogger(save_dir=args.save_dir, place=args.place, flush_every=args.flush_every)
        ctrl_logger.start()
    return ts_logger, ctrl_logger


def cmd_serie(args):
    gain_map = {
        "6.144": ADS1x15.ADS1115.PGA_6_144V,
        "4.096": ADS1x15.ADS1115.PGA_4_096V,
        "2.048": ADS1x15.ADS1115.PGA_2_048V,
        "1.024": ADS1x15.ADS1115.PGA_1_024V,
        "0.512": ADS1x15.ADS1115.PGA_0_512V,
        "0.256": ADS1x15.ADS1115.PGA_0_256V,
    }
    if args.gain not in gain_map:
        raise ValueError(f"Ganho inválido: {args.gain}")

    ads_list, f_list = init_ads_modules(args.bus, args.addr, gain_map[args.gain], args.datarate_idx)
    mapping = build_default_mapping(len(ads_list))

    compensation_enabled = bool(args.compensation)
    ctx = None
    mpc = None
    velocity_applied = None

    state = SharedState()
    state.active_paddles = list(args.active_paddles)

    if compensation_enabled:
        state.control_basis = args.control_basis if args.control_basis is not None else minimize_to_control_basis(args.minimize)
        state.score_label = compute_control_score({"h": 1.0, "v": 0.0, "d": 1.0, "a": 0.0}, state.control_basis)[2]
        ctx, mpc, _, velocity_applied = mpc_open(args.serial_mpc, velocity=args.mpc_velocity)
        for ch in (1, 2, 3):
            sent = safe_move_abs(mpc, ch, state.pos_cmd[ch], args.bounds_min, args.bounds_max, io_delay_s=args.usb_io_delay_s)
            if sent is not None:
                state.pos_cmd[ch] = sent
        if args.settle_s > 0:
            time.sleep(args.settle_s)

    ts_logger = None
    ctrl_logger = None
    saving = (args.save_mode == "on")

    if saving:
        ts_logger, ctrl_logger = build_toggle_loggers(args, compensation_enabled)
        state.ts_logger = ts_logger
        state.ctrl_logger = ctrl_logger
        print(f"Time series CSV: {ts_logger.filepath}")
        if ctrl_logger is not None:
            print(f"Controller CSV: {ctrl_logger.filepath}")
    elif args.save_mode == "toggle":
        print("Modo TOGGLE: 's' liga/desliga salvamento; 'q' sai.")
    else:
        print("Modo SOMENTE LEITURA (sem CSV).")

    print(f"I2C bus {args.bus} | addrs {[hex(a) for a in args.addr]} | gain {args.gain} | datarate_idx {args.datarate_idx}")
    print(f"Detectors: {', '.join([n.upper() for n, _, _ in mapping])}")
    if compensation_enabled:
        print(
            f"[MPC] compensation ON | basis={state.control_basis} | paddles={state.active_paddles} | "
            f"velocity_req={args.mpc_velocity} | velocity_applied={velocity_applied}"
        )
    print("-" * 110)

    t0 = time.perf_counter()
    t_acq = threading.Thread(target=acquisition_thread_func, args=(state, ads_list, f_list, mapping, args, t0), daemon=True)
    t_acq.start()

    t_ctrl = None
    if compensation_enabled:
        t_ctrl = threading.Thread(target=control_thread_func, args=(state, mpc, args), daemon=True)
        t_ctrl.start()

    def maybe_toggle(key: str | None):
        nonlocal saving, ts_logger, ctrl_logger
        if args.save_mode != "toggle" or key is None:
            return False
        if key.lower() == "q":
            return True
        if key.lower() == "s":
            saving = not saving
            if saving:
                ts_logger, ctrl_logger = build_toggle_loggers(args, compensation_enabled)
                with state.lock:
                    state.ts_logger = ts_logger
                    state.ctrl_logger = ctrl_logger
                print(f"\n[SALVAMENTO ON] Time series -> {ts_logger.filepath}")
                if ctrl_logger is not None:
                    print(f"[SALVAMENTO ON] Controller -> {ctrl_logger.filepath}")
            else:
                print("\n[SALVAMENTO OFF] (arquivos fechados)")
                if ts_logger is not None:
                    ts_logger.close()
                if ctrl_logger is not None:
                    ctrl_logger.close()
                ts_logger = None
                ctrl_logger = None
                with state.lock:
                    state.ts_logger = None
                    state.ctrl_logger = None
        return False

    poller = None
    t_last_print = -1e9
    try:
        poller = KeyPoller() if args.save_mode == "toggle" else None
        if poller:
            poller.__enter__()

        while (time.perf_counter() - t0) < args.duration:
            if poller and maybe_toggle(poller.poll()):
                break

            snap = state.snapshot()
            t_rel = snap["t_rel"]
            st = snap["stokes"]
            pos_cmd = snap["pos_cmd"]
            if not st:
                time.sleep(0.005)
                continue

            if args.print_every <= 0 or (t_rel - t_last_print) >= args.print_every:
                t_last_print = t_rel
                msg = (
                    f"{t_rel:8.4f}s | "
                    f"H:{st.get('h', float('nan')): .6f}  V:{st.get('v', float('nan')): .6f}  "
                    f"D:{st.get('d', float('nan')): .6f}  A:{st.get('a', float('nan')): .6f} | "
                    f"S1:{st.get('s1', float('nan')):+.6f}  S2:{st.get('s2', float('nan')):+.6f} | "
                    f"HVsum:{st.get('hv_sum', float('nan')):.6f}  DAsum:{st.get('da_sum', float('nan')):.6f} | "
                    f"fs~{snap['sample_rate_hz']:.2f} Hz"
                )
                if compensation_enabled:
                    msg += (
                        f" | {snap['score_label']}={snap['control_score']:+.6f} "
                        f"cmd=({pos_cmd[1]:.2f},{pos_cmd[2]:.2f},{pos_cmd[3]:.2f}) "
                        f"step={snap['last_step']:.3f} action={snap['last_action']} active={snap['active_paddles']}"
                    )
                if saving:
                    msg += " [SAVE]"
                print(msg)

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nInterrupção detectada (CTRL+C).")
    finally:
        state.running = False
        if poller:
            poller.__exit__(None, None, None)
        t_acq.join(timeout=2.0)
        if t_ctrl is not None:
            t_ctrl.join(timeout=2.0)
        if ts_logger is not None:
            ts_logger.close()
        if ctrl_logger is not None:
            ctrl_logger.close()
        with state.lock:
            state.ts_logger = None
            state.ctrl_logger = None
        if compensation_enabled:
            mpc_close(ctx, mpc)

    if ts_logger is not None and ts_logger.filepath:
        print(f"\nTime series salva em: {ts_logger.filepath}")
    if ctrl_logger is not None and ctrl_logger.filepath:
        print(f"Controller salvo em: {ctrl_logger.filepath}")


def cmd_power_impactdet(args):
    gain_map = {
        "6.144": ADS1x15.ADS1115.PGA_6_144V,
        "4.096": ADS1x15.ADS1115.PGA_4_096V,
        "2.048": ADS1x15.ADS1115.PGA_2_048V,
        "1.024": ADS1x15.ADS1115.PGA_1_024V,
        "0.512": ADS1x15.ADS1115.PGA_0_512V,
        "0.256": ADS1x15.ADS1115.PGA_0_256V,
    }
    ads_list, f_list = init_ads_modules(args.bus, args.addr, gain_map[args.gain], args.datarate_idx)
    mapping = build_default_mapping(len(ads_list))

    base = args.base.lower()
    valid_bases = [n for n, _, _ in mapping] + ["s1", "s2"]
    if base not in valid_bases:
        raise ValueError(f"base '{base}' não existe. Use uma de {valid_bases}")

    ctx, mpc, _, velocity_applied = mpc_open(args.serial_mpc, velocity=args.mpc_velocity)
    try:
        home_pos_blind(mpc)
        time.sleep(max(0.5, args.settle_s))

        scan_rows = []
        impacts_data = {1: [], 2: [], 3: []}

        for paddle in (1, 2, 3):
            for pos in range(0, 161, args.step):
                safe_move_abs(mpc, paddle, pos, 0.0, 160.0)
                time.sleep(args.settle_s)
                vals = safe_read_all(ads_list, f_list, mapping, io_delay_s=args.ads_io_delay_s)
                vals_dict = dict(zip([name for name, _, _ in mapping], vals))
                st = compute_stokes_from_vals(vals_dict)
                metric = st[base] if base in ("s1", "s2") else vals_dict.get(base, float("nan"))
                impacts_data[paddle].append(metric)
                row = {
                    "paddle": paddle, "position_deg": pos,
                    "ten_h": st["h"], "ten_v": st["v"], "ten_d": st["d"], "ten_a": st["a"],
                    "S1": st["s1"], "S2": st["s2"], "metric": metric,
                }
                scan_rows.append(row)
                print(
                    f"Pad {paddle} - {pos:3d}° -> metric({base.upper()})={metric:.6f} | "
                    f"H={st['h']:.6f} V={st['v']:.6f} D={st['d']:.6f} A={st['a']:.6f} S1={st['s1']:+.6f} S2={st['s2']:+.6f}"
                )

        impacts = {}
        for paddle, series in impacts_data.items():
            ok = [x for x in series if math.isfinite(x)]
            impact = statistics.stdev(ok) if len(ok) >= 2 else 0.0
            impacts[paddle] = impact
            print(f"Impacto do pedal {paddle}: {impact:.6f}")

        pad_min = [pad for pad, _ in sorted(impacts.items(), key=lambda x: x[1], reverse=True)]
        print(f"\npad_min (ordem de impacto em {base.upper()}): {pad_min} | velocity_applied={velocity_applied}")

        if args.save_csv:
            path = _save_pad_min_csv(args.save_dir, impacts, pad_min)
            print(f"pad_min salvo em: {path}")

        if args.save_scan_csv:
            os.makedirs(args.save_dir, exist_ok=True)
            ts = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
            scan_path = os.path.join(args.save_dir, f"power_scan_{base}_{ts}.csv")
            with open(scan_path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["paddle", "position_deg", "ten_h", "ten_v", "ten_d", "ten_a", "S1", "S2", f"metric_{base}"])
                for row in scan_rows:
                    w.writerow([
                        row["paddle"], row["position_deg"],
                        f"{row['ten_h']:.12g}", f"{row['ten_v']:.12g}", f"{row['ten_d']:.12g}", f"{row['ten_a']:.12g}",
                        f"{row['S1']:.12g}", f"{row['S2']:.12g}", f"{row['metric']:.12g}",
                    ])
            print(f"scan salvo em: {scan_path}")
    finally:
        mpc_close(ctx, mpc)


def cmd_power_impact_geral(args):
    result = power_impact_geral_4det(
        bus=args.bus,
        addr=args.addr,
        gain=args.gain,
        datarate_idx=args.datarate_idx,
        serial_mpc=args.serial_mpc,
        mpc_velocity=args.mpc_velocity,
        step=args.step,
        dwell=args.dwell,
        cycles=args.cycles,
        back_sweep=(not args.no_back_sweep),
        unit=args.unit,
        plot=(not args.no_plot),
        place=args.place,
        show=(not args.no_show),
        save_dir=args.save_dir,
        save_csv=args.save_csv,
        ads_io_delay_s=args.ads_io_delay_s,
    )

    print("\nImpacto por canal:")
    for ch in ["ten_h", "ten_v", "ten_d", "ten_a", "S1", "S2"]:
        vals = result["impacts"].get(ch, {})
        if vals:
            print(f"  {ch}: " + ", ".join(f"pad {pad}={vals[pad]:.6e}" for pad in sorted(vals)))
    print("Impacto combinado (detectores):")
    print("  " + ", ".join(f"pad {pad}={result['impacts']['combined'][pad]:.6e}" for pad in sorted(result['impacts']['combined'])))
    print(f"Ordem dos pedais: {result['pad_order']}")


def cmd_move(args):
    ctx, mpc, _, velocity_applied = mpc_open(args.serial_mpc, velocity=args.mpc_velocity)
    try:
        sent = safe_move_abs(mpc, args.paddle, float(args.pos), 0.0, 170.0)
        time.sleep(args.settle_s)
        print(f"Paddle {args.paddle} -> comando enviado para {sent:.2f} deg | velocity_applied={velocity_applied}")
    finally:
        mpc_close(ctx, mpc)


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(prog="pola_control_unificado_stokes.py")
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_ads_opts(pp):
        pp.add_argument("--bus", type=int, default=1)
        pp.add_argument("--addr", type=lambda x: int(x, 0), action="append", default=list(DEFAULT_ADDRS),
                        help="Endereço ADS1115. Repita para 2 ADS.")
        pp.add_argument("--gain", type=str, default="4.096")
        pp.add_argument("--datarate_idx", type=int, default=7, help="ADS1115 dataRate idx 0..7.")
        pp.add_argument("--ads_io_delay_s", type=float, default=0.0, help="Delay entre leituras ADS.")

    def add_mpc_opts(pp):
        pp.add_argument("--serial_mpc", type=str, default=None, help="Porta do MPC320.")
        pp.add_argument("--mpc_velocity", type=int, default=150, help="Velocidade solicitada ao MPC320.")
        pp.add_argument("--usb_io_delay_s", type=float, default=0.002, help="Delay entre comandos USB ao MPC.")

    a = sub.add_parser("serie", help="Série temporal com S1/S2 e compensação opcional por threads.")
    add_ads_opts(a)
    add_mpc_opts(a)
    a.add_argument("--duration", type=float, default=60.0)
    a.add_argument("--period", type=float, default=0.0, help="Sleep extra na thread de aquisição.")
    a.add_argument("--place", type=str, default="EQL-CTG-PBS")
    a.add_argument("--save_dir", type=str, default=DEFAULT_SAVE_DIR)
    a.add_argument("--save_mode", choices=["on", "off", "toggle"], default="on")
    a.add_argument("--flush_every", type=int, default=200)
    a.add_argument("--print_every", type=float, default=0.2)

    a.add_argument("--compensation", action="store_true", help="Ativa controle MPC.")
    a.add_argument("--minimize", choices=["h", "v", "d", "a"], default="h")
    a.add_argument("--control_basis", choices=["HV_VMAX", "HV_HMAX", "DA_DMAX", "DA_AMAX"], default=None)
    a.add_argument("--active_paddles", nargs="+", type=int, choices=[1, 2, 3], default=[1, 2, 3],
                   help="Lista de pedais que podem atuar. Default: 1 2 3")
    a.add_argument("--signal_sum_min", type=float, default=0.02)
    a.add_argument("--navg", type=int, default=2, help="Número de amostras novas usadas para avaliar cada ponto.")
    a.add_argument("--measure_timeout_s", type=float, default=0.2, help="Timeout para aguardar novas amostras do ADS.")
    a.add_argument("--control_hz", type=float, default=20.0, help="Mantido por compatibilidade; o loop usa control_sleep_s.")
    a.add_argument("--control_sleep_s", type=float, default=0.0, help="Sleep extra após cada decisão de controle.")
    a.add_argument("--settle_s", type=float, default=0.02)
    a.add_argument("--bounds_min", type=float, default=0.0)
    a.add_argument("--bounds_max", type=float, default=170.0)
    a.add_argument("--step_large_deg", type=float, default=12.0)
    a.add_argument("--step_medium_deg", type=float, default=6.0)
    a.add_argument("--step_small_deg", type=float, default=2.0)
    a.add_argument("--step_fine_deg", type=float, default=0.8)
    a.set_defaults(func=cmd_serie)

    pi = sub.add_parser("power_impactdet", help="Varredura por paddle usando base H/V/D/A ou S1/S2.")
    add_ads_opts(pi)
    add_mpc_opts(pi)
    pi.add_argument("--base", type=str, default="h", help="Base para o impacto: h/v/d/a/s1/s2")
    pi.add_argument("--step", type=int, default=8)
    pi.add_argument("--settle_s", type=float, default=0.05)
    pi.add_argument("--save_dir", type=str, default=DEFAULT_SAVE_DIR)
    pi.add_argument("--save_csv", action="store_true")
    pi.add_argument("--save_scan_csv", action="store_true")
    pi.set_defaults(func=cmd_power_impactdet)

    pg = sub.add_parser("power_impact_geral", help="Varredura geral dos 3 pedais medindo H, V, D, A, S1 e S2.")
    add_ads_opts(pg)
    add_mpc_opts(pg)
    pg.add_argument("--step", type=int, default=8)
    pg.add_argument("--dwell", type=float, default=0.8)
    pg.add_argument("--cycles", type=int, default=1)
    pg.add_argument("--no_back_sweep", action="store_true")
    pg.add_argument("--unit", type=str, default="V")
    pg.add_argument("--place", type=str, default="place")
    pg.add_argument("--save_dir", type=str, default=DEFAULT_SAVE_DIR)
    pg.add_argument("--save_csv", action="store_true")
    pg.add_argument("--no_plot", action="store_true")
    pg.add_argument("--no_show", action="store_true")
    pg.set_defaults(func=cmd_power_impact_geral)

    m = sub.add_parser("move", help="Mover um paddle do MPC320 para uma posição específica.")
    add_mpc_opts(m)
    m.add_argument("--paddle", type=int, choices=[1, 2, 3], required=True)
    m.add_argument("--pos", type=float, required=True)
    m.add_argument("--settle_s", type=float, default=0.2)
    m.set_defaults(func=cmd_move)

    return p


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
