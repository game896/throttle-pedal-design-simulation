# -*- coding: utf-8 -*-
"""
Simulacija logike provere APPS čvora (isti smer senzora, preklopljeni prozori, poluga).

Isti algoritam koji je u izveštaju dat kao pseudo-kod izvršava se nad simuliranim naponima
dva senzora, sa ubačenim greškama. Simulacija proverava LOGIKU (otkrivanje kvarova, vremena,
ponašanje izlaza), a ne tačnost – tačnost je u proračunu (proracun_greske.py). Zato senzori
nemaju šum ni greške, a uključeni su samo razdelnik i 12-bitni A/D konvertor.

Skripta:
  1) proverava da kratak spoj između signala UVEK daje grešku (T 11.8.6),
  2) proverava da spoj preko otpora nikad ne povećava izlaz,
  3) meri vreme do FAULT-a za svaki ubačeni kvar (T 11.8.8),
  4) proverava CRC-8 i pakovanje CAN poruke,
  5) crta slike za izveštaj.

Pokretanje:  python apps_sim.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# --------------------------------------------------------------------------- parametri
K = 4.0 / 90.0                 # V/deg, RTY090: 0,5-4,5 V na 90 deg
THETA = 60.0                   # deg, primer okretanja senzora (dozvoljeno 48,7-63,6 deg)
SPAN = K * THETA               # V, naponski hod
V1_0 = 0.79                    # V, APPS1 pri puštenoj pedali
DV_OFF = 0.569                 # V, razmak pravih (APPS2 iznad APPS1)
V1_100 = V1_0 + SPAN
V2_0, V2_100 = V1_0 + DV_OFF, V1_0 + DV_OFF + SPAN
V5_NOM = 5.0

DIV = 6.8 / (3.9 + 6.8)        # razdelnik signala 3,9 k / 6,8 k (10,7 k >= 10 k opterećenja senzora)
DIV5 = 5.1 / (3.9 + 5.1)       # razdelnik napajanja (PA2) 3,9 k / 5,1 k: 5,5 V -> 3,12 V < 3,3 V
ADC_REF, ADC_MAX = 3.3, 4095

MARG = 8.0                     # prozor -8 % .. 108 %
DEV_LIMIT = 10.0               # T 11.8.9
N_FAULT = 50                   # FAULT na 50. uzastopnom odbirku sa greškom
N_OK = 100                     # brisanje FAULT-a posle 100 ispravnih odbiraka ...
RELEASE = 5.0                  # ... i pedale puštene ispod 5 %
DEAD_LO, DEAD_HI = 5.0, 95.0   # mrtve zone
N_AVG = 4                      # pokretni prosek 4 odbirka
SDC_EXTRA = 100                # ECU otvara SDC ako FAULT traje još 100 ms

C_S1, C_S2, C_OUT, C_FAULT = "#2a78d6", "#eb6834", "#1baf7a", "#d03b3b"
C_INK, C_MUTED, C_GRID = "#0b0b0b", "#52514e", "#e1e0d9"


# --------------------------------------------------------------------------- model
def sensor_v(pedal_pct, v5=V5_NOM):
    """Ratiometrijski izlaz oba senzora za položaj pedale (0-100 %) i napajanje v5."""
    x = np.asarray(pedal_pct, dtype=float) / 100.0
    r = np.asarray(v5, dtype=float) / V5_NOM
    return (V1_0 + x * SPAN) * r, (V2_0 + x * SPAN) * r


def adc(v, div=DIV):
    """Razdelnik + 12-bitni A/D; vraća napon preračunat nazad na stranu senzora."""
    code = np.clip(np.round(np.asarray(v) * div / ADC_REF * ADC_MAX), 0, ADC_MAX)
    return code * ADC_REF / ADC_MAX / div


def pct(v, v0, v100):
    return (v - v0) / (v100 - v0) * 100.0


class Node:
    """Logika čvora: poziva se svake 1 ms (isto kao pseudo-kod u izveštaju)."""

    def __init__(self):
        self.buf = []
        self.t_err = 0
        self.t_ok = 0
        self.fault = True          # pri uključenju FAULT = 1 (izlaz 0 dok se pedala ne pusti)
        self.last_good = 0.0

    def step(self, v1, v2, v5):
        v5m = max(v5, 1e-3)
        n1, n2 = v1 * V5_NOM / v5m, v2 * V5_NOM / v5m        # deljenje naponom napajanja
        self.buf.append((n1, n2, v5))
        self.buf = self.buf[-N_AVG:]
        a1 = sum(b[0] for b in self.buf) / len(self.buf)
        a2 = sum(b[1] for b in self.buf) / len(self.buf)
        a5 = sum(b[2] for b in self.buf) / len(self.buf)
        p1, p2 = pct(a1, V1_0, V1_100), pct(a2, V2_0, V2_100)
        err = (not -MARG <= p1 <= 100 + MARG) or (not -MARG <= p2 <= 100 + MARG) \
            or abs(p1 - p2) > DEV_LIMIT or not 4.5 <= a5 <= 5.5
        if err:
            self.t_ok = 0
            self.t_err += 1
            if self.t_err >= N_FAULT:
                self.fault = True
        else:
            self.t_err = 0
            self.t_ok += 1
            if self.fault and self.t_ok >= N_OK and min(p1, p2) < RELEASE:
                self.fault = False
        if self.fault:
            ped = 0.0
        elif err:
            ped = self.last_good
        else:
            ped = self.last_good = min(p1, p2)
        out = float(np.clip((ped - DEAD_LO) / (DEAD_HI - DEAD_LO) * 100.0, 0.0, 100.0))
        return p1, p2, out, self.fault, err


# --------------------------------------------------------------------------- scenariji
T_FAULT = 2000


def profile(t, recover=False):
    if recover:
        return np.interp(t, [0, 300, 1000, 2600, 2750, 3050, 3400], [0, 0, 60, 60, 0, 0, 50])
    return np.interp(t, [0, 300, 1000], [0, 0, 60])


def run(name, fault_fn, t_end=3000, recover=False):
    t = np.arange(t_end)
    pedal = profile(t, recover)
    v5 = np.full(t.size, V5_NOM)
    v5 = fault_fn("v5", t, v5)
    v1, v2 = sensor_v(pedal, v5)
    v1, v2 = fault_fn("sig", t, (v1.copy(), v2.copy()), pedal=pedal, v5=v5)
    node = Node()
    rows = [node.step(a, b, c) for a, b, c in zip(adc(v1), adc(v2), adc(v5, DIV5))]
    p1, p2, out, fault, err = map(np.array, zip(*rows))
    after = fault[T_FAULT:]
    t_det = int(np.argmax(after)) if after.any() else None
    sdc = None
    if t_det is not None:
        run_len = np.argmin(after[t_det:]) if not after[t_det:].all() else after[t_det:].size
        sdc = t_det + SDC_EXTRA if run_len > SDC_EXTRA else None
    return dict(name=name, t=t, pedal=pedal, p1=p1, p2=p2, out=out, fault=fault,
                t_det=t_det, sdc=sdc)


def sc(kind_v5=None, kind_sig=None):
    def fn(what, t, x, pedal=None, v5=None):
        if what == "v5":
            return kind_v5(t, x) if kind_v5 else x
        return kind_sig(t, x[0], x[1], pedal, v5) if kind_sig else x
    return fn


m = lambda t: t >= T_FAULT


def f_open1(t, v1, v2, p, v5):
    v1[m(t)] = 0.0
    return v1, v2


def f_open1_back(t, v1, v2, p, v5):
    v1[(t >= T_FAULT) & (t < 2400)] = 0.0
    return v1, v2


def f_gnd2(t, v1, v2, p, v5):
    v2[m(t)] = 0.0
    return v1, v2


def f_5v1(t, v1, v2, p, v5):
    v1[m(t)] = v5[m(t)]
    return v1, v2


def f_short(t, v1, v2, p, v5):
    mid = (v1 + v2) / 2
    v1[m(t)], v2[m(t)] = mid[m(t)], mid[m(t)]
    return v1, v2


def f_partial(t, v1, v2, p, v5):          # spoj preko otpora: naponi se približe za 30 %
    d = v2 - v1
    v1[m(t)] += 0.15 * d[m(t)]
    v2[m(t)] -= 0.15 * d[m(t)]
    return v1, v2


def f_drift2(t, v1, v2, p, v5):           # pomeraj APPS2 za +0,32 V (12 p.p.)
    v2[m(t)] += 0.32
    return v1, v2


def f_rod1(t, v1, v2, p, v5):             # otkačena šipka APPS1: opruga vraća senzor na 0 %
    v1[m(t)] = V1_0 * v5[m(t)] / V5_NOM
    return v1, v2


def f_glitch1(t, v1, v2, p, v5):          # prekid od 30 ms – ne sme da izazove FAULT
    v1[(t >= T_FAULT) & (t < T_FAULT + 30)] = 0.0
    return v1, v2


def v5_high(t, v5):                       # napajanje poraste na 5,4 V (u dozvoljenom opsegu)
    v5[m(t)] = 5.4
    return v5


def v5_low(t, v5):                        # napajanje padne na 4,3 V (van 4,5-5,5 V)
    v5[m(t)] = 4.3
    return v5


# --------------------------------------------------------------------------- CRC i poruka
def crc8_j1850(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def pack_frame(pedal_pct, p1, p2, status, counter):
    ped = int(round(pedal_pct * 10))
    enc = lambda p: 0xFF if not -MARG <= p <= 100 + MARG else int(round(np.clip(p, 0, 100) * 2))
    d = [ped & 0xFF, ped >> 8, enc(p1), enc(p2), status, 0x00, counter & 0x0F]
    return d + [crc8_j1850(d)]


# --------------------------------------------------------------------------- slike
def style(ax):
    for s_ in ("top", "right"):
        ax.spines[s_].set_visible(False)
    for s_ in ("left", "bottom"):
        ax.spines[s_].set_color("#c3c2b7")
    ax.tick_params(colors=C_MUTED, labelsize=8.5)
    ax.grid(True, color=C_GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def fig_transfer(path):
    fig, ax = plt.subplots(figsize=(6.6, 3.8), dpi=200)
    style(ax)
    p = np.linspace(0, 100, 101)
    v1, v2 = sensor_v(p)
    mv = MARG / 100 * SPAN
    ax.axhspan(V1_0 - mv, V1_100 + mv, color=C_S1, alpha=0.08, lw=0)
    ax.axhspan(V2_0 - mv, V2_100 + mv, color=C_S2, alpha=0.08, lw=0)
    for v, c in ((V1_0 - mv, C_S1), (V1_100 + mv, C_S1), (V2_0 - mv, C_S2), (V2_100 + mv, C_S2)):
        ax.axhline(v, color=c, lw=0.7, ls=(0, (2, 2)), alpha=0.8)
    ax.plot(p, v1, color=C_S1, lw=2)
    ax.plot(p, v2, color=C_S2, lw=2)
    ax.axhline(0.5, color=C_MUTED, lw=0.8, ls=(0, (4, 3)))
    ax.axhline(4.5, color=C_MUTED, lw=0.8, ls=(0, (4, 3)))
    ax.text(101.5, v1[-1], "APPS1", fontsize=9, va="center", color=C_INK)
    ax.text(101.5, v2[-1], "APPS2", fontsize=9, va="center", color=C_INK)
    y1, y2 = sensor_v(30)
    ax.annotate("", xy=(30, y2), xytext=(30, y1), arrowprops=dict(arrowstyle="<->", color=C_MUTED, lw=1))
    ax.text(62, 1.45, "razmak 0,57 V: kratak spoj između\nsignala daje razliku ≥ 20 p.p.",
            fontsize=7.5, color=C_MUTED, va="center")
    ax.annotate("", xy=(31, (y1 + y2) / 2), xytext=(61, 1.45),
                arrowprops=dict(arrowstyle="-", color=C_MUTED, lw=0.6, ls=(0, (2, 2))))
    ax.text(40, V1_0 - mv + 0.03, ("prozor APPS1: %.2f–%.2f V" % (V1_0 - mv, V1_100 + mv)).replace(".", ","),
            fontsize=7.5, color=C_S1, va="bottom")
    ax.text(1, V2_100 + mv + 0.04, ("prozor APPS2: %.2f–%.2f V" % (V2_0 - mv, V2_100 + mv)).replace(".", ","),
            fontsize=7.5, color=C_S2, va="bottom")
    ax.text(99, 0.44, "granica senzora 0,5 V", fontsize=7, color=C_MUTED, ha="right", va="top")
    ax.text(99, 4.56, "granica senzora 4,5 V", fontsize=7, color=C_MUTED, ha="right", va="bottom")
    ax.text(50, 0.12, "0 V: prekid žice ili kratak spoj na masu", fontsize=7.5, color=C_FAULT, ha="center")
    ax.text(50, 4.88, "5 V: kratak spoj na napajanje senzora", fontsize=7.5, color=C_FAULT, ha="center")
    ax.set_xlim(0, 112)
    ax.set_ylim(0, 5.1)
    ax.set_xticks(range(0, 101, 20))
    ax.set_xlabel("Hod pedale [%]", fontsize=9, color=C_MUTED)
    ax.set_ylabel("Izlaz senzora [V]", fontsize=9, color=C_MUTED)
    fig.tight_layout()
    fig.savefig(path, facecolor="white")
    plt.close(fig)


def fig_sim(results, path):
    rows = (len(results) + 1) // 2
    fig, axes = plt.subplots(rows, 2, figsize=(7.4, 2.55 * rows), dpi=200)
    for ax, r in zip(axes.ravel(), results):
        style(ax)
        ts = r["t"] / 1000.0
        f = r["fault"]
        i = 1500
        while i < f.size:
            if f[i]:
                j = i
                while j < f.size and f[j]:
                    j += 1
                ax.axvspan(ts[i], ts[j - 1], color=C_FAULT, alpha=0.09, lw=0)
                ax.text((ts[i] + ts[j - 1]) / 2, 113, "FAULT", color=C_FAULT, fontsize=8,
                        ha="center", va="center", fontweight="bold")
                i = j
            i += 1
        ax.axvline(T_FAULT / 1000.0, color=C_MUTED, lw=0.8, ls=(0, (3, 3)))
        ax.plot(ts, np.clip(r["p1"], -20, 125), color=C_S1, lw=1.4, label="APPS1")
        ax.plot(ts, np.clip(r["p2"], -20, 125), color=C_S2, lw=1.4, label="APPS2")
        ax.plot(ts, r["out"], color=C_OUT, lw=2.0, label="izlaz ka ECU")
        ax.set_ylim(-22, 127)
        ax.set_yticks([0, 25, 50, 75, 100])
        ax.set_xlim(1.6, ts[-1])
        det = ("FAULT posle %d ms" % r["t_det"]) if r["t_det"] is not None else "bez FAULT-a"
        ax.set_title("%s\n%s" % (r["name"], det), fontsize=8.6, color=C_INK, loc="left")
        ax.set_xlabel("Vreme [s]", fontsize=8.3, color=C_MUTED)
        ax.set_ylabel("Položaj [%]", fontsize=8.3, color=C_MUTED)
    h, l = axes[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=3, frameon=False, fontsize=8.5, bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, facecolor="white")
    plt.close(fig)


def fig_block(path):
    from matplotlib.patches import FancyBboxPatch
    fig, ax = plt.subplots(figsize=(7.4, 3.6), dpi=200)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 50)
    ax.axis("off")

    def box(x, y, w, h, title, sub="", fc="#f3f3f3", ec="#898781", fs=8.3):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.2,rounding_size=1.0",
                                    fc=fc, ec=ec, lw=1.0))
        ax.text(x + w / 2, y + h / 2 + (1.5 if sub else 0), title, ha="center", va="center",
                fontsize=fs, color=C_INK, fontweight="bold")
        if sub:
            ax.text(x + w / 2, y + h / 2 - 2.0, sub, ha="center", va="center", fontsize=6.5, color=C_MUTED)

    def arrow(x0, y0, x1, y1, text="", dy=1.0, both=False):
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                    arrowprops=dict(arrowstyle="<|-|>" if both else "-|>", color="#52514e", lw=1.1,
                                    shrinkA=0, shrinkB=0, mutation_scale=9))
        if text:
            ax.text((x0 + x1) / 2, max(y0, y1) + dy, text, ha="center", va="bottom", fontsize=6.3,
                    color=C_MUTED)

    # okvir čvora
    ax.add_patch(FancyBboxPatch((24.5, 2), 39.5, 45, boxstyle="round,pad=0.2,rounding_size=1.5",
                                fc="white", ec="#c3c2b7", lw=1.0, ls=(0, (5, 3))))
    ax.text(44.25, 45.3, "APPS čvor (pored pedale)", ha="center", va="center", fontsize=7.5, color=C_MUTED)

    box(1, 30, 13, 9, "APPS1", "RTY090", fc="#e3eefb", ec=C_S1)
    box(1, 14, 13, 9, "APPS2", "RTY090", fc="#fde9e0", ec=C_S2)
    ax.text(7.5, 26.5, "pedala →" + chr(10) + "2 poluge", ha="center", va="center",
            fontsize=6.5, color=C_MUTED)

    box(27, 30, 12, 9, "Ulaz 1", "razdelnik + RC")
    box(27, 14, 12, 9, "Ulaz 2", "razdelnik + RC")
    box(27, 4, 12, 7, "Napajanje", "12 V → 5 V")
    box(45, 14, 16, 25, "STM32F103", "ADC 12 bit" + chr(10) + "provera + CAN")
    box(68.5, 23.5, 12, 6, "SN65HVD230", "", fs=7.2)
    box(87, 19, 12, 15, "ECU", "isklj. invertor," + chr(10) + "otvara SDC", fs=8.3)

    arrow(14, 34.5, 27, 34.5, "signal (K1)")
    arrow(14, 18.5, 27, 18.5, "signal (K2)")
    arrow(39, 34.5, 45, 34.5)
    arrow(39, 18.5, 45, 18.5)
    arrow(61, 26.5, 68.5, 26.5, both=True)
    arrow(80.5, 26.5, 87, 26.5, both=True)
    ax.text(83.75, 24.6, "CAN", ha="center", va="top", fontsize=6.3, color=C_MUTED)
    arrow(4, 5.5, 27, 5.5, "LV 12 V (K3)", dy=0.6)
    # 5 V iz modula: ploči i PA2 (desno) i senzorima kroz K1/K2 (levo)
    ax.plot([39, 53, 53], [7.5, 7.5, 14], color=C_FAULT, lw=0.9, ls=(0, (2, 2)))
    ax.text(54, 9.5, "5 V ploči i PA2", fontsize=6.2, color=C_FAULT, va="center")
    ax.plot([27, 20.5, 20.5], [9.0, 9.0, 31.5], color=C_FAULT, lw=0.9, ls=(0, (2, 2)))
    for y in (31.5, 15.5):
        ax.annotate("", xy=(14, y), xytext=(20.5, y),
                    arrowprops=dict(arrowstyle="-|>", color=C_FAULT, lw=0.9, ls=(0, (2, 2)),
                                    shrinkA=0, shrinkB=0, mutation_scale=8))
    ax.text(21.2, 25.5, "5 V i masa" + chr(10) + "(K1, K2)", fontsize=6.0, color=C_FAULT, va="center")
    fig.tight_layout(pad=0.2)
    fig.savefig(path, facecolor="white")
    plt.close(fig)


# --------------------------------------------------------------------------- main
def main():
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "slike")
    os.makedirs(out_dir, exist_ok=True)
    mv = MARG / 100 * SPAN
    print("Okretanje %.0f deg, hod %.3f V" % (THETA, SPAN))
    print("APPS1 %.3f..%.3f V (prozor %.3f..%.3f), APPS2 %.3f..%.3f V (prozor %.3f..%.3f)"
          % (V1_0, V1_100, V1_0 - mv, V1_100 + mv, V2_0, V2_100, V2_0 - mv, V2_100 + mv))

    # 1) kratak spoj između signala: za svaki zajednički napon mora biti greška
    bad = 0
    for v in np.arange(0.0, 5.0001, 0.002):
        n = Node()
        n.fault = False
        *_, err = n.step(v, v, V5_NOM)
        bad += (not err)
    print("Kratak spoj SIG1-SIG2: %d od %d napona NIJE prepoznato kao greška" % (bad, 2501))
    assert bad == 0

    # 2) spoj preko otpora: izmereni položaj nikad veći od stvarnog
    n_all = n_err = 0
    worst = -1e9
    for x in np.arange(0.0, 100.01, 2.0):
        a, b = sensor_v(x)
        for f1 in np.arange(0.0, 1.0001, 0.05):
            for f2 in np.arange(0.0, 1.0001 - f1, 0.05):
                if f1 == 0 and f2 == 0:
                    continue
                d = b - a
                n = Node()
                n.fault = False
                p1, p2, _, _, err = n.step(a + f1 * d, b - f2 * d, V5_NOM)
                n_all += 1
                n_err += err
                if not err:
                    worst = max(worst, min(p1, p2) - x)
    print("Spoj preko otpora: %d kombinacija, %d prepoznato kao greška; u ostalim izmereni položaj "
          "najviše %.2f p.p. iznad stvarnog" % (n_all, n_err, worst))
    assert worst <= 0.01

    # 3) scenariji
    scen = [
        ("Bez greške", sc()),
        ("Prekid žice APPS1", sc(kind_sig=f_open1)),
        ("APPS2 kratak spoj na masu", sc(kind_sig=f_gnd2)),
        ("APPS1 kratak spoj na napajanje (5 V)", sc(kind_sig=f_5v1)),
        ("Kratak spoj između signala", sc(kind_sig=f_short)),
        ("Delimičan spoj između signala", sc(kind_sig=f_partial)),
        ("Pomeraj APPS2 za +0,32 V (12 p.p.)", sc(kind_sig=f_drift2)),
        ("Otkačena šipka APPS1", sc(kind_sig=f_rod1)),
        ("Napajanje senzora 5,0 → 5,4 V", sc(kind_v5=v5_high)),
        ("Napajanje senzora pada na 4,3 V", sc(kind_v5=v5_low)),
        ("Prekid od 30 ms (smetnja)", sc(kind_sig=f_glitch1)),
    ]
    print("\n%-38s %-11s %-11s %s" % ("Scenario", "FAULT [ms]", "SDC [ms]", "izlaz pre -> posle"))
    res = {}
    for name, fn in scen:
        r = run(name, fn)
        res[name] = r
        print("%-38s %-11s %-11s %.1f -> %.1f" % (
            name, r["t_det"] if r["t_det"] is not None else "-", r["sdc"] if r["sdc"] else "-",
            r["out"][1990], r["out"][-1]))
    no_fault = ("Bez greške", "Prekid od 30 ms (smetnja)", "Napajanje senzora 5,0 → 5,4 V",
                "Delimičan spoj između signala")
    for name, r in res.items():
        if name in no_fault:
            assert r["t_det"] is None, name
        else:
            assert r["t_det"] is not None and r["t_det"] <= 55 and r["out"][-1] == 0.0, name
        assert r["out"][T_FAULT:].max() <= r["out"][1900:T_FAULT].max() + 0.5, name   # izlaz ne raste
    hi = res["Napajanje senzora 5,0 → 5,4 V"]
    print("Napajanje 5,4 V: izlaz %.2f %% -> %.2f %% (deljenje naponom napajanja)" % (hi["out"][1990], hi["out"][-1]))
    rec = run("Prekid žice APPS1, pa oporavak", sc(kind_sig=f_open1_back), t_end=3500, recover=True)
    st = res["Bez greške"]
    i_start = int(np.argmin(st["fault"]))
    print("Pokretanje: FAULT = 1 pri uključenju, obrisan u t = %d ms (pedala puštena)" % i_start)

    # 4) CRC i poruka
    assert crc8_j1850(b"123456789") == 0x4B
    print("\nCRC-8 SAE J1850('123456789') = 0x4B (očekivano)")
    print("Primer poruke (pedala 57,3 %%): %s" % " ".join("%02X" % b for b in pack_frame(57.3, 60.1, 59.8, 0, 7)))

    # 5) slike
    fig_transfer(os.path.join(out_dir, "prenosne_funkcije.png"))
    fig_sim([rec, res["Kratak spoj između signala"], res["Delimičan spoj između signala"],
             res["Otkačena šipka APPS1"], res["Napajanje senzora 5,0 → 5,4 V"],
             res["Prekid od 30 ms (smetnja)"]], os.path.join(out_dir, "simulacija.png"))
    fig_block(os.path.join(out_dir, "blok_sema.png"))
    print("\nSlike sačuvane u:", out_dir)


if __name__ == "__main__":
    main()
