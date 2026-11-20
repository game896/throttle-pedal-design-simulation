# -*- coding: utf-8 -*-
"""
Proračun najgoreg neslaganja dva APPS senzora (isti smer, preklopljeni prozori, poluga).
Sve ulazne vrednosti su najgore vrednosti iz datasheetova (izvor je naveden uz svaku).
Pokretanje:  python proracun_greske.py
"""

# --- Ulazni podaci -----------------------------------------------------------------
V_FS = 4000.0        # mV, izlaz 0,5-4,5 V             Honeywell RTY, tabela 1
TH_FS = 90.0         # deg, RTY090                      Honeywell RTY, tabela 3
LIN = 0.010          # linearnost +-1,0 % (najbolja prava, pri izmerenoj temp.)  tabela 3
ACC = 0.016          # tačnost +-1,6 % (od idealne prave, nula na 25 C)          tabela 3
SENS_BITS = 12       # rezolucija senzora              Honeywell RTY, tabela 1

VREF = 3300.0        # mV, referenca A/D (V_DDA)
ADC_BITS = 12
EL = 3               # LSB, integralna nelinearnost max       ST DS5319 Rev 20, tabela 50
ET = 5               # LSB, ukupna greška max                 ST DS5319 Rev 20, tabela 50
I_LKG = 1.0          # uA, curenje ulaza max                  ST DS5319 Rev 20, tabela 36

R1, R2 = 3.9, 6.8    # kOhm, razdelnik signala (ukupno 10,7 kOhm >= 10 kOhm opterećenja senzora)
TCR = 50e-6          # 1/C, Uni-Ohm MFR0W4F...: F = +-1 %, +-50 ppm/C   Uni-Ohm datasheet, str. 2
DT = 65.0            # C, kalibracija na 25 C, rad do -40 C (STM32F103C8T6: -40..85 C, DS5319 str. 104)

V1_0 = 790.0         # mV, napon APPS1 pri puštenoj pedali (izbor)
DV_OFF = 569.0       # mV, razmak pravih APPS2 - APPS1 (izbor; 20 p.p. na najvećem uglu)
MARG = 0.08          # prozor -8 % .. 108 % (izbor)
RES = 64.0           # mV, rezerva od 0,5 V i 4,5 V (= tačnost senzora)
LIMIT = 10.0         # p.p., T 11.8.9
MECH = 1.0           # p.p., budžet za mehaniku: razlika dve poluge + zazor + montaža (zahtev, poglavlje 3.3)

# --- Izvedene veličine -------------------------------------------------------------
k = V_FS / TH_FS                         # mV/deg
D = R2 / (R1 + R2)                       # odnos razdelnika
R_TH = R1 * R2 / (R1 + R2)               # kOhm, otpor izvora koji vidi ulaz
LSB = VREF / 2 ** ADC_BITS               # mV na ulazu A/D
LSB_S = LSB / D                          # mV preračunato na napon senzora
STEP_S = V_FS / 2 ** SENS_BITS           # mV, korak izlaza senzora
LEAK_ADC = I_LKG * R_TH                  # mV na ulazu A/D (uA x kOhm = mV)
LEAK_S = LEAK_ADC / D                    # mV na naponu senzora
dD = (R1 / (R1 + R2)) * 2 * TCR * DT     # najveća relativna promena odnosa razdelnika
# Merenje napajanja (PA2) ima SVOJ razdelnik: sa 3,6k/6,2k bi 5,5 V dalo 3,48 V > 3,3 V (A/D u zasićenju)
R1_5, R2_5 = 3.9, 5.1                    # kOhm, Uni-Ohm MFR0W4F3901A50 / MFR0W4F5101A50: 5,5 V -> 3,12 V
D5 = R2_5 / (R1_5 + R2_5)
R_TH5 = R1_5 * R2_5 / (R1_5 + R2_5)
LEAK_ADC5 = I_LKG * R_TH5                # mV na ulazu PA2
dD5 = (R1_5 / (R1_5 + R2_5)) * 2 * TCR * DT
V5_ADC = 5000.0 * D5                     # mV, napajanje senzora na ulazu PA2
S_MIN, S_MAX = k * 48.7, k * 63.6        # mV, hod koji kalibracija prihvata (poglavlje 5.2)
CAL_MIN_PP = 15.0                        # p.p., najmanja razlika pri kratkom spoju koju kalibracija prihvata
EXTRAP = (1 + (1 + 2 * MARG)) / 2        # 2,16E / 2E = 1,08: ekstrapolacija kalibracione prave do -8 % / 108 %


def end_deviation(theta, case):
    """Koliko JEDAN ispravan senzor može da odstupi na kraju hoda (0 % ili 100 %, kalibracione tačke).
    Mora biti < MARG (8 %), inače ispravan senzor izlazi iz prozora."""
    s = k * theta
    v = V1_0 + DV_OFF + s                         # najveći napon (APPS2 na 100 %)
    if case == "A":   # ista kriva kao pri kalibraciji: senzor 0 na samoj tački
        e = 2 * EL * LSB_S + 2 * LEAK_S + STEP_S
        eps5 = (2 * EL * LSB + 2 * LEAK_ADC5) / V5_ADC
    else:
        e = 2 * ACC * V_FS + 2 * ET * LSB_S + 2 * LEAK_S + STEP_S + dD * v   # dD: sopstveni razdelnik
        eps5 = (2 * ET * LSB + 2 * LEAK_ADC5) / V5_ADC + dD5                  # razdelnik PA2 je u eps5
    return (e + eps5 * v) / s * 100


def terms(case):
    """Greška JEDNOG senzora u mV (bez otpornika) i relativna greška merenja napajanja."""
    if case == "A":   # ista temperatura kao pri kalibraciji
        t = {"senzor (2 x linearnost)": 2 * LIN * V_FS,
             "A/D (2 x EL)": 2 * EL * LSB_S}
        eps5 = (2 * EL * LSB + 2 * LEAK_ADC5) / V5_ADC
    else:             # druga temperatura nego pri kalibraciji
        t = {"senzor (2 x tačnost)": 2 * ACC * V_FS,
             "A/D (2 x ET)": 2 * ET * LSB_S}
        eps5 = (2 * ET * LSB + 2 * LEAK_ADC5) / V5_ADC + dD5
    t["curenje ulaza (2 x I_lkg x R_th / D)"] = 2 * LEAK_S
    t["rezolucija senzora (2 x 0,5 koraka)"] = STEP_S
    return t, eps5


def parts_of(theta, case):
    """Doprinosi neslaganju dva senzora (p.p.), najgori slučaj, svaki izvor posebno."""
    s = k * theta
    t, eps5 = terms(case)
    parts = {name: 2 * v / s * 100 for name, v in t.items()}
    if case == "B":
        v_sum = (V1_0 + s) + (V1_0 + DV_OFF + s)       # oba napona na 100 % hoda
        parts["otpornici razdelnika (TCR)"] = dD * v_sum / s * 100
    parts["merenje napajanja (PA2)"] = eps5 * DV_OFF / s * 100
    return parts


def dp(theta, case, verbose=False):
    """Najgori slučaj: svi izvori na granici iz datasheeta, dva senzora na suprotne strane (zbir)."""
    parts = parts_of(theta, case)
    el = sum(parts.values())
    total = el + MECH
    if verbose:
        print("  ugao %.1f deg, hod %.0f mV, slučaj %s" % (theta, k * theta, case))
        for n, v in parts.items():
            print("    %-40s %6.2f p.p." % (n, v))
        print("    %-40s %6.2f p.p." % ("= električno", el))
        print("    %-40s %6.2f p.p." % ("+ mehanika (budžet, zahtev)", MECH))
        print("    %-40s %6.2f p.p." % ("= UKUPNO (najgori slučaj)", total))
        print("    %-40s %6.2f p.p." % ("verovatna procena (RSS)", rss(theta, case)))
    return total


def rss(theta, case):
    """Statistička procena: izvori nezavisni, granice iz datasheeta uzete kao krajnje vrednosti.
    Svaki doprinos v (zbir dva senzora) čine dva nezavisna dela v/2 -> RSS para je v/sqrt(2)."""
    vals = list(parts_of(theta, case).values()) + [MECH]
    return (sum((v / 2 ** 0.5) ** 2 for v in vals)) ** 0.5


def short_check(theta):
    """Kratak spoj SIG1-SIG2: izmerena razlika položaja mora biti > 10 p.p."""
    s = k * theta
    # dva kanala, svaki može da odstupi od svoje kalibracije: A/D 2 x ET, curenje 2 x I_lkg,
    # otpornici V(d_i - d_5) -> razlika najviše 2 x dD x V; senzori i mehanika ne utiču (napon je napon spoja)
    # Najveći napon za koji ima smisla kratak spoj je gornji kraj preklapanja prozora (V1_0 + 1,08 s);
    # uzima se V2_100 = V1_0 + DV_OFF + s, koji je veći, pa je procena na sigurnoj strani.
    v = V1_0 + DV_OFF + s
    # Napon spoja može biti van kalibrisanog opsega jednog senzora (do -8 % ili 108 %), pa se kalibraciona
    # prava ekstrapolira: granica je E + (1 + 2 x 0,08)E = 2,16E umesto 2E, tj. faktor EXTRAP = 1,08.
    err = (2 * 2 * ET * LSB_S + 2 * 2 * LEAK_S) * EXTRAP / s * 100 + dD * 2 * v / s * 100
    # Zajednička greška merenja napajanja eps5 skalira oba napona isto; zbog različitih nagiba
    # (oba hoda dozvoljena između S_MIN i S_MAX) daje razliku eps5 * v * (1/S_MIN - 1/S_MAX).
    _, eps5 = terms("B")
    err += eps5 * v * (1 / S_MIN - 1 / S_MAX) * 100
    return DV_OFF / s * 100, err


if __name__ == "__main__":
    print("k = %.3f mV/deg, D = %.4f, R_th = %.3f kOhm, LSB = %.4f mV (%.3f mV na senzoru)"
          % (k, D, R_TH, LSB, LSB_S))
    print("curenje: %.2f mV na A/D = %.2f mV na senzoru; dD = %.4f %%; korak senzora %.3f mV"
          % (LEAK_ADC, LEAK_S, dD * 100, STEP_S))
    for c in "AB":
        t, eps5 = terms(c)
        print("\nSlučaj %s - greška jednog senzora:" % c)
        for n, v in t.items():
            print("  %-40s %7.2f mV" % (n, v))
        print("  %-40s %7.2f mV" % ("zbir", sum(t.values())))
        print("  relativna greška merenja napajanja eps5 = %.4f %%" % (eps5 * 100))

    th_min = next(x / 10 for x in range(200, 1200) if dp(x / 10, "A") <= LIMIT)
    s_lo = (V1_0 - (500 + RES)) / MARG                       # APPS1 na -8 % >= 0,5 V + rezerva
    s_hi = ((4500 - RES) - V1_0 - DV_OFF) / (1 + MARG)       # APPS2 na 108 % <= 4,5 V - rezerva
    th_max = min(s_lo, s_hi) / k
    print("\nNajmanji ugao senzora (slučaj A sa mehanikom <= 10 p.p.): %.1f deg" % th_min)
    print("Najveći ugao senzora (prozori u 0,5-4,5 V uz rezervu %.0f mV): %.1f deg "
          "(ograničenje dole %.1f, gore %.1f)" % (RES, th_max, s_lo / k, s_hi / k))
    th_b = next((x / 10 for x in range(200, 1200) if dp(x / 10, "B") <= LIMIT), None)
    print("Slučaj B <= 10 p.p. tek od: %s deg" % th_b)

    for th in (th_min, 60.0, th_max):
        print()
        dp(th, "A", True)
        dp(th, "B", True)
        off, err = short_check(th)
        print("  kratak spoj: razmak %.1f p.p., greška merenja do %.2f p.p. -> najmanje %.1f p.p. (> 10)"
              % (off, err, off - err))
        s = k * th
        print("  naponi: APPS1 %.0f..%.0f mV (prozor %.0f..%.0f), APPS2 %.0f..%.0f mV (prozor %.0f..%.0f)"
              % (V1_0, V1_0 + s, V1_0 - MARG * s, V1_0 + (1 + MARG) * s,
                 V1_0 + DV_OFF, V1_0 + DV_OFF + s, V1_0 + DV_OFF - MARG * s,
                 V1_0 + DV_OFF + (1 + MARG) * s))
        print("  A/D kodova na ceo hod: %.0f (%.3f %% po kodu)" % (s * D / LSB, 100 / (s * D / LSB)))
    print("\nPrenosni odnos poluge za hod pedale 30 deg: %.2f .. %.2f" % (th_min / 30, th_max / 30))
    for th in (th_min, 60.0, th_max):
        print("Jedan senzor na kraju hoda, %.1f deg: A %.2f %%, B %.2f %% (margina prozora %.0f %%)"
              % (th, end_deviation(th, "A"), end_deviation(th, "B"), MARG * 100))
    worst = max(short_check(x / 10)[1] for x in range(487, 637))
    print("Kratak spoj, garancija kalibracije: %.0f - %.2f = %.2f p.p. (> 10)"
          % (CAL_MIN_PP, worst, CAL_MIN_PP - worst))
