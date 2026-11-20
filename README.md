# Senzor položaja pedale gasa (APPS) – proračun greške i simulacija

Prateći kod uz kandidatski zadatak „Projektovanje sistema koji čita poziciju pedale za gas“
(Formula Student, pravilnik FS Rules 2027). Idejno rešenje: dva Hall senzora Honeywell RTY090LVNAX,
čvor sa STM32F103C8T6, CAN komunikacija sa ECU-om i provera tačnosti (plausibility).

## Sadržaj

| Fajl | Šta radi |
|---|---|
| `proracun_greske.py` | Najgore neslaganje dva ispravna senzora (slučajevi A i B), najmanji i najveći ugao senzora, provera kratkog spoja između signala. Sve ulazne vrednosti su najgore vrednosti iz datasheetova; izvor je naveden uz svaku. |
| `simulacija/apps_sim.py` | Simulacija logike provere (merenje svake 1 ms, prozori −8 % … 108 %, razlika 10 p.p., FAULT posle 50 ms, oporavak). Proverava i CRC-8 i pakovanje CAN poruke, i crta slike. |
| `simulacija/slike/` | Slike koje simulacija generiše (blok šema, prenosne funkcije, scenariji). |

## Pokretanje

```bash
pip install -r requirements.txt
python proracun_greske.py
python simulacija/apps_sim.py
```

Testirano sa Python 3.12. Simulacija prekida izvršavanje (`assert`) ako neka provera ne prođe.

## Napomene

- Simulacija proverava **logiku** (otkrivanje kvarova, vremena, ponašanje izlaza), a ne tačnost;
  tačnost je u `proracun_greske.py`.
- Parametri (razdelnici, uglovi, naponi) su isti kao u izveštaju; promena vrednosti na vrhu fajlova
  menja sve rezultate.
- Brojevi su proračun, a ne merenje: kašnjenje senzora, vremena ECU-a i invertora i ponašanje na
  različitim temperaturama treba potvrditi na prototipu.
