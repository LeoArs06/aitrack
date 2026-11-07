# Head tracker IMU — Pro Micro

Firmware 1.0, protocollo binario v1 in `docs/hybrid/PROTOCOL.md`.
Target: Pro Micro ATmega32U4 **5 V / 16 MHz**, core Arduino AVR **1.8.8**,
FQBN `arduino:avr:leonardo`. Solo `Wire`, nessun HID.

| HW-123 / GY-521 | Pro Micro | Note |
|---|---|---|
| VCC | VCC (5 V da USB) | solo dopo la verifica del regolatore a 3,3 V |
| GND | GND | massa comune |
| SDA | 2 (SDA, PD1) | pull-up del modulo a 3,3 V |
| SCL | 3 (SCL, PD0) | pull-up del modulo a 3,3 V |
| AD0 | GND | indirizzo 0x68; il firmware prova anche 0x69 |
| INT | non collegato | pin 7 riservato; polling DATA_RDY |
| XDA, XCL | non collegati | |

Pre-check da DESIGN.md: **prima di collegare SDA/SCL**, alimentare il modulo
da 5 V e misurare circa 3,3 V all'uscita LDO e circa 3,3 V su SDA/SCL, senza
altri collegamenti. Se SDA/SCL misurano 5 V, i pull-up vanno a VCC:
**non collegare il bus; usare un level shifter**.
Il firmware disabilita subito i pull-up interni a 5 V dopo ogni `Wire.begin()`.
Con collegamento diretto rimane il margine elettrico fuori specifica:
V_IH minimo ATmega32U4 = 0,7 × 5 V = 3,5 V > 3,3 V. Usare fili corti;
se compaiono errori I²C, controllare cablaggio/pull-up e usare un BSS138.

## Schema elettrico definitivo (collegamento diretto, senza level shifter)

```
   USB-C (PC)
      │ 5 V
┌─────┴──────────── Pro Micro 5 V / 16 MHz ───┐
│ VCC (5 V) ●────────────────────────┐        │
│ GND       ●──────────────────┐     │        │
│ 2 / SDA   ●────────────┐     │     │        │   pull-up interni ATmega
│ 3 / SCL   ●──────┐     │     │     │        │   DISATTIVATI dal firmware
│ 7 (INT, riserva) ○     │     │     │        │
└──────────────────┼─────┼─────┼─────┼────────┘
                   │     │     │     │   fili corti (< 10 cm)
┌──────────────────┼─────┼─────┼─────┼──────── HW-123 (GY-521) ─┐
│               SCL●  SDA●  GND●  VCC●                          │
│                  │     │     │     │                          │
│                  │     │     │   ┌─┴──────┐ 3,3 V             │
│                  │     │     │   │  LDO   ├──────┬─────┬──── VDD MPU
│                  │     │     │   │ (662K) │      │     │      │
│                  │     │     │   └───┬────┘   R pull-up (on-board,
│                  │     │     │       │        ~2,2–4,7 kΩ verso 3,3 V)
│                  └─────┼─────┼───────┼─────────┘     │      │
│                        └─────┼───────┼───────────────┘      │
│                    AD0 ●─────┤ GND   │  → indirizzo 0x68     │
│                    INT ○  XDA ○  XCL ○   (non collegati)     │
└───────────────────────────────────────────────────────────────┘
```

- Bus I²C a riposo a **3,3 V** (pull-up del modulo): il chip IMU non vede mai
  5 V, purché i pull-up interni del Pro Micro restino spenti (lo fa il
  firmware subito dopo `Wire.begin()`, a ogni re-inizializzazione).
- Valore delle resistenze di pull-up e uscita del regolatore: da misurare
  come sopra prima di collegare SDA/SCL.
- Se servisse il level shifter (errori I²C persistenti): BSS138 a 2 canali,
  lato LV = 3,3 V del modulo, lato HV = VCC Pro Micro; nessuna modifica al
  firmware.

## Compilazione e upload

Da root del repository, con arduino-cli e core installato:

```powershell
arduino-cli core install arduino:avr@1.8.8
arduino-cli compile --fqbn arduino:avr:leonardo firmware/headtracker_imu
arduino-cli upload --fqbn arduino:avr:leonardo -p COMx firmware/headtracker_imu
```

Se l'upload fallisce sul Pro Micro, fare doppio tap sul reset e usare la porta
COM del bootloader durante la sua breve finestra di disponibilità.
Non aprire la seriale a **1200 baud**: attiva il bootloader. La diagnostica usa
115200 baud e DTR attivo. Chiudere altri monitor seriali prima di avviarla.

## Diagnostica

```powershell
python -m pip install pyserial
python firmware/tools/imu_diag.py --selftest
python firmware/tools/imu_diag.py --port COMx --seconds 60 --csv firmware/still.csv
python firmware/tools/imu_diag.py --port COMx --seconds 30 --csv firmware/shake.csv
```

Omettere `--port` per rilevamento automatico: VID 0x2341/0x1B4F/0x2A03
prioritari, poi altre porte, sempre verificando INFO dopo `?`.
Prima prova: sensore fermo per tutti i 60 s, soprattutto i primi 5 s usati
per il bias. Poi scuotere/ruotare per verificare continuità e saturazioni;
bias, rumore e deriva della prova in movimento non sono misure di quiete.
I CSV contengono i registri grezzi e i tempi di ricezione host Unix e monotono;
i campioni ricevuti nello stesso blocco USB condividono il tempo host.

Valori attesi: **200 ± 1 Hz**, campioni persi **<0,1%**, rumore gyro MPU6050
tipico circa **0,05 dps RMS** in quiete (dipende dal modulo), norma accel
circa 1 g. CRC idealmente zero. I percentili USB descrivono gli intervalli
di ricezione, inclusi batching e buchi; la deriva clock è una regressione
sui punti di minima latenza per secondo MCU, positiva se MCU più veloce.
La deriva gyro usa integrazione trapezoidale dopo i primi 5 s, con bias
stimato in quei 5 s; sui campioni mancanti interpola gli estremi.

Flag: `0x01` accel saturo, `0x02` gyro saturo, `0x04` reinizializzazione,
`0x08` campioni non trasmessi. STATUS: 1 assente, 2 errore I²C/configurazione,
3 recuperato, 4 DATA_RDY assente per oltre 50 ms. Errori I²C: controllare
alimentazione, massa, fili, pull-up e level shifter. Perdite USB: controllare
host/cavo e chiudere altri programmi sulla porta.

Comandi ASCII con newline (CR tollerato): `?` INFO, `R` reinizializza,
`0` sospende SAMPLE, `1` riprende. Le acquisizioni e `seq` proseguono anche
durante la sospensione; i buchi dopo `1` sono quindi intenzionali.
I contatori INFO sono aggiornati dalla diagnostica con `?` ogni secondo.

Decisioni: timeout Wire 2500 µs (API verificata nel core 1.8.8), backoff
100/200/400/800/1000 ms. `reinit_count` conta i tentativi di reinizializzazione
riusciti dopo il primo tentativo di boot; il primo campione dopo questi
tentativi mantiene il flag fino alla trasmissione riuscita. Un ID diverso
da 0x68 è trattato come famiglia 6500 per ACCEL_CONFIG2 e temperatura;
un clone con ID sconosciuto richiede conferma su hardware. Le righe di
campioni perse prima della prima ricezione non sono misurabili tramite gap.

Verifica senza hardware: compilazione e self-test del parser. Nessuna prova
elettrica, di upload o di acquisizione reale è stata eseguita.
