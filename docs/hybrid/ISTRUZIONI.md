# ARES Tracker — istruzioni Windows

ARES Tracker (ARCUS LAB) è un fork di AITrack con fusione IMU + webcam.

## 1. Preparare il modulo (una volta)

1. Prima di saldare SDA/SCL: alimenta l'HW-123 (VCC 5 V, GND) e misura con
   il multimetro: uscita regolatore ≈3,3 V; SDA e SCL verso GND ≈3,3 V.
   Se SDA/SCL leggono 5 V **non collegarli**: serve un level shifter.
2. Collegamenti: tabella in `firmware/README.md` (SDA→2, SCL→3, AD0→GND,
   VCC→VCC, GND→GND, INT libero).
3. Carica il firmware:
   ```
   arduino-cli compile --fqbn arduino:avr:micro firmware/headtracker_imu
   arduino-cli upload  --fqbn arduino:avr:micro -p COMx firmware/headtracker_imu
   ```
   Se l'upload non parte: doppio tocco RST→GND e rilancia subito.
4. Diagnosi (cuffie appoggiate ferme sul tavolo, 60 s):
   ```
   python firmware/tools/imu_diag.py --seconds 60
   ```
   Valori buoni: frequenza 200 ± 1 Hz, persi < 0,1 %, CRC 0, errori I²C 0,
   rumore giroscopio ≈ 0,05 °/s. Poi ripeti scuotendo le cuffie: niente
   errori né saturazioni a movimenti normali. Errori I²C ripetuti =
   collegamenti o margine di livello (vedi README firmware).

## 2. Programma

- Pronto all'uso: `software/ARES-Tracker/AresTracker.exe` (cartella portabile).
- Ricompilare: in PowerShell nella cartella `aitrack`:
  `.\tools\build.ps1 -Tests -Dist ..\ARES-Tracker`
  (scarica da solo Qt 5.15.2 e le dipendenze; serve Visual Studio Build
  Tools con C++, Python, 7-Zip).

## 3. Uso con OpenTrack e DCS

1. OpenTrack: input **UDP over network**, porta 4242 (come con AITrack
   originale). Filtro: nessuno o leggero (l'uscita è già filtrata, 200 Hz).
   Tieni il tasto di centratura di OpenTrack.
2. ARES Tracker: Mode **Hybrid IMU**, Port **Auto**, **Connect**, poi
   **Start tracking**.
3. Prima volta: **Calibrate** con il volto ben visibile. Resta fermo 2 s,
   poi muovi la testa lentamente per ~10 s: destra-sinistra, su-giù,
   inclinazione. Il programma ricava da solo l'orientamento del sensore
   sulle cuffie e il ritardo della webcam; il risultato viene salvato.
   Rifai la calibrazione se sposti il case sulle cuffie o cambi webcam.
4. Cursore **Optical correction**: più forte = la webcam riallinea prima
   (meno deriva, un po' più del suo rumore); più debole = più liscio.
   Parti da metà.
5. Mode **Webcam** = AITrack originale. Se l'IMU si scollega il programma
   passa gradualmente alla sola webcam e si ricollega da solo.

Riga di stato: frequenza IMU, campioni persi, errori CRC, frequenza
ottica, latenza stimata della webcam, deriva corretta (°/min), stato
(hybrid / gyro only = volto non visto / optical only = IMU assente).

## 4. Prove senza hardware

`set AITRACK_IMU_SIM=1` prima di avviare: l'IMU viene simulata (la rotazione
non segue la tua testa). `python tools/udp_monitor.py` mostra frequenza e
valori inviati a OpenTrack (chiudi OpenTrack prima, usa la stessa porta).

Per registrare dati reali da usare nella messa a punto: in `prefs.ini`
aggiungi `hybrid_log=true`; i file finiscono in `logs/`.
