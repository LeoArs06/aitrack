# Stato — ARES Tracker (head tracker ibrido)

Aggiornato: 7 novembre 2026, 21:30. Nome prodotto: **ARES Tracker** (ARCUS LAB),
eseguibile `AresTracker.exe`, cartella portabile `software/ARES-Tracker`. Branch `hybrid-imu` (fork `LeoArs06/aitrack`).

## Fatto (verificato su PC, senza hardware IMU)
- Build originale e build ibrida: VS Build Tools 2026 (v145), Qt 5.15.2
  (`software/deps/Qt`), `tools/build.ps1` ripetibile; cartella portabile
  `software/AITrack-hybrid`.
- Firmware `firmware/headtracker_imu` (protocollo v1, 200 Hz): compila
  (8,7 KB flash, 423 B RAM); `imu_diag.py --selftest` ok.
- Nucleo di fusione `Client/src/hybrid` + test (`tests/hybrid`): tutti verdi
  sul simulatore.
- Convenzione degli angoli verificata contro OpenCV reale:
  R = Rz(roll)·Ry(yaw)·Rx(pitch), errore 1e-13.
- Integrazione Qt (`Client/src/hybrid_qt`, UI nella finestra principale) +
  test Qt (thread, UDP, calibrazione, riconnessione, layout): verdi.
- Prova end-to-end nell'app con IMU simulata (`AITRACK_IMU_SIM=1`) e C920:
  200,0 Hz su UDP 4242, nessun pacchetto malformato, chiusura pulita.
- Correzioni upstream: landmark subpixel, selezione volto, controllo
  solvePnP, riconfigurazione UDP, preferenze ONNX, frame vuoti, filtro EA.

## Verificato su hardware (7/11, modulo cablato e chiuso nel box, COM18)
- Flash firmware ok (FQBN `arduino:avr:micro`: bootloader Arduino Micro).
- `imu_diag.py`: MPU6050 0x68, 200,4 Hz, 0 persi, 0 CRC, 0 errori I²C;
  a riposo gyro 1–3 LSB (~0,05 °/s), accel ~0,98 g.
- App con IMU vera + volto: 200,4 Hz UDP, 0 pacchetti errati, stato
  `hybrid`, ottica 24 Hz, latenza stimata 60 ms, chiusura pulita.
  **Senza calibrazione**: rotazioni non attendibili (correzione 133 °/min).

## Svolta 7/11 sera: calibrazione automatica (dati reali)
- Registrazione reale (`hybrid_log`) analizzata con `tests/hybrid/replay.cpp`
  (`hybrid_replay log.csv -1 out.csv`): webcam a **15 fps** (poca luce),
  roll neutro ~87°, pose valide 99,7%. La calibrazione guidata falliva perché
  usava 4 fotogrammi fissi (pensati per 30 fps) e una fase "fermo" impossibile
  con le cuffie in testa.
- Ora: `AutoCalibrator` in background, nessun pulsante né fase ferma. Finché
  non aggancia esce la posa webcam pura (come AITrack). Sul log reale aggancia
  in 12 s: residuo 15%, latenza 90 ms, bias = misura a tavolo, deriva 1,9 °/min.
  Uscita fusa vs webcam: ritardo -90 ms, rumore -30/40%, scarto ~1°.
- Mappatura sul casco di Leo (cuffia sinistra): yaw=-gy, pitch=+gz, roll≈+gx.
- Pulsante "Recalibrate": dimentica il montaggio e lo rifà da zero.

## Non verificato
- Modalità Webcam e Ibrida **con un volto reale** davanti alla C920
  (le correzioni upstream cambiano il percorso webcam: confrontare con
  AITrack 0.7.1 di OpenTrack con `tools/udp_monitor.py`).
- Valore del roll a testa dritta in AITrack (atteso ≈90°, limiti 0–180):
  se fosse ≈0 le pose con roll negativo verrebbero scartate dalla fusione.
- Calibrazione guidata con IMU vera; resa reale dei micromovimenti.
- I numeri della fusione (0,2° contro 4,6°) vengono da un simulatore scritto
  insieme alla fusione: indicativi, non una misura.

## Prossimi passi
1. Leo: prova in DCS con calibrazione automatica; nuove registrazioni
   `hybrid_log` → replay per mettere a punto la fusione sui dati reali.
2. Confronto modalità Webcam vs AITrack 0.7.1 e roll a testa dritta.

## Per riprendere
Leggi questo file, `DESIGN.md`, `PROTOCOL.md`. Build: `tools/build.ps1 -Tests`.
Test Qt: comandi in `Client/src/hybrid_qt/README.md`. Codex: quota esaurita
il 7/11 alle ~21, ripristino 21:59.
