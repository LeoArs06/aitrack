# Stato — head tracker ibrido

Aggiornato: 7 novembre 2026 sera. Branch `hybrid-imu` (fork `LeoArs06/aitrack`).

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

## Non verificato
- Firmware su hardware (modulo non ancora saldato).
- Modalità Webcam e Ibrida **con un volto reale** davanti alla C920
  (le correzioni upstream cambiano il percorso webcam: confrontare con
  AITrack 0.7.1 di OpenTrack con `tools/udp_monitor.py`).
- Valore del roll a testa dritta in AITrack (atteso ≈90°, limiti 0–180):
  se fosse ≈0 le pose con roll negativo verrebbero scartate dalla fusione.
- Calibrazione guidata con IMU vera; resa reale dei micromovimenti.
- I numeri della fusione (0,2° contro 4,6°) vengono da un simulatore scritto
  insieme alla fusione: indicativi, non una misura.

## Prossimi passi
1. Leo: 1 minuto davanti alla C920 → confronto Webcam vs 0.7.1 e prova
   Ibrida simulata con volto (traslazione, Hz ottici, roll a riposo).
2. Leo: saldatura, misure 3,3 V, `imu_diag.py` fermo 60 s + scossa.
3. Prova ibrida reale con `hybrid_log=true`; messa a punto con i log.

## Per riprendere
Leggi questo file, `DESIGN.md`, `PROTOCOL.md`. Build: `tools/build.ps1 -Tests`.
Test Qt: comandi in `Client/src/hybrid_qt/README.md`. Codex: quota esaurita
il 7/11 alle ~21, ripristino 21:59.
