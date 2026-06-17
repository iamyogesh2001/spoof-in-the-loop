# Spoof-in-the-Loop: GPS Spoofing Resilience in CACC Platoons

> **Submitted to:** 3rd IEEE International Conference on Vehicular Technology and Transportation Systems (ICVTTS 2026), Bangalore, India, 18–19 September 2026. IEEE VTS Bangalore Chapter.

---

## Paper

**Title:** Spoof-in-the-Loop: Simulation-Based Analysis of GPS Spoofing Resilience in Cooperative Adaptive Cruise Control Platoons with Lightweight Anomaly Detection

**Authors:**
- Yogesh Rethinapandian — Dept. of Electrical and Computer Engineering, University of Illinois Chicago, USA *(Corresponding)*
- Arun Karthik Sundararajan — IEEE Member
- Kaushik Kumar — Dept. of Data Science, University of Arizona, USA
- Smrithi Prakash — Dept. of Computer Science, SRM Institute of Science and Technology, India
- Vikram Raja — Research Associate, Research and Development, Kamuit Inc., India

**Abstract:**
Cooperative Adaptive Cruise Control (CACC) platoons rely on GPS and Vehicle-to-Vehicle communication to maintain safe inter-vehicle gaps at highway speeds. GPS spoofing can inject false position data into a vehicle's receiver, causing the CACC controller to make wrong speed decisions and precipitating rear-end collisions. The relationship between spoof magnitude and platoon safety breakdown has not been established. This paper presents *Spoof-in-the-Loop*, a simulation framework for a five-vehicle heterogeneous CACC platoon using TEXBAT-calibrated GPS spoof profiles and Oxford RobotCar noise parameters. We establish a safety envelope showing that spoofs below 18 m induce no collision, while spoofs at or above 18 m trigger collision within 19–25 s. We propose a lightweight V2V-Radar Cross-Consistency Detector exploiting the spoof-immune nature of automotive radar, flagging attacks with a True Positive Rate of 96.2% and False Positive Rate of 4.6% at a detection latency under 2.3 s, requiring no new hardware. Mitigation via radar-only CACC prevents collision at the critical threshold, while ramp spoofing remains an open residual challenge.

**Keywords:** GPS spoofing, CACC platooning, V2X security, automotive cybersecurity, anomaly detection

---

## Repository Structure

```
spoof-in-the-loop/
├── simulation.py          # Main simulation — run this to reproduce all results
├── requirements.txt       # Python dependencies
├── figures/
│   ├── fig1_gaps.png      # Fig. 2: Inter-vehicle gap dynamics (3 scenarios)
│   ├── fig3_envelope.png  # Fig. 3: Safety envelope (spoof magnitude vs collision)
│   └── fig4_latency.png   # Fig. 4: Detection latency + collision onset comparison
├── results/
│   └── summary.json       # All numerical results (collision times, AUC, latency)
└── README.md
```

---

## How to Reproduce

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Run the simulation

```bash
python simulation.py
```

This will:
- Run 5 scenarios: baseline, ramp spoof, step spoof, replay spoof, ramp + mitigation
- Run a safety envelope sweep (spoof magnitude 0–30 m)
- Compute ROC curve and detector metrics
- Generate all 3 figures in `figures/` (PNG + PDF)
- Save numerical results to `results/summary.json`

Runtime: approximately 2–3 minutes on a standard laptop.

### 3. Expected output

```
=======================================================
SPOOF-IN-THE-LOOP SIMULATION
=======================================================

[1/4] Running scenarios...
  Baseline      : collision @ None
  Ramp          : collision @ 24.37s
  Step          : collision @ 19.3s
  Replay        : collision @ 18.67s
  Mitig-Ramp    : collision @ None  (16m spoof)

[2/4] Safety envelope sweep...
[3/4] ROC curve...
[4/4] Generating figures...
```

---

## Simulation Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Platoon size | 5 vehicles | — |
| Cruise speed | 80 km/h (22.22 m/s) | — |
| Desired gap (d) | 8 m | Ploeg et al. 2014 |
| Time headway (h) | 0.5 s | Ploeg et al. 2014 |
| CACC gains (kp, kd) | 0.45, 0.90 | Ploeg et al. 2014 |
| Engine lag (τ) | 0.09–0.12 s | Rajamani 2012 |
| GPS noise (σ) | 1.2 m | Oxford RobotCar Dataset |
| Radar noise (σ) | 0.3 m | Bosch LRR3 spec |
| Spoof profiles | Ramp / Step / Replay | TEXBAT (UT Austin) |
| Detection threshold | 3.0σ (confirmed: 3 steps) | This work |

---

## Key Results

| Scenario | Collision (s) | Detection Latency (s) | Mit. Collision | Min Gap (m) |
|---|---|---|---|---|
| No Attack | — | — | — | 16.4 |
| Ramp (20 m) | 24.4 | 2.27 | 26.1 s | −3.9 |
| Step (20 m) | 19.3 | 0.02 | 21.0 s | −3.6 |
| Replay (4 s lag) | 18.7 | 0.02 | None (prevented) | −141 |

**Critical spoof threshold:** 18 m (below = safe, above = collision within 19–25 s)

**Detector performance:** AUC = 0.985, TPR = 96.2%, FPR = 4.6%

---

## Figures

### Fig. 2 — Inter-Vehicle Gap Dynamics
![Gap Dynamics](figures/fig1_gaps.png)

### Fig. 3 — Safety Envelope
![Safety Envelope](figures/fig3_envelope.png)

### Fig. 4 — Detection Latency and Collision Onset
![Detection Latency](figures/fig4_latency.png)

---

## Calibration Sources

- **GPS noise (σ = 1.2 m):** Oxford RobotCar Dataset — Maddern et al., *Int. J. Robot. Res.*, 36(1), 2017
- **GPS spoof profiles:** TEXBAT — Humphreys et al., Proc. ION GNSS, 2012
- **CACC controller:** Ploeg et al., Proc. IEEE ITSC, 2014
- **Vehicle dynamics:** Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012
- **Radar noise (σ = 0.3 m):** Bosch LRR3 long-range radar specification

---

## Citation

If you use this code or results in your work, please cite:

```bibtex
@inproceedings{rethinapandian2026spoof,
  title     = {Spoof-in-the-Loop: Simulation-Based Analysis of GPS Spoofing
               Resilience in Cooperative Adaptive Cruise Control Platoons
               with Lightweight Anomaly Detection},
  author    = {Rethinapandian, Yogesh and Sundararajan, Arun Karthik and
               Kumar, Kaushik and Prakash, Smrithi and Raja, Vikram},
  booktitle = {Proc. 3rd IEEE Int. Conf. Vehicular Technology and
               Transportation Systems (ICVTTS)},
  year      = {2026},
  month     = {September},
  address   = {Bangalore, India},
  note      = {Submitted}
}
```

---

## Disclosure

Large language model tools were used solely for drafting and editorial assistance. All simulation design, experiments, analysis, and results are the original work of the authors.

---

## License

MIT License — see [LICENSE](LICENSE) for details.
