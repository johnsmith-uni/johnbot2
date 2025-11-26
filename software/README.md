# johnbot2 Host Controller (Python)

This directory contains the **host-side controller** for the johnbot2 robot swarm.
It implements the OSC-based control and logging used in the experiments of

> *From Swarm to Individual: Emergent Individuality in Light-Mediated Robot Collectives*  [oai_citation:0‡ikegami.pdf](sediment://file_00000000697c72088fa7500d9f754807)  

Ten vibration-driven robots send light sensor values to the host.  
The host computes phototaxis motor commands according to the mapping defined in the paper
and sends motor commands back to each robot. At the same time, it logs sensor and motor
values at 24 fps to CSV for later analysis.

---

## Files

- `johnbot2_controller.py`  
  Main host controller script (OSC server + client, control law, logging).

- `requirements.txt`  
  Python dependencies for the host controller.

- (optional) `environment.yml`  
  Conda environment description, if you choose to provide one.

Log files are written to:

- `robot_logs/` (created automatically)

---

## Relation to the Paper

The controller implements the phototaxis control law described in the Methods section
of the paper (Section 2.1 and Appendix A).  [oai_citation:1‡ikegami.pdf](sediment://file_00000000697c72088fa7500d9f754807)  

For left and right sensor readings \(S_L, S_R\), the host computes:

\[
\hat r_L = \frac{S_R + 1}{S_L + S_R + 2}, \qquad
\hat r_R = \frac{S_L + 1}{S_L + S_R + 2}
\]

A sharp sigmoid

\[
\sigma_\alpha(x) = \frac{x^\alpha}{x^\alpha + (1-x)^\alpha}
\]

with \(\alpha = 8\) is applied, and motor commands \((M_L, M_R)\) are:

\[
M_L = M_{\max} \, \sigma_\alpha(\hat r_L), \qquad
M_R = M_{\max} \, \sigma_\alpha(\hat r_R),
\]

with \(M_{\max} = 200\).

The Python implementation is a direct translation of these equations.

---

## Features

- Receives light sensor values from each robot via OSC (`/sensor`)
- Computes motor commands using the paper’s phototaxis mapping
- Sends motor commands back to each robot via OSC (`/motor`)
- Optional LED control via OSC (`/LED`) if enabled
- Logs sensor and motor values at **24 fps** to CSV
- Warns if any robot stops sending data for more than 5 seconds
- Clean shutdown on `Ctrl+C` (sends stop signals to all robots)

---

## Requirements

- Python **3.8+**
- OS: Linux / macOS / Windows (tested on Linux-like environments)
- Python packages:
  - `python-osc`

All other imports (`threading`, `time`, `logging`, `csv`, `os`, `signal`, `sys`, `datetime`, `math`)
are from the Python standard library.

Install dependencies with:

```bash
pip install -r requirements.txt
