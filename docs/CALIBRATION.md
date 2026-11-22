# Calibration and motion models (MPU-6050 only)

RoadSense has no wheel encoder and no GPS on the car, so every position comes from one 6-axis IMU.
This document covers:
1. what the firmware computes,
2. which models can turn those signals into distance and heading,
3. how to fit the models from repeated start → end drives,
4. how to report the accuracy honestly.

The fitting is implemented in [`tools/fit_calibration.py`](../tools/fit_calibration.py).

---

## 1. Signals and notation

The firmware samples the MPU-6050 at $f_s = 200$ Hz, which gives $\Delta t = 5$ ms.

| Symbol | Meaning | Units |
|---|---|---|
| $\mathbf a_k$ | accelerometer sample (specific force) | g |
| $\boldsymbol\omega_k$ | gyroscope sample | deg/s |
| $\mathbf b_\omega$ | gyroscope bias | deg/s |
| $\hat{\mathbf u}$ | unit "up" vector in the sensor frame, from gravity | – |
| $\theta$ | heading, counter-clockwise from the start direction | deg |
| $(x, y)$ | position: $x$ along the start direction, $y$ to the left | m |

### 1.1 Gravity alignment, so the mounting angle doesn't matter

With the car parked, the IMU measures gravity only:

$$\hat{\mathbf u} = \frac{\bar{\mathbf a}}{\lVert\bar{\mathbf a}\rVert}, \qquad g_0 = \lVert\bar{\mathbf a}\rVert, \qquad \mathbf b_\omega = \bar{\boldsymbol\omega}$$

Here $\bar{\ }$ is the mean over 2 s at rest, taken at boot or after `calib`.

### 1.2 Yaw rate, vertical acceleration, pitch/roll rate

$$\dot\psi_k = (\boldsymbol\omega_k - \mathbf b_\omega)\cdot\hat{\mathbf u}
\qquad
a_{z,k} = \mathbf a_k\cdot\hat{\mathbf u} - g_0
\qquad
q_k = \big\lVert(\boldsymbol\omega_k-\mathbf b_\omega) - \dot\psi_k\,\hat{\mathbf u}\big\rVert$$

* $\dot\psi$ is the **raw yaw rate**: rotation about the vertical, whatever the board's orientation.
* $a_z$ is the **vertical linear acceleration**, used for surface type and potholes.
* $q$ is the pitch/roll rate, reported with each pothole.

### 1.3 Driving detector (zero-velocity detection)

Let $\sigma_a$ be the standard deviation of $\lVert\mathbf a\rVert$ over a 0.25 s window:

$$\text{driving} = \begin{cases}1 & \sigma_a > \tau_{mov}\\ 0 & \sigma_a \le \tau_{mov}\ \text{for 3 consecutive windows}\end{cases}$$

The threshold $\tau_{mov}$ is `movTh`. The detector is effectively a classifier on motor and road vibration. While the car is parked for more than 2 s (a **zero-velocity update**), the firmware re-estimates the bias with a slow low-pass filter:

$$\mathbf b_\omega \leftarrow \mathbf b_\omega + \alpha(\boldsymbol\omega_k - \mathbf b_\omega), \qquad \alpha = 0.002\ (\tau \approx 2.5\,\text{s})$$

This is the main defence against heading drift. **Park for 2–3 s before each run and at each stop.**

### 1.4 Parameter-free integrals logged for calibration

Every pose message includes three integrals, measured since the last `reset`. None of them depends on any tunable parameter:

$$T = \sum_k \text{driving}_k\,\Delta t \quad(\texttt{drv\_s}), \qquad
V = \sum_k \text{driving}_k\,\sigma_{z,k}\,\Delta t \quad(\texttt{vib\_int}), \qquad
\Psi = \sum_k \dot\psi_k\,\Delta t \quad(\texttt{yaw\_raw})$$

Here $\sigma_z$ is the 1-s RMS of $a_z$ (`vib_g`). Because they don't depend on the parameters, one dataset can be **re-integrated offline with any model** without driving again.

---

## 2. Models

### 2.1 Speed (the hard part with an IMU only)

Integrating the accelerometer twice gives errors that grow as $\tfrac12 b_a t^2$. A bias of only 0.01 g already gives about 50 m of error after 30 s. So distance has to come from a **speed model** that is switched on by the driving detector.

| Model | Speed while driving | Firmware | Free parameters |
|---|---|---|---|
| **S0 constant** | $v = v_0$ | `spdMode 0` | $v_0$ |
| **S1 vibration-proportional** | $v = v_0\,\sigma_z/\sigma_{ref}$ | `spdMode 1` | $v_0$, $\sigma_{ref}$ |
| **S2 vibration-linear** | $v = v_0 + k_v\,\sigma_z$ | `spdMode 2` | $v_0$, $k_v$ |

Integrated over a run, S0 and S2 are **linear in the logged integrals**:

$$L_{S0} = v_0\,T \qquad\qquad L_{S2} = v_0\,T + k_v\,V$$

This makes path-length fitting ordinary least squares:

$$\hat{\boldsymbol\beta} = (X^\top X)^{-1}X^\top \mathbf L,\qquad X = [\,T_i\,]\ \text{or}\ [\,T_i,\ V_i\,]$$

There is also a diagnostic model with a per-run constant, $L = v_0 T + c$. If $c < 0$, the car moves before the detector notices. The detection lag is $-c/v_0$ seconds.

**Why vibration can help:** at a fixed throttle, faster motion excites larger and higher-frequency chassis vibration, so $\sigma_z$ partly tracks speed. **Why it can hurt:** $\sigma_z$ also depends on the surface (gravel ≫ asphalt). S2 only pays off if your runs are on the same surface type as the deployment. The leave-one-out test in §3 decides this for you.

### 2.2 Heading

$$\theta(t) = s_g\,\Psi(t) \qquad\text{(H1: gyro scale factor } s_g\text{, firmware }\texttt{gyroScale})$$

$$\theta(t) = s_g\,\Psi(t) - \beta\,t \qquad\text{(H2: plus a residual bias drift }\beta\text{ in deg/s)}$$

If you only know the end headings, $s_g$ has a closed form:

$$\hat s_g = \frac{\sum_i \Psi_i\,\theta_i}{\sum_i \Psi_i^2}$$

The truth $\theta_i$ is unwrapped to the turn the gyro saw, so for example 350° is compared as −10°.

A large fitted $\beta$ means bias tracking isn't working. Usually the car didn't park long enough, or the motor heats the IMU. β is a diagnostic only, not a firmware parameter.

### 2.3 Position (dead reckoning)

The firmware and the fitting tool use the same discrete kinematics:

$$x_{k+1} = x_k + \Delta d_k\cos\theta_k,\qquad y_{k+1} = y_k + \Delta d_k\sin\theta_k,\qquad \Delta d_k = v_k\,\Delta t$$

The tool works on the 1 Hz logged series. It uses the midpoint heading $\tfrac12(\theta_k+\theta_{k+1})$ for each 1-s interval, which keeps the discretisation error well below the measurement error.

### 2.4 Joint fit from start → end runs

Take $N$ runs from the same start pose, each with a measured end point $\mathbf p_i$, optionally a path length $L_i$ and an end heading $\theta_i$. Choose the parameters $\boldsymbol\phi$ (for example $\{v_0, s_g\}$ or $\{v_0, k_v, s_g\}$) that minimise:

$$J(\boldsymbol\phi) = \sum_{i=1}^{N} \Big[\ \lVert \hat{\mathbf p}_i(\boldsymbol\phi) - \mathbf p_i\rVert^2 \;+\; w_\theta^2\,\mathrm{wrap}\big(\hat\theta_i(\boldsymbol\phi) - \theta_i\big)^2 \;+\; \big(\hat L_i(\boldsymbol\phi) - L_i\big)^2 \Big]$$

* $\hat{\mathbf p}_i$, $\hat\theta_i$ and $\hat L_i$ come from re-integrating run $i$ with the parameters $\boldsymbol\phi$, using §2.3.
* $w_\theta = 0.05$ m/deg, so 1° of heading error counts as much as 5 cm of position error.

The minimisation uses Levenberg–Marquardt. Turning runs are what make $s_g$ observable; straight runs mostly constrain $v_0$.

### 2.5 Choosing a model: leave-one-out cross-validation

Fitting error always drops as you add parameters, so the tool also reports **LOO error**. For each run $i$, it fits on the other $N-1$ runs, predicts run $i$, and takes the RMS of those prediction errors. Use the model with the lowest LOO error. That is the number to put in the report as *expected accuracy on a new drive*. Quote it in metres and as a % of the distance driven.

For the closed-form length fits, LOO comes free from the hat matrix:

$$e_{(i)} = \frac{e_i}{1 - h_{ii}}, \qquad H = X(X^\top X)^{-1}X^\top$$

---

## 3. Data-collection protocol (start → end runs)

1. **Fixed start.** Tape a start line and mark which way the car faces. Measure the start bearing with a phone compass or Google Maps, and note the start lat/lon.
2. **Fixed throttle.** Tape the transmitter trigger or use a throttle limiter, so every run uses the same throttle. This matters more than anything else.
3. **For each run:**
   1. Place the car on the start mark. Send `reset`, then `label runNN`. Keep the car still for 3 s (bias update).
   2. Drive to an end point and stop. Wait 2 s, then send `label none`.
   3. Measure the truth:
      * End point: either $(x, y)$ with a tape measure in the start frame (x along the start direction, y to the left), or lat/lon from Google Maps (right-click → coordinates). Map points are roughly ±1–3 m, so use runs of 30 m or more for those.
      * Optional: the path length driven ($L$) and the final heading relative to the start direction, counter-clockwise positive.
   4. Enter it into `calib/runs.csv`, copied from [`calib/runs_template.csv`](../calib/runs_template.csv).
4. **Mix the routes:** straight runs of different lengths, single left and single right turns, L-shapes, and one or two longer loops. **At least 8 runs; 12–20 is better.** Include both turn directions so a scale error isn't confused with an asymmetric mount.
5. Keep `tools/logger.py` running throughout. It writes `logs/session-*.jsonl`.
6. Fit and send the suggested commands to the car:
   ```bash
   python tools/fit_calibration.py logs/session-*.jsonl --runs calib/runs.csv --plot
   # if you used lat/lon end points:
   python tools/fit_calibration.py logs/*.jsonl --runs calib/runs.csv --origin 33.64301,-117.84170,90 --plot
   ```
   It prints the best model's `set spdMode/v0/kVib/gyroScale` commands. `calib/fit.png` shows every re-integrated run against the measured end points; put it in the report.
7. **Validate** on 3–5 new runs that weren't used in the fit, and report that error.

---

## 4. Beyond the baseline: models worth trying

These are ordered by effort. Each is a self-contained extension for the final report.

### 4.1 Waypoint re-anchoring (measurement update)
Dead-reckoning error grows with distance. Resetting the position at known landmarks bounds it. The firmware already does this with `anchor x y [hdg]`, and so does the dashboard's "Click true position → re-anchor car". Report the error per segment between anchors.

### 4.2 1-D Kalman filter for speed
Fuse the forward accelerometer (prediction) with the vibration speed model and zero-velocity updates (measurements):

$$v_{k+1} = v_k + (a_{x,k} - b_{a})\Delta t + w_k,\qquad b_{a,k+1} = b_{a,k} + w^{b}_k$$

$$z^{\text{vib}}_k = v_0 + k_v\sigma_{z,k} + n_k,\qquad z^{\text{ZUPT}} = 0\ \text{when parked}$$

The state is $[v, b_a]$. This keeps the fast response of the accelerometer (start/stop transients that S0 misses) without the $t^2$ drift. It needs the forward axis, which you get from $\hat{\mathbf u}$ plus a short straight acceleration at the start of a run.

### 4.3 Learned speed regression
Compute features per 1-s window, for example:
* RMS of $a_z$ and $\lVert\mathbf a\rVert$,
* band powers from an FFT (0–5, 5–15, 15–40, 40–100 Hz),
* the dominant frequency,
* gyro RMS.

Then regress them onto speed. Use ridge regression or a small random forest, trained with the per-run constraint $\sum_k \hat v_k\Delta t = L_i$ (or simply on runs at known constant speeds). Validate with leave-one-*run*-out, never with random windows, because windows from the same run are correlated.

### 4.4 Full 2-D EKF / factor graph
The state is $[x, y, \theta, v, b_\omega]$. The process model is §2.3. Measurements are:
* ZUPT ($v = 0$),
* zero-angular-rate updates ($\dot\theta = 0$) when parked,
* anchors / map clicks (absolute $x, y$),
* optionally phone GPS at 1 Hz for outdoor runs.

Smoothing the whole run offline (a Rauch–Tung–Striebel smoother) gives the best possible path for the map, even if the real-time estimate is coarser.

### 4.5 Gyro characterisation (Allan variance)
Log 30–60 min with the car still (`stream 100` → `pio device monitor | tee still.csv`), then compute the Allan deviation of $\dot\psi$. This gives:
* the angle random walk: heading noise that grows as $\sqrt{t}$,
* the bias instability: the floor on how well ZUPTs can do.

These are good numbers to compare against the datasheet.

---

## 5. Error budget to put in the report

| Source | Effect | Mitigation | How to measure |
|---|---|---|---|
| Speed varies with throttle, battery and surface | distance error ∝ speed error | fixed throttle; S2 model; fresh battery | `dcal` / length fit residuals |
| Detection lag at start and stop | constant offset per run | lower `movTh`, D2 diagnostic | sign of $c$ in D2 |
| Gyro scale error $\delta s$ | heading error $\delta s\cdot\lvert\Psi\rvert$ (1% → 3.6° per 360°) | `tcal` or the joint fit | end-heading residuals |
| Gyro bias $b$ | heading drift $b\cdot t$ | boot calib + ZUPT tracking; park often | β in model M3 |
| Heading error $\delta\theta$ | lateral error ≈ $L\,\delta\theta$ (1° → 1.7 cm/m) | all of the above | end-point scatter perpendicular to the path |
| Wheel slip (grass, gravel) | the car is "driving" but moving slower | per-surface $v_0$ / S2 | compare runs by surface |
