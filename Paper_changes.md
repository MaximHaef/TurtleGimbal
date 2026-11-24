# Changes needed in Bravo_Team

The implementation now uses the robot's body-velocity command as the model input instead of wheel angular-speed commands. The reason is the Kobuki (TurtleBot 2) base: its driver takes `(v, ω_z)` on `/commands/velocity`, and its serial protocol has no per-wheel command. The firmware drives the wheels from a (speed, radius) pair. Section and equation numbers below refer to [Bravo_Team.md](Bravo_Team.md) / `../Bravo_Team.pdf`.

Sections 1–5 do not change the method: eqs. (3), (4) and (6)–(10) keep their form, and only the input u, eq. (5) and the wording that refers to them change. Section 6 extends the method: the predictor also predicts the IMU A accelerometer reading, and the controller uses it to compensate inertial loads on the gimbal ahead of time.

## 1. Input definition

**Table I, row u_k**

- Old: `u_k` | Right/left wheel angular-speed commands | rad/s
- New: `u_k` | Commanded forward speed and yaw rate | m/s, rad/s

**Sec. II-B-1 (Physical Model), first paragraph and eq. (5)**

Old: "The physical model describes the planar motion of the differential-drive robot using the commanded angular velocities of its right and left wheels. These commands form the input vector u = [Ω_R^cmd, Ω_L^cmd]^T. The corresponding desired forward speed and yaw rate are given by (5)."

New (proposal):

> The physical model describes the planar motion of the differential-drive robot using its commanded forward speed and yaw rate, which form the input vector $\mathbf{u} = [v^{\mathrm{cmd}}, \omega^{\mathrm{cmd}}_z]^T$. The base firmware converts this command into right and left wheel speeds $v_{R,L} = v^{\mathrm{cmd}} \pm \tfrac{b}{2}\,\omega^{\mathrm{cmd}}_z$ and limits them to $v_{\max}$. When a wheel exceeds the limit, both are scaled by the same factor, which preserves the turning radius:
>
> $$
> s = \min\!\left(1,\ \frac{v_{\max}}{\max(|v_R|, |v_L|)}\right), \qquad
> \bar v^{\mathrm{cmd}} = s\,v^{\mathrm{cmd}}, \qquad
> \bar\omega^{\mathrm{cmd}}_z = s\,\omega^{\mathrm{cmd}}_z. \tag{5}
> $$

Eq. (6) then uses $\bar v^{\mathrm{cmd}}$ and $\bar\omega^{\mathrm{cmd}}_z$ in place of $v^{\mathrm{cmd}}$ and $\omega^{\mathrm{cmd}}_z$.

Simpler alternative: drop eq. (5) entirely and state that commands are kept within the wheel-speed limit, so (6) is driven by $\mathbf{u}$ directly. The equation numbers from (6) onward then shift down by one.

**Implementation:** The scaling is applied by the `base_command` relay on the robot Pi before the command reaches the driver, and the predictor uses the scaled command. If the paper keeps the saturation form, it can describe the input as "the executed command after wheel-speed saturation."

**Note:** The scaling matches the radius-preserving behaviour assumed for the Kobuki firmware. The value of v_max (about 0.7 m/s) and the scaling behaviour still need to be verified on the robot before the paper states them.

## 2. Wording that refers to wheel commands

| Location | Old | New |
|---|---|---|
| Abstract | "a wheel-command-based physical model" | "a velocity-command-based physical model" |
| Sec. I, last paragraph (line 19) | "predicts robot base motion from wheel commands" | "predicts robot base motion from velocity commands" |
| Sec. II-B, after eq. (3) | "The wheel command is assumed constant over the prediction horizon." | "The velocity command is assumed constant over the prediction horizon." |

"wheel slip" in Sec. II-B-2 stays as it is.

## 3. Table II

| Parameter | Change |
|---|---|
| Wheel radius r | Remove from the model. It is still needed in practice to convert measured wheel speeds into v and ω_z (system identification, residual targets); keep it only if that is described in the paper |
| Track width b | Keep if the saturation form of eq. (5) is used; otherwise same as r |
| Wheel-speed limit v_max | Add (m/s) if the saturation form of eq. (5) is used |

## 4. Optional additions

- **Sec. II-B-2 (Learned Model):** The context C_k = (u_k, ω_{k-H+1:k}) now mixes m/s and rad/s. A sentence that inputs are normalized before entering the network would make this explicit.
- **Sec. II-D (Development Procedure) / Fig. 3:** τ_v and τ_ω are identified from step responses in v_cmd and ω_z,cmd. Fig. 3 needs no change unless it names wheel commands.
- **Sec. II (robot description):** The Kobuki base has a built-in gyro. The paper's statement that the IMUs are part of the gimbal is still correct, but if the Kobuki gyro is used later (as context or as a reference), it has to be mentioned here and in Fig. 2.
- **Fig. 2:** No change needed. The input is labelled only with the symbol u_k.

## 5. Physical model output and initial state

The implementation goes beyond the paper's planar model in two ways:

- **Translational acceleration.** Besides the angular velocity [0, 0, ω_z], the physical model also outputs the base's translational acceleration in the base frame, [dv/dt, v·ω_z, 0] (forward plus centripetal, without gravity). If the paper keeps it, Sec. II-B-1 needs a sentence after eq. (6), for example: "From the predicted state, the base acceleration is $\mathbf{a} = [\dot v,\ v\,\omega_z,\ 0]^T$." Section 6 makes acceleration part of the predicted state, with a learned residual, and changes it to the specific force (with gravity) that the accelerometer measures.
- **Initial state.** The rollout (4) starts from $\hat x_{k|k} = x_k$. In the implementation, x_k is measured at the command time t_k: v from wheel odometry and ω_z from the Kobuki gyro. The paper does not say where x_k comes from. A short statement in Sec. II-B would close that gap.

## 6. Accelerometer prediction and inertial feedforward

**What changes.** The paper predicts only the base angular velocity and uses it for kinematic feedforward (eq. 10). The implementation also predicts what the IMU A **accelerometer** will measure. The predicted acceleration acts on the gimbal links as an inertial load (plus gravity), and the controller compensates that load with the same look-ahead idea: a torque feedforward computed from the prediction at a future time, before the load shows up as a joint error.

The quantity predicted is the **specific force** ${}^B\mathbf a = {}^B\ddot{\mathbf p} - {}^B\mathbf g$, the acceleration with gravity, as an accelerometer measures it, expressed in the base frame. Two reasons:
- It is the load on the gimbal: a link of mass m feels the force $-m\,{}^B\mathbf a$, gravity and inertia together. One term therefore compensates both the static gravity torque of an imperfectly balanced gimbal and the inertial torque from base acceleration.
- It can be measured directly: the calibrated IMU A accelerometer, rotated into B, is the ground truth for the residual targets. No gravity has to be removed with an orientation estimate.

**Sec. II-B, state and residual (after eq. 3)**

Old: "... adds the residual to the angular velocity $\boldsymbol\omega_k$ of the gimbal base, so $d_r = 3$. The state dimension is $d_x = 4$. For the state ordering used here, $L = \begin{bmatrix} \mathbf{0}_{1\times3} \\ I_3 \end{bmatrix}$ ..."

New (proposal):

> ... adds the residual to the angular velocity $\boldsymbol\omega_k$ and the specific force $\mathbf a_k$ at IMU A, so $d_r = 6$. The state is $\mathbf x = [v, \boldsymbol\omega^T, \mathbf a^T]^T$ with $d_x = 7$, and $L = \begin{bmatrix} \mathbf{0}_{1\times6} \\ I_6 \end{bmatrix}$.

The context becomes $\mathcal{C}_k = (u_k, \boldsymbol\omega_{k-H+1:k}, \mathbf a_{k-H+1:k})$, and the residual trajectories in (7) are $F \times 6$. In Sec. II-B-1, $\boldsymbol\omega = L^T\mathbf x$ becomes $\boldsymbol\omega = [I_3\ \mathbf 0]\,L^T\mathbf x$, or the paper defines selection matrices $L_\omega$, $L_a$ for the two parts.

**Sec. II-B-1, physical model (after eq. 6)**

New sentence (proposal):

> The specific force at IMU A follows from the predicted state as $\mathbf a = [\dot v,\ v\,\omega_z,\ 0]^T - {}^B\mathbf g + \dot{\boldsymbol\omega} \times \mathbf r_A + \boldsymbol\omega \times (\boldsymbol\omega \times \mathbf r_A)$, where ${}^B\mathbf g = [0, 0, -g]^T$ for level ground and $\mathbf r_A$ is the position of IMU A relative to the base's center of rotation. Tilt from uneven ground and vibration are left to the learned residual.

$\mathbf a$ is an output of the physical model, not a dynamic state: f in (3) recomputes it from v and ω at every step, and the residual corrects it. The lever-arm terms matter because IMU A is not on the Kobuki's axis of rotation: at ω_z = 2 rad/s and |r_A| = 5 cm, ω²r = 0.2 m/s², which is of the same order as the centripetal term v·ω_z at typical speeds (0.4 m/s² at 0.2 m/s).

**Sec. II-B-2, proposal weighting (eq. 8)**

No change in the first version: the weights still use only the gyroscope innovation. Extending ν with the accelerometer innovation (with $\Sigma^{\mathrm{acc}}$ and its own $P_j$) is possible but undecided (ARCHITECTURE.md §7). Eq. (9) already gives the weighted mean of the full state, $\mathbf a$ included.

**Sec. II-C, new inertial feedforward (after eq. 10)**

New text (proposal):

> Base acceleration and gravity load the gimbal joints when the link centers of mass are not on the joint axes. With link masses $m_\ell$ and center-of-mass positions $\mathbf r_{i\ell}(\mathbf q)$ relative to joint axis $\mathbf e_i$ (both in B), the load torque is $\boldsymbol\tau_{\mathrm{in}} = -G(\mathbf q)\,{}^B\mathbf a$ with $G_{i,:} = \mathbf e_i^T \sum_{\ell \ge i} m_\ell [\mathbf r_{i\ell}]_\times$. The controller adds the torque feedforward
>
> $$\boldsymbol\tau_{\mathrm{ff}} = G(\mathbf q)\,{}^B\hat{\mathbf a}(t + T_\tau), \tag{11}$$
>
> where $T_\tau$ is the delay of the torque path. Like (10), it uses the prediction at a future time, chosen by timestamp.

Notes for the paper text:
- A perfectly balanced gimbal has G = 0, and (11) only acts through imbalance. The base's angular acceleration also loads the links through their inertia, independent of balance. A term $-I\,\dot{\hat{\boldsymbol\omega}}$, with $\dot{\hat{\boldsymbol\omega}}$ from the predicted angular-velocity trajectory, can be added the same way.
- How $\boldsymbol\tau_{\mathrm{ff}}$ reaches the motors depends on the driver, which is still open: current/torque feedforward if the driver accepts it, or added to the output of the velocity PI loop if the Pi closes that loop. The paper currently states that the motors run in velocity mode (Sec. II and Fig. 2), so that sentence has to name the added torque input.
- (11) neglects the coupling between joint motion and base rotation (full rigid-body dynamics). That is acceptable for small joint rates.

**Abstract and Sec. I**

Old (abstract): "Multiple predictions are weighted using gyroscope measurements from the gimbal base. Their weighted mean provides a delay-aware angular-velocity prediction, which gimbal kinematics converts into feedforward joint-rate commands."

New (proposal): "... Their weighted mean provides a delay-aware prediction of base angular velocity and acceleration. Gimbal kinematics converts the angular velocity into feedforward joint-rate commands, and a gimbal load model converts the acceleration into feedforward torques."

Sec. I, last sentence: "... with gyroscope-weighted proposals and feedforward–feedback control ..." → "... with gyroscope-weighted proposals, kinematic and inertial feedforward, and feedback control ...".

**Table I, new rows**

| Signal | Meaning | Unit |
|---|---|---|
| $\mathbf a$ | Specific force at IMU A in the base frame (accelerometer reading) | m/s² |
| $\hat{\mathbf a}$ | Predicted specific force at $t + T_\tau$ | m/s² |
| $\boldsymbol\tau_{\mathrm{ff}}$ | Inertial feedforward joint torque | N·m |

**Table II, new rows**

| Parameter | Symbol | Unit |
|---|---|---|
| Torque-path delay | $T_\tau$ | s |
| IMU A lever arm | $\mathbf r_A$ | m |
| Link masses and centers of mass | $m_\ell$, $\mathbf r_{i\ell}$ | kg, m |
| Gravitational acceleration | $g$ | m/s² |

**Fig. 2.** Add the IMU A accelerometer $\mathbf a^{\mathrm{acc}}$ next to $\boldsymbol\omega^{\mathrm{gyr}}$, the history $\mathbf a_{k-H+1:k}$ in the context, and a second feedforward path: $\hat{\mathbf a}(t + T_\tau)$ → load model G(q) → $\boldsymbol\tau_{\mathrm{ff}}$ → motor driver.

**Sec. II-D / Fig. 3.** Add two identification steps: the gimbal mass properties (from CAD, or from static joint torques in several base tilts), and $T_\tau$. The IMU A calibration (accelerometer scale and bias, mounting rotation ${}^B R_A$, lever arm $\mathbf r_A$) comes before data collection, because the accelerometer is now ground truth for the residual targets.

**Sec. II-C, first paragraph.** "the prediction horizon must cover the actuation time" → "the prediction horizon must cover the delays of both the joint-rate path ($T_a$) and the torque path ($T_\tau$)".

