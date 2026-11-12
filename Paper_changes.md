# Changes needed in Bravo_Team

The implementation now uses the robot's body-velocity command as the model input instead of wheel angular-speed commands. The reason is the Kobuki (TurtleBot 2) base: its driver takes `(v, ω_z)` on `/commands/velocity`, and its serial protocol has no per-wheel command. The firmware drives the wheels from a (speed, radius) pair. Section and equation numbers below refer to [Bravo_Team.md](Bravo_Team.md) / `../Bravo_Team.pdf`.

The method itself does not change. Eqs. (3), (4) and (6)–(10) keep their form. Only the input u and eq. (5) change, plus the wording that refers to them.

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
