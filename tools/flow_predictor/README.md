# Flow-matching residual predictor (networks)

PyTorch models for the learned part of the disturbance predictor (paper Sec. II-B-2, eq. 7). They generate M candidate residual trajectories R̂_k ∈ ℝ^{F×3} conditioned on C_k = (u_k, ω_{k-H+1:k}). Training runs off the Pi; the predictor node runs the exported ONNX model.

| File | Content |
|---|---|
| `flow_matching.py` | `FlowMatching`: wraps a velocity network; training loss, Euler sampling, normalization, export wrapper |
| `unet.py` | `UNet1D`: 1D temporal U-Net over the F steps, FiLM conditioning |
| `dit.py` | `DiT`: transformer with one token per prediction step, adaLN-Zero conditioning |
| `layers.py` | `VelocityNet` interface, `pack_context`, time and context embeddings |
| `test_flow_matching.py` | Shapes, gradients, determinism, and a toy bimodal learning test |

## Use

```python
from flow_matching import FlowMatching
from layers import context_dim, pack_context
from unet import UNet1D          # or: from dit import DiT

net = UNet1D(horizon=F, dim=3, context_dim=context_dim(H))    # DiT(horizon=F, dim=3, context_dim=...)
fm = FlowMatching(net)                                         # sigma_min=0, tau_sampling="uniform"
fm.set_normalization(contexts, residuals)                      # training-set statistics, stored as buffers

# training: contexts (B, 2+3H) from pack_context(u, omega_hist); residuals (B, F, 3)
loss = fm(residuals, contexts)        # flow-matching loss, same as fm.loss(...)
loss.backward()

# inference: M samples, N Euler steps -> (B, M, F, 3) in rad/s
R_hat = fm.sample(contexts, num_samples=M, num_steps=N)
```

- **Loss** (`loss`, `forward`): linear path Z^τ = (1 − (1 − σ_min)τ) Z⁰ + τR, regressing v_η onto R − (1 − σ_min) Z⁰. `reduction="none"` gives the per-sample loss, e.g. for weighting.
- **Sampling** (`sample`): eq. (7) with Δτ = 1/N. The context is encoded once and shared by all M samples and N steps. `enable_grad=True` keeps the graph through the whole integration, so a loss on the generated trajectories can also be back-propagated. `return_path=True` returns every Z^{τ_n}.
- **Normalization:** C_k mixes m/s and rad/s, so contexts are normalized per feature and residuals per channel inside the module. Inputs and outputs are in physical units.
- **Export:** `fm.sampler(N)` is a module with `forward(context, noise)`, where noise is (B, M, F, 3). N is fixed and unrolled. It traces with `torch.jit.trace`; `torch.onnx.export` is not tested yet (`onnx` is not installed). Check during export whether M stays a dynamic axis, or export with the final M.

## Choices left open

H, F, M and N are TBD (ARCHITECTURE.md §5), so every class takes them as arguments. The test uses small placeholder values only. Network sizes are defaults, not tuned: `UNet1D` ≈ 2.4 M and `DiT` ≈ 1.4 M parameters at F = 10, H = 5. Each inference costs M·N network evaluations, so check the Pi 4 inference time (T3) before choosing them.

## Test

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tools/flow_predictor -p no:cacheprovider
```

Takes about 70 s on the laptop CPU, most of it in the two toy training runs.
