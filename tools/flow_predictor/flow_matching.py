"""Conditional flow matching around a velocity network (paper Sec. II-B-2, eq. 7).

Path (linear / optimal-transport, Lipman et al. 2023):

    Z^tau  = (1 - (1 - sigma_min) tau) Z^0 + tau R,     Z^0 ~ N(0, I)
    target = R - (1 - sigma_min) Z^0                    (= dZ^tau / dtau)

Training regresses v_eta(Z^tau, C, tau) onto the target (`loss`, differentiable).
Inference integrates eq. (7) with N Euler steps from M noise samples (`sample`).
`sample(..., enable_grad=True)` keeps the graph through all N steps, so a loss on
the generated trajectories can also be back-propagated through the integration.

Contexts and residuals are normalized inside this module (buffers saved with the
weights), because C_k mixes m/s and rad/s. Callers work in physical units.
"""
from typing import Optional

import torch
from torch import nn

from layers import VelocityNet


class FlowMatching(nn.Module):
    def __init__(self, net: VelocityNet, sigma_min: float = 0.0,
                 tau_sampling: str = "uniform"):
        """
        net:          velocity network (UNet1D or DiT)
        sigma_min:    width of the target distribution at tau = 1 (0 = rectified flow)
        tau_sampling: "uniform" or "logit_normal" (more weight on mid-path tau)
        """
        super().__init__()
        if tau_sampling not in ("uniform", "logit_normal"):
            raise ValueError(f"unknown tau_sampling {tau_sampling!r}")
        self.net = net
        self.sigma_min = sigma_min
        self.tau_sampling = tau_sampling
        self.register_buffer("ctx_mean", torch.zeros(net.context_dim))
        self.register_buffer("ctx_std", torch.ones(net.context_dim))
        self.register_buffer("res_mean", torch.zeros(net.dim))
        self.register_buffer("res_std", torch.ones(net.dim))

    @property
    def horizon(self) -> int:
        return self.net.horizon

    @property
    def dim(self) -> int:
        return self.net.dim

    # ---- normalization ------------------------------------------------------

    @torch.no_grad()
    def set_normalization(self, contexts: torch.Tensor, residuals: torch.Tensor,
                          eps: float = 1e-6) -> None:
        """Per-feature statistics of the training set.

        contexts:  (N_D, d_c)
        residuals: (N_D, F, d_r)  statistics per residual channel, shared over the horizon
        """
        self.ctx_mean.copy_(contexts.mean(0))
        self.ctx_std.copy_(contexts.std(0).clamp_min(eps))
        flat = residuals.reshape(-1, residuals.shape[-1])
        self.res_mean.copy_(flat.mean(0))
        self.res_std.copy_(flat.std(0).clamp_min(eps))

    def _norm_ctx(self, context: torch.Tensor) -> torch.Tensor:
        return (context - self.ctx_mean) / self.ctx_std

    def _norm_res(self, residuals: torch.Tensor) -> torch.Tensor:
        return (residuals - self.res_mean) / self.res_std

    def _denorm_res(self, z: torch.Tensor) -> torch.Tensor:
        return z * self.res_std + self.res_mean

    # ---- training -----------------------------------------------------------

    def _sample_tau(self, batch: int, device, dtype) -> torch.Tensor:
        if self.tau_sampling == "logit_normal":
            return torch.sigmoid(torch.randn(batch, device=device, dtype=dtype))
        return torch.rand(batch, device=device, dtype=dtype)

    def loss(self, residuals: torch.Tensor, context: torch.Tensor,
             tau: Optional[torch.Tensor] = None, noise: Optional[torch.Tensor] = None,
             reduction: str = "mean") -> torch.Tensor:
        """Flow-matching loss ||v_eta(Z^tau, C, tau) - (R - (1 - sigma_min) Z^0)||^2.

        residuals: (B, F, d_r) true residual trajectories R, physical units
        context:   (B, d_c)
        tau, noise: optional fixed values (B,), (B, F, d_r) for reproducibility
        reduction: "mean" -> scalar, "none" -> per-sample loss (B,)
        """
        x1 = self._norm_res(residuals)
        b = x1.shape[0]
        x0 = torch.randn_like(x1) if noise is None else noise
        if tau is None:
            tau = self._sample_tau(b, x1.device, x1.dtype)
        t = tau.view(b, 1, 1)
        z = (1 - (1 - self.sigma_min) * t) * x0 + t * x1
        target = x1 - (1 - self.sigma_min) * x0

        v = self.net(z, tau, self.net.encode_context(self._norm_ctx(context)))
        per_sample = (v - target).pow(2).mean(dim=(1, 2))
        if reduction == "none":
            return per_sample
        if reduction == "mean":
            return per_sample.mean()
        raise ValueError(f"unknown reduction {reduction!r}")

    def forward(self, residuals: torch.Tensor, context: torch.Tensor) -> torch.Tensor:
        """Training forward pass: returns the flow-matching loss (call .backward() on it)."""
        return self.loss(residuals, context)

    # ---- inference ----------------------------------------------------------

    def sample(self, context: torch.Tensor, num_samples: int, num_steps: int,
               noise: Optional[torch.Tensor] = None, enable_grad: bool = False,
               return_path: bool = False):
        """Draw M residual trajectories per context with N Euler steps (eq. 7).

        context:     (B, d_c), physical units
        num_samples: M
        num_steps:   N
        noise:       optional Z^0, (B, M, F, d_r) in normalized units
        enable_grad: keep the autograd graph through the integration
        return_path: also return Z^{tau_n} for n = 0..N, (N + 1, B, M, F, d_r), denormalized

        returns R_hat: (B, M, F, d_r), physical units
        """
        b, m, f, d = context.shape[0], num_samples, self.horizon, self.dim
        with torch.set_grad_enabled(enable_grad):
            cond = self.net.encode_context(self._norm_ctx(context))
            cond = cond.repeat_interleave(m, dim=0)                       # (B*M, cond)
            if noise is None:
                z = torch.randn(b * m, f, d, device=context.device, dtype=context.dtype)
            else:
                z = noise.reshape(b * m, f, d)
            path = [z] if return_path else None
            dtau = 1.0 / num_steps
            for n in range(num_steps):
                tau = torch.full((b * m,), n * dtau, device=z.device, dtype=z.dtype)
                z = z + dtau * self.net(z, tau, cond)
                if return_path:
                    path.append(z)
            out = self._denorm_res(z).reshape(b, m, f, d)
            if return_path:
                return out, self._denorm_res(torch.stack(path)).reshape(num_steps + 1, b, m, f, d)
            return out

    def sampler(self, num_steps: int) -> nn.Module:
        """Module with forward(context, noise) -> R_hat and a fixed N, for ONNX export.

        noise (B, M, F, d_r) is an input, so M is set by the caller at run time and
        the predictor node controls the random numbers.
        """
        return _Sampler(self, num_steps)


class _Sampler(nn.Module):
    def __init__(self, fm: FlowMatching, num_steps: int):
        super().__init__()
        self.fm, self.num_steps = fm, num_steps

    def forward(self, context: torch.Tensor, noise: torch.Tensor) -> torch.Tensor:
        return self.fm.sample(context, noise.shape[1], self.num_steps, noise=noise)
