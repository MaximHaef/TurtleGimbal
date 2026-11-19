"""Building blocks shared by the velocity networks.

Every velocity network v_eta (paper eq. 7) implements the `VelocityNet` interface:

    cond = net.encode_context(context)      # once per inference, shape (B, cond_dim)
    v    = net(z, tau, cond)                # once per Euler step

so the context C_k is encoded once and reused for all N flow steps and M samples.

Shapes:
    z        (B, F, d_r)     residual trajectory sample Z^tau
    tau      (B,)            flow time in [0, 1]
    context  (B, d_c)        flat context, see `pack_context`
"""
import math

import torch
from torch import nn


def pack_context(u: torch.Tensor, omega_hist: torch.Tensor) -> torch.Tensor:
    """Flatten C_k = (u_k, omega_{k-H+1:k}) into one vector.

    u:          (B, 2)     (v_cmd, omega_z_cmd) from /base_command
    omega_hist: (B, H, 3)  last H IMU A samples, oldest first
    returns     (B, 2 + 3H)
    """
    return torch.cat([u, omega_hist.flatten(1)], dim=1)


def context_dim(history_len: int, u_dim: int = 2, omega_dim: int = 3) -> int:
    return u_dim + history_len * omega_dim


def timestep_embedding(tau: torch.Tensor, dim: int, scale: float = 1000.0,
                       max_period: float = 10000.0) -> torch.Tensor:
    """Sinusoidal embedding of the flow time tau in [0, 1] -> (B, dim)."""
    half = dim // 2
    freqs = torch.exp(-math.log(max_period)
                      * torch.arange(half, device=tau.device, dtype=torch.float32) / half)
    args = (scale * tau.float())[:, None] * freqs[None]
    emb = torch.cat([torch.cos(args), torch.sin(args)], dim=-1)
    if dim % 2:
        emb = torch.cat([emb, torch.zeros_like(emb[:, :1])], dim=-1)
    return emb.to(tau.dtype)


class TimeEmbedding(nn.Module):
    """tau -> sinusoidal features -> MLP -> (B, out_dim)."""

    def __init__(self, out_dim: int, freq_dim: int = 128):
        super().__init__()
        self.freq_dim = freq_dim
        self.mlp = nn.Sequential(nn.Linear(freq_dim, out_dim), nn.SiLU(),
                                 nn.Linear(out_dim, out_dim))

    def forward(self, tau: torch.Tensor) -> torch.Tensor:
        return self.mlp(timestep_embedding(tau, self.freq_dim))


class ContextEncoder(nn.Module):
    """MLP on the flat (normalized) context -> (B, out_dim)."""

    def __init__(self, in_dim: int, out_dim: int, hidden_dim: int = 256,
                 num_layers: int = 2, dropout: float = 0.0):
        super().__init__()
        layers, d = [], in_dim
        for _ in range(num_layers):
            layers += [nn.Linear(d, hidden_dim), nn.SiLU(), nn.Dropout(dropout)]
            d = hidden_dim
        layers.append(nn.Linear(d, out_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, context: torch.Tensor) -> torch.Tensor:
        return self.net(context)


class VelocityNet(nn.Module):
    """Interface for v_eta(Z^tau, C_k, tau). Subclasses set the attributes below."""

    horizon: int        # F
    dim: int            # d_r
    context_dim: int    # d_c

    def encode_context(self, context: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError

    def forward(self, z: torch.Tensor, tau: torch.Tensor, cond: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError
