"""Diffusion Transformer (DiT) velocity network.

Each of the F prediction steps is one token (d_r values -> hidden). A learned
positional embedding marks the step index. Conditioning (context + flow time)
modulates every block through adaLN-Zero (Peebles & Xie, 2023): the block's
LayerNorm shift/scale and the residual gates are regressed from the condition,
with the gates initialised to zero so each block starts as the identity.
"""
import torch
from torch import nn

from layers import ContextEncoder, TimeEmbedding, VelocityNet


def _modulate(x: torch.Tensor, shift: torch.Tensor, scale: torch.Tensor) -> torch.Tensor:
    return x * (1 + scale.unsqueeze(1)) + shift.unsqueeze(1)


class DiTBlock(nn.Module):
    def __init__(self, hidden: int, heads: int, mlp_ratio: float, dropout: float):
        super().__init__()
        self.norm1 = nn.LayerNorm(hidden, elementwise_affine=False, eps=1e-6)
        self.attn = nn.MultiheadAttention(hidden, heads, dropout=dropout, batch_first=True)
        self.norm2 = nn.LayerNorm(hidden, elementwise_affine=False, eps=1e-6)
        mlp_hidden = int(hidden * mlp_ratio)
        self.mlp = nn.Sequential(nn.Linear(hidden, mlp_hidden), nn.GELU(approximate="tanh"),
                                 nn.Dropout(dropout), nn.Linear(mlp_hidden, hidden))
        self.ada = nn.Sequential(nn.SiLU(), nn.Linear(hidden, 6 * hidden))
        nn.init.zeros_(self.ada[-1].weight)
        nn.init.zeros_(self.ada[-1].bias)

    def forward(self, x: torch.Tensor, c: torch.Tensor) -> torch.Tensor:
        sh1, sc1, g1, sh2, sc2, g2 = self.ada(c).chunk(6, dim=-1)
        h = _modulate(self.norm1(x), sh1, sc1)
        x = x + g1.unsqueeze(1) * self.attn(h, h, h, need_weights=False)[0]
        h = _modulate(self.norm2(x), sh2, sc2)
        return x + g2.unsqueeze(1) * self.mlp(h)


class DiT(VelocityNet):
    """
    horizon:     F (number of tokens)
    dim:         d_r (3)
    context_dim: d_c (2 + 3H, see layers.pack_context)
    """

    def __init__(self, horizon: int, dim: int, context_dim: int, hidden: int = 128,
                 depth: int = 4, heads: int = 4, mlp_ratio: float = 4.0,
                 dropout: float = 0.0, context_hidden: int = 256):
        super().__init__()
        self.horizon, self.dim, self.context_dim = horizon, dim, context_dim

        self.context_encoder = ContextEncoder(context_dim, hidden, context_hidden)
        self.time_embedding = TimeEmbedding(hidden)

        self.token = nn.Linear(dim, hidden)
        self.pos = nn.Parameter(torch.randn(1, horizon, hidden) * 0.02)
        self.blocks = nn.ModuleList(DiTBlock(hidden, heads, mlp_ratio, dropout)
                                    for _ in range(depth))

        self.final_norm = nn.LayerNorm(hidden, elementwise_affine=False, eps=1e-6)
        self.final_ada = nn.Sequential(nn.SiLU(), nn.Linear(hidden, 2 * hidden))
        self.final = nn.Linear(hidden, dim)
        for lin in (self.final_ada[-1], self.final):
            nn.init.zeros_(lin.weight)
            nn.init.zeros_(lin.bias)

    def encode_context(self, context: torch.Tensor) -> torch.Tensor:
        return self.context_encoder(context)

    def forward(self, z: torch.Tensor, tau: torch.Tensor, cond: torch.Tensor) -> torch.Tensor:
        c = cond + self.time_embedding(tau)
        x = self.token(z) + self.pos
        for blk in self.blocks:
            x = blk(x, c)
        shift, scale = self.final_ada(c).chunk(2, dim=-1)
        return self.final(_modulate(self.final_norm(x), shift, scale))   # (B, F, d_r)
