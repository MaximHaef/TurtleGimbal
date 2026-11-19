"""1D temporal U-Net velocity network (convolutions over the F prediction steps).

The d_r residual channels are the conv channels, the horizon F is the length axis.
Conditioning (context + flow time) enters every residual block through FiLM
(per-channel scale and shift), as in temporal U-Nets for trajectory diffusion.
F does not have to be a power of two: the input is zero-padded to a multiple of
2^(levels-1) and the output cropped back.
"""
import torch
import torch.nn.functional as Fn
from torch import nn

from layers import ContextEncoder, TimeEmbedding, VelocityNet


def _norm(channels: int, groups: int) -> nn.GroupNorm:
    return nn.GroupNorm(min(groups, channels), channels)


class ResBlock1D(nn.Module):
    def __init__(self, in_ch: int, out_ch: int, cond_dim: int, kernel_size: int,
                 groups: int, dropout: float):
        super().__init__()
        pad = kernel_size // 2
        self.norm1 = _norm(in_ch, groups)
        self.conv1 = nn.Conv1d(in_ch, out_ch, kernel_size, padding=pad)
        self.film = nn.Linear(cond_dim, 2 * out_ch)
        self.norm2 = _norm(out_ch, groups)
        self.drop = nn.Dropout(dropout)
        self.conv2 = nn.Conv1d(out_ch, out_ch, kernel_size, padding=pad)
        self.skip = nn.Conv1d(in_ch, out_ch, 1) if in_ch != out_ch else nn.Identity()
        nn.init.zeros_(self.film.weight)
        nn.init.zeros_(self.film.bias)

    def forward(self, x: torch.Tensor, c: torch.Tensor) -> torch.Tensor:
        h = self.conv1(Fn.silu(self.norm1(x)))
        scale, shift = self.film(c).unsqueeze(-1).chunk(2, dim=1)
        h = self.norm2(h) * (1 + scale) + shift
        h = self.conv2(self.drop(Fn.silu(h)))
        return h + self.skip(x)


class UNet1D(VelocityNet):
    """
    horizon:       F
    dim:           d_r (3)
    context_dim:   d_c (2 + 3H, see layers.pack_context)
    base_channels: channels at the finest level
    channel_mults: one entry per level; len(channel_mults) - 1 downsamplings
    """

    def __init__(self, horizon: int, dim: int, context_dim: int, base_channels: int = 32,
                 channel_mults=(1, 2, 4), blocks_per_level: int = 2, cond_dim: int = 256,
                 kernel_size: int = 5, groups: int = 8, dropout: float = 0.0,
                 context_hidden: int = 256):
        super().__init__()
        self.horizon, self.dim, self.context_dim = horizon, dim, context_dim
        self.factor = 2 ** (len(channel_mults) - 1)

        self.context_encoder = ContextEncoder(context_dim, cond_dim, context_hidden)
        self.time_embedding = TimeEmbedding(cond_dim)

        block = lambda i, o: ResBlock1D(i, o, cond_dim, kernel_size, groups, dropout)
        chans = [base_channels * m for m in channel_mults]
        self.inp = nn.Conv1d(dim, chans[0], kernel_size, padding=kernel_size // 2)

        self.down = nn.ModuleList()
        skip_chans, ch = [], chans[0]
        for lvl, out in enumerate(chans):
            blocks = nn.ModuleList()
            for _ in range(blocks_per_level):
                blocks.append(block(ch, out))
                ch = out
                skip_chans.append(ch)
            last = lvl == len(chans) - 1
            down = nn.Identity() if last else nn.Conv1d(ch, ch, 3, stride=2, padding=1)
            self.down.append(nn.ModuleDict({"blocks": blocks, "down": down}))

        self.mid = nn.ModuleList([block(ch, ch), block(ch, ch)])

        self.up = nn.ModuleList()
        for lvl, out in reversed(list(enumerate(chans))):
            blocks = nn.ModuleList()
            for _ in range(blocks_per_level):
                blocks.append(block(ch + skip_chans.pop(), out))
                ch = out
            first = lvl == 0
            up = nn.Identity() if first else nn.ConvTranspose1d(ch, ch, 4, stride=2, padding=1)
            self.up.append(nn.ModuleDict({"blocks": blocks, "up": up}))

        self.out = nn.Sequential(_norm(ch, groups), nn.SiLU(),
                                 nn.Conv1d(ch, dim, kernel_size, padding=kernel_size // 2))
        nn.init.zeros_(self.out[-1].weight)
        nn.init.zeros_(self.out[-1].bias)

    def encode_context(self, context: torch.Tensor) -> torch.Tensor:
        return self.context_encoder(context)

    def forward(self, z: torch.Tensor, tau: torch.Tensor, cond: torch.Tensor) -> torch.Tensor:
        c = cond + self.time_embedding(tau)
        x = z.transpose(1, 2)                                   # (B, d_r, F)
        pad = (-self.horizon) % self.factor
        if pad:
            x = Fn.pad(x, (0, pad))

        h = self.inp(x)
        skips = []
        for level in self.down:
            for blk in level["blocks"]:
                h = blk(h, c)
                skips.append(h)
            h = level["down"](h)
        for blk in self.mid:
            h = blk(h, c)
        for level in self.up:
            for blk in level["blocks"]:
                h = blk(torch.cat([h, skips.pop()], dim=1), c)
            h = level["up"](h)

        return self.out(h)[..., :self.horizon].transpose(1, 2)  # (B, F, d_r)
