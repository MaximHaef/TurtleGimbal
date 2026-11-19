"""Shape, gradient and learning tests for the flow-matching predictor networks.

    python3 -m pytest tools/flow_predictor -p no:cacheprovider
"""
import pytest
import torch

from dit import DiT
from flow_matching import FlowMatching
from layers import context_dim, pack_context
from unet import UNet1D

H, D_R = 5, 3      # test values only; H, F, M, N for the robot are TBD (ARCHITECTURE.md §5)


def make_net(kind, horizon, small=True):
    dc = context_dim(H)
    if kind == "unet":
        return UNet1D(horizon, D_R, dc, base_channels=16 if small else 32,
                      channel_mults=(1, 2), blocks_per_level=1, cond_dim=64, context_hidden=64)
    return DiT(horizon, D_R, dc, hidden=32 if small else 64, depth=2, heads=2, context_hidden=64)


def random_context(b):
    return pack_context(torch.randn(b, 2), torch.randn(b, H, 3))


@pytest.mark.parametrize("kind", ["unet", "dit"])
@pytest.mark.parametrize("horizon", [1, 7, 16])
def test_shapes(kind, horizon):
    fm = FlowMatching(make_net(kind, horizon))
    ctx = random_context(4)
    v = fm.net(torch.randn(4, horizon, D_R), torch.rand(4), fm.net.encode_context(ctx))
    assert v.shape == (4, horizon, D_R)
    out, path = fm.sample(ctx, num_samples=6, num_steps=3, return_path=True)
    assert out.shape == (4, 6, horizon, D_R)
    assert path.shape == (4, 4, 6, horizon, D_R)
    assert torch.allclose(path[-1], out)


@pytest.mark.parametrize("kind", ["unet", "dit"])
def test_loss_backprop(kind):
    torch.manual_seed(0)
    fm = FlowMatching(make_net(kind, 8))
    loss = fm(torch.randn(5, 8, D_R), random_context(5))
    assert loss.ndim == 0
    loss.backward()
    grads = [p.grad for p in fm.parameters() if p.requires_grad]
    assert all(g is not None for g in grads)
    assert any(g.abs().sum() > 0 for g in grads)


@pytest.mark.parametrize("kind", ["unet", "dit"])
def test_sample_grad_flag(kind):
    torch.manual_seed(0)
    fm = FlowMatching(make_net(kind, 8))
    fm(torch.randn(5, 8, D_R), random_context(5)).backward()   # make output layers non-zero
    with torch.no_grad():
        for p in fm.parameters():
            p.add_(0.05 * torch.randn_like(p))
    fm.zero_grad()
    ctx = random_context(2)
    assert not fm.sample(ctx, 3, 4).requires_grad
    out = fm.sample(ctx, 3, 4, enable_grad=True)
    out.pow(2).mean().backward()
    assert any(p.grad is not None and p.grad.abs().sum() > 0 for p in fm.parameters())


def test_fixed_noise_is_deterministic_and_one_step_is_euler():
    torch.manual_seed(0)
    fm = FlowMatching(make_net("dit", 6)).eval()
    with torch.no_grad():
        for p in fm.parameters():
            p.add_(0.05 * torch.randn_like(p))
    ctx, noise = random_context(2), torch.randn(2, 3, 6, D_R)
    a = fm.sample(ctx, 3, 5, noise=noise)
    b = fm.sample(ctx, 3, 5, noise=noise)
    assert torch.equal(a, b)
    one = fm.sample(ctx, 3, 1, noise=noise)
    cond = fm.net.encode_context(ctx).repeat_interleave(3, 0)
    z0 = noise.reshape(6, 6, D_R)
    with torch.no_grad():
        expect = z0 + fm.net(z0, torch.zeros(6), cond)
    assert torch.allclose(one, expect.reshape(2, 3, 6, D_R), atol=1e-6)


def test_normalization_roundtrip():
    fm = FlowMatching(make_net("unet", 4))
    ctx = random_context(100) * 3 + 1
    res = torch.randn(100, 4, D_R) * torch.tensor([0.1, 2.0, 5.0]) + 0.5
    fm.set_normalization(ctx, res)
    assert torch.allclose(fm._denorm_res(fm._norm_res(res)), res, atol=1e-5)
    n = fm._norm_res(res).reshape(-1, D_R)
    assert torch.allclose(n.mean(0), torch.zeros(D_R), atol=1e-4)
    assert torch.allclose(n.std(0), torch.ones(D_R), atol=1e-4)


@pytest.mark.parametrize("kind", ["unet", "dit"])
def test_learns_context_dependent_bimodal_residuals(kind):
    """Context sign s picks the mean; each sample is +-1 offset (two modes)."""
    torch.manual_seed(0)
    f = 8
    ramp = torch.linspace(0, 1, f).view(1, f, 1)

    def data(b):
        ctx = random_context(b)
        s = torch.sign(ctx[:, :1]).view(b, 1, 1)
        mode = torch.randint(0, 2, (b, 1, 1)).float() * 2 - 1
        return 2 * s * ramp + mode + 0.05 * torch.randn(b, f, D_R), ctx

    fm = FlowMatching(make_net(kind, f, small=False))
    fm.set_normalization(*reversed(data(2000)))
    opt = torch.optim.AdamW(fm.parameters(), lr=2e-3)
    first = None
    for step in range(600):
        res, ctx = data(256)
        loss = fm(res, ctx)
        opt.zero_grad()
        loss.backward()
        opt.step()
        first = loss.item() if first is None else first
    assert loss.item() < 0.6 * first

    fm.eval()
    ctx = torch.zeros(2, context_dim(H))
    ctx[0, 0], ctx[1, 0] = 1.0, -1.0
    with torch.no_grad():
        out = fm.sample(ctx, num_samples=400, num_steps=20)
    for i, s in enumerate((1.0, -1.0)):
        offset = (out[i] - 2 * s * ramp[0]).mean(dim=(1, 2))      # per sample, ~ +-1
        assert (offset > 0.5).float().mean() > 0.2     # both modes present
        assert (offset < -0.5).float().mean() > 0.2
        assert ((offset.abs() - 1).abs() < 0.4).float().mean() > 0.8
