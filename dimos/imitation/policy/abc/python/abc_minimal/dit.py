"""ABC-DiT policy implementation for the released bottles-in-bin checkpoints.

Includes the CLIP text encoder and DINOv3 vision backbone needed to run the model.
"""

import gzip
import html
import math
import urllib.request
from functools import lru_cache
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from abc_minimal.config import ClipConfig, DiTConfig

# CLIP ViT-B/32 text encoder.

SOT_TOKEN = "<|startoftext|>"
EOT_TOKEN = "<|endoftext|>"


def task_name_to_prompt(task_name):
    """Convert production task names like open_the_pen_caps to CLIP prompt text."""
    return " ".join(task_name.replace("-", " ").replace("_", " ").split())


def _load_clip_text_deps():
    try:
        import ftfy
        import regex
    except ImportError as exc:
        raise RuntimeError(
            "CLIP text embedding requires the 'ftfy' and 'regex' packages. "
            "Run `uv sync` after pulling this version, or install them manually."
        ) from exc
    return ftfy, regex


def _download_if_missing(url, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        urllib.request.urlretrieve(url, path)


def ensure_clip_text_assets(config: ClipConfig):
    """Download CLIP ViT-B/32 text assets if needed."""
    root = Path(config.cache_dir).expanduser()
    b32_path = root / config.model_name
    bpe_path = root / config.bpe_name
    _download_if_missing(config.model_url, b32_path)
    _download_if_missing(config.bpe_url, bpe_path)
    return b32_path, bpe_path


@lru_cache()
def _bytes_to_unicode():
    bs = (
        list(range(ord("!"), ord("~") + 1))
        + list(range(ord("¡"), ord("¬") + 1))
        + list(range(ord("®"), ord("ÿ") + 1))
    )
    cs = bs[:]
    n = 0
    for b in range(256):
        if b not in bs:
            bs.append(b)
            cs.append(256 + n)
            n += 1
    return dict(zip(bs, [chr(c) for c in cs]))


def _get_pairs(word):
    return set(zip(word, word[1:]))


class CLIPBPETokenizer:
    """Small copy of OpenAI CLIP's BPE tokenizer, scoped to text encoding."""

    def __init__(self, bpe_path):
        _ftfy, regex = _load_clip_text_deps()
        self.ftfy = _ftfy
        self.regex = regex
        with gzip.open(bpe_path, "rt", encoding="utf-8") as f:
            merges = [tuple(l.split()) for l in f.read().split("\n")[1 : 49152 - 256 - 2 + 1]]
        self.byte_encoder = _bytes_to_unicode()
        vocab = list(self.byte_encoder.values())
        vocab = vocab + [v + "</w>" for v in vocab]
        for merge in merges:
            vocab.append("".join(merge))
        vocab.extend([SOT_TOKEN, EOT_TOKEN])
        self.encoder = {v: i for i, v in enumerate(vocab)}
        self.bpe_ranks = dict(zip(merges, range(len(merges))))
        self.cache = {SOT_TOKEN: SOT_TOKEN, EOT_TOKEN: EOT_TOKEN}
        self.pat = regex.compile(
            r"<\|startoftext\|>|<\|endoftext\|>|\'s|\'t|\'re|\'ve|\'m|\'ll|\'d|"
            r"[\p{L}]+|[\p{N}]|[^\s\p{L}\p{N}]+",
            regex.IGNORECASE,
        )

    def _basic_clean(self, text):
        return html.unescape(html.unescape(self.ftfy.fix_text(text))).strip()

    def _whitespace_clean(self, text):
        return self.regex.sub(r"\s+", " ", text).strip()

    def bpe(self, token):
        if token in self.cache:
            return self.cache[token]
        word = tuple(token[:-1]) + (token[-1] + "</w>",)
        pairs = _get_pairs(word)
        if not pairs:
            return token + "</w>"
        while True:
            bigram = min(pairs, key=lambda p: self.bpe_ranks.get(p, float("inf")))
            if bigram not in self.bpe_ranks:
                break
            first, second = bigram
            new_word = []
            i = 0
            while i < len(word):
                try:
                    j = word.index(first, i)
                except ValueError:
                    new_word.extend(word[i:])
                    break
                new_word.extend(word[i:j])
                if word[j] == first and j < len(word) - 1 and word[j + 1] == second:
                    new_word.append(first + second)
                    i = j + 2
                else:
                    new_word.append(word[j])
                    i = j + 1
            word = tuple(new_word)
            if len(word) == 1:
                break
            pairs = _get_pairs(word)
        out = " ".join(word)
        self.cache[token] = out
        return out

    def encode(self, text):
        text = self._whitespace_clean(self._basic_clean(text)).lower()
        tokens = []
        for token in self.regex.findall(self.pat, text):
            token = "".join(self.byte_encoder[b] for b in token.encode("utf-8"))
            tokens.extend(self.encoder[piece] for piece in self.bpe(token).split(" "))
        return tokens


class CLIPQuickGELU(nn.Module):
    def forward(self, x):
        return x * torch.sigmoid(1.702 * x)


class CLIPTextBlock(nn.Module):
    def __init__(self, width, heads, mask):
        super().__init__()
        self.attn = nn.MultiheadAttention(width, heads)
        self.ln_1 = nn.LayerNorm(width)
        self.mlp = nn.Sequential()
        self.mlp.add_module("c_fc", nn.Linear(width, width * 4))
        self.mlp.add_module("gelu", CLIPQuickGELU())
        self.mlp.add_module("c_proj", nn.Linear(width * 4, width))
        self.ln_2 = nn.LayerNorm(width)
        self.register_buffer("mask", mask, persistent=False)

    def forward(self, x):
        x_ln = self.ln_1(x)
        x = x + self.attn(x_ln, x_ln, x_ln, need_weights=False, attn_mask=self.mask)[0]
        x = x + self.mlp(self.ln_2(x))
        return x


class CLIPTextTower(nn.Module):
    def __init__(self, state_dict):
        super().__init__()
        embed_dim = state_dict["text_projection"].shape[1]
        context_length = state_dict["positional_embedding"].shape[0]
        vocab_size = state_dict["token_embedding.weight"].shape[0]
        width = state_dict["ln_final.weight"].shape[0]
        heads = width // 64
        layers = len(
            {k.split(".")[2] for k in state_dict if k.startswith("transformer.resblocks")}
        )
        mask = torch.empty(context_length, context_length).fill_(float("-inf")).triu_(1)

        self.context_length = context_length
        self.token_embedding = nn.Embedding(vocab_size, width)
        self.positional_embedding = nn.Parameter(torch.empty(context_length, width))
        self.transformer = nn.Module()
        self.transformer.resblocks = nn.Sequential(
            *[CLIPTextBlock(width, heads, mask) for _ in range(layers)]
        )
        self.ln_final = nn.LayerNorm(width)
        self.text_projection = nn.Parameter(torch.empty(width, embed_dim))

    def forward(self, text):
        x = self.token_embedding(text) + self.positional_embedding
        x = x.permute(1, 0, 2)
        x = self.transformer.resblocks(x)
        x = x.permute(1, 0, 2)
        x = self.ln_final(x)
        return x[torch.arange(x.shape[0], device=x.device), text.argmax(dim=-1)] @ self.text_projection


class CLIPTextEmbedder:
    """OpenAI CLIP ViT-B/32 text encoder that returns normalized 512-d vectors.
    Holds a CPU memo cache keyed by prompt so repeats skip BPE+transformer."""

    def __init__(self, config: ClipConfig, device="cpu"):
        b32_path, bpe_path = ensure_clip_text_assets(config)
        self.device = torch.device(device)
        try:
            state_dict = torch.jit.load(str(b32_path), map_location="cpu").state_dict()
        except RuntimeError:
            state_dict = torch.load(b32_path, map_location="cpu", weights_only=False)
        self.tokenizer = CLIPBPETokenizer(bpe_path)
        self.model = CLIPTextTower(state_dict).eval().to(self.device)
        text_keys = {
            k: v
            for k, v in state_dict.items()
            if k.startswith(
                (
                    "token_embedding",
                    "positional_embedding",
                    "transformer.resblocks",
                    "ln_final",
                    "text_projection",
                )
            )
        }
        missing, unexpected = self.model.load_state_dict(text_keys, strict=False)
        if missing or unexpected:
            raise RuntimeError(f"bad CLIP text weights: missing={missing} unexpected={unexpected}")
        self._cache = {}

    @torch.no_grad()
    def encode(self, texts):
        if isinstance(texts, str):
            texts = [texts]
        fresh = [t for t in dict.fromkeys(texts) if t not in self._cache]
        if fresh:
            context = torch.zeros(
                len(fresh), self.model.context_length, dtype=torch.long, device=self.device
            )
            for i, text in enumerate(fresh):
                token_ids = [
                    self.tokenizer.encoder[SOT_TOKEN],
                    *self.tokenizer.encode(text),
                    self.tokenizer.encoder[EOT_TOKEN],
                ]
                if len(token_ids) > self.model.context_length:
                    raise RuntimeError(
                        f"Input {text!r} is too long for CLIP context length "
                        f"{self.model.context_length}"
                    )
                context[i, : len(token_ids)] = torch.tensor(
                    token_ids, dtype=torch.long, device=self.device
                )
            features = self.model(context)
            features = features / features.norm(dim=-1, keepdim=True)
            for i, text in enumerate(fresh):
                self._cache[text] = features[i].cpu()
        out = torch.stack([self._cache[t] for t in texts], dim=0)
        return out.to(self.device)


def encode_clip_text(texts, config: ClipConfig, device="cpu"):
    """Encode exact prompt text with OpenAI CLIP ViT-B/32."""
    return CLIPTextEmbedder(config, device=device).encode(texts)


def encode_clip_task_name(task_names, config: ClipConfig, device="cpu"):
    """Encode task names after production-style dash/underscore replacement."""
    if isinstance(task_names, str):
        task_names = [task_names]
    return encode_clip_text([task_name_to_prompt(t) for t in task_names], config, device=device)


# DINOv3 ViT-B/16 vision encoder.


def _rope_rotate_half(x):
    x1, x2 = x.chunk(2, dim=-1)
    return torch.cat([-x2, x1], dim=-1)


class DinoRope(nn.Module):
    """RoPE over the 2D patch grid (base=100, separate coord normalization).

    rescale_coords=2 applies a random log-uniform rescale of the coordinates
    during training only — part of the pretraining distribution, kept for
    finetuning fidelity.
    """

    def __init__(self, embed_dim, num_heads, base=100.0, rescale_coords=2.0):
        super().__init__()
        d_head = embed_dim // num_heads
        self.d_head = d_head
        self.rescale_coords = rescale_coords
        self.register_buffer("periods", torch.empty(d_head // 4), persistent=True)
        with torch.no_grad():
            self.periods.copy_(
                base ** (2 * torch.arange(d_head // 4, dtype=torch.float32) / (d_head // 2))
            )

    def forward(self, H, W):
        dev = self.periods.device
        coords_h = torch.arange(0.5, H, device=dev, dtype=torch.float32) / H
        coords_w = torch.arange(0.5, W, device=dev, dtype=torch.float32) / W
        coords = torch.stack(torch.meshgrid(coords_h, coords_w, indexing="ij"), dim=-1)
        coords = coords.flatten(0, 1)
        coords = 2.0 * coords - 1.0
        if self.training and self.rescale_coords is not None:
            r = np.log(self.rescale_coords)
            rescale = torch.empty(1, device=dev).uniform_(-r, r).exp()
            coords = coords * rescale
        angles = 2 * math.pi * coords[:, :, None] / self.periods[None, None, :]
        angles = angles.flatten(1, 2).tile(2)
        return torch.sin(angles), torch.cos(angles)


class LinearKMaskedBias(nn.Linear):
    """qkv Linear whose k-third of the bias is masked to zero (DINOv3 quirk)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.register_buffer("bias_mask", torch.full_like(self.bias, math.nan))

    def forward(self, x):
        return F.linear(x, self.weight, self.bias * self.bias_mask.to(self.bias.dtype))


class DinoAttention(nn.Module):
    def __init__(self, dim, num_heads):
        super().__init__()
        self.num_heads = num_heads
        self.qkv = LinearKMaskedBias(dim, dim * 3, bias=True)
        self.proj = nn.Linear(dim, dim, bias=True)

    def forward(self, x, rope=None):
        B, N, C = x.shape
        qkv = self.qkv(x).reshape(B, N, 3, self.num_heads, C // self.num_heads)
        q, k, v = [t.transpose(1, 2) for t in torch.unbind(qkv, 2)]
        if rope is not None:
            sin, cos = rope
            n_prefix = N - sin.shape[-2]  # cls + storage tokens are not rotated
            q_dt, k_dt = q.dtype, k.dtype
            q, k = q.to(sin.dtype), k.to(sin.dtype)
            q = torch.cat(
                [q[:, :, :n_prefix], q[:, :, n_prefix:] * cos + _rope_rotate_half(q[:, :, n_prefix:]) * sin],
                dim=-2,
            )
            k = torch.cat(
                [k[:, :, :n_prefix], k[:, :, n_prefix:] * cos + _rope_rotate_half(k[:, :, n_prefix:]) * sin],
                dim=-2,
            )
            q, k = q.to(q_dt), k.to(k_dt)
        x = F.scaled_dot_product_attention(q, k, v)
        return self.proj(x.transpose(1, 2).reshape(B, N, C))


class LayerScale(nn.Module):
    def __init__(self, dim, init_values=1e-5):
        super().__init__()
        self.gamma = nn.Parameter(init_values * torch.ones(dim))

    def forward(self, x):
        return x * self.gamma


class DinoMlp(nn.Module):
    def __init__(self, dim, hidden):
        super().__init__()
        self.fc1 = nn.Linear(dim, hidden)
        self.act = nn.GELU()
        self.fc2 = nn.Linear(hidden, dim)

    def forward(self, x):
        return self.fc2(self.act(self.fc1(x)))


class DinoBlock(nn.Module):
    def __init__(self, dim, num_heads, ffn_ratio=4.0):
        super().__init__()
        self.norm1 = nn.LayerNorm(dim, eps=1e-5)
        self.attn = DinoAttention(dim, num_heads)
        self.ls1 = LayerScale(dim)
        self.norm2 = nn.LayerNorm(dim, eps=1e-5)
        self.mlp = DinoMlp(dim, int(dim * ffn_ratio))
        self.ls2 = LayerScale(dim)

    def forward(self, x, rope=None):
        x = x + self.ls1(self.attn(self.norm1(x), rope=rope))
        x = x + self.ls2(self.mlp(self.norm2(x)))
        return x


class DinoPatchEmbed(nn.Module):
    def __init__(self, patch_size=16, in_chans=3, embed_dim=768):
        super().__init__()
        self.proj = nn.Conv2d(in_chans, embed_dim, kernel_size=patch_size, stride=patch_size)

    def forward(self, x):
        x = self.proj(x)  # (B, D, H/16, W/16)
        return x.flatten(2).transpose(1, 2), x.shape[2], x.shape[3]


class DinoVisionTransformer(nn.Module):
    """DINOv3 ViT-B/16 with 4 storage tokens. encode_image_tokens() returns
    (B, 1+196, 768) = CLS + patch tokens (storage tokens dropped), matching
    the production vision backbone interface."""

    N_STORAGE_TOKENS = 4

    def __init__(self, embed_dim, depth, num_heads):
        super().__init__()
        self.embed_dim = embed_dim
        self.patch_embed = DinoPatchEmbed(embed_dim=embed_dim)
        self.cls_token = nn.Parameter(torch.empty(1, 1, embed_dim))
        self.storage_tokens = nn.Parameter(torch.empty(1, self.N_STORAGE_TOKENS, embed_dim))
        self.mask_token = nn.Parameter(torch.empty(1, embed_dim))
        self.rope_embed = DinoRope(embed_dim, num_heads)
        self.blocks = nn.ModuleList(DinoBlock(embed_dim, num_heads) for _ in range(depth))
        self.norm = nn.LayerNorm(embed_dim, eps=1e-5)
        self.init_weights()

    def init_weights(self):
        """Match production's models/dinov3/vision_transformer.py:init_weights_vit.
        Crucially, this fills `bias_mask` (otherwise NaN-initialized) so that
        the K-third of every qkv bias is masked to 0 — without this, a fresh
        DINOv3 produces NaN on its very first forward pass."""
        nn.init.normal_(self.cls_token, std=0.02)
        nn.init.normal_(self.storage_tokens, std=0.02)
        nn.init.zeros_(self.mask_token)
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.trunc_normal_(m.weight, std=0.02)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
                if isinstance(m, LinearKMaskedBias):
                    o = m.out_features
                    m.bias_mask.fill_(1)
                    m.bias_mask[o // 3 : 2 * o // 3].fill_(0)
            elif isinstance(m, nn.LayerNorm):
                m.reset_parameters()
            elif isinstance(m, LayerScale):
                nn.init.constant_(m.gamma, 1e-5)
            elif isinstance(m, DinoPatchEmbed):
                # Match nn.Conv2d default
                m.proj.reset_parameters()

    def encode_image_tokens(self, images):
        x, H, W = self.patch_embed(images)
        B = x.shape[0]
        cls_token = self.cls_token + 0 * self.mask_token  # production quirk, kept
        x = torch.cat(
            [cls_token.expand(B, -1, -1), self.storage_tokens.expand(B, -1, -1), x], dim=1
        )
        rope = self.rope_embed(H, W)
        for blk in self.blocks:
            x = blk(x, rope=rope)
        x = self.norm(x)
        cls_out = x[:, :1]
        patches = x[:, 1 + self.N_STORAGE_TOKENS :]
        return torch.cat([cls_out, patches], dim=1)


class DinoVisionBackbone(nn.Module):
    """Wrapper around DinoVisionTransformer with an optional bf16-autocast
    forward path. The wrapper keeps the production checkpoint key layout
    (`img_backbone.dinov3_model.*`) so the slim 200k checkpoint loads with
    zero missing/unexpected keys. Set bf16 with set_bfloat16(True): the
    DINO forward then runs under autocast(bf16) on CUDA, cutting
    vision-encoder activation memory roughly in half. Tokens are cast back
    to fp32 on the way out so the surrounding DiT stays dtype-stable.
    """

    def __init__(self, config: DiTConfig):
        super().__init__()
        self.dinov3_model = DinoVisionTransformer(
            embed_dim=config.vit_embed_dim,
            depth=config.vit_depth,
            num_heads=config.vit_num_heads,
        )
        self.bfloat16 = False

    def set_bfloat16(self, enabled: bool = True):
        self.bfloat16 = bool(enabled)

    def encode_image_tokens(self, images):
        if self.bfloat16 and images.is_cuda:
            with torch.autocast(device_type="cuda", dtype=torch.bfloat16):
                tokens = self.dinov3_model.encode_image_tokens(images)
            return tokens.to(torch.float32)
        return self.dinov3_model.encode_image_tokens(images)


# ABC-DiT policy.


def modulate(x, shift, scale):
    if shift.ndim == 2:
        shift = shift.unsqueeze(1)
        scale = scale.unsqueeze(1)
    return x * (1 + scale) + shift


def gate_residual(gate, residual):
    if gate.ndim == 2:
        gate = gate.unsqueeze(1)
    return gate * residual


def get_1d_sincos_pos_embed(embed_dim, length):
    omega = np.arange(embed_dim // 2, dtype=np.float64)
    omega /= embed_dim / 2.0
    omega = 1.0 / 10000**omega
    out = np.einsum("m,d->md", np.arange(length, dtype=np.float64), omega)
    return np.concatenate([np.sin(out), np.cos(out)], axis=1)


class TimestepEmbedder(nn.Module):
    def __init__(self, hidden_size, frequency_embedding_size=256):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(frequency_embedding_size, hidden_size, bias=True),
            nn.SiLU(),
            nn.Linear(hidden_size, hidden_size, bias=True),
        )
        self.frequency_embedding_size = frequency_embedding_size
        half = frequency_embedding_size // 2
        freqs = torch.exp(
            -math.log(10000) * torch.arange(half, dtype=torch.float32) / half
        )
        self.register_buffer("freqs", freqs, persistent=False)

    def timestep_embedding(self, t):
        freqs = self.freqs
        if freqs.device != t.device:
            freqs = freqs.to(device=t.device)
        args = t[:, None].float() * freqs[None]
        return torch.cat([torch.cos(args), torch.sin(args)], dim=-1)

    def forward(self, t):
        t_shape = t.shape
        t_freq = self.timestep_embedding(t.reshape(-1))
        t_emb = self.mlp(t_freq.to(self.mlp[0].weight.dtype))
        return t_emb.reshape(*t_shape, -1)


class DiTAttention(nn.Module):
    """Self-attention over action tokens (timm-equivalent, qkv_bias=True)."""

    def __init__(self, dim, num_heads):
        super().__init__()
        self.num_heads = num_heads
        self.qkv = nn.Linear(dim, dim * 3, bias=True)
        self.proj = nn.Linear(dim, dim, bias=True)

    def forward(self, x):
        B, N, C = x.shape
        qkv = self.qkv(x).reshape(B, N, 3, self.num_heads, C // self.num_heads)
        q, k, v = qkv.permute(2, 0, 3, 1, 4).unbind(0)
        x = F.scaled_dot_product_attention(q, k, v)
        return self.proj(x.transpose(1, 2).reshape(B, N, C))


class DiTMlp(nn.Module):
    def __init__(self, dim, hidden):
        super().__init__()
        self.fc1 = nn.Linear(dim, hidden)
        self.act = nn.GELU(approximate="tanh")
        self.fc2 = nn.Linear(hidden, dim)

    def forward(self, x):
        return self.fc2(self.act(self.fc1(x)))


class DiTBlock(nn.Module):
    """AdaLN-Zero DiT block with vision cross-attention (9-way modulation)."""

    def __init__(self, hidden_size, num_heads, mlp_ratio=4.0):
        super().__init__()
        self.norm1 = nn.LayerNorm(hidden_size, elementwise_affine=False, eps=1e-6)
        self.attn = DiTAttention(hidden_size, num_heads)
        self.norm2 = nn.LayerNorm(hidden_size, elementwise_affine=False, eps=1e-6)
        self.mlp = DiTMlp(hidden_size, int(hidden_size * mlp_ratio))
        self.norm_xattn = nn.LayerNorm(hidden_size, elementwise_affine=False, eps=1e-6)
        self.norm_xattn_kv = nn.LayerNorm(hidden_size, elementwise_affine=False, eps=1e-6)
        self.cross_attn = nn.MultiheadAttention(hidden_size, num_heads, batch_first=True)
        self.adaLN_modulation = nn.Sequential(
            nn.SiLU(), nn.Linear(hidden_size, 9 * hidden_size, bias=True)
        )

    def forward(self, x, c, vision_tokens):
        (
            shift_msa, scale_msa, gate_msa,
            shift_xattn, scale_xattn, gate_xattn,
            shift_mlp, scale_mlp, gate_mlp,
        ) = self.adaLN_modulation(c).chunk(9, dim=-1)

        x = x + gate_residual(gate_msa, self.attn(modulate(self.norm1(x), shift_msa, scale_msa)))

        x_normed = modulate(self.norm_xattn(x), shift_xattn, scale_xattn)
        kv = self.norm_xattn_kv(vision_tokens)
        xattn_out, _ = self.cross_attn(x_normed, kv, kv, need_weights=False)
        x = x + gate_residual(gate_xattn, xattn_out)

        x = x + gate_residual(gate_mlp, self.mlp(modulate(self.norm2(x), shift_mlp, scale_mlp)))
        return x


class FinalLayer(nn.Module):
    def __init__(self, hidden_size, action_dim):
        super().__init__()
        self.norm_final = nn.LayerNorm(hidden_size, elementwise_affine=False, eps=1e-6)
        self.linear = nn.Linear(hidden_size, action_dim, bias=True)
        self.adaLN_modulation = nn.Sequential(
            nn.SiLU(), nn.Linear(hidden_size, 2 * hidden_size, bias=True)
        )

    def forward(self, x, c):
        shift, scale = self.adaLN_modulation(c).chunk(2, dim=-1)
        return self.linear(modulate(self.norm_final(x), shift, scale))


class PoolMlp(nn.Module):
    def __init__(self, in_dim, hidden_dim):
        super().__init__()
        self.fc1 = nn.Linear(in_dim, hidden_dim)
        self.act = nn.GELU()
        self.fc2 = nn.Linear(hidden_dim, in_dim)

    def forward(self, x):
        return self.fc2(self.act(self.fc1(x)))


class AttentionPoolBlock(nn.Module):
    """Learnable queries cross-attend to ViT tokens (per camera)."""

    def __init__(self, embed_dim, num_heads, mlp_ratio=4):
        super().__init__()
        self.ln_1 = nn.LayerNorm(embed_dim)
        self.attention = nn.MultiheadAttention(embed_dim, num_heads, batch_first=True)
        self.ln_2 = nn.LayerNorm(embed_dim)
        self.mlp = PoolMlp(embed_dim, int(mlp_ratio * embed_dim))

    def forward(self, x, queries):
        x_kv = self.ln_1(x)
        x_q = self.ln_1(queries)
        out, _ = self.attention(x_q, x_kv, x_kv, need_weights=False)
        return self.mlp(self.ln_2(out)) + out


class DiTPolicy(nn.Module):
    """Minimal ABC-DiT: loads the production dit_xL pretraining checkpoint."""

    def __init__(self, config: DiTConfig):
        super().__init__()
        self.config = config
        H = config.hidden_size
        self.camera_keys = list(config.camera_keys)
        self.chunk_length = config.chunk_length
        self.action_dim = config.action_dim

        self.x_embedder = nn.Linear(config.state_dim, H)
        self.y_embedder = nn.Linear(config.action_dim, H)
        # Checkpoint compatibility only; unused in forward.
        self.img_proj = nn.Linear(config.vit_embed_dim, H)
        self.img_proj.requires_grad_(False)
        self.t_embedder = TimestepEmbedder(H)
        self.pos_embed = nn.Parameter(torch.zeros(1, config.chunk_length, H), requires_grad=False)

        self.img_backbone = DinoVisionBackbone(config)

        self.apool_queries = nn.ParameterDict(
            {
                cam: nn.Parameter(
                    torch.randn(1, config.vision_pool_num_queries, config.vit_embed_dim) * 0.02
                )
                for cam in self.camera_keys
            }
        )
        self.apool = nn.ModuleDict(
            {
                cam: AttentionPoolBlock(
                    config.vit_embed_dim,
                    config.vision_pool_num_heads,
                    config.vision_pool_mlp_ratio,
                )
                for cam in self.camera_keys
            }
        )
        self.vision_tokens_proj = nn.Linear(config.vit_embed_dim, H)
        self.vision_camera_embed = nn.Embedding(len(self.camera_keys), H)

        self.task_to_hidden = nn.Linear(config.task_embed_dim, H)
        self.blocks = nn.ModuleList(
            DiTBlock(H, config.num_heads, config.mlp_ratio) for _ in range(config.depth)
        )
        self.final_layer = FinalLayer(H, config.action_dim)

        # cond = [state, task, timestep] -> hidden (vision goes via cross-attn)
        self.cond_proj = nn.Sequential(
            nn.Linear(3 * H, H), nn.SiLU(), nn.Linear(H, H), nn.LayerNorm(H)
        )

        self.register_buffer("clip_mean", torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))
        self.register_buffer("clip_std", torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1))

        pos = get_1d_sincos_pos_embed(H, config.chunk_length)
        self.pos_embed.data.copy_(torch.from_numpy(pos).float().unsqueeze(0))

    def build_vision_tokens(self, images):
        """images: dict cam -> (B, 3, 224, 224), already ImageNet-normalized.
        Returns (B, num_cameras * queries, hidden)."""
        pooled = []
        for cam in self.camera_keys:
            tokens = self.img_backbone.encode_image_tokens(images[cam])
            tokens = tokens.to(self.apool_queries[cam].dtype)
            queries = self.apool_queries[cam].expand(tokens.shape[0], -1, -1)
            pooled.append(self.apool[cam](tokens, queries))
        tokens_by_camera = torch.stack(pooled, dim=1)  # (B, Nc, K, vit_dim)
        B, Nc, K, D = tokens_by_camera.shape
        vision_tokens = self.vision_tokens_proj(
            tokens_by_camera.reshape(B * Nc * K, D).to(self.vision_tokens_proj.weight.dtype)
        ).reshape(B, Nc, K, -1)
        cam_emb = self.vision_camera_embed(torch.arange(Nc, device=vision_tokens.device))
        vision_tokens = vision_tokens + cam_emb[None, :, None, :]
        return vision_tokens.reshape(B, Nc * K, -1)

    def compute_cond(self, state, task_vec_clip, t_cond):
        """state (B,14); task_vec_clip (B,512); t_cond (B,) or (B,T).
        Returns conditioning c: (B,H) or (B,T,H)."""
        model_dtype = self.x_embedder.weight.dtype
        cond_dtype = self.cond_proj[0].weight.dtype
        st_vec = self.x_embedder(state.to(model_dtype))
        task_vec_h = self.task_to_hidden(task_vec_clip.to(self.task_to_hidden.weight.dtype))
        task_vec_h = task_vec_h.to(model_dtype)
        t_vec = self.t_embedder(t_cond.to(model_dtype))
        cond_parts = [st_vec, task_vec_h, t_vec]
        if t_vec.ndim == 3:
            T = t_vec.shape[1]
            cond_parts = [
                p.unsqueeze(1).expand(-1, T, -1) if p.ndim == 2 else p for p in cond_parts
            ]
        cond_concat = torch.cat(cond_parts, dim=-1).to(cond_dtype)
        if cond_dtype == torch.float32 and cond_concat.is_cuda:
            with torch.autocast(device_type="cuda", enabled=False):
                return self.cond_proj(cond_concat).to(model_dtype)
        return self.cond_proj(cond_concat).to(model_dtype)

    def predict_velocity(self, x_t, c, vision_tokens):
        z = self.y_embedder(x_t) + self.pos_embed.data[:, : x_t.shape[1], :]
        for block in self.blocks:
            z = block(z, c, vision_tokens)
        return self.final_layer(z, c)

    def forward(
        self,
        batch,
        noise=None,
        t=None,
        max_action_prefix=0,
        prefix_conditioning_prob=1.0,
        prefix_noise_scale=0.0,
    ):
        """Flow-matching training loss with optional action-prefix conditioning.
        batch: state (B,14), images dict, actions (B,30,14), task_vec_clip (B,512),
        optional state_is_masked (B,) bool."""
        state = batch["state"]
        actions = batch["actions"]
        N, T_chunk, D_action = actions.shape

        if noise is None:
            noise = torch.randn_like(actions)
        if t is None:
            t = torch.rand(N, 1, 1, device=state.device, dtype=actions.dtype)

        if max_action_prefix > 0:
            apply_prefix = torch.rand(N, device=state.device) < prefix_conditioning_prob
            if "state_is_masked" in batch:
                apply_prefix = apply_prefix & ~batch["state_is_masked"].to(state.device)
            delay = torch.randint(0, max_action_prefix, (N,), device=state.device)
            delay = torch.where(apply_prefix, delay, torch.zeros_like(delay))
            prefix_mask = torch.arange(T_chunk, device=state.device)[None, :] < delay[:, None]
            prefix_mask_expanded = prefix_mask.unsqueeze(-1)
            t_per_pos = torch.where(prefix_mask_expanded, torch.zeros_like(t), t)
        else:
            prefix_mask_expanded = None
            t_per_pos = t

        x_t = (1 - t_per_pos) * actions + t_per_pos * noise
        if prefix_noise_scale > 0.0 and prefix_mask_expanded is not None:
            x_t = x_t + prefix_mask_expanded.float() * torch.randn_like(x_t) * prefix_noise_scale

        vision_tokens = self.build_vision_tokens(batch["images"])
        t_cond = t_per_pos.squeeze(-1) if prefix_mask_expanded is not None else t[:, 0, 0]
        c = self.compute_cond(state, batch["task_vec_clip"], t_cond)
        v_t = self.predict_velocity(x_t, c, vision_tokens)

        u_t = noise - actions
        if prefix_mask_expanded is not None:
            postfix_mask = ~prefix_mask_expanded
            masked_loss = ((u_t - v_t) ** 2) * postfix_mask.float()
            return masked_loss.sum() / (postfix_mask.float().sum() * D_action + 1e-8)
        return F.mse_loss(u_t, v_t)

    @torch.no_grad()
    def sample_actions(self, batch, num_steps=10, noise=None):
        """Euler flow integration from noise to actions (production tau=1 path).
        Vision tokens and the static conditioning are computed once and reused
        across steps, like production infer()."""
        state = batch["state"]
        B = state.shape[0]
        model_dtype = self.y_embedder.weight.dtype
        if noise is None:
            noise = torch.randn(
                B,
                self.chunk_length,
                self.action_dim,
                device=state.device,
                dtype=model_dtype,
            )
        x_t = noise.to(device=state.device, dtype=model_dtype)
        vision_tokens = self.build_vision_tokens(batch["images"])
        dt = -1.0 / num_steps
        for i in range(num_steps):
            t = torch.full((B,), 1.0 + i * dt, device=state.device, dtype=model_dtype)
            c = self.compute_cond(state, batch["task_vec_clip"], t)
            v = self.predict_velocity(x_t, c, vision_tokens)
            x_t = x_t + v * dt
        return x_t

    @torch.no_grad()
    def sample_actions_rtc(self, batch, action_prefix, prefix_length: int, num_steps=10, noise=None):
        """Euler sampling with per-position action-prefix conditioning."""
        state = batch["state"]
        B = state.shape[0]
        model_dtype = self.y_embedder.weight.dtype
        if noise is None:
            noise = torch.randn(
                B,
                self.chunk_length,
                self.action_dim,
                device=state.device,
                dtype=model_dtype,
            )
        x_t = noise.to(device=state.device, dtype=model_dtype)
        action_prefix = action_prefix.to(device=state.device, dtype=model_dtype)

        prefix_pos = torch.arange(self.chunk_length, device=state.device) < prefix_length
        prefix_mask = prefix_pos.view(1, self.chunk_length, 1).expand_as(x_t)
        prefix_t_mask = prefix_pos.view(1, self.chunk_length).expand(B, self.chunk_length)
        x_t = torch.where(prefix_mask, action_prefix, x_t)

        vision_tokens = self.build_vision_tokens(batch["images"])
        dt = -1.0 / num_steps
        for i in range(num_steps):
            t = torch.full(
                (B, self.chunk_length),
                1.0 + i * dt,
                device=state.device,
                dtype=model_dtype,
            )
            t = torch.where(prefix_t_mask, torch.zeros_like(t), t)
            c = self.compute_cond(state, batch["task_vec_clip"], t)
            v = self.predict_velocity(x_t, c, vision_tokens)
            x_t = x_t + v * dt
            x_t = torch.where(prefix_mask, action_prefix, x_t)
        return x_t


def load_pretrained(model, ckpt_path):
    """Load the slim production checkpoint (model-only, prefixes stripped)."""
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False, mmap=True)
    sd = ckpt["model"] if "model" in ckpt else ckpt
    sd = {k[len("_orig_mod."):] if k.startswith("_orig_mod.") else k: v for k, v in sd.items()}
    missing, unexpected = model.load_state_dict(sd, strict=False)
    if unexpected:
        raise RuntimeError(f"unexpected checkpoint keys: {unexpected[:8]}")
    if missing:
        raise RuntimeError(f"missing checkpoint keys: {missing[:8]}")
    return ckpt


if __name__ == "__main__":
    model = DiTPolicy(DiTConfig())
    n_params = sum(p.numel() for p in model.parameters())
    print(f"DiTPolicy built: {n_params / 1e9:.3f}B params")
