# Ego spatial sectors (agreed)

Reference figure: [`assets/ego_spatial_sectors_reference.png`](assets/ego_spatial_sectors_reference.png)

## Coordinates

- Robot plane: `(+forward, +left)` meters
- `θ = atan2(left, forward)` in degrees, **left-positive**
- `r = hypot(forward, left)`
- Eligible only if `0.5 ≤ r ≤ 5` m; otherwise reject / do not publish

## Shared ahead cone

| Label | Angle | Families |
|-------|-------|----------|
| Ahead | `\|θ\| ≤ 45°` | `fore_aft`, `any_ahead`, `ego_turn` → ahead |

## `fore_aft` / `any_ahead`

| Label | Angle | Action |
|-------|-------|--------|
| Ahead | `\|θ\| ≤ 45°` | publish |
| Behind | `\|θ\| ≥ 135°` | publish (`fore_aft` only) |
| Side | `45° < \|θ\| < 135°` | reject |

`any_ahead` is true only if some instance lies in the **Ahead** cone (not the forward half-plane).

## `ego_turn`

| Answer | Angle |
|--------|-------|
| ahead | `\|θ\| ≤ 45°` |
| left | `45° < θ < 135°` |
| right | `−135° < θ < −45°` |
| turn around | `\|θ\| ≥ 135°` (keep the question; do **not** reject) |

## `next_to`

Keep landmark pairing, but the resolved anchor must sit in the family’s allowed sector and `r ≤ 5` m.
