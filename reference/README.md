# Reference papers

Fetched, not committed — arXiv's licence covers distribution *there*, this crate is MIT, and the
standards body owns its own document. `reference/*.pdf` is gitignored; this file is the record of
what belongs here, and an entry says what it is the source *of*, so a claim in the repository can
be traced to one.

| File | Source | sha256 |
| --- | --- | --- |
| `sola-quaternion-kinematics.pdf` | J. Solà, *Quaternion kinematics for the error-state Kalman filter*, [arXiv:1711.02508v1](https://arxiv.org/abs/1711.02508) | `6a3d3d3933c425eb6cef049290c2b48b71a90f5189e9810575d83d3a922faaca` |
| `nga-wgs84.pdf` | NGA.STND.0036 v1.0.0, *World Geodetic System 1984*, 2014-07-08 | `5edc1cf7411d4df01cae5205ee19f661265c45a55cca60b9351f6ae5abade512` |

```bash
curl -sSL -o reference/sola-quaternion-kinematics.pdf https://arxiv.org/pdf/1711.02508v1
curl -sSL -o reference/nga-wgs84.pdf "https://earth-info.nga.mil/php/download.php?file=coord-wgs84"
```

Solà is the primary source for the error-state formulation and the Jacobians in
[`EQUATIONS.md`](../EQUATIONS.md), which maps its equations onto his; cite it by equation number,
not by page, since a later version would renumber pages and not equations. arXiv holds v1 only.

The WGS 84 standard is the source of the ellipsoid constants in `src/geodetic.rs` — `WGS84_A` from
its Table 3.1, `WGS84_E2` from its Table 3.5 — and of nothing else here. It defines the ellipsoid
but never writes the geodetic-to-ECEF conversion of equation (43), so that citation stays Groves
(2.112), which is a book and therefore the one claim in this repository a reader cannot check
without buying something.
