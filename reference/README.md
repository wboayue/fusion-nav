# Reference papers

Fetched, not committed — arXiv's licence covers distribution *there*, this crate is MIT, and the
standards body owns its own document. `reference/*.pdf` is gitignored; this file is the record of
what belongs here, and an entry says what it is the source *of*, so a claim in the repository can
be traced to one.

| File | Source | sha256 |
| --- | --- | --- |
| `sola-quaternion-kinematics.pdf` | J. Solà, *Quaternion kinematics for the error-state Kalman filter*, [arXiv:1711.02508v1](https://arxiv.org/abs/1711.02508) | `6a3d3d3933c425eb6cef049290c2b48b71a90f5189e9810575d83d3a922faaca` |
| `nga-wgs84.pdf` | NGA.STND.0036 v1.0.0, *World Geodetic System 1984*, 2014-07-08 | `5edc1cf7411d4df01cae5205ee19f661265c45a55cca60b9351f6ae5abade512` |
| `WMM2025COF.zip` | NCEI, *World Magnetic Model 2025* coefficients, [WMM2025COF.zip](https://www.ncei.noaa.gov/products/world-magnetic-model/wmm-coefficients) | `2e76569370d081f2cd7919490218bd094ca9afde347b198eff5621e0af460d03` |
| `zanetti-dsouza-consider.pdf` | R. Zanetti and C. D'Souza, *Recursive Implementations of the Consider Filter*, AAS preprint, NASA NTRS [20120010515](https://ntrs.nasa.gov/citations/20120010515) | `4ea34db951911cfd21818ebd54c71f3adfff00e8c351c740a00f78af883a8189` |

```bash
curl -sSL -o reference/sola-quaternion-kinematics.pdf https://arxiv.org/pdf/1711.02508v1
curl -sSL -o reference/nga-wgs84.pdf "https://earth-info.nga.mil/php/download.php?file=coord-wgs84"
curl -sSL -o reference/zanetti-dsouza-consider.pdf https://ntrs.nasa.gov/api/citations/20120010515/downloads/20120010515.pdf
curl -sSL -o reference/WMM2025COF.zip https://www.ncei.noaa.gov/sites/default/files/2024-12/WMM2025COF.zip
```

Solà is the primary source for the error-state formulation and the Jacobians in
[`EQUATIONS.md`](../EQUATIONS.md), which maps its equations onto his; cite it by equation number,
not by page, since a later version would renumber pages and not equations. arXiv holds v1 only.

The WGS 84 standard is the source of the ellipsoid constants in `src/geodetic.rs` — `WGS84_A` from
its Table 3.1, `WGS84_E2` from its Table 3.5 — and of nothing else here. It defines the ellipsoid
but never writes the geodetic-to-ECEF conversion of equation (43), so that citation stays Groves
(2.112), which is a book and therefore the one claim in this repository a reader cannot check
without buying something.

Zanetti and D'Souza is the source for the consider (Schmidt–Kalman) update that equation (30′)
was measured against and did not adopt: Joseph form holding for any gain, their (5), the consider
gain as the optimal gain with its parameter rows zeroed, (26), and the parameter covariance left
unchanged by an update, (29). Schmidt's own 1966 chapter is not freely available; this paper
states the algebra and cites it.

NCEI's WMM2025 archive is the source of the declination table in `src/magnetic.rs`, through
`tools/declination.py`; NCEI states the model is in the public domain. Its `WMM2025.COF` is
byte-identical to the `WMM_2025.COF` that `pygeomag` 1.1.0 ships, whose sha256 the tool checks
before reading it, and `pygeomag` reproduces the archive's `WMM2025_TestValues.txt` declinations
to 0.005°, the two decimals they are printed to. So the tool reads the published model, not a copy
that only resembles it.
