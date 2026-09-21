# Reference papers

Fetched, not committed — arXiv's licence covers distribution *there*, and this crate is MIT.
`reference/*.pdf` is gitignored; this file is the record of what belongs here.

| File | Source | sha256 |
| --- | --- | --- |
| `sola-quaternion-kinematics.pdf` | J. Solà, *Quaternion kinematics for the error-state Kalman filter*, [arXiv:1711.02508v1](https://arxiv.org/abs/1711.02508) | `6a3d3d3933c425eb6cef049290c2b48b71a90f5189e9810575d83d3a922faaca` |

```bash
curl -sSL -o reference/sola-quaternion-kinematics.pdf https://arxiv.org/pdf/1711.02508v1
```

Solà is the primary source for the error-state formulation and the Jacobians in
[`EQUATIONS.md`](../EQUATIONS.md); cite it by equation number there, not by page here, since a
later arXiv version renumbers pages and not equations.
