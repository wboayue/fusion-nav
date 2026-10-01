"""The `# GNSS noise parameters` header line `examples/replay/main.rs --r-policy px4` reads.

Beside the converters rather than in one, since every converter writes it: the PX4 corpus
with the values its log's EKF2 flew, UrbanNav with PX4's defaults. Standard library only.
"""

# The parameters PX4 floors and caps a receiver's reported accuracy with, and
# their defaults at c4e4ef98 (`src/modules/ekf2/params_gnss.yaml:30-72`,
# `module.yaml:67-71` for EKF2_NOAID_NOISE) for a log that does not carry one.
GNSS_NOISE_PARAMETERS = [
    ("EKF2_GPS_P_NOISE", 0.5),
    ("EKF2_GPS_V_NOISE", 0.3),
    ("EKF2_NOAID_NOISE", 10.0),
]


def gnss_noise_note(params):
    """The header line `examples/replay/main.rs --r-policy px4` reads its floors from.

    The values this log's EKF2 bounded its receiver with, so a floored replay is
    compared against the `R` EKF2 actually fused rather than a default it may not
    have run: `2c42096b` flew `EKF2_GPS_V_NOISE` 0.3, not the 0.5 #113 floored it
    at. A parameter the log does not carry falls back to PX4's default and says so. The rule the harness applies to them is its own, cited
    there; this line carries values only.
    """
    cells = []
    for name, default in GNSS_NOISE_PARAMETERS:
        if name in params:
            cells.append(f"{name} {float(params[name]):.6g}")
        else:
            cells.append(f"{name} {default:.6g} (PX4 default; not in the log)")
    return "GNSS noise parameters: " + ", ".join(cells)
