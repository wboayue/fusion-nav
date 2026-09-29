"""The local tangent plane both converters place fixes and truth on.

One copy, so the PX4 corpus and UrbanNav agree with each other and with the filter
about where a latitude, longitude and height are. Standard library only.
"""

import math

# WGS84, for the local tangent plane about the first GNSS fix.
WGS84_A = 6_378_137.0
WGS84_E2 = 6.694_379_990_141e-3


def geodetic_to_ned(lat, lon, alt, lat0, lon0, alt0):
    """Local tangent plane about (lat0, lon0, alt0). Degrees in, meters out.

    Exact, the same conversion as `LocalOrigin::to_ned` in src/geodetic.rs, so the
    replay corpus and the filter agree about where a fix is: both points to
    Earth-centered coordinates, then the difference rotated onto the origin's
    north, east and down.
    """
    x, y, z = ecef(lat, lon, alt)
    x0, y0, z0 = ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    phi, lam = math.radians(lat0), math.radians(lon0)
    sp, cp, sl, cl = math.sin(phi), math.cos(phi), math.sin(lam), math.cos(lam)
    north = -sp * cl * dx - sp * sl * dy + cp * dz
    east = -sl * dx + cl * dy
    down = -cp * cl * dx - cp * sl * dy - sp * dz
    return north, east, down


def ecef(lat, lon, alt):
    """Earth-centered, Earth-fixed meters of a WGS84 position in degrees."""
    phi, lam = math.radians(lat), math.radians(lon)
    s = math.sin(phi)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    return (
        (n + alt) * math.cos(phi) * math.cos(lam),
        (n + alt) * math.cos(phi) * math.sin(lam),
        (n * (1.0 - WGS84_E2) + alt) * s,
    )
