# SPDX-License-Identifier: MIT
"""A GPS orbit model for predicting days ahead.

Forces: Earth's gravity field to degree and order 4 (EGM2008), the Sun and the Moon (ERFA), and
solar radiation pressure as five ECOM-like terms (constant along the Sun direction, the solar
panel axis and the third axis, plus a once-per-revolution term on the third), switched off in
Earth's shadow. The equations of motion are integrated in GCRS; Earth orientation comes from ERFA
(IAU 2006/2000A precession-nutation) with polar motion and UT1 from the IGS ERP file. Treating the
Earth's axis as fixed in space instead left ~20 m of error over two days.
"""
import math

import erfa
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares

GM_E, R_E = 3.986004418e14, 6378136.3
GM_SUN, GM_MOON = 1.32712440018e20, 4.9028e12
AU = 1.495978707e11
JD_GPS0 = 2444244.5               # Julian date of the GPS epoch, 1980-01-06
TT_GPS = 51.184                   # TT - GPS, s
UTC_GPS = -18.0                   # UTC - GPS since 2017, s

# EGM2008, fully normalised. Degree 6 was tried and changed the fit by under 0.1 m.
_CBAR = {(2, 0): -0.484165143790815e-3, (2, 1): -0.206615509074176e-9, (2, 2): 0.243938357328313e-5,
         (3, 0): 0.957161207093473e-6, (3, 1): 0.203046201047864e-5, (3, 2): 0.904787894809528e-6,
         (3, 3): 0.721321757121568e-6, (4, 0): 0.539965866638991e-6, (4, 1): -0.536157389388867e-6,
         (4, 2): 0.350501623962649e-6, (4, 3): 0.990856766672321e-6, (4, 4): -0.188519633023033e-6}
_SBAR = {(2, 1): 0.138441389137979e-8, (2, 2): -0.140027370385934e-5, (3, 1): 0.248200415856872e-6,
         (3, 2): -0.619005475177618e-6, (3, 3): 0.141434926192941e-5, (4, 1): -0.473567346518086e-6,
         (4, 2): 0.662480026275829e-6, (4, 3): -0.200956723567452e-6, (4, 4): 0.308803882149194e-6}
NMAX = 4
_C = np.zeros((NMAX + 1, NMAX + 1))
_S = np.zeros((NMAX + 1, NMAX + 1))
_C[0, 0] = 1.0
for (_n, _m), _v in _CBAR.items():
    _f = math.sqrt((2 - (_m == 0)) * (2 * _n + 1) * math.factorial(_n - _m) / math.factorial(_n + _m))
    _C[_n, _m] = _v * _f
    _S[_n, _m] = _SBAR.get((_n, _m), 0.0) * _f


def gravity(r):
    """Geopotential acceleration in the Earth-fixed frame (Montenbruck & Gill, eq. 3.33)."""
    x, y, z = r
    r2 = x * x + y * y + z * z
    rho = R_E * R_E / r2
    x0, y0, z0 = R_E * x / r2, R_E * y / r2, R_E * z / r2
    N = NMAX + 2
    V = np.zeros((N, N))
    W = np.zeros((N, N))
    V[0, 0] = R_E / math.sqrt(r2)
    V[1, 0] = z0 * V[0, 0]
    for n in range(2, N):
        V[n, 0] = ((2 * n - 1) * z0 * V[n - 1, 0] - (n - 1) * rho * V[n - 2, 0]) / n
    for m in range(1, N):
        V[m, m] = (2 * m - 1) * (x0 * V[m - 1, m - 1] - y0 * W[m - 1, m - 1])
        W[m, m] = (2 * m - 1) * (x0 * W[m - 1, m - 1] + y0 * V[m - 1, m - 1])
        if m + 1 < N:
            V[m + 1, m] = (2 * m + 1) * z0 * V[m, m]
            W[m + 1, m] = (2 * m + 1) * z0 * W[m, m]
        for n in range(m + 2, N):
            V[n, m] = ((2 * n - 1) * z0 * V[n - 1, m] - (n + m - 1) * rho * V[n - 2, m]) / (n - m)
            W[n, m] = ((2 * n - 1) * z0 * W[n - 1, m] - (n + m - 1) * rho * W[n - 2, m]) / (n - m)
    ax = ay = az = 0.0
    for n in range(NMAX + 1):
        for m in range(n + 1):
            C, S = _C[n, m], _S[n, m]
            if m == 0:
                ax -= C * V[n + 1, 1]
                ay -= C * W[n + 1, 1]
                az -= (n + 1) * C * V[n + 1, 0]
            else:
                f = 0.5 * (n - m + 1) * (n - m + 2)
                ax += 0.5 * (-C * V[n + 1, m + 1] - S * W[n + 1, m + 1]) + f * (C * V[n + 1, m - 1] + S * W[n + 1, m - 1])
                ay += 0.5 * (-C * W[n + 1, m + 1] + S * V[n + 1, m + 1]) + f * (-C * W[n + 1, m - 1] + S * V[n + 1, m - 1])
                az += (n - m + 1) * (-C * V[n + 1, m] - S * W[n + 1, m])
    return GM_E / (R_E * R_E) * np.array([ax, ay, az])


class EarthOrientation:
    """GCRS <-> ITRS from ERFA and IGS ERP rows (gps seconds, xp rad, yp rad, UT1-UTC s)."""

    def __init__(self, erp, t_start, t_end):
        self.erp = erp
        # Precession-nutation changes slowly: tabulate it every 30 minutes and interpolate.
        self.grid = np.arange(t_start - 3600, t_end + 86400, 1800.0)
        self.npb = np.array([erfa.pnm06a(JD_GPS0, (t + TT_GPS) / 86400.0) for t in self.grid])

    def _eop(self, t):
        ts = [row[0] for row in self.erp]
        return [float(np.interp(t, ts, [row[k] for row in self.erp])) for k in (1, 2, 3)]

    def c2t(self, t):
        xp, yp, dut1 = self._eop(t)
        k = min(max(int((t - self.grid[0]) // 1800), 0), len(self.grid) - 2)
        f = (t - self.grid[k]) / 1800.0
        npb = self.npb[k] * (1 - f) + self.npb[k + 1] * f
        era = erfa.era00(JD_GPS0, (t + UTC_GPS + dut1) / 86400.0)
        c, s = math.cos(era), math.sin(era)
        rz = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        return erfa.pom00(xp, yp, erfa.sp00(JD_GPS0, (t + TT_GPS) / 86400.0)) @ rz @ npb


def read_erp(text):
    """IGS ERP file -> rows (gps seconds, xp rad, yp rad, UT1-UTC s), extended ten days ahead with
    UT1 drifting by the last length of day and the pole held."""
    arc = math.pi / 180 / 3600 * 1e-6
    rows = []
    for line in text.splitlines():
        f = line.split()
        if f and f[0][0].isdigit() and len(f) >= 5:
            t = (float(f[0]) - 44244.0) * 86400 - UTC_GPS
            rows.append((t, float(f[1]) * arc, float(f[2]) * arc, float(f[3]) * 1e-7, float(f[4]) * 1e-7))
    if not rows:
        raise ValueError("no ERP rows")
    t, xp, yp, ut, lod = rows[-1]
    rows += [(t + d * 86400, xp, yp, ut - lod * d, lod) for d in range(1, 11)]
    return [row[:4] for row in rows]


def _lagrange(ts, ys, t, order=10):
    i = int(np.searchsorted(ts, t))
    lo = max(0, min(len(ts) - order, i - order // 2))
    xs = ts[lo:lo + order]
    out = 0
    for j in range(order):
        w = 1.0
        for m in range(order):
            if m != j:
                w *= (t - xs[m]) / (xs[j] - xs[m])
        out = out + w * ys[lo + j]
    return out


class Orbit:
    def __init__(self, eo, t0):
        self.eo, self.t0 = eo, t0

    @staticmethod
    def _bodies(t):
        d1, d2 = JD_GPS0, (t + TT_GPS) / 86400.0
        pvh, _ = erfa.epv00(d1, d2)
        return -pvh[0] * AU, erfa.moon98(d1, d2)[0] * AU

    def _rhs(self, t, y, srp):
        r, v = y[:3], y[3:]
        m = self.eo.c2t(t)
        a = m.T @ gravity(m @ r)
        sun, moon = self._bodies(t)
        for s, gm in ((sun, GM_SUN), (moon, GM_MOON)):
            d = s - r
            a += gm * (d / np.linalg.norm(d) ** 3 - s / np.linalg.norm(s) ** 3)
        ss = sun / np.linalg.norm(sun)
        proj = r @ ss
        if proj > 0 or np.linalg.norm(r - proj * ss) > R_E:          # not in Earth's shadow
            eD = sun - r
            eD /= np.linalg.norm(eD)
            er = r / np.linalg.norm(r)
            eY = np.cross(eD, er)
            eY /= np.linalg.norm(eY)
            eB = np.cross(eD, eY)
            h = np.cross(r, v)
            h /= np.linalg.norm(h)
            sp = ss - (ss @ h) * h
            sp /= np.linalg.norm(sp)
            u = math.atan2(np.cross(sp, er) @ h, sp @ er)
            D0, Y0, B0, Bc, Bs = srp
            a += 1e-9 * (D0 * eD + Y0 * eY + (B0 + Bc * math.cos(u) + Bs * math.sin(u)) * eB)
        return np.concatenate([v, a])

    def propagate(self, state, srp, times):
        sol = solve_ivp(self._rhs, (self.t0, float(max(times))), state, method="DOP853",
                        t_eval=times, rtol=1e-11, atol=1e-5, args=(srp,))
        return sol.y[:3].T

    def fit(self, times, positions_gcrs, start=None):
        """Initial state and radiation-pressure terms that best reproduce the positions. Returns
        (state, srp, rms m, solver evaluations). The arc must start at t0. start: an optional
        (state, srp) to begin from, such as the previous run's carried forward; it changes only
        where the solver starts, not what it fits."""
        ts, xs = np.asarray(times, float), np.asarray(positions_gcrs, float)
        if start is None:
            v0 = (_lagrange(ts, xs, ts[0] + 1.0) - _lagrange(ts, xs, ts[0] - 1.0)) / 2.0
            x0 = np.concatenate([xs[0], v0, np.zeros(5)])
        else:
            x0 = np.concatenate([start[0], start[1]])
        scale = np.array([1, 1, 1, 1e-3, 1e-3, 1e-3, 1, 1, 1, 1, 1])

        def res(p):
            q = x0 + p * scale
            return (self.propagate(q[:6], q[6:], ts) - xs).ravel()

        def jac(p):
            f0 = res(p)
            cols = []
            for j in range(11):
                step = np.zeros(11)
                step[j] = 1e-2 if j < 6 else 1.0
                cols.append((res(p + step) - f0) / step[j])
            return np.array(cols).T

        sol = least_squares(res, np.zeros(11), jac=jac, method="lm", xtol=1e-12, ftol=1e-12, max_nfev=40)
        q = x0 + sol.x * scale
        return q[:6], q[6:], math.sqrt(float(np.mean(sol.fun ** 2))), int(sol.nfev)
