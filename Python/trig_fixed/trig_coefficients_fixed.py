#!/usr/bin/env python3
"""
trig_coefficients_fixed.py

Compute fixed-point polynomial coefficients for sin/cos approximation at a given
Q factor (2^q scale), targeting a quarter-wave polynomial evaluator.

C evaluator structure (degree-4 example, shared-power sin+cos):

    theta2 = (theta * theta) >> q
    theta3 = (theta2 * theta) >> q
    theta4 = (theta3 * theta) >> q
    sin_out = (sc1*theta4 + sc2*theta3 + sc3*theta2 + sc4*theta + (sc5<<q)) >> q
    cos_out = (cc1*theta4 + cc2*theta3 + cc3*theta2 + cc4*theta + (cc5<<q)) >> q

where theta is in Qq format, reduced to [0, HALF_PI_QB] by quadrant logic.

This is equivalent to evaluating float polynomial p(u) where u = theta / 2^q
is the angle in radians [0, pi/2], and p(u) approximates sin(u)*2^q or cos(u)*2^q.

Coefficient naming (descending power): sc1=a_d, sc2=a_{d-1}, ..., sc_{d+1}=a_0
The constant term (sc_{d+1}) appears as (sc<<q) in the C accumulator before the
final >>q, so it is the raw rounded float coefficient, not pre-shifted.

Usage:
    python trig_coefficients_fixed.py --q 14
    python trig_coefficients_fixed.py --q 12 --degree 4 --plot
    python trig_coefficients_fixed.py --q 14 --degree 5
    python trig_coefficients_fixed.py --q 14 --compare   # compare against existing coefficients
"""

import numpy as np
import argparse
from scipy.optimize import linprog


def get_angle_constants(q):
    """Return dict of Qq-scaled angle constants."""
    return {
        f'HALF_PI_Q{q}B':             int(round(np.pi / 2 * 2**q)),
        f'PI_Q{q}B':                  int(round(np.pi * 2**q)),
        f'TWO_PI_Q{q}B':              int(round(2 * np.pi * 2**q)),
        f'THREE_BY_TWO_PI_Q{q}B':     int(round(3 * np.pi / 2 * 2**q)),
    }


def _fp_design_matrix(q, degree, n_samples):
    """
    Build design matrix using actual fixed-point power basis.

    Each column corresponds to a polynomial term as evaluated by the C fixed-point
    evaluator: theta^k computed via cascaded (>>q) integer shifts, then divided by
    2^q to give the float equivalent seen by the coefficient multiplier.

    Columns are in descending power order: [theta^d/2^q, ..., theta/2^q, 1].
    The constant column is 1 (not 2^q) because in the C evaluator the constant
    coefficient is applied as (sc_const << q) before the final >> q, which cancels.
    """
    u = np.linspace(0, np.pi / 2, n_samples)
    theta = np.round(u * float(2**q)).astype(np.int64)
    q_scale = float(2**q)

    # Compute integer powers via cascaded >>q shifts — identical to C evaluator
    pow_int = [None] * (degree + 1)
    pow_int[1] = theta.copy()
    for k in range(2, degree + 1):
        pow_int[k] = (pow_int[k - 1] * theta) >> q

    # Assemble columns descending: degree, degree-1, ..., 1, constant
    cols = [pow_int[k].astype(np.float64) / q_scale for k in range(degree, 0, -1)]
    cols.append(np.ones(n_samples, dtype=np.float64))

    return np.column_stack(cols), u


def _minimax_lp(A, y):
    """
    Solve the minimax (L-inf) polynomial fit as a linear program.

    Finds coefficients c minimizing max|A @ c - y|.
    Variables: x = [c_0, ..., c_{n-1}, t].  Minimize t subject to:
        A @ c - y  <=  t * ones
       -A @ c + y  <=  t * ones
    """
    n_samples, n_coeffs = A.shape
    c_obj = np.zeros(n_coeffs + 1)
    c_obj[-1] = 1.0  # minimize t

    ones_col = np.ones((n_samples, 1))
    A_ub = np.block([[ A, -ones_col],
                     [-A, -ones_col]])
    b_ub = np.concatenate([y, -y])

    result = linprog(c_obj, A_ub=A_ub, b_ub=b_ub, method='highs')
    if not result.success:
        raise RuntimeError(f"Minimax LP failed: {result.message}")
    return result.x[:n_coeffs]


def get_trig_coefficients(q, degree=4, n_samples=5000, method='minimax_fp'):
    """
    Fit polynomial approximations for sin(u)*2^q and cos(u)*2^q over u in [0, pi/2].

    Both functions require separate coefficient sets (different shapes over this interval).

    Args:
        q:         Q factor — output is scaled by 2^q
        degree:    polynomial degree (default 4)
        n_samples: sample points for fit (default 5000; minimax needs far fewer than polyfit)
        method:    'minimax_fp' — minimax LP fit using actual fixed-point basis (default);
                   'polyfit'   — least-squares float polynomial fit (original approach,
                                 kept for comparison)

    Returns:
        sin_coeffs: list of int, descending power order [a_d, ..., a_1, a_0]
        cos_coeffs: list of int, descending power order [b_d, ..., b_1, b_0]
    """
    u = np.linspace(0, np.pi / 2, n_samples)
    sin_target = np.sin(u) * float(2**q)
    cos_target = np.cos(u) * float(2**q)

    if method == 'polyfit':
        sin_f = np.polyfit(u, sin_target, degree)
        cos_f = np.polyfit(u, cos_target, degree)
        return list(np.round(sin_f).astype(int)), list(np.round(cos_f).astype(int))

    # minimax_fp: minimax LP over fixed-point basis vectors
    A, u = _fp_design_matrix(q, degree, n_samples)
    sin_target = np.sin(u) * float(2**q)
    cos_target = np.cos(u) * float(2**q)

    sin_f = _minimax_lp(A, sin_target)
    cos_f = _minimax_lp(A, cos_target)

    return list(np.round(sin_f).astype(int)), list(np.round(cos_f).astype(int))


def eval_poly_fixed(coeffs, q, u_float):
    """
    Evaluate the polynomial using integer fixed-point arithmetic mirroring the C evaluator.

    Args:
        coeffs:  list of int coefficients, descending power order
        q:       Q factor
        u_float: numpy array of float angles in [0, pi/2] (radians)

    Returns:
        numpy float array of Qq-scaled approximation values
    """
    degree = len(coeffs) - 1
    theta = np.round(u_float * float(2**q)).astype(np.int64)

    # Build power array where powers[k] = theta^k in Qq format
    #   powers[0] = 2^q  (constant term: coeff * 2^q, then >>q gives coeff)
    #   powers[1] = theta
    #   powers[k] = (powers[k-1] * theta) >> q  for k >= 2
    powers = [None] * (degree + 1)
    powers[0] = np.full_like(theta, 1 << q, dtype=np.int64)
    if degree >= 1:
        powers[1] = theta.copy()
    for k in range(2, degree + 1):
        powers[k] = (powers[k - 1] * theta) >> q

    # Coeffs are [a_d, a_{d-1}, ..., a_1, a_0]; coeffs[i] corresponds to u^(degree-i)
    result = np.zeros_like(theta, dtype=np.int64)
    for i, c in enumerate(coeffs):
        k = degree - i
        result += np.int64(c) * powers[k]

    result >>= q
    return result.astype(np.float64)


def compute_errors(coeffs, q, is_cos=False, n_eval=200000):
    """Return (max_abs_err_lsb, rms_err_lsb) for the fixed-point evaluator."""
    u = np.linspace(0, np.pi / 2, n_eval)
    exact = (np.cos(u) if is_cos else np.sin(u)) * float(2**q)
    approx = eval_poly_fixed(coeffs, q, u)
    err = approx - exact
    return float(np.max(np.abs(err))), float(np.sqrt(np.mean(err**2)))


def print_c_output(q, degree, sin_coeffs, cos_coeffs, show_errors=True):
    constants = get_angle_constants(q)

    print(f"/* ===== Q{q} Fixed-Point Trig Coefficients (degree-{degree} polynomial) ===== */")
    print(f"/* Evaluator: p(u) ~= sin/cos(u) * 2^{q},  u in [0, pi/2],  output [{-(1<<q)}, {1<<q}] */")
    print()
    print("/* Angle constants */")
    for name, val in constants.items():
        print(f"#define {name:<40} {val}")
    print()

    print(f"/* Sin polynomial coefficients (scale 2^{q}, descending power) */")
    for i, c in enumerate(sin_coeffs):
        power = degree - i
        label = f"sc{i+1}_q{q}b"
        comment = f"  /* u^{power} */" if power > 0 else ""
        print(f"static const int32_t {label:<20} = {c:8d};{comment}")
    print()

    print(f"/* Cos polynomial coefficients (scale 2^{q}, descending power) */")
    for i, c in enumerate(cos_coeffs):
        power = degree - i
        label = f"cc{i+1}_q{q}b"
        comment = f"  /* u^{power} */" if power > 0 else ""
        print(f"static const int32_t {label:<20} = {c:8d};{comment}")
    print()

    if show_errors:
        q_scale = float(1 << q)
        sin_max, sin_rms = compute_errors(sin_coeffs, q, is_cos=False)
        cos_max, cos_rms = compute_errors(cos_coeffs, q, is_cos=True)
        print("/* Fixed-point approximation error vs exact: */")
        print(f"/*   sin: max={sin_max:.2f} LSB ({sin_max/q_scale*100:.4f}%), rms={sin_rms:.3f} LSB */")
        print(f"/*   cos: max={cos_max:.2f} LSB ({cos_max/q_scale*100:.4f}%), rms={cos_rms:.3f} LSB */")


def compare_against_existing(q, degree, sin_coeffs, cos_coeffs):
    """Print comparison of newly fitted coefficients vs existing hardcoded values."""
    existing = {
        12: {
            'sin': [117, -834, 85, 4078, 1],
            'cos': None,  # existing code uses sin(theta + HALF_PI) instead
        },
        14: {
            'sin': [469, -3339, 342, 16310, 4],
            'cos': None,
        },
    }
    if q not in existing:
        print(f"/* No existing coefficients to compare for q={q} */")
        return

    ref = existing[q]
    print(f"\n/* === Comparison vs existing hardcoded coefficients (q={q}) === */")

    if ref['sin'] is not None and len(ref['sin']) == len(sin_coeffs):
        print(f"/*   {'term':<8} {'existing':>10}  {'new':>10}  {'delta':>8} */")
        for i, (e, n) in enumerate(zip(ref['sin'], sin_coeffs)):
            power = degree - i
            print(f"/*   sc{i+1} (u^{power}): {e:>10d}  {n:>10d}  {n-e:>+8d} */")

        ref_max, ref_rms = compute_errors(ref['sin'], q, is_cos=False)
        new_max, new_rms = compute_errors(sin_coeffs, q, is_cos=False)
        print(f"/*   sin error (existing): max={ref_max:.2f} LSB, rms={ref_rms:.3f} LSB */")
        print(f"/*   sin error (new fit):  max={new_max:.2f} LSB, rms={new_rms:.3f} LSB */")


def plot_errors(q, degree, sin_coeffs, cos_coeffs):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot")
        return

    N = 20000
    q_scale = float(1 << q)
    u = np.linspace(0, np.pi / 2, N)

    sin_exact  = np.sin(u) * q_scale
    cos_exact  = np.cos(u) * q_scale
    sin_approx = eval_poly_fixed(sin_coeffs, q, u)
    cos_approx = eval_poly_fixed(cos_coeffs, q, u)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f'Q{q} Fixed-Point Sin/Cos Polynomial (degree {degree})', fontsize=13)

    for ax, exact, approx, name in [
        (axes[0, 0], sin_exact, sin_approx, 'Sin'),
        (axes[0, 1], cos_exact, cos_approx, 'Cos'),
    ]:
        ax.plot(u, exact,  label='exact',  linewidth=2)
        ax.plot(u, approx, '--', label='approx', linewidth=1, alpha=0.8)
        ax.set_title(f'{name} Magnitude')
        ax.set_xlabel('u (radians)')
        ax.set_ylabel(f'value (Q{q})')
        ax.legend()
        ax.grid(True, alpha=0.4)

    for ax, exact, approx, name in [
        (axes[1, 0], sin_exact, sin_approx, 'Sin'),
        (axes[1, 1], cos_exact, cos_approx, 'Cos'),
    ]:
        err = approx - exact
        ax.plot(u, err, color='tab:red')
        ax.set_title(f'{name} Error (approx - exact)')
        ax.set_xlabel('u (radians)')
        ax.set_ylabel('error (LSB)')
        ax.axhline(0, color='k', linewidth=0.5)
        ax.grid(True, alpha=0.4)
        ax.set_ylim([-2, 2])

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Compute Q-scaled fixed-point sin/cos polynomial coefficients',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--q',       type=int, default=14,
                        help='Q factor: output is scaled by 2^q (default: 14)')
    parser.add_argument('--degree',  type=int, default=4,
                        help='Polynomial degree (default: 4)')
    parser.add_argument('--samples', type=int, default=5000,
                        help='Sample count for fit (default: 5000)')
    parser.add_argument('--method', choices=['minimax_fp', 'polyfit'], default='minimax_fp',
                        help='Fit method: minimax_fp (default) or polyfit')
    parser.add_argument('--plot',    action='store_true',
                        help='Show magnitude and error plots')
    parser.add_argument('--compare', action='store_true',
                        help='Compare output against existing hardcoded coefficients')
    args = parser.parse_args()

    sin_coeffs, cos_coeffs = get_trig_coefficients(args.q, args.degree, args.samples, args.method)
    print_c_output(args.q, args.degree, sin_coeffs, cos_coeffs)

    if args.compare:
        compare_against_existing(args.q, args.degree, sin_coeffs, cos_coeffs)

    if args.plot:
        plot_errors(args.q, args.degree, sin_coeffs, cos_coeffs)


if __name__ == '__main__':
    main()
