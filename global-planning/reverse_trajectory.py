#!/usr/bin/env python3
import numpy as np
import pandas as pd
import argparse

def normalize_angle_rad(angle):
    """Normalize angle to [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def reverse_trajectory(df):
    # Expected column names
    cols = {
        's': 's_m',
        'x': 'x_m',
        'y': 'y_m',
        'psi': 'psi_rad',
        'kappa': 'kappa_radpm',
        'vx': 'vx_mps',
        'ax': 'ax_mps2'
    }

    df_rev = df.iloc[::-1].reset_index(drop=True)

    # Recompute arc length from reversed coordinates
    dx = df_rev[cols['x']].diff().fillna(0.0)
    dy = df_rev[cols['y']].diff().fillna(0.0)
    df_rev[cols['s']] = np.sqrt(dx**2 + dy**2).cumsum()

    # Rotate heading by π and normalize
    df_rev[cols['psi']] = normalize_angle_rad(df_rev[cols['psi']] + np.pi)

    # Flip curvature sign (dψ/ds changes sign)
    df_rev[cols['kappa']] = -df_rev[cols['kappa']]

    return df_rev

def main():
    parser = argparse.ArgumentParser(description="Reverse a trajectory CSV file.")
    parser.add_argument("input", help="Input trajectory file (semicolon-separated CSV)")
    parser.add_argument("output", help="Output reversed trajectory file")
    args = parser.parse_args()

    df = pd.read_csv(args.input, sep=';')
    df = df.rename(columns=lambda c: c.strip())  # clean up spaces

    df_rev = reverse_trajectory(df)

    df_rev.to_csv(args.output, sep=';', index=False, float_format="%.7f")
    print(f"✅ Reversed trajectory saved to {args.output}")

if __name__ == "__main__":
    main()
