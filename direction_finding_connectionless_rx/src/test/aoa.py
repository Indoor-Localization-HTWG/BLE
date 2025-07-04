import numpy as np

def estimate_aoa_with_reference(iq_list, fc, dt, d,
                                sample_rate=1e6, n_ref=8, c=3e8):
    """
    Estimate AoA from nRF CTE IQ samples, accounting for the 8-sample reference period.

    Parameters
    ----------
    iq_list : list of (I, Q)
        Full sequence: first n_ref are reference samples, then alternating Ant0/Ant1.
    fc : float
        Carrier frequency (Hz).
    dt : float
        Antenna‐switch delay (s).
    d : float
        Antenna spacing (m).
    sample_rate : float
        IQ sampling rate (Hz). Default 1e6 for nRF.
    n_ref : int
        Number of initial reference samples. Default 8.
    c : float
        Speed of light (m/s).

    Returns
    -------
    aoas : np.ndarray
        Per‐pair AoA estimates in degrees.
    aoa_mean : float
        Mean AoA over all pairs.
    cfo_est : float
        Estimated CFO in Hz (for debug / further use).
    """
    # 1) convert all to complex
    cplx = np.array([i + 1j*q for (i, q) in iq_list], dtype=np.complex128)
    
    # 2) split reference vs switching
    c_ref = cplx[:n_ref]
    c_sw  = cplx[n_ref:]
    
    # 3) calibration: remove DC/gain+phase offset
    #    divide by mean of reference samples
    calib = np.mean(c_ref)
    c_sw  = c_sw / calib
    
    # 4) CFO estimation from ref period (linear phase slope)
    Ts      = 1.0 / sample_rate
    if len(c_ref) > 1:
        # total phase drift over reference
        Δφ_ref = np.angle(c_ref[-1] / c_ref[0])
        total_t = (n_ref - 1) * Ts
        cfo_est = Δφ_ref / (2*np.pi * total_t)  # in Hz
        # remove CFO ramp from switching samples
        # time of each switching sample measured from end of reference
        t_sw = np.arange(len(c_sw)) * Ts
        c_sw *= np.exp(-1j * 2*np.pi * cfo_est * (t_sw + total_t))
    else:
        cfo_est = 0.0
    
    # 5) compute the phase advance due to the 2 µs antenna‐switch
    φ_sw = 2*np.pi * fc * dt
    φ_sw = (φ_sw + np.pi) % (2*np.pi) - np.pi

    # 6) pair up Ant0/Ant1 and do the Δφ→θ
    lam = c / fc
    aoas = []
    for k in range(len(c_sw)//2):
        c0 = c_sw[2*k]
        c1 = c_sw[2*k + 1]
        dφ_raw = np.angle(c1) - np.angle(c0)
        dφ = (dφ_raw - φ_sw + np.pi) % (2*np.pi) - np.pi
        sinθ = (dφ * lam) / (2*np.pi * d)
        sinθ = np.clip(sinθ, -1.0, 1.0)
        θ = np.degrees(np.arcsin(sinθ))
        aoas.append(θ)
    
    aoas = np.array(aoas)
    return aoas, float(np.mean(aoas)), float(cfo_est)


# ─── example ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    iq = [(-39,-34),(40,38),(-43,-33),(39,29),(-47,-29),(45,26),
          (-49,-26),(54,-1),(12,0),(-51,-11),(8,0),(-54,1),
          (13,-8),(-53,20),(13,-8),(-45,33),(6,-12),(-27,45),
          (0,-12),(-16,44),(0,-12),(-7,51),(-5,-13),(12,52),
          (-9,-13),(24,49)]
    fc  = 2.402e9   # BLE ch37
    dt  = 2e-6      # 2 µs sw delay
    d   = 0.05      # 5 cm ant spacing

    aoas, mean_aoa, cfo = estimate_aoa_with_reference(iq, fc, dt, d)
    print("Per‐pair AoAs (°):", aoas)
    print("Mean AoA      (°):", mean_aoa)
    print("Estimated CFO (Hz):", cfo)