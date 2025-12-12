# Drone Localization via TDOA Triangulation: Improvements & Physics-Mathematics Advice

## Executive Summary

Your system uses **TDOA (Time Difference of Arrival)** with **GCC-PHAT** for 3D triangulation. Current issues: high residual errors (500-1300m), poor height estimation, and low confidence scores. This document provides physics-based improvements and realistic recommendations.

---

## 1. PHYSICS & MATHEMATICS FUNDAMENTALS

### 1.1 TDOA Accuracy Limits

**Fundamental Equation:**
```
τ_ij = (d_i - d_j) / c
where:
  τ_ij = TDOA between sensors i and j (seconds)
  d_i = distance from source to sensor i (meters)
  c = speed of sound (m/s)
```

**Key Physics Constraints:**

1. **TDOA Resolution Limit:**
   - At 16 kHz sampling: theoretical resolution = 1/16000 = 62.5 μs
   - With GCC-PHAT interpolation (16x): ~3.9 μs
   - Distance error per μs: c × 1μs = 343 × 10⁻⁶ = 0.343 mm per sensor pair
   - **BUT**: For a source at 500m with 100m sensor baseline:
     - Angular error: δθ ≈ (c × δτ) / baseline = (343 × 3.9μs) / 100m ≈ 0.013° 
     - Distance error: δd ≈ d² × δθ / baseline ≈ 500² × 0.013° / 100 ≈ **32.5m**
   - **Your 500-1300m residuals suggest TDOA errors of 1.5-4ms, which is 100-1000x worse than theoretical!**

2. **Far-Field vs Near-Field:**
   - **Far-field**: distance >> baseline (your case: 500m >> 100m)
   - Distance accuracy degrades as **d²/baseline**
   - Height accuracy is **worst** in far-field (sensors are nearly coplanar)
   - **Recommendation**: Use sensors with maximum vertical separation for height accuracy

3. **Geometric Dilution of Precision (GDOP):**
   - Poor sensor geometry → large position errors even with good TDOA
   - **Optimal**: Sensors form a tetrahedron (3D spread)
   - **Your case**: Sensors likely in a plane → poor height resolution
   - **GDOP formula**: σ_position = GDOP × σ_TDOA × c
   - For planar arrays, GDOP_z (height) can be 10-100× worse than GDOP_xy

### 1.2 Speed of Sound Corrections

**Current**: Fixed at 343 m/s (20°C, sea level)

**Reality**: Speed of sound varies with:
```
c(T, h) = 331.3 × √(1 + T/273.15) - 0.006 × h/1000
where:
  T = temperature (°C)
  h = altitude (meters)
```

**At 500m altitude, 25°C:**
- c ≈ 346.5 m/s (1% error = 3.5m error per 100m distance)
- **For 500m source**: ~17.5m systematic error

**Recommendations:**
1. Measure local temperature and altitude
2. Use weather station data or estimate from sensor altitude
3. Consider wind effects (adds ~5-10 m/s effective speed variation)
4. **Quick fix**: c = 331.3 + 0.606 × T(°C) - 0.006 × altitude(m)/1000

---

## 2. SENSOR GEOMETRY OPTIMIZATION

### 2.1 Current Issues

- **15 sensors in likely planar arrangement** → poor height resolution
- **Baseline ~100m** → good for horizontal, poor for vertical
- **No vertical diversity** → height estimation fundamentally limited

### 2.2 Recommendations

1. **Use Sensors with Maximum Vertical Spread:**
   - If sensors are at different elevations, use that information
   - Height accuracy ≈ baseline_vertical / baseline_horizontal × horizontal_accuracy
   - **If vertical baseline = 5m, horizontal = 100m**: height error ≈ 20× horizontal error

2. **Sensor Selection for Triangulation:**
   - Don't use all 16 sensors (redundant, adds noise)
   - **Optimal**: 4-6 well-spaced sensors forming a tetrahedron
   - Select sensors based on:
     - Maximum baseline (for distance accuracy)
     - Maximum vertical spread (for height accuracy)
     - Strong signal strength (for TDOA quality)
   - **Algorithm**: Select top 4-6 sensors by signal strength × geometric spread

3. **Weighted Triangulation:**
   - Weight sensor pairs by:
     - Signal quality (SNR)
     - Baseline length (longer = better)
     - Geometric diversity (prefer non-coplanar sensors)

---

## 3. TDOA ACCURACY IMPROVEMENTS

### 3.1 GCC-PHAT Enhancements

**Current Issues:**
- No frequency weighting (all frequencies treated equally)
- No noise filtering
- No outlier rejection

**Improvements:**

1. **Frequency-Weighted GCC-PHAT:**
   ```python
   # Weight by signal-to-noise ratio in each frequency band
   # Drone signals are strongest at 400-3000 Hz
   weights = np.abs(fft1) * np.abs(fft2)  # SNR weighting
   gcc_phat = cross_psd / (magnitude + noise_floor)  # Instead of just magnitude
   ```

2. **Sub-sample Peak Interpolation:**
   ```python
   # Current: integer peak index
   # Better: Parabolic interpolation around peak
   peak_idx = np.argmax(np.abs(correlation))
   if peak_idx > 0 and peak_idx < len(correlation) - 1:
       y1, y2, y3 = correlation[peak_idx-1], correlation[peak_idx], correlation[peak_idx+1]
       # Parabolic fit: find true peak
       offset = 0.5 * (y1 - y3) / (y1 - 2*y2 + y3)
       delay = (peak_idx + offset) / sample_rate
   ```

3. **Multi-Band GCC-PHAT:**
   - Compute TDOA in multiple frequency bands
   - Weight by band energy
   - Average or use robust statistics (median)

4. **Outlier Rejection:**
   - Compute TDOA for all pairs
   - Reject pairs with:
     - Low correlation quality (< threshold)
     - Physically impossible delays (|τ| > max_baseline/c)
     - Inconsistent with other pairs (statistical outlier)

### 3.2 Signal Preprocessing

**Current**: Bandpass filter 400-3000 Hz

**Improvements:**

1. **Adaptive Filtering:**
   - Use notch filters to remove known interference (power lines, etc.)
   - Dynamic bandpass based on detected fundamental frequency

2. **Wind Noise Reduction:**
   - High-pass filter > 100 Hz (wind noise is low-frequency)
   - Use spectral subtraction for stationary noise

3. **Coherence Filtering:**
   - Only use frequency bins where signals are coherent across sensors
   - Coherence = |cross_spectrum|² / (power1 × power2)
   - Threshold: coherence > 0.5

---

## 4. ALGORITHM IMPROVEMENTS

### 4.1 Triangulation Solver Enhancements

**Current**: Least squares with multiple initial guesses

**Issues:**
- No constraints (allows negative heights, unrealistic positions)
- No uncertainty quantification
- Poor handling of outliers

**Improvements:**

1. **Constrained Optimization:**
   ```python
   # Add bounds to optimization
   bounds = (
       (x_min, x_max),  # Based on sensor network extent
       (y_min, y_max),
       (50, 1000)       # Height: 50-1000m (reasonable for drones)
   )
   result = least_squares(residual_function, initial_guess, 
                         bounds=bounds, method='trf')
   ```

2. **Robust Statistics:**
   - Use Huber loss instead of L2 (less sensitive to outliers)
   - Or use RANSAC: randomly sample sensor pairs, find consensus

3. **Uncertainty Estimation:**
   ```python
   # Compute covariance matrix from Jacobian
   J = result.jac  # Jacobian matrix
   cov = np.linalg.inv(J.T @ J) * (residual_error**2)
   position_uncertainty = np.sqrt(np.diag(cov))  # 1-sigma uncertainty
   ```

4. **Multi-Hypothesis Tracking:**
   - Maintain multiple position hypotheses
   - Update with Kalman filter or particle filter
   - Use temporal consistency (drone can't jump 1km in 10 seconds)

### 4.2 Height Estimation Specific Improvements

**Problem**: Height is hardest to estimate (sensors are coplanar)

**Solutions:**

1. **Height Constraint from Expected Position:**
   - If you know drone is at 450-500m, use that as strong prior
   - Bayesian approach: P(position | TDOA, prior) ∝ P(TDOA | position) × P(position | prior)

2. **Multi-Frame Fusion:**
   - Average height over multiple time windows
   - Use temporal smoothing (drone height changes slowly)

3. **Geometric Height Estimation:**
   - If horizontal position is accurate, use elevation angle from strongest sensor
   - height ≈ horizontal_distance × tan(elevation_angle)
   - Less accurate but more stable than pure TDOA

---

## 5. CALIBRATION & ERROR SOURCES

### 5.1 Sensor Position Calibration

**Critical**: Sensor positions must be accurate to < 1m for 500m source localization

**Recommendations:**
1. **GPS Accuracy**: Standard GPS = 3-5m error → **Use RTK GPS** (cm-level)
2. **Coordinate System**: Ensure all sensors in same reference frame
3. **Height Calibration**: Measure sensor heights relative to ground (not GPS altitude)
4. **Regular Re-calibration**: Check sensor positions periodically

### 5.2 Clock Synchronization

**Critical**: All sensors must be synchronized to < 1ms for 500m accuracy

**Current**: Assumes perfect synchronization (may not be true!)

**Recommendations:**
1. **Hardware Sync**: Use GPS-disciplined clocks or PTP (Precision Time Protocol)
2. **Software Sync**: Cross-correlate known reference signals
3. **Drift Correction**: Estimate and correct clock drift over time

### 5.3 Sensor Response Calibration

**Issues:**
- Different microphones have different frequency responses
- Different gains/amplifications
- Phase delays in electronics

**Solutions:**
1. **Calibration Signals**: Play known signals, measure response
2. **Equalization**: Apply inverse filter to normalize responses
3. **Gain Normalization**: Normalize by RMS energy per sensor

---

## 6. PRACTICAL IMPLEMENTATION RECOMMENDATIONS

### 6.1 Immediate Improvements (Easy Wins)

1. **Speed of Sound Correction:**
   ```python
   # Add to config
   TEMPERATURE_C = 25.0  # Measure this!
   ALTITUDE_M = 500.0    # Sensor altitude
   speed_of_sound = 331.3 + 0.606 * TEMPERATURE_C - 0.006 * ALTITUDE_M / 1000
   ```

2. **Height Constraints:**
   ```python
   # In triangulation, add bounds
   bounds = ((x_min, x_max), (y_min, y_max), (400, 600))  # 400-600m height
   ```

3. **Sensor Selection:**
   - Use only top 6 sensors by signal strength
   - Prefer sensors with maximum baseline

4. **Outlier Rejection:**
   - Reject TDOA pairs with correlation quality < 0.3
   - Reject physically impossible delays

### 6.2 Medium-Term Improvements

1. **Sub-sample Peak Interpolation** (see Section 3.1)
2. **Frequency-Weighted GCC-PHAT** (see Section 3.1)
3. **Robust Statistics** (Huber loss, RANSAC)
4. **Multi-frame Temporal Smoothing**

### 6.3 Long-Term Improvements

1. **Kalman Filter Tracking:**
   - Model drone dynamics (velocity, acceleration)
   - Predict position, update with TDOA measurements
   - Handles missing/outlier measurements gracefully

2. **Machine Learning Enhancement:**
   - Train on known drone positions
   - Learn sensor-specific biases
   - Improve height estimation from horizontal position + signal features

3. **Multi-Modal Fusion:**
   - Combine TDOA with:
     - Signal strength (rough distance)
     - Frequency analysis (Doppler shift → velocity)
     - Visual tracking (if available)

---

## 7. EXPECTED ACCURACY & LIMITATIONS

### 7.1 Theoretical Limits

**Best Case (ideal conditions):**
- TDOA accuracy: 3.9 μs (with interpolation)
- Horizontal accuracy: ~10-20m at 500m distance
- Height accuracy: ~50-100m at 500m (with planar array)

**Realistic (your current system):**
- TDOA accuracy: ~1-4ms (from your residuals)
- Horizontal accuracy: ~50-200m at 500m distance
- Height accuracy: ~200-500m at 500m (very poor)

### 7.2 Why Height is So Hard

1. **Geometric**: Sensors are coplanar → height poorly constrained
2. **Far-field**: At 500m, small height changes → tiny TDOA differences
3. **Noise**: Height TDOA differences are small → easily corrupted by noise

**Example:**
- Source at (0, 0, 500m), sensors at (±50m, 0, 0m)
- Height change of 100m → TDOA change of only ~0.3ms
- With 1ms TDOA noise → height uncertainty ≈ 300m!

---

## 8. VALIDATION & TESTING

### 8.1 Ground Truth Validation

1. **Known Position Tests:**
   - Place speaker at known position (GPS + laser rangefinder)
   - Play drone-like sounds
   - Compare estimated vs. actual position

2. **Synthetic Data:**
   - Generate signals with known delays
   - Test triangulation algorithm
   - Verify accuracy

### 8.2 Performance Metrics

Track these metrics:
- **Position Error**: RMS distance from ground truth
- **Height Error**: RMS height difference
- **Residual Error**: TDOA fit quality
- **Confidence Calibration**: Does confidence=0.9 mean 90% within error bounds?

---

## 9. CODE-SPECIFIC RECOMMENDATIONS

### 9.1 Priority Fixes

1. **Add speed of sound correction** (temperature, altitude)
2. **Add height bounds** (400-600m for drones)
3. **Implement sub-sample peak interpolation** in GCC-PHAT
4. **Add outlier rejection** for TDOA pairs
5. **Select best sensors** (top 6 by signal strength × geometric spread)

### 9.2 Algorithm Improvements

1. **Constrained optimization** with bounds
2. **Robust statistics** (Huber loss or RANSAC)
3. **Temporal smoothing** (average over multiple frames)
4. **Uncertainty quantification** (covariance matrix)

### 9.3 Signal Processing

1. **Frequency-weighted GCC-PHAT**
2. **Coherence filtering**
3. **Multi-band TDOA** (compute in multiple frequency bands)

---

## 10. REALISTIC EXPECTATIONS

### What You Can Achieve:

- **Horizontal Position**: 20-50m accuracy at 500m (with improvements)
- **Height**: 50-100m accuracy IF you have vertical sensor diversity, otherwise 200-500m
- **Update Rate**: Real-time (every 10 seconds is fine)

### What You Cannot Achieve:

- **Sub-meter accuracy** at 500m with current sensor geometry
- **Reliable height** without vertical sensor diversity
- **Perfect accuracy** in noisy environments

---

## 11. QUICK REFERENCE: IMPLEMENTATION CHECKLIST

- [ ] Add temperature/altitude-based speed of sound correction
- [ ] Add height bounds (400-600m) to optimization
- [ ] Implement sub-sample peak interpolation
- [ ] Add TDOA outlier rejection (correlation quality, physical limits)
- [ ] Select top 6 sensors by signal strength × geometric spread
- [ ] Add constrained optimization with bounds
- [ ] Implement temporal smoothing (multi-frame averaging)
- [ ] Add uncertainty quantification (covariance matrix)
- [ ] Validate with known position tests
- [ ] Calibrate sensor positions (RTK GPS if possible)
- [ ] Verify clock synchronization (< 1ms)

---

## References & Further Reading

1. **TDOA Fundamentals**: "Time Difference of Arrival (TDOA) Localization" - academic papers
2. **GCC-PHAT**: Knapp & Carter, "The Generalized Correlation Method for Estimation of Time Delay"
3. **Sensor Array Processing**: Van Trees, "Optimal Array Processing"
4. **Geometric Dilution of Precision**: GPS literature (same concept applies)

---

**Last Updated**: Based on analysis of your current implementation
**Contact**: Review this document and prioritize improvements based on your constraints and requirements

