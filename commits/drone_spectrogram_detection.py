#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drone detection using spectrogram analysis on 10-second chunks.
Uses existing core algorithms: TriangulationEngine, sensor positions from KML.
Creates 2D and 3D maps with detections.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Ellipse, FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import soundfile as sf
from scipy.signal import spectrogram, find_peaks, butter, filtfilt
from scipy.stats import chi2
import os
import sys
import time
from src.kml_parser import get_sensor_positions_xyz
from src.triangulation import TriangulationEngine, TriangulationResult

# Fix Windows console encoding for emojis
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# ========================
# CONFIGURATIONS
# ========================
WAV_FILE = "multi-20251122-141610-627897594.wav"
KML_FILE = "Sensor-Locations-BOP-Dharma.kml"
# Use 10s chunks for faster processing and more stable TDOA estimates
CHUNK_DURATION = 10.0  # seconds per chunk
# Widen the fundamental band slightly to catch lower rotor tones
DRONE_FREQ_BAND = (100, 2000)  # Hz - fundamental frequency range (lowered to capture 100-600 Hz fundamentals)
FILTER_BAND = (100, 8000)  # Hz - bandpass filter (extended to capture low fundamentals and harmonics)
HARMONIC_SEARCH_BAND = (100, 2000)  # Hz - search for harmonics up to 2000 Hz (as observed in Audacity)
MIN_HARMONICS = 2  # Fundamental + 1 harmonic (balanced sensitivity)
ENERGY_THRESHOLD = 0.00002  # Lower threshold to catch valid signals
MIN_HARMONIC_RATIO = 0.5  # Slightly looser harmonic energy requirement
MIN_SNR_DB = 10.0  # Balanced SNR threshold for proper detections
HARMONIC_TOLERANCE = 0.08  # 8% tolerance for harmonic frequency matching (lenient enough)
CROP_START = 0.0   # Process from 0 seconds (start of file)
CROP_END = 145.0   # Process until 145 seconds (extended by 5 more seconds)
PROCESS_ALL_CHUNKS = False  # Only process chunks with detections
DEBUG_MODE = False  # Disable verbose debug logging for speed

# Speed of sound correction parameters
TEMPERATURE_C = 25.0  # Ambient temperature in Celsius (MEASURE THIS!)
ALTITUDE_M = 500.0    # Sensor altitude in meters (from KML or GPS)

# Height bounds used in triangulation.
# NOTE: The array in this experiment is almost planar (all sensors ~same Z),
# so TDOA provides weak constraints in height. Use realistic drone altitude bounds.
MIN_HEIGHT_M = 0.0      # Ground level
MAX_HEIGHT_M = 500.0    # Maximum altitude for consumer drones (DJI typically 500m max)
TYPICAL_DRONE_HEIGHT_M = 450.0  # Typical operating altitude for regularization (realistic for consumer drones) height

# TDOA quality thresholds
MIN_CORRELATION_QUALITY = 0.01  # Minimal threshold - accept any correlation above noise
MAX_SENSORS_TO_USE = 6  # Maximum number of sensors to use for triangulation

# ========================
# FUNCTIONS
# ========================

def apply_bandpass_filter(audio_data, sample_rate, low_freq=100.0, high_freq=5000.0):
    """Apply bandpass filter to enhance drone signal detection."""
    print(f"[FILTER] Applying bandpass filter ({low_freq}-{high_freq} Hz)...")
    sys.stdout.flush()
    
    nyquist = sample_rate / 2
    low = low_freq / nyquist
    high = high_freq / nyquist
    
    if low >= 1.0 or high >= 1.0:
        low = min(low, 0.9)
        high = min(high, 0.95)
    
    if low <= 0 or high <= low:
        low = 100.0 / nyquist
        high = 3000.0 / nyquist
    
    try:
        b, a = butter(2, [low, high], btype='band')
        filtered_data = np.zeros_like(audio_data)
        
        for ch in range(audio_data.shape[0]):
            try:
                filtered_data[ch] = filtfilt(b, a, audio_data[ch])
            except:
                filtered_data[ch] = audio_data[ch]
        
        print(f"   [OK] Filtering complete")
        sys.stdout.flush()
        return filtered_data
    except Exception as e:
        print(f"   [ERROR] Filter failed: {e}, using original")
        return audio_data

def detect_drone_spectrogram(signal, fs, min_fund_hz=100.0, max_fund_hz=2000.0, min_harmonics=2):
    """
    Detect drone-like signatures using spectrogram analysis with improved harmonic pattern matching.
    Requires actual harmonic series (fundamental + multiple harmonics) typical of drone rotors.
    Improved to detect fundamentals from 100-2000 Hz and harmonics up to 2000 Hz.
    
    Returns:
        (has_drone, fundamental_freq, confidence_score): Detection result
    """
    if signal.size < fs // 2:  # Need at least 0.5 second
        return False, 0.0, 0.0
    
    # Compute spectrogram with better frequency resolution for low frequencies
    # Use longer window for better low-frequency resolution
    nperseg = min(8192, max(4096, fs // 2))  # Increased for better low-freq resolution
    noverlap = nperseg // 2
    f, t, Sxx = spectrogram(signal, fs=fs, nperseg=nperseg, noverlap=noverlap, 
                            window='hann', scaling='density')
    
    # Convert to dB
    Sxx_db = 10.0 * np.log10(Sxx + 1e-12)
    
    # Average over time to get stable spectrum
    spectrum = np.mean(Sxx, axis=1)
    spectrum_db = 10.0 * np.log10(spectrum + 1e-12)
    
    # Calculate noise floor (use lower frequencies < 50 Hz and higher frequencies > 3000 Hz)
    noise_band_mask = (f < 50) | (f > 3000)
    if np.any(noise_band_mask):
        noise_floor_db = np.median(spectrum_db[noise_band_mask])
    else:
        noise_floor_db = np.median(spectrum_db)
    
    # Focus on fundamental frequency band for finding candidate fundamentals
    fund_band_mask = (f >= min_fund_hz) & (f <= max_fund_hz)
    if not np.any(fund_band_mask):
        return False, 0.0, 0.0
    
    f_fund_band = f[fund_band_mask]
    spec_fund_band_db = spectrum_db[fund_band_mask]
    
    # Extended range for harmonics: up to 2000 Hz (as observed in Audacity)
    harmonic_max_freq = min(2000.0, max_fund_hz * 10)  # Allow up to 10th harmonic or 2000 Hz
    extended_band_mask = (f >= min_fund_hz) & (f <= harmonic_max_freq)
    f_extended = f[extended_band_mask]
    spec_extended_db = spectrum_db[extended_band_mask]
    
    # Find peaks in the fundamental band with stricter criteria
    fund_band_max = np.max(spec_fund_band_db) if len(spec_fund_band_db) > 0 else noise_floor_db
    snr = fund_band_max - noise_floor_db
    
    # Require minimum SNR
    if snr < MIN_SNR_DB:
        return False, 0.0, 0.0
    
    # Peak detection with stricter height requirement in fundamental band
    min_peak_height = noise_floor_db + MIN_SNR_DB
    min_distance = int(fs / (nperseg * 2))  # Minimum distance between peaks (frequency resolution)
    peaks, peak_props = find_peaks(spec_fund_band_db, height=min_peak_height, distance=max(1, min_distance))
    
    if peaks.size < 2:  # Need at least 2 peaks to form a harmonic series
        return False, 0.0, 0.0
    
    peak_freqs = f_fund_band[peaks]
    peak_mags = spec_fund_band_db[peaks]
    
    # Check for harmonic patterns: try each peak as potential fundamental
    best_fund = 0.0
    best_harmonic_count = 0
    best_confidence = 0.0
    best_harmonic_energies = []
    
    for fund_idx, candidate_fund in enumerate(peak_freqs):
        # Calculate how many harmonics we can check up to 2000 Hz
        max_harmonic_num = int(harmonic_max_freq / candidate_fund)
        if max_harmonic_num < min_harmonics:
            continue  # Can't get enough harmonics
        
        harmonic_count = 1  # Count the fundamental itself
        harmonic_energies = [peak_mags[fund_idx]]
        harmonic_freqs = [candidate_fund]
        
        # Check for harmonics at 2x, 3x, 4x, ... up to 2000 Hz or 10th harmonic
        # Improved: allow some missing harmonics but require at least min_harmonics total
        consecutive_missing = 0
        max_consecutive_missing = 2  # Allow up to 2 consecutive missing harmonics
        
        for harmonic_num in range(2, min(max_harmonic_num + 1, 11)):  # Check up to 10th harmonic
            expected_harmonic_freq = candidate_fund * harmonic_num
            
            # Stop if we exceed harmonic search range (2000 Hz)
            if expected_harmonic_freq > harmonic_max_freq:
                break
            
            # Find closest frequency in spectrum
            freq_idx = np.argmin(np.abs(f - expected_harmonic_freq))
            actual_freq = f[freq_idx]
            
            # Improved tolerance: use percentage-based tolerance, but more lenient for higher harmonics
            # Higher harmonics may have slightly more frequency drift
            base_tolerance = candidate_fund * HARMONIC_TOLERANCE
            tolerance = base_tolerance * (1.0 + 0.1 * (harmonic_num - 2))  # Slightly more lenient for higher harmonics
            
            if abs(actual_freq - expected_harmonic_freq) > tolerance:
                consecutive_missing += 1
                if consecutive_missing > max_consecutive_missing:
                    # If we've missed too many consecutive harmonics, stop
                    # But don't break if we already have enough harmonics
                    if harmonic_count >= min_harmonics:
                        break  # We have enough, stop searching
                    else:
                        continue  # Keep searching for more harmonics
            
            # Check if there's significant energy at this frequency
            harmonic_energy_db = spectrum_db[freq_idx]
            
            # Harmonic should be at least MIN_SNR_DB above noise floor
            # More lenient for higher harmonics (they may be weaker)
            harmonic_min_height = min_peak_height - (harmonic_num - 2) * 2.0  # Allow 2 dB less per harmonic
            if harmonic_energy_db < harmonic_min_height:
                consecutive_missing += 1
                if consecutive_missing > max_consecutive_missing and harmonic_count >= min_harmonics:
                    break
                continue
            
            consecutive_missing = 0  # Reset counter when we find a harmonic
            
            # Check if this harmonic is stronger than nearby frequencies (local peak)
            freq_bin_width = f[1] - f[0] if len(f) > 1 else 1.0
            # Search range based on frequency resolution
            search_range = max(1, int(0.05 * expected_harmonic_freq / freq_bin_width))  # ±5% of frequency
            
            start_idx = max(0, freq_idx - search_range)
            end_idx = min(len(spectrum_db), freq_idx + search_range + 1)
            local_max = np.max(spectrum_db[start_idx:end_idx])
            
            # Harmonic should be close to local maximum (more lenient for higher harmonics)
            max_allowed_diff = 2.0 + (harmonic_num - 2) * 1.0  # Allow more tolerance for higher harmonics
            if harmonic_energy_db < (local_max - max_allowed_diff):
                consecutive_missing += 1
                if consecutive_missing > max_consecutive_missing and harmonic_count >= min_harmonics:
                    break
                continue
            
            harmonic_count += 1
            harmonic_energies.append(harmonic_energy_db)
            harmonic_freqs.append(actual_freq)
            
            # If we've found enough harmonics and we're past 2000 Hz, we can stop
            if harmonic_count >= min_harmonics and expected_harmonic_freq >= 2000.0:
                break
        
        # Require minimum number of harmonics (fundamental + at least min_harmonics-1 harmonics)
        if harmonic_count >= min_harmonics:
            # Calculate confidence based on:
            # 1. Number of harmonics found
            # 2. Strength of harmonics relative to noise
            # 3. Consistency of harmonic amplitudes
            
            avg_harmonic_energy = np.mean(harmonic_energies)
            harmonic_snr = avg_harmonic_energy - noise_floor_db
            
            # Improved confidence calculation
            # Bonus for harmonics reaching up to 2000 Hz
            harmonics_reach_2000hz = any(hf >= 1900.0 for hf in harmonic_freqs)
            reach_bonus = 0.1 if harmonics_reach_2000hz else 0.0
            
            # Confidence increases with more harmonics and better SNR
            confidence = min(1.0, (
                0.25 * (harmonic_count / 8.0) +  # Up to 25% for harmonic count (up to 8 harmonics)
                0.35 * min(1.0, harmonic_snr / 30.0) +  # Up to 35% for SNR
                0.25 * min(1.0, (harmonic_count - min_harmonics + 1) / 5.0) +  # Up to 25% for exceeding minimum
                0.05 * reach_bonus  # 5% bonus for reaching 2000 Hz
            ))
            
            if harmonic_count > best_harmonic_count or (harmonic_count == best_harmonic_count and confidence > best_confidence):
                best_fund = candidate_fund
                best_harmonic_count = harmonic_count
                best_confidence = confidence
                best_harmonic_energies = harmonic_energies
    
    # Only return detection if we found a proper harmonic series
    if best_harmonic_count >= min_harmonics and best_fund > 0.0:
        return True, best_fund, best_confidence
    
    return False, 0.0, 0.0

def combine_cluster_channels(audio_chunk):
    """
    Combine channels 16-20 into single cluster signal (sensor 16).
    Returns shape (16, samples): channels 0-14 are individual sensors, channel 15 is cluster.
    """
    if audio_chunk.shape[0] < 20:
        return audio_chunk
    
    result = np.zeros((16, audio_chunk.shape[1]))
    result[:15] = audio_chunk[:15]  # Individual sensors 1-15
    cluster_signal = np.mean(audio_chunk[15:20], axis=0)  # Average of channels 16-20
    result[15] = cluster_signal
    return result

def process_audio_detection(wav_path, kml_path):
    """
    Process audio file in 10-second chunks, detect drone sounds, and triangulate.
    """
    print("[DRONE DETECTION] Drone Detection using Spectrogram Analysis")
    print("=" * 70)
    
    # Load audio
    print(f"[LOAD] Loading: {wav_path}")
    sys.stdout.flush()
    audio_data, sample_rate = sf.read(wav_path, always_2d=True)
    audio_data = audio_data.T  # Shape: (channels, samples)
    
    duration_s = audio_data.shape[1] / sample_rate
    print(f"   [OK] Loaded: {audio_data.shape[0]} channels, {duration_s:.1f}s ({duration_s/60:.1f} min), {sample_rate}Hz")
    sys.stdout.flush()
    
    # Crop audio (handle None as "end of file")
    crop_start_sample = int(CROP_START * sample_rate)
    if CROP_END is None:
        crop_end_sample = audio_data.shape[1]
    else:
        crop_end_sample = int(CROP_END * sample_rate)
    crop_end_sample = min(crop_end_sample, audio_data.shape[1])
    
    if crop_start_sample < audio_data.shape[1]:
        audio_data = audio_data[:, crop_start_sample:crop_end_sample]
        cropped_duration = audio_data.shape[1] / sample_rate
        crop_end_display = f"{CROP_END:.0f}" if CROP_END is not None else "end"
        print(f"   [CROP] Cropped to {CROP_START:.0f}-{crop_end_display}s ({cropped_duration:.1f}s duration)")
        print(f"   [TARGET] Focusing on area where drone may appear (full file)")
        sys.stdout.flush()
    else:
        print(f"   [WARN] Crop start ({CROP_START}s) exceeds file duration, using full file")
        sys.stdout.flush()
    
    # Apply filter
    filtered_audio = apply_bandpass_filter(audio_data, sample_rate, 
                                          low_freq=FILTER_BAND[0], 
                                          high_freq=FILTER_BAND[1])
    
    # Load sensors
    print(f"[SENSORS] Loading sensors: {kml_path}")
    sys.stdout.flush()
    sensor_names, sensor_positions_array = get_sensor_positions_xyz(
        kml_path, add_opposite_sensors=True
    )
    print(f"   [OK] Loaded: {len(sensor_names)} sensors")
    sys.stdout.flush()
    
    # Calculate speed of sound with temperature and altitude correction
    # c(T, h) = 331.3 + 0.606*T - 0.006*h/1000
    speed_of_sound = 331.3 + 0.606 * TEMPERATURE_C - 0.006 * ALTITUDE_M / 1000.0
    print(f"[TRIANG] Initializing triangulation engine...")
    print(f"   [INFO] Speed of sound: {speed_of_sound:.2f} m/s (T={TEMPERATURE_C}°C, h={ALTITUDE_M}m)")
    sys.stdout.flush()
    
    # Create config - NO regularization, let TDOAs determine position naturally
    from src.triangulation import TriangulationConfig
    config = TriangulationConfig(
        temperature_c=TEMPERATURE_C,
        altitude_m=ALTITUDE_M,
        regularization_weight=0.0  # NO regularization - use only TDOA data
    )
    
    triangulation_engine = TriangulationEngine(
        sensor_positions=sensor_positions_array,
        speed_of_sound=speed_of_sound,
        config=config
    )
    
    # Find sensors 5 and 6 for initial guess (drone was observed at 500m above sensors 5-6)
    sensor_5_idx = None
    sensor_6_idx = None
    for i, name in enumerate(sensor_names):
        if "005" in name or "5" in name.split()[-1]:
            sensor_5_idx = i
        if "006" in name or "6" in name.split()[-1]:
            sensor_6_idx = i
    
    # Calculate midpoint between sensors 5 and 6 at 500m height
    if sensor_5_idx is not None and sensor_6_idx is not None:
        sensor_5_pos = sensor_positions_array[sensor_5_idx]
        sensor_6_pos = sensor_positions_array[sensor_6_idx]
        sensors_5_6_midpoint = (sensor_5_pos + sensor_6_pos) / 2.0
        expected_drone_position = sensors_5_6_midpoint.copy()
        expected_drone_position[2] = 500.0  # 500m above ground
        print(f"   [INFO] Expected drone position: above sensors 5-6 at 500m")
        print(f"   [INFO] Sensor 5 position: ({sensor_5_pos[0]:.1f}, {sensor_5_pos[1]:.1f}, {sensor_5_pos[2]:.1f})")
        print(f"   [INFO] Sensor 6 position: ({sensor_6_pos[0]:.1f}, {sensor_6_pos[1]:.1f}, {sensor_6_pos[2]:.1f})")
        print(f"   [INFO] Expected position: ({expected_drone_position[0]:.1f}, {expected_drone_position[1]:.1f}, {expected_drone_position[2]:.1f})")
        sys.stdout.flush()
    else:
        expected_drone_position = None
        print(f"   [WARN] Could not find sensors 5-6, using default initial guesses")
        sys.stdout.flush()
    
    # Process in 2-minute chunks
    chunk_samples = int(CHUNK_DURATION * sample_rate)
    total_samples = filtered_audio.shape[1]
    num_chunks = (total_samples + chunk_samples - 1) // chunk_samples
    
    print(f"\n[PROCESS] Processing: {num_chunks} chunk(s) of {CHUNK_DURATION/60:.1f} minutes each")
    print(f"   Fundamental search band: {DRONE_FREQ_BAND[0]}-{DRONE_FREQ_BAND[1]} Hz (captures 100-2000 Hz fundamentals)")
    print(f"   Harmonic search range: up to {HARMONIC_SEARCH_BAND[1]} Hz (as observed in Audacity)")
    print(f"   Min harmonics required: {MIN_HARMONICS} (fundamental + {MIN_HARMONICS-1} harmonics)")
    print(f"   Evaluating all channels (1-based indexing; no channel priority)")
    print(f"   Energy threshold: {ENERGY_THRESHOLD}")
    print(f"   Min SNR: {MIN_SNR_DB} dB")
    print(f"   Harmonic tolerance: {HARMONIC_TOLERANCE*100:.1f}%")
    print("")
    sys.stdout.flush()
    
    detections = []
    start_time = time.time()
    
    # Store expected position for use in triangulation
    expected_pos = expected_drone_position if 'expected_drone_position' in locals() else None
    
    # Temporal smoothing: maintain history of recent detections
    recent_detections = []  # For temporal smoothing
    MAX_HISTORY = 3  # Number of recent detections to consider
    
    # Store TDOA data for visualization
    tdoa_data = []  # Store TDOA matrices and quality metrics
    
    for chunk_idx in range(num_chunks):
        start_sample = chunk_idx * chunk_samples
        end_sample = min(start_sample + chunk_samples, total_samples)
        chunk = filtered_audio[:, start_sample:end_sample]
        
        chunk_duration_actual = (end_sample - start_sample) / sample_rate
        timestamp = start_sample / sample_rate
        
        # Calculate absolute timestamp (including crop offset)
        abs_timestamp = CROP_START + timestamp
        
        print(f"\n[CHUNK] Processing chunk {chunk_idx+1}/{num_chunks} ({chunk_duration_actual:.1f}s, abs_t={abs_timestamp:.1f}s = {abs_timestamp/60:.2f}min)...")
        sys.stdout.flush()
        
        # Combine cluster channels
        processed_chunk = combine_cluster_channels(chunk)
        
        # Check signal energy
        rms_levels = np.sqrt(np.mean(processed_chunk**2, axis=1))
        max_energy = np.max(rms_levels)
        active_channels = np.sum(rms_levels > ENERGY_THRESHOLD)
        
        print(f"   [ENERGY] Signal energy: {max_energy:.6f}, active channels: {active_channels}")
        if DEBUG_MODE:
            print(f"   [DEBUG] Channel energies: {[f'{e:.6f}' for e in rms_levels[:5]]}...")
        sys.stdout.flush()
        
        if max_energy < ENERGY_THRESHOLD:
            print(f"   [SKIP] Very low signal energy ({max_energy:.6f}) - skipping")
            sys.stdout.flush()
            continue
        
        # Very lenient: process if we have at least 1 active channel
        if active_channels < 1:
            print(f"   [SKIP] No active channels - skipping")
            sys.stdout.flush()
            continue
        
        # Evaluate all channels (1-based indexing) without prioritization
        has_drone = False
        fundamental = 0.0
        confidence = 0.0
        detection_channel = None
        
        max_channels = min(15, processed_chunk.shape[0])
        for ch_idx, ch_signal in enumerate(processed_chunk[:max_channels], start=1):
            has_drone, fundamental, confidence = detect_drone_spectrogram(
                ch_signal, sample_rate,
                DRONE_FREQ_BAND[0],
                HARMONIC_SEARCH_BAND[1],
                MIN_HARMONICS
            )
            if has_drone:
                detection_channel = ch_idx  # 1-based channel index
                print(f"   [DETECT] Drone detected on channel {detection_channel}")
                sys.stdout.flush()
                break
            elif DEBUG_MODE and ch_idx <= 5:
                print(f"   [DEBUG] Channel {ch_idx}: no detection")
        
        # If still not detected, try average of first 15 channels
        if not has_drone:
            avg_signal = np.mean(processed_chunk[:15], axis=0) if processed_chunk.shape[0] >= 15 else processed_chunk[4]
            has_drone, fundamental, confidence = detect_drone_spectrogram(
                avg_signal, sample_rate,
                DRONE_FREQ_BAND[0],
                HARMONIC_SEARCH_BAND[1],
                MIN_HARMONICS
            )
            if DEBUG_MODE and not has_drone:
                print(f"   [DEBUG] Average of 15 channels: no detection")
        
        # Last resort: cluster channel (channel 16-20 combined)
        if not has_drone:
            cluster_signal = processed_chunk[15] if processed_chunk.shape[0] >= 16 else processed_chunk[4]
            has_drone, fundamental, confidence = detect_drone_spectrogram(
                cluster_signal, sample_rate, 
                DRONE_FREQ_BAND[0], 
                HARMONIC_SEARCH_BAND[1],
                MIN_HARMONICS
            )
            if DEBUG_MODE and not has_drone:
                print(f"   [DEBUG] Cluster channel: no detection")
        
        # Only process chunks with actual drone harmonics detected
        if not has_drone:
            print(f"   [SKIP] No drone harmonic pattern detected (need {MIN_HARMONICS} harmonics with SNR>{MIN_SNR_DB}dB) - skipping chunk")
            sys.stdout.flush()
            continue
        
        print(f"   [DETECT] Drone detected! (fundamental={fundamental:.1f} Hz, confidence={confidence:.3f}, min {MIN_HARMONICS} harmonics found)")
        sys.stdout.flush()
        
        # ============================================================
        # STEP 1: Compute TDOAs explicitly using GCC-PHAT
        # ============================================================
        print(f"   [TDOA] Computing TDOAs using GCC-PHAT from actual audio data...")
        sys.stdout.flush()
        
        # Compute TDOA matrix with uncertainties
        tdoa_result = triangulation_engine.compute_tdoa_matrix(
            processed_chunk.T,  # Shape: (samples, channels)
            sample_rate,
            method='gcc_phat',
            frequency_band=DRONE_FREQ_BAND,
            min_correlation_quality=MIN_CORRELATION_QUALITY,
            return_uncertainties=True
        )
        
        if len(tdoa_result) == 3:
            tdoa_matrix, quality_matrix, uncertainty_matrix = tdoa_result
        else:
            tdoa_matrix, quality_matrix = tdoa_result
            uncertainty_matrix = None
        
        # Analyze TDOA results
        valid_tdoas = tdoa_matrix[tdoa_matrix != 0]
        valid_qualities = quality_matrix[quality_matrix > 0]
        
        if len(valid_tdoas) == 0:
            print(f"   [SKIP] No valid TDOAs found at quality>={MIN_CORRELATION_QUALITY} - skipping chunk")
            sys.stdout.flush()
            continue
        
        # Require a minimum number of valid TDOA pairs (lowered to allow processing)
        num_valid_pairs = len(valid_tdoas)
        MIN_TDOA_PAIRS = 1  # Very low threshold - allow processing with even 1 valid pair
        if num_valid_pairs < MIN_TDOA_PAIRS:
            print(f"   [SKIP] Not enough valid TDOA pairs ({num_valid_pairs}/{MIN_TDOA_PAIRS} required) - skipping triangulation")
            sys.stdout.flush()
            continue
        
        print(f"   [TDOA] Found {num_valid_pairs} valid TDOA pairs")
        # Compute physical TDOA limit based on sensor geometry
        # Max possible TDOA is max baseline / speed_of_sound
        sensor_distances = np.linalg.norm(
            sensor_positions_array[:, np.newaxis, :] - sensor_positions_array[np.newaxis, :, :],
            axis=2
        )
        max_baseline = np.max(sensor_distances)
        max_tdoa_phys = max_baseline / speed_of_sound
        
        # Reject physically impossible or very large TDOAs
        invalid_mask = np.abs(tdoa_matrix) > max_tdoa_phys
        if np.any(invalid_mask):
            tdoa_matrix[invalid_mask] = 0.0
            quality_matrix[invalid_mask] = 0.0
            if uncertainty_matrix is not None:
                uncertainty_matrix[invalid_mask] = np.inf
        
        # Recompute valid TDOAs after physical filtering
        valid_tdoas = tdoa_matrix[tdoa_matrix != 0]
        valid_qualities = quality_matrix[quality_matrix > 0]
        
        # Re-check minimum pairs after physical filtering
        if len(valid_tdoas) < MIN_TDOA_PAIRS:
            print(f"   [SKIP] Not enough valid TDOA pairs after filtering ({len(valid_tdoas)}/{MIN_TDOA_PAIRS} required) - skipping triangulation")
            sys.stdout.flush()
            continue
        
        print(f"   [TDOA] TDOA range: {np.min(valid_tdoas)*1000:.2f} to {np.max(valid_tdoas)*1000:.2f} ms")
        print(f"   [TDOA] Mean |TDOA|: {np.mean(np.abs(valid_tdoas))*1000:.2f} ms")
        print(f"   [TDOA] Quality range: {np.min(valid_qualities):.3f} to {np.max(valid_qualities):.3f}")
        print(f"   [TDOA] Mean quality: {np.mean(valid_qualities):.3f}")
        
        # Store TDOA data for visualization
        tdoa_data.append({
            'timestamp': abs_timestamp,
            'tdoa_matrix': tdoa_matrix.copy(),
            'quality_matrix': quality_matrix.copy(),
            'uncertainty_matrix': uncertainty_matrix.copy() if uncertainty_matrix is not None else None
        })
        
        # Show top TDOA pairs
        if DEBUG_MODE:
            # Find pairs with highest quality
            pair_qualities = []
            for i in range(tdoa_matrix.shape[0]):
                for j in range(i+1, tdoa_matrix.shape[1]):
                    if tdoa_matrix[i, j] != 0:
                        pair_qualities.append((i, j, tdoa_matrix[i, j], quality_matrix[i, j]))
            pair_qualities.sort(key=lambda x: x[3], reverse=True)
            print(f"   [DEBUG] Top 5 TDOA pairs:")
            for idx, (i, j, tdoa, qual) in enumerate(pair_qualities[:5]):
                print(f"      Pair {i}-{j}: {tdoa*1000:.2f} ms, quality={qual:.3f}")
        
        sys.stdout.flush()
        
        # Calculate bounds for optimization (based on sensor network extent)
        # Use wider bounds to allow TDOA-based positions, not just sensor extent
        sensor_centroid = np.mean(sensor_positions_array, axis=0)
        sensor_extent = np.max(np.abs(sensor_positions_array - sensor_centroid), axis=0)
        
        # Estimate source distance from TDOA magnitudes
        # Larger TDOAs suggest farther sources
        mean_tdoa_ms = np.mean(np.abs(valid_tdoas)) * 1000 if len(valid_tdoas) > 0 else 400
        # Rough distance estimate: TDOA ~ baseline / c, so distance ~ baseline * c / TDOA
        # For typical baselines ~100m and TDOA ~500ms, distance ~700m
        estimated_distance = mean_tdoa_ms * 0.7  # Rough scaling factor (meters per ms)
        estimated_distance = np.clip(estimated_distance, 300, 2000)  # Reasonable range
        
        # Use much wider bounds: allow positions up to estimated_distance from centroid
        # This prevents hitting bounds when TDOAs suggest positions outside sensor extent
        # No height bounds - use real triangulation values
        bounds = (
            np.array([sensor_centroid[0] - estimated_distance, 
                     sensor_centroid[1] - estimated_distance, 
                     -10000.0]),  # Very wide height range - no artificial bounds
            np.array([sensor_centroid[0] + estimated_distance, 
                     sensor_centroid[1] + estimated_distance, 
                     10000.0])   # Very wide height range - no artificial bounds
        )
        
        if DEBUG_MODE:
            print(f"   [DEBUG] Bounds: X=[{bounds[0][0]:.1f}, {bounds[1][0]:.1f}], "
                  f"Y=[{bounds[0][1]:.1f}, {bounds[1][1]:.1f}], "
                  f"Z=[{bounds[0][2]:.1f}, {bounds[1][2]:.1f}]")
            print(f"   [DEBUG] Estimated distance from TDOA: {estimated_distance:.1f}m")
        
        # ============================================================
        # STEP 2: Triangulate using computed TDOAs (NO hard-coded position)
        # ============================================================
        print(f"   [TRIANG] Triangulating from actual TDOA measurements...")
        sys.stdout.flush()
        
        # Compute signal strengths for sensor selection and weighting
        signal_strengths = np.sqrt(np.mean(processed_chunk**2, axis=1))
        
        # ============================================================
        # IMPROVED: Enhanced TDOA weighting based on signal quality
        # ============================================================
        # Improve uncertainty matrix with better weighting
        # Weight by: signal strength, baseline length, and correlation quality
        if uncertainty_matrix is not None:
            enhanced_uncertainty = uncertainty_matrix.copy()
        else:
            enhanced_uncertainty = np.full_like(tdoa_matrix, np.inf)
        
        # Compute baseline lengths for all pairs
        baseline_lengths = np.zeros_like(tdoa_matrix)
        for i in range(len(sensor_positions_array)):
            for j in range(i + 1, len(sensor_positions_array)):
                baseline = np.linalg.norm(sensor_positions_array[i] - sensor_positions_array[j])
                baseline_lengths[i, j] = baseline
                baseline_lengths[j, i] = baseline
        
        # Enhance uncertainty weighting
        for i in range(tdoa_matrix.shape[0]):
            for j in range(i + 1, tdoa_matrix.shape[1]):
                if tdoa_matrix[i, j] != 0 and quality_matrix[i, j] > 0:
                    # Signal strength weight (average of both sensors)
                    signal_weight = (signal_strengths[i] + signal_strengths[j]) / (2.0 * (np.max(signal_strengths) + 1e-10))
                    
                    # Baseline weight (longer baseline = better accuracy)
                    baseline = baseline_lengths[i, j]
                    baseline_weight = baseline / (max_baseline + 1e-10)
                    
                    # Quality weight (from correlation)
                    quality = quality_matrix[i, j]
                    
                    # Combined weight: prefer high signal, long baseline, high quality
                    combined_weight = (0.4 * signal_weight + 0.3 * baseline_weight + 0.3 * quality)
                    
                    # Reduce uncertainty for well-weighted pairs
                    if enhanced_uncertainty[i, j] < np.inf:
                        enhanced_uncertainty[i, j] = enhanced_uncertainty[i, j] / (combined_weight + 0.1)
                    else:
                        # If uncertainty was infinite, set based on quality
                        enhanced_uncertainty[i, j] = (1.0 - combined_weight) * 0.01  # 0.01s max uncertainty
                    
                    enhanced_uncertainty[j, i] = enhanced_uncertainty[i, j]
        
        if DEBUG_MODE:
            # Count well-weighted pairs
            well_weighted = np.sum((enhanced_uncertainty < np.inf) & (enhanced_uncertainty < 0.01))
            print(f"   [WEIGHTING] Enhanced {well_weighted} TDOA pairs with improved weighting")
        
        # ============================================================
        # IMPROVED: Outlier rejection for TDOA pairs
        # ============================================================
        # Reject TDOA pairs that are statistical outliers
        # This improves accuracy by removing bad measurements
        tdoa_values = []
        tdoa_pairs = []
        for i in range(tdoa_matrix.shape[0]):
            for j in range(i + 1, tdoa_matrix.shape[1]):
                if tdoa_matrix[i, j] != 0 and quality_matrix[i, j] > MIN_CORRELATION_QUALITY:
                    tdoa_values.append(abs(tdoa_matrix[i, j]))
                    tdoa_pairs.append((i, j))
        
        if len(tdoa_values) > 4:
            # Use IQR (Interquartile Range) method for outlier detection
            q1 = np.percentile(tdoa_values, 25)
            q3 = np.percentile(tdoa_values, 75)
            iqr = q3 - q1
            lower_bound = q1 - 1.5 * iqr
            upper_bound = q3 + 1.5 * iqr
            
            # Mark outliers (but don't remove them completely - just reduce weight)
            outlier_count = 0
            for idx, (i, j) in enumerate(tdoa_pairs):
                tdoa_val = abs(tdoa_matrix[i, j])
                if tdoa_val < lower_bound or tdoa_val > upper_bound:
                    # Reduce weight for outliers
                    if enhanced_uncertainty[i, j] < np.inf:
                        enhanced_uncertainty[i, j] = enhanced_uncertainty[i, j] * 5.0  # Increase uncertainty
                        enhanced_uncertainty[j, i] = enhanced_uncertainty[i, j]
                    outlier_count += 1
            
            if DEBUG_MODE and outlier_count > 0:
                print(f"   [OUTLIER] Identified {outlier_count} outlier TDOA pairs (reduced weight)")
        
        # ============================================================
        # IMPROVED: Better initial guess using multiple methods
        # ============================================================
        # Method 1: Temporal smoothing (if previous detections available)
        initial_guesses = []
        
        if detections and len(detections) > 0:
            # Use most recent detection as prior
            last_detection = detections[-1]
            last_pos = np.array(last_detection['position'])
            
            # Predict position based on velocity (if we have multiple detections)
            if len(detections) >= 2:
                prev_pos = np.array(detections[-2]['position'])
                dt = abs_timestamp - detections[-2]['timestamp']
                if dt > 0:
                    velocity = (last_pos - prev_pos) / dt
                    # Extrapolate position (with damping)
                    predicted_pos = last_pos + velocity * (abs_timestamp - last_detection['timestamp']) * 0.5
                    initial_guesses.append(predicted_pos)
                else:
                    initial_guesses.append(last_pos.copy())
            else:
                initial_guesses.append(last_pos.copy())
        
        # Method 2: TDOA-based rough estimate
        # Use TDOA patterns to estimate rough position
        if len(valid_tdoas) >= 3:
            # Find sensor pair with largest TDOA (likely closest to source)
            max_tdoa_idx = np.argmax([abs(tdoa_matrix[i, j]) 
                                      for i in range(tdoa_matrix.shape[0]) 
                                      for j in range(i+1, tdoa_matrix.shape[1]) 
                                      if tdoa_matrix[i, j] != 0])
            
            # Get the sensor pair
            pair_idx = 0
            best_i, best_j = 0, 1
            for i in range(tdoa_matrix.shape[0]):
                for j in range(i+1, tdoa_matrix.shape[1]):
                    if tdoa_matrix[i, j] != 0:
                        if pair_idx == max_tdoa_idx:
                            best_i, best_j = i, j
                            break
                        pair_idx += 1
                if pair_idx > max_tdoa_idx:
                    break
            
            # Estimate position near the sensor with earlier arrival
            if tdoa_matrix[best_i, best_j] > 0:
                # Sensor j receives signal first
                tdoa_est = sensor_positions_array[best_j].copy()
            else:
                # Sensor i receives signal first
                tdoa_est = sensor_positions_array[best_i].copy()
            
            # Estimate distance from TDOA magnitude
            tdoa_mag = abs(tdoa_matrix[best_i, best_j])
            est_dist = tdoa_mag * speed_of_sound * 0.5  # Rough estimate
            tdoa_est[2] = np.clip(est_dist * 0.5, 100.0, 600.0)  # Height estimate
            initial_guesses.append(tdoa_est)
        
        # Method 3: Sensor centroid with height from TDOA statistics
        centroid_guess = sensor_centroid.copy()
        if len(valid_tdoas) > 0:
            # Estimate height from TDOA statistics
            mean_tdoa = np.mean(np.abs(valid_tdoas))
            # Rough height estimate: larger TDOAs suggest farther/higher sources
            height_est = np.clip(mean_tdoa * speed_of_sound * 0.3, 100.0, 600.0)
            centroid_guess[2] = height_est
        else:
            centroid_guess[2] = 400.0
        initial_guesses.append(centroid_guess)
        
        # Select best initial guess (prefer temporal if available)
        if len(initial_guesses) > 0:
            initial_guess = initial_guesses[0]  # Prefer temporal
            # Ensure it's within bounds
            initial_guess = np.clip(initial_guess, bounds[0], bounds[1])
        else:
            initial_guess = sensor_centroid.copy()
            initial_guess[2] = 400.0
            initial_guess = np.clip(initial_guess, bounds[0], bounds[1])
        
        if DEBUG_MODE:
            print(f"   [DEBUG] Initial guess: ({initial_guess[0]:.1f}, {initial_guess[1]:.1f}, {initial_guess[2]:.1f})")
        
        try:
            # Use least_squares method with improved weighting
            # The TDOAs from the audio should determine the height naturally
            result = triangulation_engine.triangulate_least_squares(
                tdoa_matrix,
                tdoa_uncertainties=enhanced_uncertainty,
                initial_guess=initial_guess,
                bounds=bounds,
                use_huber_loss=True,  # Robust loss to reduce impact of outlier TDOAs
                position_prior=initial_guess if len(detections) > 0 else None  # Use temporal prior if available
            )
            
            # If least_squares fails or has high error, try robust method
            if result is None or result.residual_error > 1000.0:
                print(f"   [TRIANG] Least squares error high ({result.residual_error:.1f}m), trying robust method...")
                result = triangulation_engine.triangulate_robust(
                    tdoa_matrix,
                    tdoa_uncertainties=enhanced_uncertainty,
                    signal_strengths=signal_strengths,
                    expected_position=initial_guess if len(detections) > 0 else None,  # Use temporal prior
                    bounds=bounds
                )
            
            if not result:
                print(f"   [ERROR] Triangulation failed")
                sys.stdout.flush()
                continue
            
            # ============================================================
            # IMPROVED: Iterative refinement
            # ============================================================
            # Refine the result using the first result as new initial guess
            if result.residual_error > 50.0 and result.residual_error < 500.0:
                # Only refine if error is moderate (not too high, not too low)
                refined_result = triangulation_engine.triangulate_least_squares(
                    tdoa_matrix,
                    tdoa_uncertainties=enhanced_uncertainty,
                    initial_guess=result.position,  # Use previous result as starting point
                    bounds=bounds,
                    use_huber_loss=True,
                    position_prior=result.position  # Strong prior from previous result
                )
                
                if refined_result and refined_result.residual_error < result.residual_error:
                    result = refined_result
                    if DEBUG_MODE:
                        print(f"   [REFINE] Iterative refinement improved error: {result.residual_error:.2f}m")
            
            # GDOP gate: allow infinite GDOP for coplanar sensors (height from TDOA stats)
            # Very lenient - allow high GDOP to process all detections
            gdop = getattr(result, "gdop", None)
            if gdop is not None and not np.isinf(gdop) and gdop > 50.0:  # Very high threshold
                print(f"   [WARN] GDOP very high (gdop={gdop:.1f}) but processing anyway")
                sys.stdout.flush()
                # Don't skip - process anyway
            elif gdop is not None and np.isinf(gdop):
                # Infinite GDOP is expected for coplanar sensors - height is poorly constrained
                # Use TDOA measurements to solve for height more accurately
                print(f"   [INFO] Infinite GDOP (coplanar sensors) - solving for height from TDOA measurements")
                
                # Get the triangulated X, Y position
                drone_x = result.position[0]
                drone_y = result.position[1]
                
                # Solve for height using TDOA measurements
                # For each sensor pair: TDOA = (d_i - d_j) / c
                # Where d_i = sqrt((x_i - x)^2 + (y_i - y)^2 + (z_i - h)^2)
                # We know x, y from triangulation, so we can solve for h
                
                def height_residual(h):
                    """Compute TDOA residuals for a given height."""
                    residuals = []
                    # Ensure h is a scalar float
                    if isinstance(h, (np.ndarray, list)):
                        h = float(h[0] if len(h) > 0 else h)
                    else:
                        h = float(h)
                    
                    for i in range(len(sensor_positions_array)):
                        for j in range(i + 1, len(sensor_positions_array)):
                            if tdoa_matrix[i, j] is not None and not np.isnan(tdoa_matrix[i, j]):
                                # Distance from source to sensor i
                                dx_i = sensor_positions_array[i, 0] - drone_x
                                dy_i = sensor_positions_array[i, 1] - drone_y
                                dz_i = sensor_positions_array[i, 2] - h
                                dist_i = np.sqrt(dx_i**2 + dy_i**2 + dz_i**2)
                                
                                # Distance from source to sensor j
                                dx_j = sensor_positions_array[j, 0] - drone_x
                                dy_j = sensor_positions_array[j, 1] - drone_y
                                dz_j = sensor_positions_array[j, 2] - h
                                dist_j = np.sqrt(dx_j**2 + dy_j**2 + dz_j**2)
                                
                                # Predicted TDOA
                                predicted_tdoa = (dist_i - dist_j) / speed_of_sound
                                
                                # Observed TDOA
                                observed_tdoa = tdoa_matrix[i, j]
                                
                                # Residual (weighted by quality if available)
                                quality = uncertainty_matrix[i, j] if uncertainty_matrix[i, j] > 0 else 1.0
                                weight = 1.0 / (quality + 0.1)  # Higher quality = higher weight
                                residual = weight * (predicted_tdoa - observed_tdoa)
                                residuals.append(residual)
                    
                    return np.array(residuals)
                
                # Optimize height using least squares
                from scipy.optimize import least_squares
                
                # Reasonable height range for drones: 0-1000m (extended for accuracy)
                height_bounds = (0.0, 1000.0)
                
                # Initial height guess from triangulation result (clipped to reasonable range)
                initial_height = np.clip(result.position[2], height_bounds[0], height_bounds[1])
                
                try:
                    # Solve for height that minimizes TDOA residuals
                    height_result = least_squares(
                        height_residual,
                        x0=initial_height,
                        bounds=height_bounds,
                        method='trf',  # Trust Region Reflective algorithm
                        ftol=1e-6,
                        xtol=1e-6,
                        max_nfev=100
                    )
                    
                    if height_result.success:
                        optimized_height = float(height_result.x[0])
                        residual_norm = np.linalg.norm(height_result.fun)
                        
                        # Only use optimized height if residual is reasonable (stricter for accuracy)
                        if residual_norm < 5.0:  # Stricter threshold for better height accuracy
                            result.position[2] = optimized_height
                            print(f"   [HEIGHT] Optimized height: {optimized_height:.1f}m (residual: {residual_norm:.3f})")
                        else:
                            # If residual is too high, use a more conservative estimate
                            # Use median of height estimates from individual TDOA pairs
                            height_estimates = []
                            for i in range(len(sensor_positions_array)):
                                for j in range(i + 1, len(sensor_positions_array)):
                                    if tdoa_matrix[i, j] is not None and not np.isnan(tdoa_matrix[i, j]):
                                        # Estimate height from this TDOA pair
                                        horizontal_dist_i = np.sqrt(
                                            (sensor_positions_array[i, 0] - drone_x)**2 +
                                            (sensor_positions_array[i, 1] - drone_y)**2
                                        )
                                        horizontal_dist_j = np.sqrt(
                                            (sensor_positions_array[j, 0] - drone_x)**2 +
                                            (sensor_positions_array[j, 1] - drone_y)**2
                                        )
                                        # Use TDOA to estimate total distance difference
                                        tdoa_dist = abs(tdoa_matrix[i, j]) * speed_of_sound
                                        # Estimate height from geometry
                                        avg_horizontal = (horizontal_dist_i + horizontal_dist_j) / 2.0
                                        if tdoa_dist > avg_horizontal:
                                            h_est = np.sqrt(tdoa_dist**2 - avg_horizontal**2)
                                            if 0 < h_est < 1000:
                                                height_estimates.append(h_est)
                            
                            if len(height_estimates) > 0:
                                optimized_height = np.median(height_estimates)
                                result.position[2] = optimized_height
                                print(f"   [HEIGHT] Estimated from TDOA pairs: {optimized_height:.1f}m")
                            else:
                                # Fallback: use initial guess clipped to reasonable range
                                result.position[2] = np.clip(initial_height, 100.0, 500.0)
                                print(f"   [HEIGHT] Using clipped initial guess: {result.position[2]:.1f}m")
                    else:
                        # Optimization failed, try median estimate from TDOA pairs
                        height_estimates = []
                        for i in range(len(sensor_positions_array)):
                            for j in range(i + 1, len(sensor_positions_array)):
                                if tdoa_matrix[i, j] is not None and not np.isnan(tdoa_matrix[i, j]):
                                    # Estimate height from this TDOA pair
                                    horizontal_dist_i = np.sqrt(
                                        (sensor_positions_array[i, 0] - drone_x)**2 +
                                        (sensor_positions_array[i, 1] - drone_y)**2
                                    )
                                    horizontal_dist_j = np.sqrt(
                                        (sensor_positions_array[j, 0] - drone_x)**2 +
                                        (sensor_positions_array[j, 1] - drone_y)**2
                                    )
                                    # Use TDOA to estimate total distance difference
                                    tdoa_dist = abs(tdoa_matrix[i, j]) * speed_of_sound
                                    # Estimate height from geometry: h^2 = R^2 - d_h^2
                                    # For each sensor, estimate R from TDOA pattern
                                    if tdoa_dist > abs(horizontal_dist_i - horizontal_dist_j):
                                        # Try to solve for height using both sensors
                                        # Simplified: use average horizontal distance
                                        avg_horizontal = (horizontal_dist_i + horizontal_dist_j) / 2.0
                                        if tdoa_dist > avg_horizontal:
                                            h_est = np.sqrt(tdoa_dist**2 - avg_horizontal**2)
                                            if 0 < h_est < 1000:
                                                height_estimates.append(h_est)
                        
                        if len(height_estimates) > 0:
                            optimized_height = np.median(height_estimates)
                            result.position[2] = optimized_height
                            print(f"   [HEIGHT] Estimated from TDOA pairs (fallback): {optimized_height:.1f}m")
                        else:
                            # Final fallback: use initial guess clipped to reasonable range
                            result.position[2] = np.clip(initial_height, 100.0, 500.0)
                            print(f"   [HEIGHT] Using clipped initial guess: {result.position[2]:.1f}m")
                        
                except Exception as e:
                    # If optimization fails, use reasonable default
                    print(f"   [HEIGHT] Optimization error: {e}, using clipped value")
                    result.position[2] = np.clip(result.position[2], 100.0, 500.0)
            
            # Ensure height is reasonable for drones (0-1000m)
            if result.position[2] < 0 or result.position[2] > 1000:
                # Clip to reasonable range
                old_height = result.position[2]
                result.position[2] = np.clip(result.position[2], 0.0, 1000.0)
                if abs(old_height - result.position[2]) > 10:
                    print(f"   [HEIGHT] Clipped unrealistic height {old_height:.1f}m to {result.position[2]:.1f}m")
            
            # Validation
            sensor_centroid = np.mean(sensor_positions_array, axis=0)
            distance_from_centroid = np.linalg.norm(result.position - sensor_centroid)
            
            # VERY lenient validation - accept results based on detection confidence and reasonable position
            # Use detection confidence if triangulation confidence is too low
            effective_confidence = max(result.confidence, confidence * 0.1)  # Use detection confidence as fallback
            
            # Check if position is reasonable (not NaN/Inf and within reasonable bounds)
            # No height bounds - accept any real triangulation height value
            position_reasonable = (
                not np.any(np.isnan(result.position)) and
                not np.any(np.isinf(result.position)) and
                np.all(np.abs(result.position[:2]) < 100000.0)  # X, Y within 100km (very lenient)
                # Height: accept any real value from triangulation (no bounds)
            )
            
            # Very lenient validation - accept all reasonable detections
            is_valid = (
                position_reasonable and
                (result.confidence >= 0.0001 or confidence > 0.1) and  # Very low threshold
                result.residual_error < 5000.0 and  # Allow very high error
                distance_from_centroid < 50000.0  # Allow very far distances
            )
            
            if is_valid:
                # Use the position directly from TDOA optimization - NO post-processing
                # The TDOA-based optimization should determine height naturally
                # Only apply temporal smoothing to X, Y coordinates (height is already from TDOAs)
                smoothed_position = result.position.copy()
                
                if len(recent_detections) > 0:
                    # Weighted average with recent positions (exponential decay)
                    # Only smooth X, Y - keep Z (height) from current TDOA optimization
                    alpha = 0.7  # Weight for current measurement
                    recent_positions = np.array([d['position'] for d in recent_detections[-MAX_HISTORY:]])
                    weights = np.array([alpha * (0.3 ** i) for i in range(len(recent_positions))])
                    weights = weights / weights.sum()
                    # Smooth only X, Y coordinates
                    smoothed_xy = np.sum(recent_positions[:, :2] * weights[:, np.newaxis], axis=0) * (1 - alpha) + smoothed_position[:2] * alpha
                    smoothed_position[:2] = smoothed_xy
                    # Keep Z (height) from current TDOA optimization - don't smooth it
                    
                    # Update result with smoothed position
                    result.position = smoothed_position
                
                # Find nearest sensor
                distances_to_sensors = np.linalg.norm(
                    sensor_positions_array - result.position, axis=1
                )
                nearest_sensor_idx = np.argmin(distances_to_sensors)
                nearest_sensor_distance = distances_to_sensors[nearest_sensor_idx]
                
                # Identify sensors used for triangulation based on signal strength
                # Use sensors with significant signal contribution
                signal_strengths = np.sqrt(np.mean(processed_chunk**2, axis=1))
                signal_threshold = np.max(signal_strengths) * 0.1  # At least 10% of max signal
                sensors_used_indices = np.where(signal_strengths > signal_threshold)[0].tolist()
                
                # If we have num_sensors_used, use top N sensors by signal strength
                if result.num_sensors_used > 0:
                    top_sensor_indices = np.argsort(signal_strengths)[-result.num_sensors_used:]
                    sensors_used_indices = sorted(top_sensor_indices.tolist())
                
                # Ensure we have at least 3 sensors (minimum for triangulation)
                if len(sensors_used_indices) < 3:
                    top_sensor_indices = np.argsort(signal_strengths)[-min(4, len(signal_strengths)):]
                    sensors_used_indices = sorted(top_sensor_indices.tolist())
                
                detection = {
                    'timestamp': abs_timestamp,  # Use absolute timestamp
                    'position': result.position.copy(),
                    'height': result.position[2],  # Height (Z coordinate)
                    'confidence': effective_confidence,  # Use effective confidence
                    'residual_error': result.residual_error,
                    'method': result.method,
                    'num_sensors_used': result.num_sensors_used,
                    'sensors_used_indices': sensors_used_indices,  # Indices of sensors used
                    'fundamental_freq': fundamental,
                    'detection_confidence': confidence,
                    'nearest_sensor_idx': nearest_sensor_idx,
                    'nearest_sensor_name': sensor_names[nearest_sensor_idx],
                    'nearest_sensor_distance': nearest_sensor_distance
                }
                detections.append(detection)
                
                # Add to recent detections for temporal smoothing
                recent_detections.append(detection)
                if len(recent_detections) > MAX_HISTORY:
                    recent_detections.pop(0)
                
                print(f"      [OK] Triangulated: ({result.position[0]:6.1f}, {result.position[1]:6.1f}, {result.position[2]:6.1f}) "
                      f"conf={effective_confidence:.3f} (tri={result.confidence:.3f}, det={confidence:.3f}) "
                      f"error={result.residual_error:.1f}m nearest={sensor_names[nearest_sensor_idx]}")
                print(f"      [TDOA] Based on {len(valid_tdoas)} TDOA measurements from actual audio")
                if expected_pos is not None:
                    dist_from_expected = np.linalg.norm(result.position - expected_pos)
                    print(f"      [INFO] Distance from expected position: {dist_from_expected:.1f}m")
                sys.stdout.flush()
            else:
                print(f"   [WARN] Triangulation result invalid (conf={result.confidence:.3f}, det_conf={confidence:.3f}, "
                      f"error={result.residual_error:.1f}m, dist={distance_from_centroid:.1f}m)")
                sys.stdout.flush()
        
        except Exception as e:
            import traceback
            error_msg = str(e)
            print(f"   [ERROR] Triangulation error: {error_msg}")
            if DEBUG_MODE:
                print(f"   [DEBUG] Full traceback:")
                traceback.print_exc()
            sys.stdout.flush()
    
    elapsed_total = time.time() - start_time
    print(f"\n[COMPLETE] Analysis Complete!")
    print(f"   [TIME] Total time: {elapsed_total:.1f}s")
    print(f"   [STATS] Processed: {num_chunks} chunks")
    print(f"   [DETECTIONS] Valid detections: {len(detections)}")
    sys.stdout.flush()
    
    if detections:
        confidences = [d['confidence'] for d in detections]
        print(f"   [CONF] Confidence: avg={np.mean(confidences):.3f}, range={np.min(confidences):.3f}-{np.max(confidences):.3f}")
        sys.stdout.flush()
    
    return sensor_names, sensor_positions_array.tolist(), detections, tdoa_data

def create_uncertainty_ellipse(position, residual_error, confidence_level=0.95, num_sensors=6,
                              covariance_matrix=None, uncertainty_ellipsoid=None):
    """
    Calculate uncertainty ellipse parameters for 2D visualization.
    Enhanced to use actual covariance matrix if available.
    
    Args:
        position: 3D position
        residual_error: Residual error
        confidence_level: Confidence level (default 0.95)
        num_sensors: Number of sensors used
        covariance_matrix: Optional 3x3 covariance matrix from TriangulationResult
        uncertainty_ellipsoid: Optional ellipsoid dict from TriangulationResult
        
    Returns:
        (a, b, theta): Semi-major axis, semi-minor axis, rotation angle (radians)
    """
    # If we have actual covariance matrix, use it
    if covariance_matrix is not None and uncertainty_ellipsoid is not None:
        # Extract 2D covariance (x, y components)
        cov_2d = covariance_matrix[:2, :2]
        
        # Eigenvalue decomposition
        eigenvals, eigenvecs = np.linalg.eigh(cov_2d)
        idx = np.argsort(eigenvals)[::-1]
        eigenvals = eigenvals[idx]
        eigenvecs = eigenvecs[:, idx]
        
        # Chi-squared value for 2D (2 degrees of freedom)
        chi2_val = chi2.ppf(confidence_level, df=2)
        
        # Semi-axes
        a = np.sqrt(eigenvals[0] * chi2_val)
        b = np.sqrt(eigenvals[1] * chi2_val)
        
        # Rotation angle
        theta = np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0])
        
        return a, b, theta
    
    # Fallback: approximate uncertainty
    chi2_val = chi2.ppf(confidence_level, df=2)
    base_uncertainty = residual_error / np.sqrt(num_sensors) if num_sensors > 0 else residual_error
    a = base_uncertainty * np.sqrt(chi2_val)
    b = base_uncertainty * np.sqrt(chi2_val)
    theta = 0.0
    
    return a, b, theta

def create_2d_3d_maps(sensor_names, sensor_positions, detections, output_dir="plots"):
    """Create aesthetic 2D & 3D maps with dark background, round markers, minimal professional styling.
    Generates comprehensive PDF report with summary, statistics, and explanations (IEEE/academic ready)."""
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"\n[MAPS] Creating Professional PDF Report with Dark Theme Visualizations...")
    sys.stdout.flush()
    
    # Create PDF file (overwrites existing)
    pdf_path = os.path.join(output_dir, "drone_localization_report.pdf")
    
    # Use dark background for professional aesthetic
    plt.style.use('dark_background')
    
    # Ensure we use a non-interactive backend to avoid display issues
    import matplotlib
    matplotlib.use('Agg')  # Use non-interactive backend
    
    # Define dark theme color palette (professional, minimal)
    COLORS = {
        'bg': '#0a0a0a',           # Near-black background
        'grid': '#1a1a1a',         # Dark grid lines
        'text': '#e0e0e0',         # Light text
        'sensor': '#4A90E2',       # Blue for sensors
        'cluster': '#F5A623',      # Orange for cluster
        'detection': '#E94B3C',     # Red for detections
        'trajectory': '#50C878',   # Green for trajectory
        'uncertainty_high': '#2ECC71',  # Green for high confidence uncertainty
        'uncertainty_med': '#F39C12',   # Orange for medium confidence
        'uncertainty_low': '#E74C3C',   # Red for low confidence
        'accent': '#9B59B6',       # Purple accent
        'axis': '#666666'          # Gray for axes
    }
    
    # Delete or rename existing PDF if it exists to avoid corruption
    if os.path.exists(pdf_path):
        try:
            os.remove(pdf_path)
        except (PermissionError, OSError) as e:
            # If file is locked (open in another program), rename it
            try:
                backup_path = pdf_path.replace('.pdf', f'_backup_{int(time.time())}.pdf')
                os.rename(pdf_path, backup_path)
                print(f"   [INFO] Renamed locked PDF to: {backup_path}")
            except:
                # If rename also fails, create new file with timestamp
                pdf_path = pdf_path.replace('.pdf', f'_{int(time.time())}.pdf')
                print(f"   [INFO] Creating new PDF: {pdf_path}")
    
    try:
        with PdfPages(pdf_path) as pdf:
            # Set PDF metadata (IEEE/academic ready)
            d = pdf.infodict()
            d['Title'] = 'Drone Localization Analysis Report'
            d['Author'] = 'Acoustic Localization System'
            d['Subject'] = 'Time Difference of Arrival (TDOA) Based Drone Localization'
            d['Keywords'] = 'Drone Detection, TDOA, Triangulation, Acoustic Localization, Sensor Array'
            d['Creator'] = 'Drone Detection and Localization System'
            d['CreationDate'] = time.strftime('%Y-%m-%d %H:%M:%S')
            
            # Separate sensor types and sort consistently
            # Sort by index to ensure consistent ordering in both 2D and 3D views
            original_indices = sorted([i for i, name in enumerate(sensor_names) if "CLUSTER" not in name])
            cluster_indices = [i for i, name in enumerate(sensor_names) if "CLUSTER" in name]
            
            positions_arr = np.array(sensor_positions, dtype=float)
            
            # Extract sensor positions in sorted order (EXACT same order for 2D and 3D)
            # Store as lists to ensure consistent iteration order
            orig_x = [float(sensor_positions[i][0]) for i in original_indices]
            orig_y = [float(sensor_positions[i][1]) for i in original_indices]
            orig_z = [float(sensor_positions[i][2]) for i in original_indices]
            
            # Store sensor names in same order for reference
            orig_names = [sensor_names[i] for i in original_indices]
            
            cluster_pos = sensor_positions[cluster_indices[0]] if cluster_indices else None
        
            # Initialize variables
            sorted_indices = []
            det_x, det_y, det_z = [], [], []
            confidences, timestamps = [], []
            
            if detections:
                det_x = [d['position'][0] for d in detections]
                det_y = [d['position'][1] for d in detections]
                det_z = [d['position'][2] for d in detections]
                confidences = [d['confidence'] for d in detections]
                timestamps = [d['timestamp'] for d in detections]
                sorted_indices = np.argsort(timestamps)
                
                # Extract enhanced data if available
                gdop_values = [d.get('gdop', None) for d in detections]
                covariance_matrices = [d.get('covariance_matrix', None) for d in detections]
                uncertainty_ellipsoids = [d.get('uncertainty_ellipsoid', None) for d in detections]
                is_far_field = [d.get('is_far_field', None) for d in detections]
                estimated_distances = [d.get('estimated_distance', None) for d in detections]
            else:
                gdop_values = []
                covariance_matrices = []
                uncertainty_ellipsoids = []
                is_far_field = []
                estimated_distances = []
            
            # ============================================
            # PAGE 1: EXECUTIVE SUMMARY & STATISTICS
            # ============================================
            # REMOVED: First page (Executive Summary) - not clear, skipped
            # fig_summary = plt.figure(figsize=(11, 8.5), facecolor=COLORS['bg'])
            # ax_summary = fig_summary.add_subplot(111, facecolor=COLORS['bg'])
            # ax_summary.axis('off')
            # 
            # # Title
            # title_text = "DRONE LOCALIZATION ANALYSIS REPORT"
            # ax_summary.text(0.5, 0.95, title_text, transform=ax_summary.transAxes,
            #               fontsize=24, weight='bold', ha='center', color=COLORS['text'],
            #               family='sans-serif')
            # 
            # # Summary statistics
            # if detections:
            #     sorted_detections = sorted(detections, key=lambda d: d['timestamp'])
            #     positions_arr = np.array([d['position'] for d in sorted_detections])
            #     heights_arr = np.array([d['height'] for d in sorted_detections])
            #     errors_arr = np.array([d['residual_error'] for d in detections])
            #     conf_arr = np.array([d['confidence'] for d in detections])
            #     timestamps_arr = np.array([d['timestamp'] for d in sorted_detections])
            #     
            #     # Calculate statistics
            #     total_detections = len(detections)
            #     avg_height = np.mean(heights_arr)
            #     std_height = np.std(heights_arr)
            #     min_height, max_height = np.min(heights_arr), np.max(heights_arr)
            #     avg_error = np.mean(errors_arr)
            #     std_error = np.std(errors_arr)
            #     avg_confidence = np.mean(conf_arr)
            #     duration = timestamps_arr[-1] - timestamps_arr[0] if len(timestamps_arr) > 1 else 0
            #     
            #     # Position statistics
            #     x_range = np.max(positions_arr[:, 0]) - np.min(positions_arr[:, 0])
            #     y_range = np.max(positions_arr[:, 1]) - np.min(positions_arr[:, 1])
            #     avg_x = np.mean(positions_arr[:, 0])
            #     avg_y = np.mean(positions_arr[:, 1])
            #     
            #     summary_text = f"""
            # EXECUTIVE SUMMARY
            # 
            # Detection Statistics:
            #   • Total Detections: {total_detections}
            #   • Analysis Duration: {duration:.1f} seconds ({duration/60:.2f} minutes)
            #   • Detection Rate: {total_detections/max(duration, 1)*60:.2f} detections/minute
            # 
            # Position Statistics:
            #   • Average Position: ({avg_x:.1f}, {avg_y:.1f}) m
            #   • X Range: {x_range:.1f} m
            #   • Y Range: {y_range:.1f} m
            # 
            # Altitude Statistics:
            #   • Mean Altitude: {avg_height:.1f} m
            #   • Standard Deviation: {std_height:.1f} m
            #   • Range: {min_height:.1f} - {max_height:.1f} m
            # 
            # Quality Metrics:
            #   • Average Confidence: {avg_confidence:.3f}
            #   • Average Residual Error: {avg_error:.1f} m
            #   • Error Standard Deviation: {std_error:.1f} m
            # 
            # Sensor Configuration:
            #   • Active Sensors: {len(original_indices)}
            #   • Cluster Sensor: {'Present' if cluster_pos else 'Not Present'}
            # """
            # else:
            #     summary_text = """
            # EXECUTIVE SUMMARY
            # 
            # No detections found in the analyzed audio data.
            # 
            # Please verify:
            #   • Audio file contains drone signals
            #   • Frequency bands are correctly configured
            #   • Sensor positions are accurate
            # """
            # 
            # ax_summary.text(0.1, 0.85, summary_text, transform=ax_summary.transAxes,
            #               fontsize=11, va='top', ha='left', color=COLORS['text'],
            #               family='monospace', linespacing=1.5)
            # 
            # # Explanation section
            # explanation_text = """
            # METHODOLOGY
            # 
            # This report presents results from Time Difference of Arrival (TDOA) based acoustic
            # localization of drone signals. The system uses a distributed sensor array to detect
            # and triangulate drone positions in 3D space.
            # 
            # Key Techniques:
            #   1. Spectrogram Analysis: Harmonic detection in 100-2000 Hz band
            #   2. GCC-PHAT: Generalized Cross-Correlation with Phase Transform for TDOA estimation
            #   3. Weighted Least Squares: Robust triangulation with uncertainty weighting
            #   4. Uncertainty Estimation: Cramer-Rao Lower Bound (CRLB) based error analysis
            # 
            # The detected positions represent the estimated 3D location of the acoustic source
            # (drone) at each time instant, with uncertainty ellipses indicating the confidence
            # region for each detection.
            # """
            # 
            # ax_summary.text(0.1, 0.35, explanation_text, transform=ax_summary.transAxes,
            #               fontsize=10, va='top', ha='left', color=COLORS['text'],
            #               family='sans-serif', linespacing=1.4)
            # 
            # plt.tight_layout()
            # pdf.savefig(fig_summary, bbox_inches='tight', dpi=300, facecolor=COLORS['bg'])
            # plt.close(fig_summary)
            # print(f"   [PAGE 1] Executive Summary saved")
            # sys.stdout.flush()
            
            # ============================================
            # PAGE 2: 2D TOP VIEW - DARK THEME
            # ============================================
            fig_2d = plt.figure(figsize=(11, 8.5), facecolor=COLORS['bg'])
            ax1 = fig_2d.add_subplot(111, facecolor=COLORS['bg'])
            
            # Professional grid with dark theme
            ax1.grid(True, alpha=0.15, linestyle='-', linewidth=0.5, which='major', color=COLORS['grid'])
            ax1.grid(True, alpha=0.08, linestyle='--', linewidth=0.3, which='minor', color=COLORS['grid'])
            ax1.minorticks_on()
            ax1.set_axisbelow(True)
            
            # Plot sensors with round markers
            if original_indices:
                sensor_scatter = ax1.scatter(orig_x, orig_y, c=COLORS['sensor'], s=180, 
                           marker='o', label=f'Sensors ({len(original_indices)})', 
                           alpha=0.85, edgecolors='white', linewidth=1.5, zorder=4)
                # Add sensor labels
                for idx, (x, y) in enumerate(zip(orig_x, orig_y)):
                    ax1.annotate(f'S{original_indices[idx]+1}', (x, y), 
                               xytext=(6, 6), textcoords='offset points',
                               fontsize=8, weight='bold', color=COLORS['text'],
                               bbox=dict(boxstyle='round,pad=0.3', facecolor=COLORS['bg'], 
                                        edgecolor=COLORS['sensor'], alpha=0.9, linewidth=1),
                               zorder=6)
            
            # Plot cluster with round marker
            if cluster_pos is not None:
                ax1.scatter([cluster_pos[0]], [cluster_pos[1]], c=COLORS['cluster'], s=400, 
                           marker='o', label='Cluster Sensor', alpha=0.9,
                           edgecolors='white', linewidth=2, zorder=4)
                ax1.annotate('CLUSTER', (cluster_pos[0], cluster_pos[1]),
                           xytext=(10, 10), textcoords='offset points',
                           fontsize=9, weight='bold', color=COLORS['text'],
                           bbox=dict(boxstyle='round,pad=0.4', facecolor=COLORS['bg'],
                                    edgecolor=COLORS['cluster'], alpha=0.9, linewidth=1.5),
                           zorder=6)
            
            # Plot detections with uncertainty visualization (dark theme)
            if detections:
                # Plot uncertainty ellipses with dark theme colors
                for i, det in enumerate(detections):
                    x, y = det['position'][0], det['position'][1]
                    
                    # Use actual covariance if available
                    cov_matrix = covariance_matrices[i] if i < len(covariance_matrices) else None
                    ellipsoid = uncertainty_ellipsoids[i] if i < len(uncertainty_ellipsoids) else None
                    
                    a, b, theta = create_uncertainty_ellipse(
                        det['position'], det['residual_error'], 
                        confidence_level=0.95, num_sensors=det.get('num_sensors_used', 6),
                        covariance_matrix=cov_matrix, uncertainty_ellipsoid=ellipsoid
                    )
                    
                    # Show uncertainty ellipses with color coding based on confidence
                    confidence = det.get('confidence', 0.5)
                    if a > 10:  # Show ellipses for significant uncertainty
                        # Color based on confidence (dark theme)
                        if confidence > 0.7:
                            ellipse_color = COLORS['uncertainty_high']
                            alpha = 0.2
                        elif confidence > 0.4:
                            ellipse_color = COLORS['uncertainty_med']
                            alpha = 0.25
                        else:
                            ellipse_color = COLORS['uncertainty_low']
                            alpha = 0.3
                        
                        ellipse = Ellipse(xy=(x, y), width=2*a, height=2*b, angle=np.degrees(theta),
                                        alpha=alpha, facecolor=ellipse_color, 
                                        edgecolor=ellipse_color, linewidth=1, linestyle='--',
                                        zorder=1)
                        ax1.add_patch(ellipse)
                
                # Plot detections with round markers, size based on confidence
                sizes = [120 + conf * 80 for conf in confidences]  # Size: 120-200
                scatter2d = ax1.scatter(det_x, det_y, c=timestamps, s=sizes,
                                      cmap='plasma', alpha=0.85, edgecolors='white', 
                                      linewidth=1.5, marker='o',
                                      label=f'Detections ({len(detections)})', zorder=5)
                
                # Add height annotations for key detections (sparse)
                if len(detections) <= 25:  # Only annotate if not too many
                    for i, (x, y, z, conf) in enumerate(zip(det_x, det_y, det_z, confidences)):
                        if i % max(1, len(detections) // 8) == 0 or conf > 0.75:
                            ax1.annotate(f'{z:.0f}m', (x, y),
                                       xytext=(0, -18), textcoords='offset points',
                                       fontsize=7, weight='bold', color=COLORS['text'],
                                       ha='center',
                                       bbox=dict(boxstyle='round,pad=0.2', facecolor=COLORS['bg'],
                                                edgecolor=COLORS['detection'], alpha=0.9, linewidth=1),
                                       zorder=7)
            else:
                scatter2d = None
                gdop_values = []
                covariance_matrices = []
                uncertainty_ellipsoids = []
                is_far_field = []
                estimated_distances = []
            
            # Professional labels and title (dark theme)
            ax1.set_xlabel('X Position (m)', fontsize=12, weight='bold', labelpad=10, color=COLORS['text'])
            ax1.set_ylabel('Y Position (m)', fontsize=12, weight='bold', labelpad=10, color=COLORS['text'])
            ax1.set_title('2D Top View - Drone Localization', fontsize=16, pad=15, weight='bold', color=COLORS['text'])
            
            # Set axis colors for dark theme
            ax1.tick_params(colors=COLORS['text'], labelsize=10)
            ax1.spines['bottom'].set_color(COLORS['axis'])
            ax1.spines['top'].set_color(COLORS['axis'])
            ax1.spines['left'].set_color(COLORS['axis'])
            ax1.spines['right'].set_color(COLORS['axis'])
            
            # Professional legend (dark theme)
            legend_elements = []
            if original_indices:
                legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['sensor'], 
                                                 markersize=10, markeredgecolor='white', markeredgewidth=1.5,
                                                 label=f'Sensors ({len(original_indices)})'))
            if cluster_pos is not None:
                legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['cluster'], 
                                                 markersize=12, markeredgecolor='white', markeredgewidth=2,
                                                 label='Cluster Sensor'))
            if detections:
                legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['detection'], 
                                                 markersize=10, markeredgecolor='white', markeredgewidth=1.5,
                                                 label=f'Detections ({len(detections)})'))
                # Add uncertainty legend
                from matplotlib.patches import Patch
                legend_elements.append(Patch(facecolor=COLORS['uncertainty_high'], edgecolor=COLORS['uncertainty_high'], 
                                            alpha=0.3, label='Uncertainty (High)'))
                legend_elements.append(Patch(facecolor=COLORS['uncertainty_med'], edgecolor=COLORS['uncertainty_med'], 
                                            alpha=0.3, label='Uncertainty (Med)'))
            
            # Position legend with dark theme styling
            leg = ax1.legend(handles=legend_elements, loc='upper right', fontsize=9, 
                           framealpha=0.95, edgecolor=COLORS['axis'], fancybox=True, frameon=True,
                           shadow=False, borderpad=1.0, labelspacing=0.8)
            leg.get_frame().set_facecolor(COLORS['bg'])
            leg.get_frame().set_linewidth(1)
            for text in leg.get_texts():
                text.set_color(COLORS['text'])
            
            ax1.set_aspect('equal')
            
            # Add statistics text box (dark theme)
            if detections:
                avg_conf = np.mean(confidences)
                avg_error = np.mean([d['residual_error'] for d in detections])
                avg_gdop = np.mean([g for g in gdop_values if g is not None]) if any(gdop_values) else None
                
                stats_text = f'Detections: {len(detections)}\n'
                stats_text += f'Avg Confidence: {avg_conf:.2f}\n'
                stats_text += f'Avg Error: {avg_error:.1f} m'
                if avg_gdop is not None:
                    stats_text += f'\nAvg GDOP: {avg_gdop:.2f}'
                
                ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes,
                        fontsize=9, verticalalignment='top', color=COLORS['text'],
                        bbox=dict(boxstyle='round', facecolor=COLORS['bg'], alpha=0.9,
                                edgecolor=COLORS['axis'], linewidth=1),
                        family='monospace')
            
            if detections:
                all_x = orig_x + det_x
                all_y = orig_y + det_y
                margin = max(100, (max(all_x) - min(all_x)) * 0.12)
                ax1.set_xlim([min(all_x) - margin, max(all_x) + margin])
                ax1.set_ylim([min(all_y) - margin, max(all_y) + margin])
                
                # Professional colorbar (dark theme)
                if scatter2d is not None:
                    cbar = plt.colorbar(scatter2d, ax=ax1, label='Time (s)', shrink=0.7, pad=0.02, aspect=25)
                    cbar.ax.tick_params(labelsize=9, colors=COLORS['text'])
                    cbar.set_label('Time (s)', fontsize=10, weight='bold', color=COLORS['text'], labelpad=8)
                    cbar.outline.set_linewidth(1)
                    cbar.outline.set_edgecolor(COLORS['axis'])
            else:
                if original_indices:
                    margin = max(100, (max(orig_x) - min(orig_x)) * 0.15)
                    ax1.set_xlim([min(orig_x) - margin, max(orig_x) + margin])
                    ax1.set_ylim([min(orig_y) - margin, max(orig_y) + margin])
            
            plt.tight_layout()
            pdf.savefig(fig_2d, bbox_inches='tight', dpi=300, facecolor=COLORS['bg'])
            plt.close(fig_2d)
            print(f"   [PAGE 2] 2D Top View saved")
            sys.stdout.flush()
            
            # ============================================
            # PAGE 3: 3D VIEW - DARK THEME WITH ALTITUDE DOTS
            # ============================================
            fig_3d = plt.figure(figsize=(11, 8.5), facecolor=COLORS['bg'])
            ax2 = fig_3d.add_subplot(111, projection='3d')
            ax2.xaxis.pane.fill = False
            ax2.yaxis.pane.fill = False
            ax2.zaxis.pane.fill = False
            ax2.xaxis.pane.set_edgecolor(COLORS['grid'])
            ax2.yaxis.pane.set_edgecolor(COLORS['grid'])
            ax2.zaxis.pane.set_edgecolor(COLORS['grid'])
            ax2.xaxis.pane.set_alpha(0.1)
            ax2.yaxis.pane.set_alpha(0.1)
            ax2.zaxis.pane.set_alpha(0.1)
            ax2.set_facecolor(COLORS['bg'])
            
            # Plot sensors with round markers (dark theme)
            if original_indices:
                ax2.scatter(orig_x, orig_y, orig_z, c=COLORS['sensor'], s=120, marker='o',
                           label=f'Sensors ({len(original_indices)})',
                           alpha=0.8, edgecolors='white', linewidth=1.5, zorder=3)
                # Add sensor labels
                for idx in range(len(original_indices)):
                    x, y, z = orig_x[idx], orig_y[idx], orig_z[idx]
                    sensor_label = f' S{original_indices[idx]+1}'
                    ax2.text(x, y, z, sensor_label, fontsize=7, 
                           color=COLORS['text'], weight='bold', zorder=4)
            
            if cluster_pos is not None:
                ax2.scatter([cluster_pos[0]], [cluster_pos[1]], [cluster_pos[2]],
                           c=COLORS['cluster'], s=300, marker='o', label='Cluster Sensor',
                           alpha=0.85, edgecolors='white', linewidth=2, zorder=3)
                ax2.text(cluster_pos[0], cluster_pos[1], cluster_pos[2], ' CLUSTER',
                        fontsize=8, color=COLORS['text'], weight='bold', zorder=4)
            
            if detections:
                # Ground plane with dark theme
                x_range = max(det_x) - min(det_x) if det_x else 1000
                y_range = max(det_y) - min(det_y) if det_y else 1000
                x_center = (max(det_x) + min(det_x)) / 2 if det_x else 0
                y_center = (max(det_y) + min(det_y)) / 2 if det_y else 0
                
                # Create subtle ground plane
                xx = np.linspace(x_center - x_range*0.8, x_center + x_range*0.8, 15)
                yy = np.linspace(y_center - y_range*0.8, y_center + y_range*0.8, 15)
                XX, YY = np.meshgrid(xx, yy)
                ZZ = np.zeros_like(XX)
                ax2.plot_surface(XX, YY, ZZ, alpha=0.08, color=COLORS['grid'], zorder=0, 
                               linewidth=0.3, edgecolor=COLORS['grid'], antialiased=True)
                
                # Plot altitude dots: show altitude with dots at ground level
                # Create dots at ground (z=0) for each detection to show altitude reference
                ax2.scatter(det_x, det_y, [0]*len(det_x), c=det_z, s=40, 
                           cmap='viridis', alpha=0.4, marker='o', edgecolors='none',
                           zorder=1, label='Altitude Reference')
                
                # Draw vertical lines from ground to detection (altitude visualization)
                for i, det in enumerate(detections):
                    x, y, z = det['position'][0], det['position'][1], det['position'][2]
                    confidence = det.get('confidence', 0.5)
                    
                    # Draw vertical line from ground to detection
                    ax2.plot([x, x], [y, y], [0, z], color=COLORS['trajectory'], 
                           alpha=0.3, linewidth=1, linestyle='--', zorder=1)
                    
                    # Add height annotation (sparse)
                    if i % max(1, len(detections) // 10) == 0 or confidence > 0.75:
                        ax2.text(x, y, z + 25, f'{z:.0f}m', fontsize=7, 
                               color=COLORS['text'], weight='bold', ha='center',
                               bbox=dict(boxstyle='round,pad=0.2', facecolor=COLORS['bg'],
                                        edgecolor=COLORS['detection'], alpha=0.9, linewidth=1),
                               zorder=6)
                
                # Plot detections with round markers, size based on confidence
                sizes = [100 + conf * 60 for conf in confidences]  # Size: 100-160
                scatter3d = ax2.scatter(det_x, det_y, det_z, c=timestamps,
                                       s=sizes, cmap='plasma', alpha=0.85, edgecolors='white',
                                       linewidth=1.5, marker='o',
                                       label=f'Detections ({len(detections)})', zorder=5)
                
            else:
                scatter3d = None
            
            # Professional labels (dark theme)
            ax2.set_xlabel('X Position (m)', fontsize=11, weight='bold', labelpad=10, color=COLORS['text'])
            ax2.set_ylabel('Y Position (m)', fontsize=11, weight='bold', labelpad=10, color=COLORS['text'])
            ax2.set_zlabel('Altitude (m)', fontsize=11, weight='bold', labelpad=10, color=COLORS['text'])
            ax2.set_title('3D View - Drone Localization with Altitude', fontsize=14, pad=12, weight='bold', color=COLORS['text'])
            
            # Set tick colors
            ax2.tick_params(colors=COLORS['text'], labelsize=9)
            
            # Optimal viewing angle
            ax2.view_init(elev=25, azim=50)
            ax2.grid(True, alpha=0.15, linestyle='-', linewidth=0.4, color=COLORS['grid'])
            
            # Professional legend (dark theme)
            if detections or original_indices or cluster_pos:
                leg = ax2.legend(loc='upper left', fontsize=8, framealpha=0.95, 
                          edgecolor=COLORS['axis'], fancybox=True, shadow=False)
                leg.get_frame().set_facecolor(COLORS['bg'])
                leg.get_frame().set_linewidth(1)
                for text in leg.get_texts():
                    text.set_color(COLORS['text'])
            
            # Add altitude reference lines at key heights
            if detections and det_z:
                min_z, max_z = min(det_z), max(det_z)
                if max_z > 100:
                    # Add horizontal reference lines at key heights
                    for ref_z in [0, 200, 400, 600, 800]:
                        if ref_z <= max_z + 50:
                            # Create a small line segment for reference
                            x_lim = ax2.get_xlim()
                            y_lim = ax2.get_ylim()
                            ax2.plot([x_lim[0], x_lim[1]], [y_lim[0], y_lim[0]], [ref_z, ref_z], 
                                   color=COLORS['axis'], linestyle=':', alpha=0.2, linewidth=0.5)
            
            # Set equal aspect ratio for proper 3D visualization
            if detections:
                all_x = orig_x + det_x
                all_y = orig_y + det_y
                all_z = orig_z + det_z
                max_range = np.array([max(all_x) - min(all_x), max(all_y) - min(all_y), max(all_z) - min(all_z)]).max() / 2.0
                mid_x = (max(all_x) + min(all_x)) * 0.5
                mid_y = (max(all_y) + min(all_y)) * 0.5
                mid_z = (max(all_z) + min(all_z)) * 0.5
                ax2.set_xlim(mid_x - max_range, mid_x + max_range)
                ax2.set_ylim(mid_y - max_range, mid_y + max_range)
                ax2.set_zlim(max(0, mid_z - max_range), mid_z + max_range)
                
                # Professional colorbar (dark theme)
                if scatter3d is not None:
                    cbar = plt.colorbar(scatter3d, ax=ax2, label='Time (s)', shrink=0.6, pad=0.04, aspect=20)
                    cbar.ax.tick_params(labelsize=8, colors=COLORS['text'])
                    cbar.set_label('Time (s)', fontsize=9, weight='bold', color=COLORS['text'], labelpad=8)
                    cbar.outline.set_linewidth(1)
                    cbar.outline.set_edgecolor(COLORS['axis'])
                
                # Add statistics box (dark theme)
                avg_height = np.mean(det_z)
                height_std = np.std(det_z)
                stats_text = f'Avg Altitude: {avg_height:.0f} m\n'
                stats_text += f'Range: {min(det_z):.0f}-{max(det_z):.0f} m\n'
                stats_text += f'Std Dev: {height_std:.0f} m'
                
                ax2.text2D(0.02, 0.98, stats_text, transform=ax2.transAxes,
                         fontsize=8, verticalalignment='top', color=COLORS['text'],
                         bbox=dict(boxstyle='round', facecolor=COLORS['bg'], alpha=0.9,
                                 edgecolor=COLORS['axis'], linewidth=1),
                         family='monospace')
            
            plt.tight_layout()
            pdf.savefig(fig_3d, bbox_inches='tight', dpi=300, facecolor=COLORS['bg'])
            plt.close(fig_3d)
            print(f"   [PAGE 3] 3D View with Altitude Dots saved")
            sys.stdout.flush()
            
            # ============================================
            # PAGE 4+: Additional Analysis Plots (Dark Theme)
            # ============================================
            if detections:
                heights_arr = np.array([d['height'] for d in detections])
                errors_arr = np.array([d['residual_error'] for d in detections])
                conf_arr = np.array([d['confidence'] for d in detections])
                sorted_detections = sorted(detections, key=lambda d: d['timestamp'])
                timestamps_arr = np.array([d['timestamp'] for d in sorted_detections])
                positions_arr = np.array([d['position'] for d in sorted_detections])
                time_min = timestamps_arr / 60.0
                
                # Page 4: Time Series - Position (Dark Theme)
                fig_ts = plt.figure(figsize=(11, 8.5), facecolor=COLORS['bg'])
                ax_ts = fig_ts.add_subplot(111, facecolor=COLORS['bg'])
                
                # Smooth positions slightly for clearer trajectory lines
                def _smooth(arr, win=3):
                    if len(arr) < win:
                        return arr
                    kernel = np.ones(win) / win
                    pad = win // 2
                    padded = np.pad(arr, (pad, pad), mode='edge')
                    return np.convolve(padded, kernel, mode='valid')
                
                sm_x_ts = _smooth(positions_arr[:, 0])
                sm_y_ts = _smooth(positions_arr[:, 1])
                
                # Professional plots with dark theme
                ax_ts.plot(time_min, sm_x_ts, COLORS['sensor'], linewidth=2.5, marker='o', markersize=6,
                          markerfacecolor=COLORS['bg'], markeredgewidth=1.5, markeredgecolor=COLORS['sensor'],
                          label='X Position', zorder=3)
                ax_ts.plot(time_min, sm_y_ts, COLORS['cluster'], linewidth=2.5, marker='o', markersize=6,
                          markerfacecolor=COLORS['bg'], markeredgewidth=1.5, markeredgecolor=COLORS['cluster'],
                          label='Y Position', zorder=3)
                
                # Add shaded regions for variation (dark theme)
                if len(time_min) > 1:
                    ax_ts.fill_between(time_min, positions_arr[:, 0] - np.std(positions_arr[:, 0])*0.5,
                                     positions_arr[:, 0] + np.std(positions_arr[:, 0])*0.5,
                                     alpha=0.15, color=COLORS['sensor'], zorder=1)
                    ax_ts.fill_between(time_min, positions_arr[:, 1] - np.std(positions_arr[:, 1])*0.5,
                                     positions_arr[:, 1] + np.std(positions_arr[:, 1])*0.5,
                                     alpha=0.15, color=COLORS['cluster'], zorder=1)
                
                ax_ts.set_xlabel('Time (minutes)', fontsize=11, weight='bold', color=COLORS['text'])
                ax_ts.set_ylabel('Position (m)', fontsize=11, weight='bold', color=COLORS['text'])
                ax_ts.set_title('Position Over Time', fontsize=14, pad=12, weight='bold', color=COLORS['text'])
                ax_ts.grid(True, alpha=0.15, linestyle='-', linewidth=0.5, which='major', color=COLORS['grid'])
                ax_ts.grid(True, alpha=0.08, linestyle='--', linewidth=0.3, which='minor', color=COLORS['grid'])
                ax_ts.minorticks_on()
                ax_ts.tick_params(colors=COLORS['text'], labelsize=9)
                ax_ts.spines['bottom'].set_color(COLORS['axis'])
                ax_ts.spines['top'].set_color(COLORS['axis'])
                ax_ts.spines['left'].set_color(COLORS['axis'])
                ax_ts.spines['right'].set_color(COLORS['axis'])
                
                leg = ax_ts.legend(fontsize=9, framealpha=0.95, loc='best', shadow=False, 
                           edgecolor=COLORS['axis'], fancybox=True)
                leg.get_frame().set_facecolor(COLORS['bg'])
                leg.get_frame().set_linewidth(1)
                for text in leg.get_texts():
                    text.set_color(COLORS['text'])
                
                # Add statistics (dark theme)
                stats_text = f'X Range: {np.min(positions_arr[:, 0]):.1f} - {np.max(positions_arr[:, 0]):.1f} m\n'
                stats_text += f'Y Range: {np.min(positions_arr[:, 1]):.1f} - {np.max(positions_arr[:, 1]):.1f} m'
                ax_ts.text(0.98, 0.02, stats_text, transform=ax_ts.transAxes,
                         fontsize=8, verticalalignment='bottom', horizontalalignment='right', color=COLORS['text'],
                         bbox=dict(boxstyle='round', facecolor=COLORS['bg'], alpha=0.9,
                                 edgecolor=COLORS['axis'], linewidth=1),
                         family='monospace')
                
                plt.tight_layout()
                pdf.savefig(fig_ts, bbox_inches='tight', dpi=300, facecolor=COLORS['bg'])
                plt.close(fig_ts)
                print(f"   [PAGE 4] Time Series - Position saved")
                sys.stdout.flush()
                
                # Page 5: Time Series - Altitude (Dark Theme)
                fig_ht = plt.figure(figsize=(11, 8.5), facecolor=COLORS['bg'])
                ax_ht = fig_ht.add_subplot(111, facecolor=COLORS['bg'])
                
                # Professional altitude plot (dark theme)
                ax_ht.plot(time_min, heights_arr, COLORS['detection'], linewidth=2.5, marker='o', markersize=6,
                          markerfacecolor=COLORS['bg'], markeredgewidth=1.5, markeredgecolor=COLORS['detection'],
                          label='Estimated Altitude', zorder=3)
                
                # Expected range visualization (dark theme)
                ax_ht.axhline(y=450, color=COLORS['cluster'], linestyle='--', alpha=0.5, linewidth=2, 
                             label='Expected Range (450-500m)')
                ax_ht.axhline(y=500, color=COLORS['cluster'], linestyle='--', alpha=0.5, linewidth=2)
                ax_ht.fill_between([time_min[0], time_min[-1]], 450, 500, alpha=0.12, color=COLORS['cluster'],
                                  label='Expected Zone', zorder=1)
                
                # Add confidence-based error bars if available
                if len(heights_arr) == len(confidences):
                    uncertainties = [(1.0 - conf) * 50.0 for conf in confidences]
                    ax_ht.errorbar(time_min, heights_arr, yerr=uncertainties, fmt='none',
                                 ecolor=COLORS['detection'], alpha=0.25, capsize=2, capthick=1, zorder=2)
                
                ax_ht.set_xlabel('Time (minutes)', fontsize=11, weight='bold', color=COLORS['text'])
                ax_ht.set_ylabel('Altitude (m)', fontsize=11, weight='bold', color=COLORS['text'])
                ax_ht.set_title('Altitude Over Time', fontsize=14, pad=12, weight='bold', color=COLORS['text'])
                ax_ht.grid(True, alpha=0.15, linestyle='-', linewidth=0.5, which='major', color=COLORS['grid'])
                ax_ht.grid(True, alpha=0.08, linestyle='--', linewidth=0.3, which='minor', color=COLORS['grid'])
                ax_ht.minorticks_on()
                ax_ht.tick_params(colors=COLORS['text'], labelsize=9)
                ax_ht.spines['bottom'].set_color(COLORS['axis'])
                ax_ht.spines['top'].set_color(COLORS['axis'])
                ax_ht.spines['left'].set_color(COLORS['axis'])
                ax_ht.spines['right'].set_color(COLORS['axis'])
                
                leg = ax_ht.legend(fontsize=9, framealpha=0.95, loc='best', shadow=False,
                           edgecolor=COLORS['axis'], fancybox=True)
                leg.get_frame().set_facecolor(COLORS['bg'])
                leg.get_frame().set_linewidth(1)
                for text in leg.get_texts():
                    text.set_color(COLORS['text'])
                
                # Add statistics (dark theme)
                avg_h = np.mean(heights_arr)
                std_h = np.std(heights_arr)
                stats_text = f'Mean: {avg_h:.1f} m\nStd Dev: {std_h:.1f} m\n'
                stats_text += f'Range: {np.min(heights_arr):.1f}-{np.max(heights_arr):.1f} m'
                ax_ht.text(0.98, 0.02, stats_text, transform=ax_ht.transAxes,
                         fontsize=8, verticalalignment='bottom', horizontalalignment='right', color=COLORS['text'],
                         bbox=dict(boxstyle='round', facecolor=COLORS['bg'], alpha=0.9,
                                 edgecolor=COLORS['axis'], linewidth=1),
                         family='monospace')
                
                plt.tight_layout()
                pdf.savefig(fig_ht, bbox_inches='tight', dpi=300, facecolor=COLORS['bg'])
                plt.close(fig_ht)
                print(f"   [PAGE 5] Time Series - Altitude saved")
                sys.stdout.flush()
    
                # Page 6: Quality Metrics (Dark Theme with GDOP)
                fig_qual = plt.figure(figsize=(11, 8.5), facecolor=COLORS['bg'])
                ax_qual = fig_qual.add_subplot(111, facecolor=COLORS['bg'])
                
                # Confidence plot (dark theme)
                line1 = ax_qual.plot(time_min, conf_arr, COLORS['sensor'], linewidth=2.5, marker='o', markersize=6,
                            markerfacecolor=COLORS['bg'], markeredgewidth=1.5, markeredgecolor=COLORS['sensor'],
                            label='Confidence', zorder=3)
                
                # Fill area under confidence curve
                ax_qual.fill_between(time_min, 0, conf_arr, alpha=0.15, color=COLORS['sensor'], zorder=1)
                
                # Error plot on twin axis
                ax_qual_twin = ax_qual.twinx()
                line2 = ax_qual_twin.plot(time_min, errors_arr, COLORS['detection'], linewidth=2.5, marker='o', markersize=6,
                                 linestyle='--', markerfacecolor=COLORS['bg'], markeredgewidth=1.5,
                                 markeredgecolor=COLORS['detection'], label='Residual Error', zorder=3)
                
                # GDOP plot if available
                if any(gdop_values):
                    gdop_arr = np.array([g if g is not None else np.nan for g in gdop_values])
                    if not np.all(np.isnan(gdop_arr)):
                        ax_qual_twin2 = ax_qual.twinx()
                        ax_qual_twin2.spines['right'].set_position(('outward', 60))
                        line3 = ax_qual_twin2.plot(time_min, gdop_arr, COLORS['accent'], linewidth=2, 
                                                  marker='^', markersize=5, linestyle=':',
                                                  markerfacecolor=COLORS['bg'], markeredgewidth=1.5,
                                                  markeredgecolor=COLORS['accent'], label='GDOP', zorder=2)
                        ax_qual_twin2.set_ylabel('GDOP', fontsize=10, weight='bold', color=COLORS['accent'], labelpad=12)
                        ax_qual_twin2.tick_params(axis='y', labelcolor=COLORS['accent'], labelsize=8)
                        ax_qual_twin2.axhline(y=5.0, color=COLORS['accent'], linestyle=':', alpha=0.4, linewidth=1.5,
                                            label='GDOP Threshold (5.0)')
                
                ax_qual.set_ylabel('Confidence', fontsize=11, weight='bold', color=COLORS['sensor'], labelpad=10)
                ax_qual_twin.set_ylabel('Error (m)', fontsize=11, weight='bold', color=COLORS['detection'], labelpad=10)
                ax_qual.set_xlabel('Time (minutes)', fontsize=11, weight='bold', color=COLORS['text'])
                ax_qual.set_title('Quality Metrics Over Time', fontsize=14, pad=12, weight='bold', color=COLORS['text'])
                ax_qual.grid(True, alpha=0.15, linestyle='-', linewidth=0.5, which='major', color=COLORS['grid'])
                ax_qual.grid(True, alpha=0.08, linestyle='--', linewidth=0.3, which='minor', color=COLORS['grid'])
                ax_qual.minorticks_on()
                ax_qual.tick_params(axis='y', labelcolor=COLORS['sensor'], labelsize=9, colors=COLORS['text'])
                ax_qual.tick_params(axis='x', colors=COLORS['text'], labelsize=9)
                ax_qual_twin.tick_params(axis='y', labelcolor=COLORS['detection'], labelsize=9)
                ax_qual.set_ylim([0, 1.1])
                ax_qual.spines['bottom'].set_color(COLORS['axis'])
                ax_qual.spines['top'].set_color(COLORS['axis'])
                ax_qual.spines['left'].set_color(COLORS['axis'])
                ax_qual.spines['right'].set_color(COLORS['axis'])
                
                # Combined legend (dark theme)
                lines = line1 + line2
                if any(gdop_values) and not np.all(np.isnan(gdop_arr)):
                    lines += line3
                labels = [l.get_label() for l in lines]
                leg = ax_qual.legend(lines, labels, loc='upper left', fontsize=8, framealpha=0.95,
                             shadow=False, edgecolor=COLORS['axis'], fancybox=True)
                leg.get_frame().set_facecolor(COLORS['bg'])
                leg.get_frame().set_linewidth(1)
                for text in leg.get_texts():
                    text.set_color(COLORS['text'])
                
                # Add statistics (dark theme)
                stats_text = f'Avg Confidence: {np.mean(conf_arr):.3f}\n'
                stats_text += f'Avg Error: {np.mean(errors_arr):.1f} m\n'
                if any(gdop_values) and not np.all(np.isnan(gdop_arr)):
                    stats_text += f'Avg GDOP: {np.nanmean(gdop_arr):.2f}'
                ax_qual.text(0.98, 0.02, stats_text, transform=ax_qual.transAxes,
                           fontsize=8, verticalalignment='bottom', horizontalalignment='right', color=COLORS['text'],
                           bbox=dict(boxstyle='round', facecolor=COLORS['bg'], alpha=0.9,
                                   edgecolor=COLORS['axis'], linewidth=1),
                           family='monospace')
                
                plt.tight_layout()
                pdf.savefig(fig_qual, bbox_inches='tight', dpi=300, facecolor=COLORS['bg'])
                plt.close(fig_qual)
                print(f"   [PAGE 6] Quality Metrics saved")
                sys.stdout.flush()
                
                # Page 7: Frequency Analysis (Dark Theme)
                frequencies = np.array([d.get('fundamental_freq', 0.0) for d in sorted_detections])
                fig_freq = plt.figure(figsize=(11, 8.5), facecolor=COLORS['bg'])
                ax_freq = fig_freq.add_subplot(111, facecolor=COLORS['bg'])
                
                # Professional frequency plot (dark theme)
                ax_freq.plot(time_min, frequencies, COLORS['accent'], linewidth=2.5, marker='o', markersize=6,
                            markerfacecolor=COLORS['bg'], markeredgewidth=1.5, markeredgecolor=COLORS['accent'],
                            label='Fundamental Frequency', zorder=3)
                
                # Drone frequency band visualization (dark theme)
                ax_freq.axhline(y=400, color=COLORS['cluster'], linestyle='--', alpha=0.5, linewidth=2,
                              label='Drone Band (400-3000 Hz)')
                ax_freq.axhline(y=3000, color=COLORS['cluster'], linestyle='--', alpha=0.5, linewidth=2)
                ax_freq.fill_between([time_min[0], time_min[-1]], 400, 3000, alpha=0.12, color=COLORS['cluster'],
                                   zorder=1, label='Expected Range')
                
                # Add mean frequency line
                mean_freq = np.mean(frequencies)
                ax_freq.axhline(y=mean_freq, color=COLORS['accent'], linestyle=':', alpha=0.6, linewidth=1.5,
                              label=f'Mean: {mean_freq:.0f} Hz')
                
                ax_freq.set_xlabel('Time (minutes)', fontsize=11, weight='bold', color=COLORS['text'])
                ax_freq.set_ylabel('Frequency (Hz)', fontsize=11, weight='bold', color=COLORS['text'])
                ax_freq.set_title('Fundamental Frequency Over Time', fontsize=14, pad=12, weight='bold', color=COLORS['text'])
                ax_freq.grid(True, alpha=0.15, linestyle='-', linewidth=0.5, which='major', color=COLORS['grid'])
                ax_freq.grid(True, alpha=0.08, linestyle='--', linewidth=0.3, which='minor', color=COLORS['grid'])
                ax_freq.minorticks_on()
                ax_freq.tick_params(colors=COLORS['text'], labelsize=9)
                ax_freq.spines['bottom'].set_color(COLORS['axis'])
                ax_freq.spines['top'].set_color(COLORS['axis'])
                ax_freq.spines['left'].set_color(COLORS['axis'])
                ax_freq.spines['right'].set_color(COLORS['axis'])
                
                leg = ax_freq.legend(fontsize=8, framealpha=0.95, loc='best', shadow=False,
                             edgecolor=COLORS['axis'], fancybox=True)
                leg.get_frame().set_facecolor(COLORS['bg'])
                leg.get_frame().set_linewidth(1)
                for text in leg.get_texts():
                    text.set_color(COLORS['text'])
                
                # Add statistics (dark theme)
                stats_text = f'Mean: {mean_freq:.0f} Hz\n'
                stats_text += f'Range: {np.min(frequencies):.0f}-{np.max(frequencies):.0f} Hz\n'
                stats_text += f'Std Dev: {np.std(frequencies):.0f} Hz'
                ax_freq.text(0.98, 0.02, stats_text, transform=ax_freq.transAxes,
                           fontsize=8, verticalalignment='bottom', horizontalalignment='right', color=COLORS['text'],
                           bbox=dict(boxstyle='round', facecolor=COLORS['bg'], alpha=0.9,
                                   edgecolor=COLORS['axis'], linewidth=1),
                           family='monospace')
                
                plt.tight_layout()
                pdf.savefig(fig_freq, bbox_inches='tight', dpi=300, facecolor=COLORS['bg'])
                plt.close(fig_freq)
                print(f"   [PAGE 7] Frequency Analysis saved")
                sys.stdout.flush()
                
                # Page 8: Error Distribution (Dark Theme)
                fig_err = plt.figure(figsize=(11, 8.5), facecolor=COLORS['bg'])
                ax_err = fig_err.add_subplot(111, facecolor=COLORS['bg'])
                
                # Professional histogram (dark theme)
                n_bins = min(15, max(5, len(errors_arr) // 3))
                n, bins, patches = ax_err.hist(errors_arr, bins=n_bins, alpha=0.7, color=COLORS['detection'], 
                                              edgecolor=COLORS['axis'], linewidth=1, zorder=2)
                
                # Color bars by value (gradient for dark theme)
                for i, (bar, val) in enumerate(zip(patches, n)):
                    intensity = 0.5 + 0.5 * val / max(n) if max(n) > 0 else 0.5
                    bar.set_facecolor(plt.cm.Reds(intensity))
                
                # Mean and median lines (dark theme)
                mean_err = np.mean(errors_arr)
                median_err = np.median(errors_arr)
                std_err = np.std(errors_arr)
                
                ax_err.axvline(x=mean_err, color=COLORS['sensor'], linestyle='--', linewidth=2.5, 
                              label=f'Mean: {mean_err:.1f} m', zorder=3)
                ax_err.axvline(x=median_err, color=COLORS['cluster'], linestyle='--', linewidth=2.5,
                              label=f'Median: {median_err:.1f} m', zorder=3)
                
                # Add normal distribution overlay (dark theme)
                from scipy.stats import norm
                x_fit = np.linspace(errors_arr.min(), errors_arr.max(), 100)
                y_fit = norm.pdf(x_fit, mean_err, std_err) * len(errors_arr) * (bins[1] - bins[0])
                ax_err.plot(x_fit, y_fit, color=COLORS['text'], linewidth=2, alpha=0.5, label='Normal Fit', zorder=4)
                
                ax_err.set_xlabel('Residual Error (m)', fontsize=11, weight='bold', color=COLORS['text'])
                ax_err.set_ylabel('Count', fontsize=11, weight='bold', color=COLORS['text'])
                ax_err.set_title('Error Distribution', fontsize=14, pad=12, weight='bold', color=COLORS['text'])
                ax_err.grid(True, alpha=0.15, linestyle='-', linewidth=0.5, axis='y', which='major', color=COLORS['grid'])
                ax_err.grid(True, alpha=0.08, linestyle='--', linewidth=0.3, axis='y', which='minor', color=COLORS['grid'])
                ax_err.minorticks_on()
                ax_err.tick_params(colors=COLORS['text'], labelsize=9)
                ax_err.spines['bottom'].set_color(COLORS['axis'])
                ax_err.spines['top'].set_color(COLORS['axis'])
                ax_err.spines['left'].set_color(COLORS['axis'])
                ax_err.spines['right'].set_color(COLORS['axis'])
                
                leg = ax_err.legend(loc='best', fontsize=8, framealpha=0.95, shadow=False,
                            edgecolor=COLORS['axis'], fancybox=True)
                leg.get_frame().set_facecolor(COLORS['bg'])
                leg.get_frame().set_linewidth(1)
                for text in leg.get_texts():
                    text.set_color(COLORS['text'])
                
                # Add statistics (dark theme)
                stats_text = f'Mean: {mean_err:.2f} m\n'
                stats_text += f'Median: {median_err:.2f} m\n'
                stats_text += f'Std Dev: {std_err:.2f} m\n'
                stats_text += f'Min: {np.min(errors_arr):.2f} m\n'
                stats_text += f'Max: {np.max(errors_arr):.2f} m'
                ax_err.text(0.98, 0.98, stats_text, transform=ax_err.transAxes,
                          fontsize=8, verticalalignment='top', horizontalalignment='right', color=COLORS['text'],
                          bbox=dict(boxstyle='round', facecolor=COLORS['bg'], alpha=0.9,
                                  edgecolor=COLORS['axis'], linewidth=1),
                          family='monospace')
                
                plt.tight_layout()
                pdf.savefig(fig_err, bbox_inches='tight', dpi=300, facecolor=COLORS['bg'])
                plt.close(fig_err)
                print(f"   [PAGE 8] Error Distribution saved")
                sys.stdout.flush()
            
            # PDF is automatically closed when exiting the 'with' block
            # Force flush to ensure all data is written
            import gc
            gc.collect()
        
    except Exception as e:
        print(f"   [ERROR] Failed to create PDF: {e}")
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        # Try to remove corrupted file
        if os.path.exists(pdf_path):
            try:
                os.remove(pdf_path)
            except:
                pass
        raise
    
    # Verify PDF was created and is valid
    if not os.path.exists(pdf_path):
        raise FileNotFoundError(f"PDF file was not created: {pdf_path}")
    
    # Check file size
    file_size = os.path.getsize(pdf_path)
    if file_size < 1000:  # Less than 1KB is suspicious
        raise ValueError(f"PDF file is too small ({file_size} bytes), may be corrupted")
    
    print(f"\n   [SAVED] PDF Report: {pdf_path} ({file_size/1024:.2f} KB)")
    sys.stdout.flush()
    
    return pdf_path, pdf_path

def create_timeseries_plots(detections, output_dir="plots"):
    """Create time series plots: position, height, speed over time."""

def create_timeseries_plots(detections, output_dir="plots"):
    """Create time series plots: position, height, speed over time."""
    if not detections:
        return
    
    print(f"   [PLOT] Creating time series plots...")
    sys.stdout.flush()
    
    # Sort by timestamp
    sorted_detections = sorted(detections, key=lambda d: d['timestamp'])
    timestamps = np.array([d['timestamp'] for d in sorted_detections])
    positions = np.array([d['position'] for d in sorted_detections])
    heights = np.array([d['height'] for d in sorted_detections])
    confidences = np.array([d['confidence'] for d in sorted_detections])
    errors = np.array([d['residual_error'] for d in sorted_detections])
    
    # Calculate speed (m/s)
    speeds = np.zeros(len(sorted_detections))
    for i in range(len(sorted_detections) - 1):
        dt = timestamps[i+1] - timestamps[i]
        if dt > 0:
            dx = positions[i+1, 0] - positions[i, 0]
            dy = positions[i+1, 1] - positions[i, 1]
            dz = positions[i+1, 2] - positions[i, 2]
            speeds[i] = np.sqrt(dx**2 + dy**2 + dz**2) / dt
    speeds[-1] = speeds[-2] if len(speeds) > 1 else 0.0
    
    # Convert timestamps to minutes for readability
    time_min = timestamps / 60.0
    
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Drone Localization - Time Series Analysis', fontsize=16, weight='bold', y=0.995)
    
    # X Position
    axes[0].plot(time_min, positions[:, 0], 'b-', linewidth=2, marker='o', markersize=6, label='X Position')
    axes[0].fill_between(time_min, positions[:, 0] - errors, positions[:, 0] + errors, 
                        alpha=0.3, color='blue', label='Uncertainty')
    axes[0].set_ylabel('X Position (m)', fontsize=12, weight='bold')
    axes[0].grid(True, alpha=0.3, linestyle='--')
    axes[0].legend(loc='upper right', fontsize=10)
    
    # Y Position
    axes[1].plot(time_min, positions[:, 1], 'g-', linewidth=2, marker='o', markersize=6, label='Y Position')
    axes[1].fill_between(time_min, positions[:, 1] - errors, positions[:, 1] + errors, 
                        alpha=0.3, color='green', label='Uncertainty')
    axes[1].set_ylabel('Y Position (m)', fontsize=12, weight='bold')
    axes[1].grid(True, alpha=0.3, linestyle='--')
    axes[1].legend(loc='upper right', fontsize=10)
    
    # Height
    axes[2].plot(time_min, heights, 'r-', linewidth=2, marker='o', markersize=6, label='Height')
    axes[2].fill_between(time_min, heights - errors, heights + errors, 
                        alpha=0.3, color='red', label='Uncertainty')
    axes[2].axhline(y=450, color='orange', linestyle='--', alpha=0.5, label='Expected (450m)')
    axes[2].axhline(y=500, color='orange', linestyle='--', alpha=0.5, label='Expected (500m)')
    axes[2].set_ylabel('Height (m)', fontsize=12, weight='bold')
    axes[2].grid(True, alpha=0.3, linestyle='--')
    axes[2].legend(loc='upper right', fontsize=10)
    
    # Speed
    axes[3].plot(time_min, speeds, 'purple', linewidth=2, marker='o', markersize=6, label='Speed')
    axes[3].set_ylabel('Speed (m/s)', fontsize=12, weight='bold')
    axes[3].set_xlabel('Time (minutes)', fontsize=12, weight='bold')
    axes[3].grid(True, alpha=0.3, linestyle='--')
    axes[3].legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, "drone_detection_timeseries.png")
    fig.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"   [SAVED] Time Series: {output_file}")
    sys.stdout.flush()
    plt.close(fig)

def create_quality_metrics_plots(detections, output_dir="plots"):
    """Create quality metrics plots: confidence, error, sensors used over time."""
    if not detections:
        return
    
    print(f"   [PLOT] Creating quality metrics plots...")
    sys.stdout.flush()
    
    sorted_detections = sorted(detections, key=lambda d: d['timestamp'])
    timestamps = np.array([d['timestamp'] for d in sorted_detections])
    confidences = np.array([d['confidence'] for d in sorted_detections])
    errors = np.array([d['residual_error'] for d in sorted_detections])
    num_sensors = np.array([d.get('num_sensors_used', 6) for d in sorted_detections])
    detection_conf = np.array([d.get('detection_confidence', 0.0) for d in sorted_detections])
    
    time_min = timestamps / 60.0
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Drone Localization - Quality Metrics', fontsize=16, weight='bold', y=0.995)
    
    # Confidence
    axes[0, 0].plot(time_min, confidences, 'b-', linewidth=2, marker='o', markersize=6, label='Triangulation Confidence')
    axes[0, 0].plot(time_min, detection_conf, 'g--', linewidth=2, marker='s', markersize=5, label='Detection Confidence')
    axes[0, 0].axhline(y=0.5, color='orange', linestyle='--', alpha=0.5, label='Threshold (0.5)')
    axes[0, 0].set_ylabel('Confidence', fontsize=12, weight='bold')
    axes[0, 0].set_title('Confidence Over Time', fontsize=12, weight='bold')
    axes[0, 0].grid(True, alpha=0.3, linestyle='--')
    axes[0, 0].legend(loc='best', fontsize=9)
    axes[0, 0].set_ylim([0, 1.1])
    
    # Residual Error
    axes[0, 1].plot(time_min, errors, 'r-', linewidth=2, marker='o', markersize=6, label='Residual Error')
    axes[0, 1].axhline(y=200, color='orange', linestyle='--', alpha=0.5, label='Target (<200m)')
    axes[0, 1].set_ylabel('Residual Error (m)', fontsize=12, weight='bold')
    axes[0, 1].set_title('Triangulation Error Over Time', fontsize=12, weight='bold')
    axes[0, 1].grid(True, alpha=0.3, linestyle='--')
    axes[0, 1].legend(loc='best', fontsize=9)
    
    # Number of Sensors Used
    axes[1, 0].bar(time_min, num_sensors, width=0.5, alpha=0.7, color='purple', edgecolor='black', linewidth=1)
    axes[1, 0].axhline(y=4, color='orange', linestyle='--', alpha=0.5, label='Minimum (4)')
    axes[1, 0].set_ylabel('Number of Sensors', fontsize=12, weight='bold')
    axes[1, 0].set_xlabel('Time (minutes)', fontsize=12, weight='bold')
    axes[1, 0].set_title('Sensors Used for Triangulation', fontsize=12, weight='bold')
    axes[1, 0].grid(True, alpha=0.3, linestyle='--', axis='y')
    axes[1, 0].legend(loc='best', fontsize=9)
    axes[1, 0].set_ylim([0, max(16, max(num_sensors) + 2)])
    
    # Error vs Confidence scatter
    axes[1, 1].scatter(confidences, errors, c=time_min, cmap='viridis', s=100, alpha=0.7, edgecolors='black', linewidth=1)
    axes[1, 1].set_xlabel('Confidence', fontsize=12, weight='bold')
    axes[1, 1].set_ylabel('Residual Error (m)', fontsize=12, weight='bold')
    axes[1, 1].set_title('Error vs Confidence', fontsize=12, weight='bold')
    axes[1, 1].grid(True, alpha=0.3, linestyle='--')
    cbar = plt.colorbar(axes[1, 1].collections[0], ax=axes[1, 1], label='Time (min)')
    cbar.ax.tick_params(labelsize=9)
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, "drone_detection_quality_metrics.png")
    fig.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"   [SAVED] Quality Metrics: {output_file}")
    sys.stdout.flush()
    plt.close(fig)

def create_frequency_analysis_plots(detections, output_dir="plots"):
    """Create frequency analysis plots: fundamental frequency over time."""
    if not detections:
        return
    
    print(f"   [PLOT] Creating frequency analysis plots...")
    sys.stdout.flush()
    
    sorted_detections = sorted(detections, key=lambda d: d['timestamp'])
    timestamps = np.array([d['timestamp'] for d in sorted_detections])
    frequencies = np.array([d.get('fundamental_freq', 0.0) for d in sorted_detections])
    
    time_min = timestamps / 60.0
    
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('Drone Localization - Frequency Analysis', fontsize=16, weight='bold', y=0.995)
    
    # Fundamental Frequency over Time
    axes[0].plot(time_min, frequencies, 'b-', linewidth=2, marker='o', markersize=6, label='Fundamental Frequency')
    axes[0].axhline(y=400, color='orange', linestyle='--', alpha=0.5, label='Band: 400-3000 Hz')
    axes[0].axhline(y=3000, color='orange', linestyle='--', alpha=0.5)
    axes[0].fill_between([time_min[0], time_min[-1]], 400, 3000, alpha=0.1, color='orange')
    axes[0].set_ylabel('Frequency (Hz)', fontsize=12, weight='bold')
    axes[0].set_title('Detected Fundamental Frequency Over Time', fontsize=12, weight='bold')
    axes[0].grid(True, alpha=0.3, linestyle='--')
    axes[0].legend(loc='best', fontsize=10)
    
    # Frequency Histogram
    axes[1].hist(frequencies, bins=20, alpha=0.7, color='blue', edgecolor='black', linewidth=1)
    axes[1].axvline(x=400, color='orange', linestyle='--', alpha=0.5, label='Band: 400-3000 Hz')
    axes[1].axvline(x=3000, color='orange', linestyle='--', alpha=0.5)
    axes[1].set_xlabel('Frequency (Hz)', fontsize=12, weight='bold')
    axes[1].set_ylabel('Count', fontsize=12, weight='bold')
    axes[1].set_title('Frequency Distribution', fontsize=12, weight='bold')
    axes[1].grid(True, alpha=0.3, linestyle='--', axis='y')
    axes[1].legend(loc='best', fontsize=10)
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, "drone_detection_frequency_analysis.png")
    fig.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"   [SAVED] Frequency Analysis: {output_file}")
    sys.stdout.flush()
    plt.close(fig)

def create_error_analysis_plots(detections, output_dir="plots"):
    """Create error analysis plots: error distributions and correlations."""
    if not detections:
        return
    
    print(f"   [PLOT] Creating error analysis plots...")
    sys.stdout.flush()
    
    errors = np.array([d['residual_error'] for d in detections])
    confidences = np.array([d['confidence'] for d in detections])
    num_sensors = np.array([d.get('num_sensors_used', 6) for d in detections])
    heights = np.array([d['height'] for d in detections])
    
    # Calculate distances from sensor centroid (approximate)
    positions = np.array([d['position'] for d in detections])
    distances = np.linalg.norm(positions, axis=1)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Drone Localization - Error Analysis', fontsize=16, weight='bold', y=0.995)
    
    # Error Histogram
    axes[0, 0].hist(errors, bins=15, alpha=0.7, color='red', edgecolor='black', linewidth=1)
    axes[0, 0].axvline(x=np.mean(errors), color='blue', linestyle='--', linewidth=2, label=f'Mean: {np.mean(errors):.1f}m')
    axes[0, 0].axvline(x=np.median(errors), color='green', linestyle='--', linewidth=2, label=f'Median: {np.median(errors):.1f}m')
    axes[0, 0].set_xlabel('Residual Error (m)', fontsize=12, weight='bold')
    axes[0, 0].set_ylabel('Count', fontsize=12, weight='bold')
    axes[0, 0].set_title('Error Distribution', fontsize=12, weight='bold')
    axes[0, 0].grid(True, alpha=0.3, linestyle='--', axis='y')
    axes[0, 0].legend(loc='best', fontsize=9)
    
    # Error vs Number of Sensors
    axes[0, 1].scatter(num_sensors, errors, alpha=0.6, s=100, c='blue', edgecolors='black', linewidth=1)
    axes[0, 1].set_xlabel('Number of Sensors Used', fontsize=12, weight='bold')
    axes[0, 1].set_ylabel('Residual Error (m)', fontsize=12, weight='bold')
    axes[0, 1].set_title('Error vs Sensor Count', fontsize=12, weight='bold')
    axes[0, 1].grid(True, alpha=0.3, linestyle='--')
    
    # Error vs Distance
    axes[1, 0].scatter(distances, errors, alpha=0.6, s=100, c='green', edgecolors='black', linewidth=1)
    axes[1, 0].set_xlabel('Distance from Origin (m)', fontsize=12, weight='bold')
    axes[1, 0].set_ylabel('Residual Error (m)', fontsize=12, weight='bold')
    axes[1, 0].set_title('Error vs Distance', fontsize=12, weight='bold')
    axes[1, 0].grid(True, alpha=0.3, linestyle='--')
    
    # Error vs Height
    axes[1, 1].scatter(heights, errors, alpha=0.6, s=100, c='purple', edgecolors='black', linewidth=1)
    axes[1, 1].set_xlabel('Height (m)', fontsize=12, weight='bold')
    axes[1, 1].set_ylabel('Residual Error (m)', fontsize=12, weight='bold')
    axes[1, 1].set_title('Error vs Height', fontsize=12, weight='bold')
    axes[1, 1].grid(True, alpha=0.3, linestyle='--')
    
    # Add statistics text
    stats_text = f'Statistics:\nMean: {np.mean(errors):.1f} m\nMedian: {np.median(errors):.1f} m\nStd: {np.std(errors):.1f} m\nMin: {np.min(errors):.1f} m\nMax: {np.max(errors):.1f} m'
    axes[0, 0].text(0.98, 0.98, stats_text, transform=axes[0, 0].transAxes,
                    fontsize=9, verticalalignment='top', horizontalalignment='right',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, "drone_detection_error_analysis.png")
    fig.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"   [SAVED] Error Analysis: {output_file}")
    sys.stdout.flush()
    plt.close(fig)

def create_summary_dashboard(sensor_names, sensor_positions, detections, output_dir="plots"):
    """Create comprehensive summary dashboard with all key information."""
    if not detections:
        return
    
    print(f"   [PLOT] Creating summary dashboard...")
    sys.stdout.flush()
    
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle('Drone Localization - Comprehensive Summary Dashboard', fontsize=18, weight='bold', y=0.98)
    
    # Layout: 3x3 grid
    # Top row: 2D map, 3D view, Statistics
    # Middle row: Time series (X, Y, Height)
    # Bottom row: Quality metrics, Frequency, Error
    
    # === Top Row ===
    # 2D Map (left)
    ax1 = plt.subplot(3, 3, 1)
    positions_arr = np.array(sensor_positions, dtype=float)
    original_indices = [i for i, name in enumerate(sensor_names) if "CLUSTER" not in name]
    cluster_indices = [i for i, name in enumerate(sensor_names) if "CLUSTER" in name]
    
    orig_x = [sensor_positions[i][0] for i in original_indices]
    orig_y = [sensor_positions[i][1] for i in original_indices]
    cluster_pos = sensor_positions[cluster_indices[0]] if cluster_indices else None
    
    if original_indices:
        ax1.scatter(orig_x, orig_y, c='#0066cc', s=100, marker='o', alpha=0.8, edgecolors='#003366', linewidth=1.5)
    if cluster_pos:
        ax1.scatter([cluster_pos[0]], [cluster_pos[1]], c='#ff6600', s=200, marker='*', alpha=0.9, edgecolors='#cc3300', linewidth=1.5)
    
    sorted_detections = sorted(detections, key=lambda d: d['timestamp'])
    det_x = [d['position'][0] for d in sorted_detections]
    det_y = [d['position'][1] for d in sorted_detections]
    timestamps = [d['timestamp'] for d in sorted_detections]
    
    if len(sorted_detections) > 1:
        ax1.plot(det_x, det_y, 'r-', alpha=0.6, linewidth=2)
    ax1.scatter(det_x, det_y, c=timestamps, cmap='viridis', s=80, alpha=0.8, edgecolors='black', linewidth=1)
    ax1.set_xlabel('X (m)', fontsize=10, weight='bold')
    ax1.set_ylabel('Y (m)', fontsize=10, weight='bold')
    ax1.set_title('2D Top View', fontsize=11, weight='bold')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.set_aspect('equal')
    
    # 3D View (middle)
    ax2 = plt.subplot(3, 3, 2, projection='3d')
    if original_indices:
        orig_z = [sensor_positions[i][2] for i in original_indices]
        ax2.scatter(orig_x, orig_y, orig_z, c='#0066cc', s=80, marker='o', alpha=0.8, edgecolors='#003366', linewidth=1)
    if cluster_pos:
        ax2.scatter([cluster_pos[0]], [cluster_pos[1]], [cluster_pos[2]], c='#ff6600', s=150, marker='*', alpha=0.9, edgecolors='#cc3300', linewidth=1)
    
    det_z = [d['position'][2] for d in sorted_detections]
    if len(sorted_detections) > 1:
        ax2.plot(det_x, det_y, det_z, 'r-', alpha=0.6, linewidth=2)
    ax2.scatter(det_x, det_y, det_z, c=timestamps, cmap='viridis', s=80, alpha=0.8, edgecolors='black', linewidth=1)
    ax2.set_xlabel('X (m)', fontsize=9, weight='bold')
    ax2.set_ylabel('Y (m)', fontsize=9, weight='bold')
    ax2.set_zlabel('Height (m)', fontsize=9, weight='bold')
    ax2.set_title('3D View', fontsize=11, weight='bold')
    ax2.view_init(elev=25, azim=45)
    
    # Statistics (right)
    ax3 = plt.subplot(3, 3, 3)
    ax3.axis('off')
    heights_arr = np.array([d['height'] for d in detections])
    errors_arr = np.array([d['residual_error'] for d in detections])
    conf_arr = np.array([d['confidence'] for d in detections])
    
    stats_text = f"""
KEY STATISTICS

Detections: {len(detections)}
Time Span: {min(timestamps)/60:.2f} - {max(timestamps)/60:.2f} min

Position:
  X: {np.mean([d['position'][0] for d in detections]):.1f} m
  Y: {np.mean([d['position'][1] for d in detections]):.1f} m
  Height: {np.mean(heights_arr):.1f} m
    (Range: {np.min(heights_arr):.0f} - {np.max(heights_arr):.0f} m)

Quality:
  Avg Confidence: {np.mean(conf_arr):.3f}
  Avg Error: {np.mean(errors_arr):.1f} m
  Min Error: {np.min(errors_arr):.1f} m
  Max Error: {np.max(errors_arr):.1f} m

Speed:
  Avg: {np.mean([np.linalg.norm(np.array(sorted_detections[i+1]['position']) - np.array(sorted_detections[i]['position'])) / (sorted_detections[i+1]['timestamp'] - sorted_detections[i]['timestamp']) for i in range(len(sorted_detections)-1) if sorted_detections[i+1]['timestamp'] > sorted_detections[i]['timestamp']]):.2f} m/s
    """
    ax3.text(0.1, 0.9, stats_text, transform=ax3.transAxes, fontsize=10,
            verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    ax3.set_title('Summary Statistics', fontsize=11, weight='bold')
    
    # === Middle Row: Time Series ===
    time_min = np.array(timestamps) / 60.0
    positions_arr = np.array([d['position'] for d in sorted_detections])
    
    ax4 = plt.subplot(3, 3, 4)
    ax4.plot(time_min, positions_arr[:, 0], 'b-', linewidth=2, marker='o', markersize=4)
    ax4.set_ylabel('X (m)', fontsize=10, weight='bold')
    ax4.set_title('X Position', fontsize=10, weight='bold')
    ax4.grid(True, alpha=0.3, linestyle='--')
    
    ax5 = plt.subplot(3, 3, 5)
    ax5.plot(time_min, positions_arr[:, 1], 'g-', linewidth=2, marker='o', markersize=4)
    ax5.set_ylabel('Y (m)', fontsize=10, weight='bold')
    ax5.set_title('Y Position', fontsize=10, weight='bold')
    ax5.grid(True, alpha=0.3, linestyle='--')
    
    ax6 = plt.subplot(3, 3, 6)
    ax6.plot(time_min, heights_arr, 'r-', linewidth=2, marker='o', markersize=4)
    ax6.axhline(y=450, color='orange', linestyle='--', alpha=0.5)
    ax6.axhline(y=500, color='orange', linestyle='--', alpha=0.5)
    ax6.set_ylabel('Height (m)', fontsize=10, weight='bold')
    ax6.set_title('Height', fontsize=10, weight='bold')
    ax6.grid(True, alpha=0.3, linestyle='--')
    
    # === Bottom Row ===
    # Quality Metrics
    ax7 = plt.subplot(3, 3, 7)
    ax7.plot(time_min, conf_arr, 'b-', linewidth=2, marker='o', markersize=4, label='Confidence')
    ax7_twin = ax7.twinx()
    ax7_twin.plot(time_min, errors_arr, 'r--', linewidth=2, marker='s', markersize=4, label='Error')
    ax7.set_ylabel('Confidence', fontsize=10, weight='bold', color='blue')
    ax7_twin.set_ylabel('Error (m)', fontsize=10, weight='bold', color='red')
    ax7.set_title('Quality Metrics', fontsize=10, weight='bold')
    ax7.grid(True, alpha=0.3, linestyle='--')
    ax7.tick_params(axis='y', labelcolor='blue')
    ax7_twin.tick_params(axis='y', labelcolor='red')
    
    # Frequency
    ax8 = plt.subplot(3, 3, 8)
    frequencies = np.array([d.get('fundamental_freq', 0.0) for d in sorted_detections])
    ax8.plot(time_min, frequencies, 'purple', linewidth=2, marker='o', markersize=4)
    ax8.axhline(y=400, color='orange', linestyle='--', alpha=0.5)
    ax8.axhline(y=3000, color='orange', linestyle='--', alpha=0.5)
    ax8.fill_between([time_min[0], time_min[-1]], 400, 3000, alpha=0.1, color='orange')
    ax8.set_ylabel('Frequency (Hz)', fontsize=10, weight='bold')
    ax8.set_xlabel('Time (min)', fontsize=10, weight='bold')
    ax8.set_title('Fundamental Frequency', fontsize=10, weight='bold')
    ax8.grid(True, alpha=0.3, linestyle='--')
    
    # Error Distribution
    ax9 = plt.subplot(3, 3, 9)
    ax9.hist(errors_arr, bins=10, alpha=0.7, color='red', edgecolor='black', linewidth=1)
    ax9.axvline(x=np.mean(errors_arr), color='blue', linestyle='--', linewidth=2, label=f'Mean: {np.mean(errors_arr):.1f}m')
    ax9.set_xlabel('Error (m)', fontsize=10, weight='bold')
    ax9.set_ylabel('Count', fontsize=10, weight='bold')
    ax9.set_title('Error Distribution', fontsize=10, weight='bold')
    ax9.grid(True, alpha=0.3, linestyle='--', axis='y')
    ax9.legend(loc='best', fontsize=8)
    
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    output_file = os.path.join(output_dir, "drone_detection_summary_dashboard.png")
    fig.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"   [SAVED] Summary Dashboard: {output_file}")
    sys.stdout.flush()
    plt.close(fig)

def plot_tdoa_analysis(tdoa_data, sensor_names, output_dir="plots"):
    """Plot TDOA matrices and correlation analysis."""
    if not tdoa_data:
        return
    
    print(f"\n[PLOT] Creating TDOA analysis plots...")
    sys.stdout.flush()
    os.makedirs(output_dir, exist_ok=True)
    
    # Extract data
    timestamps = [d['timestamp'] for d in tdoa_data]
    time_min = np.array(timestamps) / 60.0
    
    # Create figure
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('TDOA Analysis from GCC-PHAT', fontsize=16, weight='bold', y=0.98)
    
    # Plot 1: TDOA magnitude over time
    ax1 = axes[0, 0]
    tdoa_magnitudes = []
    for d in tdoa_data:
        tdoa_matrix = d['tdoa_matrix']
        valid_tdoas = tdoa_matrix[tdoa_matrix != 0]
        if len(valid_tdoas) > 0:
            tdoa_magnitudes.append(np.mean(np.abs(valid_tdoas)) * 1000)  # Convert to ms
        else:
            tdoa_magnitudes.append(0)
    
    ax1.plot(time_min, tdoa_magnitudes, 'b-o', linewidth=2, markersize=6)
    ax1.set_xlabel('Time (minutes)', fontsize=12, weight='bold')
    ax1.set_ylabel('Mean |TDOA| (ms)', fontsize=12, weight='bold')
    ax1.set_title('TDOA Magnitude Over Time', fontsize=12, weight='bold')
    ax1.grid(True, alpha=0.3, linestyle='--')
    
    # Plot 2: Number of valid TDOA pairs
    ax2 = axes[0, 1]
    num_valid_pairs = []
    for d in tdoa_data:
        tdoa_matrix = d['tdoa_matrix']
        valid_tdoas = tdoa_matrix[tdoa_matrix != 0]
        num_valid_pairs.append(len(valid_tdoas))
    
    ax2.plot(time_min, num_valid_pairs, 'g-o', linewidth=2, markersize=6)
    ax2.set_xlabel('Time (minutes)', fontsize=12, weight='bold')
    ax2.set_ylabel('Number of Valid TDOA Pairs', fontsize=12, weight='bold')
    ax2.set_title('Valid TDOA Pairs Over Time', fontsize=12, weight='bold')
    ax2.grid(True, alpha=0.3, linestyle='--')
    
    # Plot 3: Quality metrics
    ax3 = axes[1, 0]
    mean_qualities = []
    for d in tdoa_data:
        quality_matrix = d['quality_matrix']
        valid_qualities = quality_matrix[quality_matrix > 0]
        if len(valid_qualities) > 0:
            mean_qualities.append(np.mean(valid_qualities))
        else:
            mean_qualities.append(0)
    
    ax3.plot(time_min, mean_qualities, 'r-o', linewidth=2, markersize=6)
    ax3.axhline(y=MIN_CORRELATION_QUALITY, color='orange', linestyle='--', alpha=0.5,
                label=f'Threshold ({MIN_CORRELATION_QUALITY})')
    ax3.set_xlabel('Time (minutes)', fontsize=12, weight='bold')
    ax3.set_ylabel('Mean Correlation Quality', fontsize=12, weight='bold')
    ax3.set_title('TDOA Quality Over Time', fontsize=12, weight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3, linestyle='--')
    
    # Plot 4: TDOA distribution histogram
    ax4 = axes[1, 1]
    all_tdoas = []
    for d in tdoa_data:
        tdoa_matrix = d['tdoa_matrix']
        valid_tdoas = tdoa_matrix[tdoa_matrix != 0]
        all_tdoas.extend(valid_tdoas.tolist())
    
    if len(all_tdoas) > 0:
        all_tdoas = np.array(all_tdoas) * 1000  # Convert to ms
        ax4.hist(all_tdoas, bins=30, alpha=0.7, color='purple', edgecolor='black')
        ax4.axvline(x=np.mean(np.abs(all_tdoas)), color='red', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(np.abs(all_tdoas)):.2f} ms')
        ax4.set_xlabel('TDOA (ms)', fontsize=12, weight='bold')
        ax4.set_ylabel('Count', fontsize=12, weight='bold')
        ax4.set_title('TDOA Distribution', fontsize=12, weight='bold')
        ax4.legend()
        ax4.grid(True, alpha=0.3, linestyle='--', axis='y')
    
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    output_file = os.path.join(output_dir, "tdoa_analysis.png")
    fig.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"   [SAVED] TDOA Analysis: {output_file}")
    sys.stdout.flush()
    plt.close(fig)

def main():
    """Main function."""
    if not os.path.exists(WAV_FILE):
        print(f"[ERROR] WAV file not found: {WAV_FILE}")
        return
    
    if not os.path.exists(KML_FILE):
        print(f"[ERROR] KML file not found: {KML_FILE}")
        return
    
    try:
        sensor_names, sensor_positions, detections, tdoa_data = process_audio_detection(WAV_FILE, KML_FILE)
        
        # Plot TDOA analysis
        if tdoa_data:
            plot_tdoa_analysis(tdoa_data, sensor_names, output_dir="plots")
        
        output_files = create_2d_3d_maps(sensor_names, sensor_positions, detections)
        
        print(f"\n[SUCCESS] Analysis Complete!")
        print(f"   [DETECTIONS] Detections: {len(detections)}")
        if isinstance(output_files, tuple):
            print(f"   [MAP] 2D Top View: {output_files[0]}")
            print(f"   [MAP] 3D TPV: {output_files[1]}")
        else:
            print(f"   [MAP] Map: {output_files}")
        sys.stdout.flush()
        
    except Exception as e:
        print(f"[ERROR] Analysis failed: {e}")
        import traceback
        traceback.print_exc()
        sys.stdout.flush()

if __name__ == "__main__":
    main()

