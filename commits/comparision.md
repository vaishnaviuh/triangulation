# Code Comparison Report: triangulation vs commits

## Executive Summary

This report compares the Python code files between the `triangulation/src` folder and the `commits/src` folder. The analysis reveals that most files are identical, with one major difference in the `triangulation.py` file, which has been significantly enhanced in the commits folder.

---

## Files Comparison

### Identical Files (No Changes)

The following files are **identical** between both folders:

1. **`audio_io.py`** - Audio file I/O operations (load, save, segment extraction)
2. **`audio.py`** - Audio recording and simulation functions
3. **`classify.py`** - Sound classification module (drone, mechanical, clap detection)
4. **`config.py`** - Configuration dataclasses (AudioConfig, GeometryConfig, etc.)
5. **`doa.py`** - Direction of Arrival (DOA) estimation functions
6. **`kml_parser.py`** - KML file parser for sensor positions
7. **`pipeline.py`** - Signal processing pipeline
8. **`plotting.py`** - Visualization functions for 2D/3D plots
9. **`utils.py`** - General utility functions
10. **`utils/logger.py`** - Structured logging utilities
11. **`utils/retry.py`** - Retry mechanisms with exponential backoff
12. **`utils/validators.py`** - Input validation utilities
13. **`utils/__init__.py`** - Utils package initialization

**Total: 13 files are identical**

---

## Files with Differences

### 1. `triangulation.py` - MAJOR ENHANCEMENTS

**Status:** Significantly enhanced in `commits/src/triangulation.py`

**Key Differences:**

#### A. New Classes and Utilities

**In commits folder only:**
- **`TriangulationConfig`** dataclass - Configuration for triangulation engine with:
  - Temperature, humidity, altitude parameters
  - Atmospheric refraction modeling
  - Multipath detection settings
  - GDOP threshold
  - Huber loss parameters
  - Regularization weights

- **`SpeedOfSoundCalculator`** class - Advanced speed of sound calculations:
  - Temperature-based calculations
  - Humidity corrections
  - Altitude/pressure corrections
  - Speed gradient computation for refraction

- **`UncertaintyEstimator`** class - TDOA and position uncertainty estimation:
  - Cramér-Rao Lower Bound (CRLB) computation
  - Position covariance matrix calculation
  - Uncertainty ellipsoid computation

- **`GDOPCalculator`** class - Geometric Dilution of Precision:
  - GDOP computation from Jacobian matrices
  - Weighted GDOP calculations

#### B. Enhanced TriangulationResult

**In commits folder:**
- Added fields:
  - `covariance_matrix` - Position covariance matrix (3x3)
  - `uncertainty_ellipsoid` - Uncertainty ellipsoid parameters
  - `gdop` - Geometric Dilution of Precision value
  - `tdoa_uncertainties` - TDOA uncertainty matrix
  - `is_far_field` - Far-field detection flag
  - `estimated_distance` - Estimated distance from sensor centroid

#### C. Enhanced TriangulationEngine Methods

**New/Enhanced methods in commits folder:**

1. **`_compute_jacobian()`** - Computes Jacobian matrix for TDOA residuals
2. **`_is_far_field()`** - Determines if source is in far-field
3. **`_detect_multipath()`** - Detects multipath using secondary peaks
4. **`_compute_coherence()`** - Computes coherence function between signals
5. **`compute_tdoa_matrix()`** - Enhanced with:
   - Frequency band weighting
   - Uncertainty estimation
   - Quality matrix computation
   - Outlier rejection
6. **`_gcc_phat_delay_enhanced()`** - Enhanced GCC-PHAT with:
   - Coherence computation
   - Multipath detection
   - Frequency weighting
   - Sub-sample interpolation
7. **`_huber_loss()`** - Huber loss function for robust optimization
8. **`triangulate_least_squares()`** - Enhanced with:
   - Uncertainty weighting
   - Huber loss option
   - Position prior regularization
   - Covariance matrix computation
   - GDOP calculation
   - Far-field detection
9. **`triangulate_robust()`** - Enhanced with:
   - Multiple initial guesses
   - GDOP-aware result selection
   - Uncertainty-aware optimization
10. **`triangulate_audio_chunk()`** - Enhanced with:
    - Frequency band filtering
    - Sensor selection (D-optimal design)
    - Uncertainty propagation
    - Bounds support
11. **`_select_optimal_sensors()`** - New method for optimal sensor selection
12. **`_compute_jacobian_for_sensors()`** - Jacobian computation for sensor subsets

#### D. Enhanced Initialization

**In commits folder:**
- `TriangulationEngine.__init__()` now accepts:
  - Optional `speed_of_sound` parameter
  - Optional `config` parameter (TriangulationConfig)
  - Automatic speed of sound calculation from atmospheric conditions
  - Sensor position normalization for numerical stability

#### E. Enhanced Validation

**In commits folder:**
- `validate_triangulation_result()` now checks:
  - GDOP threshold
  - Enhanced position bounds validation

#### Summary of Changes

| Aspect | triangulation/src | commits/src |
|--------|-------------------|-------------|
| Lines of Code | ~495 lines | ~1078 lines |
| Classes | 1 (TriangulationEngine) | 4 (TriangulationEngine, SpeedOfSoundCalculator, UncertaintyEstimator, GDOPCalculator) |
| Dataclasses | 1 (TriangulationResult) | 2 (TriangulationResult, TriangulationConfig) |
| TDOA Methods | Basic GCC-PHAT | Enhanced GCC-PHAT with coherence, multipath detection |
| Uncertainty | None | Full uncertainty propagation (CRLB, covariance, ellipsoids) |
| GDOP | Not computed | Computed and used for validation |
| Atmospheric Corrections | Fixed speed of sound (343.0 m/s) | Dynamic calculation with temp/humidity/altitude |
| Robust Optimization | Basic | Huber loss, multiple initial guesses |
| Sensor Selection | All sensors used | Optimal sensor selection (D-optimal design) |

---

## Files Only in commits Folder

### 1. `analyze_positions.py`
- **Location:** `commits/analyze_positions.py`
- **Purpose:** Analyzes detection position accuracy
- **Functionality:**
  - Loads detections from `drone_spectrogram_detection.py`
  - Analyzes position statistics (X, Y, Z ranges)
  - Checks residual errors
  - Validates confidence scores
  - Checks for suspicious patterns (identical positions, high residuals)
  - Analyzes sensor geometry
  - Provides recommendations for accuracy improvements

### 2. `drone_spectrogram_detection.py`
- **Location:** `commits/drone_spectrogram_detection.py`
- **Size:** 2496 lines
- **Purpose:** Drone detection using spectrogram analysis
- **Functionality:**
  - Processes audio files in chunks
  - Applies bandpass filtering
  - Detects drone signatures using spectrogram analysis
  - Triangulates detected drones
  - Creates 2D and 3D visualization maps
  - Handles cluster channels (combines channels 16-20)
  - Very aggressive detection settings for maximum sensitivity

### 3. `triangulate.py` (in main triangulation folder)
- **Location:** `triangulation/triangulate.py`
- **Note:** This file appears to be identical to `commits/drone_spectrogram_detection.py` (same 858 lines)

---

## Summary Statistics

| Category | Count |
|----------|-------|
| **Identical Files** | 13 |
| **Files with Differences** | 1 (`triangulation.py`) |
| **New Files in commits** | 2 (`analyze_positions.py`, `drone_spectrogram_detection.py`) |
| **Total Files Compared** | 14 |

---

## Key Improvements in commits Folder

### 1. **Physics-Based Enhancements**
- Atmospheric corrections for speed of sound
- Temperature, humidity, and altitude considerations
- Atmospheric refraction modeling

### 2. **Uncertainty Quantification**
- TDOA uncertainty estimation (CRLB)
- Position covariance matrices
- Uncertainty ellipsoids
- GDOP calculation

### 3. **Robustness Improvements**
- Multipath detection
- Signal coherence analysis
- Huber loss for robust optimization
- Multiple initial guesses for optimization
- Outlier rejection

### 4. **Advanced Algorithms**
- D-optimal sensor selection
- Far-field vs near-field detection
- Enhanced GCC-PHAT with frequency weighting
- Sub-sample interpolation for TDOA

### 5. **Numerical Stability**
- Sensor position normalization
- Regularization for optimization
- Improved matrix conditioning checks

---

## Recommendations

1. **Consider merging** the enhanced `triangulation.py` from commits folder into the main triangulation folder
2. **Review** the new files (`analyze_positions.py` and `drone_spectrogram_detection.py`) for integration
3. **Test** the enhanced triangulation engine with real data to validate improvements
4. **Document** the new features and configuration options

---

## Conclusion

The commits folder contains significant enhancements to the triangulation engine, particularly in:
- Physics-based modeling (atmospheric corrections)
- Uncertainty quantification
- Robustness and numerical stability
- Advanced signal processing techniques

The rest of the codebase remains unchanged, indicating focused improvements on the core triangulation algorithm.

