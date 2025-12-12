#!/usr/bin/env python3
"""Analyze detection position accuracy."""
import sys
import numpy as np
from drone_spectrogram_detection import process_audio_detection

# Load detections
print("Loading detections...")
sensor_names, sensor_positions, detections, tdoa_data = process_audio_detection(
    'multi-20251122-141610-627897594.wav',
    'Sensor-Locations-BOP-Dharma.kml'
)

if len(detections) == 0:
    print("No detections found!")
    sys.exit(1)

# Extract positions
positions = np.array([d['position'] for d in detections])
residual_errors = np.array([d['residual_error'] for d in detections])
confidences = np.array([d['confidence'] for d in detections])

print(f"\n{'='*60}")
print(f"POSITION ACCURACY ANALYSIS")
print(f"{'='*60}\n")

print(f"Total detections: {len(detections)}")
print(f"\nPosition Statistics:")
print(f"  X: min={positions[:, 0].min():.2f}m, max={positions[:, 0].max():.2f}m, std={positions[:, 0].std():.2f}m")
print(f"  Y: min={positions[:, 1].min():.2f}m, max={positions[:, 1].max():.2f}m, std={positions[:, 1].std():.2f}m")
print(f"  Z: min={positions[:, 2].min():.2f}m, max={positions[:, 2].max():.2f}m, std={positions[:, 2].std():.2f}m")

print(f"\nResidual Errors (TDOA fit quality):")
print(f"  min={residual_errors.min():.1f}m, max={residual_errors.max():.1f}m, mean={residual_errors.mean():.1f}m, std={residual_errors.std():.1f}m")

print(f"\nConfidence Scores:")
print(f"  min={confidences.min():.3f}, max={confidences.max():.3f}, mean={confidences.mean():.3f}")

# Check for suspicious patterns
print(f"\n{'='*60}")
print(f"ACCURACY ISSUES:")
print(f"{'='*60}\n")

# Check if X positions are all the same
x_unique = len(np.unique(np.round(positions[:, 0], 1)))
if x_unique == 1:
    print(f"⚠️  WARNING: All X positions are identical ({positions[0, 0]:.1f}m)")
    print(f"   This suggests hitting a bound or systematic error!")
else:
    print(f"✓ X positions vary ({x_unique} unique values)")

# Check residual errors
high_residual = np.sum(residual_errors > 150)
if high_residual > 0:
    print(f"⚠️  WARNING: {high_residual}/{len(detections)} detections have residual errors > 150m")
    print(f"   High residuals indicate poor TDOA fit to triangulated position")
else:
    print(f"✓ Residual errors are reasonable (< 150m)")

# Check sensor geometry
sensor_positions_array = np.array(sensor_positions)
sensor_centroid = np.mean(sensor_positions_array, axis=0)
sensor_extent = np.max(np.abs(sensor_positions_array - sensor_centroid), axis=0)

print(f"\nSensor Array Geometry:")
print(f"  Centroid: ({sensor_centroid[0]:.1f}, {sensor_centroid[1]:.1f}, {sensor_centroid[2]:.1f})")
print(f"  Extent: X={sensor_extent[0]:.1f}m, Y={sensor_extent[1]:.1f}m, Z={sensor_extent[2]:.1f}m")
print(f"  Number of sensors: {len(sensor_positions)}")

# Check if sensors are coplanar (poor for height estimation)
z_range = sensor_positions_array[:, 2].max() - sensor_positions_array[:, 2].min()
if z_range < 10:
    print(f"⚠️  WARNING: Sensors are nearly coplanar (Z range = {z_range:.1f}m)")
    print(f"   This causes poor height estimation (GDOP_z is very high)")
else:
    print(f"✓ Sensors have good vertical spread (Z range = {z_range:.1f}m)")

# Check detection positions relative to sensors
distances = np.linalg.norm(positions - sensor_centroid, axis=1)
print(f"\nDetection Distances from Sensor Centroid:")
print(f"  min={distances.min():.1f}m, max={distances.max():.1f}m, mean={distances.mean():.1f}m")

# Check if positions are reasonable
print(f"\n{'='*60}")
print(f"RECOMMENDATIONS:")
print(f"{'='*60}\n")

if x_unique == 1:
    print("1. X positions are all identical - check if optimization is hitting bounds")
    print("2. Review initial guess and bounds for X coordinate")

if high_residual > len(detections) * 0.5:
    print("3. High residual errors suggest:")
    print("   - TDOA measurements may have systematic errors")
    print("   - Sensor geometry may be poor (check GDOP)")
    print("   - Speed of sound may be incorrect")

if z_range < 10:
    print("4. With coplanar sensors, height accuracy is fundamentally limited")
    print("   - Consider using sensors at different elevations")
    print("   - Height estimates are based on TDOA patterns, not geometry")

print("\n" + "="*60)

