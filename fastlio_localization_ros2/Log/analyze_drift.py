import csv
import math

data = []
with open('/home/simit/FAST_LIO_LOCALIZATION2/Log/drift_diagnostic.csv', 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    for i, row in enumerate(reader):
        vals = [float(v) for v in row]
        data.append({
            'frame': i,
            'timestamp': vals[0],
            'pos_x': vals[1], 'pos_y': vals[2], 'pos_z': vals[3],
            'rot_x': vals[4], 'rot_y': vals[5], 'rot_z': vals[6],
            'vel_x': vals[7], 'vel_y': vals[8], 'vel_z': vals[9],
            'bg_x': vals[10], 'bg_y': vals[11], 'bg_z': vals[12],
            'ba_x': vals[13], 'ba_y': vals[14], 'ba_z': vals[15],
            'grav_x': vals[16], 'grav_y': vals[17], 'grav_z': vals[18],
            'delta_pos_x': vals[19], 'delta_pos_y': vals[20], 'delta_pos_z': vals[21],
            'delta_rot_x': vals[22], 'delta_rot_y': vals[23], 'delta_rot_z': vals[24],
            'P_d0': vals[25], 'P_d1': vals[26], 'P_d2': vals[27],
            'P_d3': vals[28], 'P_d4': vals[29], 'P_d5': vals[30],
            'P_d6': vals[31], 'P_d7': vals[32], 'P_d8': vals[33],
            'P_d9': vals[34], 'P_d10': vals[35], 'P_d11': vals[36],
            'P_d12': vals[37], 'P_d13': vals[38], 'P_d14': vals[39],
            'P_d15': vals[40], 'P_d16': vals[41], 'P_d17': vals[42],
            'P_d18': vals[43], 'P_d19': vals[44], 'P_d20': vals[45],
            'P_d21': vals[46], 'P_d22': vals[47],
            'effct_feat_num': vals[48],
            'feats_down_size': vals[49],
            'res_mean': vals[50],
            'icp_time': vals[51],
            'kdtree_size': vals[52],
            'global_match': vals[53],
        })

print("=" * 80)
print("DRIFT DIAGNOSTIC ANALYSIS")
print("=" * 80)
print(f"Total frames: {len(data)}")
print(f"Time range: {data[0]['timestamp']:.3f} to {data[-1]['timestamp']:.3f}s")
print()

# 1. Find stutter frames: anomalously large delta_pos or delta_rot
print("=" * 80)
print("1. STUTTER FRAME DETECTION")
print("=" * 80)

for d in data:
    d['delta_pos_mag'] = math.sqrt(d['delta_pos_x']**2 + d['delta_pos_y']**2 + d['delta_pos_z']**2)
    d['delta_rot_mag'] = math.sqrt(d['delta_rot_x']**2 + d['delta_rot_y']**2 + d['delta_rot_z']**2)

delta_pos_mags = [d['delta_pos_mag'] for d in data]
delta_rot_mags = [d['delta_rot_mag'] for d in data]

mean_dp = sum(delta_pos_mags) / len(delta_pos_mags)
std_dp = (sum((x - mean_dp)**2 for x in delta_pos_mags) / len(delta_pos_mags))**0.5
mean_dr = sum(delta_rot_mags) / len(delta_rot_mags)
std_dr = (sum((x - mean_dr)**2 for x in delta_rot_mags) / len(delta_rot_mags))**0.5

print(f"delta_pos_mag: mean={mean_dp:.6f}, std={std_dp:.6f}")
print(f"delta_rot_mag: mean={mean_dr:.6f} rad, std={std_dr:.6f} rad")
print()

# Find top 20 largest delta_pos and delta_rot frames
print("--- Top 20 frames by |delta_pos| ---")
sorted_by_dpos = sorted(data, key=lambda d: d['delta_pos_mag'], reverse=True)
for i, d in enumerate(sorted_by_dpos[:20]):
    print(f"  Frame {d['frame']:4d} t={d['timestamp']:8.3f}s  |delta_pos|={d['delta_pos_mag']:.6f}m  "
          f"dp=({d['delta_pos_x']:+.6f},{d['delta_pos_y']:+.6f},{d['delta_pos_z']:+.6f})  "
          f"|delta_rot|={d['delta_rot_mag']:.6f}rad({math.degrees(d['delta_rot_mag']):.3f}deg)  "
          f"global_match={int(d['global_match'])}  effct_feat={int(d['effct_feat_num'])}")

print()
print("--- Top 20 frames by |delta_rot| ---")
sorted_by_drot = sorted(data, key=lambda d: d['delta_rot_mag'], reverse=True)
for i, d in enumerate(sorted_by_drot[:20]):
    print(f"  Frame {d['frame']:4d} t={d['timestamp']:8.3f}s  |delta_rot|={d['delta_rot_mag']:.6f}rad({math.degrees(d['delta_rot_mag']):.3f}deg)  "
          f"dr=({d['delta_rot_x']:+.6f},{d['delta_rot_y']:+.6f},{d['delta_rot_z']:+.6f})  "
          f"|delta_pos|={d['delta_pos_mag']:.6f}m  "
          f"global_match={int(d['global_match'])}  effct_feat={int(d['effct_feat_num'])}")

# 2. check_safe_update analysis
print()
print("=" * 80)
print("2. CHECK_SAFE_UPDATE ANALYSIS (threshold: 0.2m pos, 5deg=0.087rad rot)")
print("=" * 80)

violations_pos = [d for d in data if d['delta_pos_mag'] > 0.2]
violations_rot = [d for d in data if d['delta_rot_mag'] > math.radians(5)]
violations_any = [d for d in data if d['delta_pos_mag'] > 0.2 or d['delta_rot_mag'] > math.radians(5)]

print(f"Frames with |delta_pos| > 0.2m: {len(violations_pos)}")
print(f"Frames with |delta_rot| > 5 deg (0.087 rad): {len(violations_rot)}")
print(f"Frames violating either: {len(violations_any)}")

if violations_any:
    print()
    print("VIOLATION FRAMES:")
    for d in violations_any:
        print(f"  Frame {d['frame']:4d} t={d['timestamp']:8.3f}s  "
              f"|delta_pos|={d['delta_pos_mag']:.6f}m  |delta_rot|={d['delta_rot_mag']:.6f}rad({math.degrees(d['delta_rot_mag']):.3f}deg)  "
              f"global_match={int(d['global_match'])}  effct_feat={int(d['effct_feat_num'])}  "
              f"pos=({d['pos_x']:.3f},{d['pos_y']:.3f},{d['pos_z']:.3f})")
else:
    print("NO violations found. check_safe_update threshold is not being exceeded.")
    print()
    print("Max values:")
    max_pos = max(data, key=lambda d: d['delta_pos_mag'])
    max_rot = max(data, key=lambda d: d['delta_rot_mag'])
    print(f"  Max |delta_pos| = {max_pos['delta_pos_mag']:.6f}m at frame {max_pos['frame']} (t={max_pos['timestamp']:.3f}s)")
    print(f"  Max |delta_rot| = {max_rot['delta_rot_mag']:.6f}rad ({math.degrees(max_rot['delta_rot_mag']):.3f}deg) at frame {max_rot['frame']} (t={max_rot['timestamp']:.3f}s)")

# 3. Cumulative drift pattern
print()
print("=" * 80)
print("3. CUMULATIVE DRIFT PATTERN")
print("=" * 80)

# Show position every N frames
N = max(1, len(data) // 20)
print(f"Position at every ~{N} frames:")
print(f"  {'Frame':>5s} {'Time':>8s} {'pos_x':>10s} {'pos_y':>10s} {'pos_z':>10s} {'|pos|':>10s} "
      f"{'rot_x':>8s} {'rot_y':>8s} {'rot_z':>8s} {'gmatch':>7s} {'feat':>5s} {'res':>8s}")
for i in range(0, len(data), N):
    d = data[i]
    pos_mag = math.sqrt(d['pos_x']**2 + d['pos_y']**2 + d['pos_z']**2)
    print(f"  {d['frame']:5d} {d['timestamp']:8.3f} {d['pos_x']:10.4f} {d['pos_y']:10.4f} {d['pos_z']:10.4f} {pos_mag:10.4f} "
          f"{math.degrees(d['rot_x']):8.3f} {math.degrees(d['rot_y']):8.3f} {math.degrees(d['rot_z']):8.3f} "
          f"{int(d['global_match']):7d} {int(d['effct_feat_num']):5d} {d['res_mean']:8.5f}")

# Compute frame-to-frame position change magnitude (before EKF correction)
print()
print("--- Cumulative position drift analysis ---")
# Break data into segments
print(f"  Start position: ({data[0]['pos_x']:.4f}, {data[0]['pos_y']:.4f}, {data[0]['pos_z']:.4f})")
print(f"  End position:   ({data[-1]['pos_x']:.4f}, {data[-1]['pos_y']:.4f}, {data[-1]['pos_z']:.4f})")

total_pos_drift = math.sqrt(data[-1]['pos_x']**2 + data[-1]['pos_y']**2 + data[-1]['pos_z']**2)
print(f"  Total |position| at end: {total_pos_drift:.4f}m")

# Check if drift is gradual or sudden
# Look at the derivative of position magnitude
pos_mags = [math.sqrt(d['pos_x']**2 + d['pos_y']**2 + d['pos_z']**2) for d in data]
dpos_mag_changes = [(pos_mags[i+1] - pos_mags[i]) for i in range(len(pos_mags)-1)]

# Find the biggest single-frame position magnitude jumps
pos_jumps = [(i, dpos_mag_changes[i]) for i in range(len(dpos_mag_changes))]
pos_jumps.sort(key=lambda x: abs(x[1]), reverse=True)

print()
print("--- Top 15 single-frame position magnitude jumps ---")
for idx, jump in pos_jumps[:15]:
    d = data[idx+1]
    print(f"  Frame {idx+1:4d}-> {idx+2:4d} t={d['timestamp']:8.3f}s  d|pos|={jump:+.6f}m  "
          f"|pos|={pos_mags[idx+1]:.4f}  global_match={int(d['global_match'])}  "
          f"pos=({d['pos_x']:.4f},{d['pos_y']:.4f},{d['pos_z']:.4f})")

# 4. P covariance analysis
print()
print("=" * 80)
print("4. P COVARIANCE ANALYSIS")
print("=" * 80)

print("Position covariances (P_d0=x, P_d1=y, P_d2=z) and rotation (P_d3, P_d4, P_d5):")
print(f"  {'Frame':>5s} {'Time':>8s} {'P_d0':>12s} {'P_d1':>12s} {'P_d2':>12s} {'P_d3':>12s} {'P_d4':>12s} {'P_d5':>12s} {'gmatch':>7s}")
for i in range(0, len(data), N):
    d = data[i]
    print(f"  {d['frame']:5d} {d['timestamp']:8.3f} {d['P_d0']:12.8f} {d['P_d1']:12.8f} {d['P_d2']:12.8f} "
          f"{d['P_d3']:12.8f} {d['P_d4']:12.8f} {d['P_d5']:12.8f} {int(d['global_match']):7d}")

# Min/max of key P diagonals
p0_vals = [d['P_d0'] for d in data]
p3_vals = [d['P_d3'] for d in data]
print()
print(f"  P_d0 (pos_x): min={min(p0_vals):.8f}, max={max(p0_vals):.8f}")
print(f"  P_d3 (rot_x): min={min(p3_vals):.8f}, max={max(p3_vals):.8f}")

# Check if P is collapsing (getting too small to allow corrections)
# P_d6, P_d7, P_d8 are velocity covariance
p6_vals = [d['P_d6'] for d in data]
p9_vals = [d['P_d9'] for d in data]
print(f"  P_d6 (vel_x): min={min(p6_vals):.8f}, max={max(p6_vals):.8f}")
print(f"  P_d9 (bias_g_x): min={min(p9_vals):.8f}, max={max(p9_vals):.8f}")

# 5. Global match analysis
print()
print("=" * 80)
print("5. GLOBAL MATCH ANALYSIS")
print("=" * 80)

# Find when global_match drops to 0
gm_values = [d['global_match'] for d in data]
has_global = [d for d in data if d['global_match'] > 0]
no_global = [d for d in data if d['global_match'] == 0]
print(f"Frames with global_match > 0: {len(has_global)}")
print(f"Frames with global_match = 0: {len(no_global)}")

if has_global:
    print(f"Global match range when active: {min(d['global_match'] for d in has_global):.0f} to {max(d['global_match'] for d in has_global):.0f}")
    last_global_frame = max(d['frame'] for d in has_global)
    print(f"Last frame with global match: {last_global_frame} (t={data[last_global_frame]['timestamp']:.3f}s)")

if no_global:
    first_no_global = min(d['frame'] for d in no_global)
    print(f"First frame with NO global match: {first_no_global} (t={data[first_no_global]['timestamp']:.3f}s)")

# Show global_match transitions
print()
print("--- Global match transition frames ---")
prev_gm = data[0]['global_match']
transitions = []
for d in data[1:]:
    if d['global_match'] != prev_gm:
        if (prev_gm > 0 and d['global_match'] == 0) or (prev_gm == 0 and d['global_match'] > 0):
            transitions.append(d)
            print(f"  Frame {d['frame']:4d} t={d['timestamp']:8.3f}s  global_match: {prev_gm:.0f} -> {d['global_match']:.0f}  "
                  f"pos=({d['pos_x']:.4f},{d['pos_y']:.4f},{d['pos_z']:.4f})  "
                  f"effct_feat={int(d['effct_feat_num'])}  kdtree={int(d['kdtree_size'])}")
    prev_gm = d['global_match']

# 6. Rotation analysis
print()
print("=" * 80)
print("6. ROTATION DIVERGENCE ANALYSIS")
print("=" * 80)

print(f"Rotation over time (degrees):")
print(f"  {'Frame':>5s} {'Time':>8s} {'rot_x(deg)':>10s} {'rot_y(deg)':>10s} {'rot_z(deg)':>10s} {'gmatch':>7s} {'feat':>5s}")
for i in range(0, len(data), N):
    d = data[i]
    print(f"  {d['frame']:5d} {d['timestamp']:8.3f} {math.degrees(d['rot_x']):10.4f} {math.degrees(d['rot_y']):10.4f} {math.degrees(d['rot_z']):10.4f} "
          f"{int(d['global_match']):7d} {int(d['effct_feat_num']):5d}")

# Find where rotation starts diverging significantly
rot_x_vals = [math.degrees(d['rot_x']) for d in data]
rot_y_vals = [math.degrees(d['rot_y']) for d in data]
rot_z_vals = [math.degrees(d['rot_z']) for d in data]

print()
print(f"  rot_x range: {min(rot_x_vals):.4f} to {max(rot_x_vals):.4f} deg")
print(f"  rot_y range: {min(rot_y_vals):.4f} to {max(rot_y_vals):.4f} deg")
print(f"  rot_z range: {min(rot_z_vals):.4f} to {max(rot_z_vals):.4f} deg")

# 7. Residual analysis
print()
print("=" * 80)
print("7. RESIDUAL ANALYSIS (res_mean)")
print("=" * 80)

res_vals = [d['res_mean'] for d in data]
print(f"res_mean: min={min(res_vals):.6f}, max={max(res_vals):.6f}, mean={sum(res_vals)/len(res_vals):.6f}")

# Top 20 highest residual frames
print()
print("--- Top 20 highest res_mean frames ---")
sorted_by_res = sorted(data, key=lambda d: d['res_mean'], reverse=True)
for d in sorted_by_res[:20]:
    print(f"  Frame {d['frame']:4d} t={d['timestamp']:8.3f}s  res_mean={d['res_mean']:.6f}  "
          f"|delta_pos|={d['delta_pos_mag']:.6f}  global_match={int(d['global_match'])}  "
          f"effct_feat={int(d['effct_feat_num'])}  pos=({d['pos_x']:.3f},{d['pos_y']:.3f},{d['pos_z']:.3f})")

# 8. Timestamp gaps
print()
print("=" * 80)
print("8. TIMESTAMP GAP ANALYSIS")
print("=" * 80)

dt_vals = [(data[i+1]['timestamp'] - data[i]['timestamp'], i) for i in range(len(data)-1)]
dt_vals.sort(key=lambda x: x[0], reverse=True)

print(f"Frame interval: mean={sum(x[0] for x in dt_vals)/len(dt_vals):.6f}s")
print()
print("--- Top 15 largest timestamp gaps ---")
for dt, idx in dt_vals[:15]:
    d = data[idx+1]
    print(f"  Frame {idx:4d}->{idx+1:4d} t={data[idx]['timestamp']:.3f}->{d['timestamp']:.3f}  dt={dt:.6f}s  "
          f"global_match={int(d['global_match'])}  icp_time={d['icp_time']:.6f}s")

# 9. KEY FINDING: Correlation between global_match loss and drift
print()
print("=" * 80)
print("9. DRIFT vs GLOBAL_MATCH CORRELATION")
print("=" * 80)

# Find the transition point where global_match drops to 0
transition_frames = []
prev_gm = data[0]['global_match']
for i in range(1, len(data)):
    if data[i]['global_match'] == 0 and prev_gm > 0:
        transition_frames.append(i)
    prev_gm = data[i]['global_match']

if transition_frames:
    tf = transition_frames[0]
    print(f"Global match first drops to 0 at frame {tf} (t={data[tf]['timestamp']:.3f}s)")
    print()
    print(f"Position BEFORE loss (frame {tf-1}): ({data[tf-1]['pos_x']:.4f}, {data[tf-1]['pos_y']:.4f}, {data[tf-1]['pos_z']:.4f})")
    print(f"Position AT loss (frame {tf}):        ({data[tf]['pos_x']:.4f}, {data[tf]['pos_y']:.4f}, {data[tf]['pos_z']:.4f})")
    
    pos_at_loss = math.sqrt(data[tf]['pos_x']**2 + data[tf]['pos_y']**2 + data[tf]['pos_z']**2)
    pos_at_end = math.sqrt(data[-1]['pos_x']**2 + data[-1]['pos_y']**2 + data[-1]['pos_z']**2)
    
    # Show 10 frames before and after transition
    print()
    print("--- 10 frames before/after global_match loss ---")
    for i in range(max(0, tf-10), min(len(data), tf+10)):
        d = data[i]
        marker = " <<< LOSS" if i == tf else ""
        print(f"  Frame {d['frame']:4d} t={d['timestamp']:8.3f}  "
              f"pos=({d['pos_x']:8.4f},{d['pos_y']:8.4f},{d['pos_z']:8.4f})  "
              f"rot=({math.degrees(d['rot_x']):7.3f},{math.degrees(d['rot_y']):7.3f},{math.degrees(d['rot_z']):7.3f})deg  "
              f"|dp|={d['delta_pos_mag']:.6f}  gm={int(d['global_match'])}  feat={int(d['effct_feat_num'])}  "
              f"res={d['res_mean']:.5f}{marker}")
    
    print()
    print(f"|position| at global_match loss: {pos_at_loss:.4f}m")
    print(f"|position| at end: {pos_at_end:.4f}m")
    print(f"Drift AFTER global_match loss: {pos_at_end - pos_at_loss:.4f}m")
    print()
    
    # Compute drift rate after global match loss
    if tf < len(data) - 1:
        time_after_loss = data[-1]['timestamp'] - data[tf]['timestamp']
        drift_after = pos_at_end - pos_at_loss
        print(f"Time after loss: {time_after_loss:.3f}s")
        print(f"Average drift rate after loss: {drift_after/time_after_loss:.4f}m/s")
else:
    print("No global_match loss transition found - global_match may be 0 throughout or never lost.")

# 10. Check if drift is due to accumulated small errors vs. single jumps
print()
print("=" * 80)
print("10. DRIFT MECHANISM ANALYSIS")
print("=" * 80)

# Divide into segments and compute drift per segment
seg_size = max(1, len(data) // 10)
print(f"Drift per {seg_size}-frame segment:")
print(f"  {'Seg':>4s} {'Frames':>10s} {'Time':>8s} {'dPos_x':>10s} {'dPos_y':>10s} {'dPos_z':>10s} {'d|pos|':>10s} {'avg_gm':>8s} {'avg_feat':>9s} {'avg_res':>9s}")
for seg in range(0, len(data), seg_size):
    seg_data = data[seg:min(seg+seg_size+1, len(data))]
    d0, d1 = seg_data[0], seg_data[-1]
    dpx = d1['pos_x'] - d0['pos_x']
    dpy = d1['pos_y'] - d0['pos_y']
    dpz = d1['pos_z'] - d0['pos_z']
    dp_mag = math.sqrt(dpx**2 + dpy**2 + dpz**2)
    avg_gm = sum(d['global_match'] for d in seg_data) / len(seg_data)
    avg_feat = sum(d['effct_feat_num'] for d in seg_data) / len(seg_data)
    avg_res = sum(d['res_mean'] for d in seg_data) / len(seg_data)
    print(f"  {seg//seg_size:4d} {seg_data[0]['frame']:4d}-{seg_data[-1]['frame']:4d} "
          f"{d1['timestamp']-d0['timestamp']:8.3f}s {dpx:10.4f} {dpy:10.4f} {dpz:10.4f} {dp_mag:10.4f} "
          f"{avg_gm:8.1f} {avg_feat:9.1f} {avg_res:9.6f}")

# 11. Velocity analysis - is velocity diverging?
print()
print("=" * 80)
print("11. VELOCITY ANALYSIS")
print("=" * 80)
vel_mags = [math.sqrt(d['vel_x']**2 + d['vel_y']**2 + d['vel_z']**2) for d in data]
print(f"  |velocity|: min={min(vel_mags):.4f}, max={max(vel_mags):.4f}, end={vel_mags[-1]:.4f} m/s")
print(f"  Velocity at end: ({data[-1]['vel_x']:.4f}, {data[-1]['vel_y']:.4f}, {data[-1]['vel_z']:.4f}) m/s")
# Find when velocity starts growing
for i in range(0, len(data), N):
    d = data[i]
    print(f"  Frame {d['frame']:4d} t={d['timestamp']:7.3f}  vel=({d['vel_x']:+8.4f},{d['vel_y']:+8.4f},{d['vel_z']:+8.4f})  |vel|={vel_mags[i]:.4f}")

# 12. FINAL SUMMARY
print()
print("=" * 80)
print("12. SUMMARY AND ROOT CAUSE")
print("=" * 80)
