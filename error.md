[INFO] [1784522567.899934318]: [path_catalog] midpoint check: B4(1.0012,4.3750), B5(1.4988,4.3750) -> A1(1.2500,4.3750)
[INFO] [1784522567.901175694]: [path_catalog] midpoint check: B60(1.0012,0.1250), B61(1.4988,0.1250) -> A2(1.2500,0.1250)
[INFO] [1784522567.901901180]: [path_catalog] A1 virtual slot: id=101 row=0 dock=(1.2500,4.3750) pre=(1.2500,4.1000) yaw=90.0deg
[INFO] [1784522567.902029478]: [path_catalog] A2 virtual slot: id=102 row=7 dock=(1.2500,0.1250) pre=(1.2500,0.4000) yaw=-90.0deg
[WARN] [1784522567.902083676]: [path_catalog] selected depot=A1 direction=from_depot target_row=-1 targets (66): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65]
[WARN] [1784522567.902367971]: [planner][row1-debug] route=A1_TO_B src=101 tgt=61 src_corr=1 tgt_corr=4 target=(1.499,0.400 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.304 far_gap=0.584 horiz=0.374 min_x=0.676
[WARN] [1784522567.902514068]: [planner][row1-debug] initial_reverse tgt=61 same_corr=0 start_to_right=1 start_lane_y=3.791 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.890 actual_end=(0.890,3.791) curve_start=(1.250,4.129) curve_pts=56 direct_ok=1
[WARN] [1784522567.902672365]: [planner][row1-debug] center detour tgt=61 aux=(0.792,0.709) stage=(1.937,0.709) heading=180.0deg turn_pts=71
[WARN] [1784522567.902751264]: [planner][row1-debug] skeleton tgt=61 points=8 terminal_reverse=0 goal_lane_y=0.709 terminal_stop_y=0.400 final_ref_x=1.125
[WARN] [1784522567.902840662]: [planner][row1-debug] skeleton[0]=(0.890,3.791)
[WARN] [1784522567.902911060]: [planner][row1-debug] skeleton[1]=(2.155,3.791)
[WARN] [1784522567.902994659]: [planner][row1-debug] skeleton[2]=(2.155,2.950)
[WARN] [1784522567.903042658]: [planner][row1-debug] skeleton[3]=(1.250,2.950)
[WARN] [1784522567.903109157]: [planner][row1-debug] skeleton[4]=(1.250,1.550)
[WARN] [1784522567.903404951]: [planner][row1-debug] skeleton[5]=(1.125,1.550)
[WARN] [1784522567.903481049]: [planner][row1-debug] skeleton[6]=(1.125,0.709)
[WARN] [1784522567.903541148]: [planner][row1-debug] skeleton[7]=(0.792,0.709)
[WARN] [1784522567.903632846]: [planner][row1-debug] lane_shift rejected tgt=61 j=2 terminal=0 lateral=0.905 lead_in=0.472 lead_out=0.883
[WARN] [1784522567.903728944]: [planner][row1-debug] lane_shift accepted tgt=61 j=4 terminal=0 lateral=0.125 lead_in=0.252 lead_out=0.252
[WARN] [1784522567.904246334]: [planner][row1-debug] clothoid infeasible tgt=61 j=6 p=(1.125, 0.709) prev=(1.125, 1.550) next=(0.792, 0.709) prev_len=0.840 next_len=0.333 limit=0.330
[WARN] [1784522567.904380531]: [planner] slot 61: using local arc fallback only at infeasible turns; accepted lane shifts and clothoids are preserved
[WARN] [1784522567.904558828]: [planner][row1-debug] local arc fallback tgt=61 j=6 radius=0.150 max_radius=0.150 pts=41
[WARN] [1784522567.904852122]: [path_catalog][debug-layers] A1_to_B61 layers=6 arc=1
[WARN] [1784522567.904963021]: [path_catalog][debug-layers] layer[0] type=skeleton label=route_skeleton pts=8
[WARN] [1784522567.905012319]: [path_catalog][debug-layers] S0=(0.890,3.791)
[WARN] [1784522567.905088218]: [path_catalog][debug-layers] S1=(2.155,3.791)
[WARN] [1784522567.905182516]: [path_catalog][debug-layers] S2=(2.155,2.950)
[WARN] [1784522567.905275814]: [path_catalog][debug-layers] S3=(1.250,2.950)
[WARN] [1784522567.905358412]: [path_catalog][debug-layers] S4=(1.250,1.550)
[WARN] [1784522567.905451010]: [path_catalog][debug-layers] S5=(1.125,1.550)
[WARN] [1784522567.905509710]: [path_catalog][debug-layers] S6=(1.125,0.709)
[WARN] [1784522567.905618708]: [path_catalog][debug-layers] S7=(0.792,0.709)
[WARN] [1784522567.905693606]: [path_catalog][debug-layers] layer[1] type=clothoid label=clothoid pts=80
[WARN] [1784522567.905801504]: [path_catalog][debug-layers] layer[2] type=clothoid label=clothoid pts=59
[WARN] [1784522567.905876302]: [path_catalog][debug-layers] layer[3] type=clothoid label=clothoid pts=87
[WARN] [1784522567.905955401]: [path_catalog][debug-layers] layer[4] type=lane_shift label=lane_shift pts=52
[WARN] [1784522567.906038499]: [path_catalog][debug-layers] layer[5] type=arc_fallback label=local_arc_fallback pts=41
[WARN] [1784522567.906215696]: [path_catalog] single path A1_to_B61 row=7 col=5 wpts=595 len=7.738 arc=1 animate=1
[WARN] [1784522567.906578488]: [path_catalog] published single path markers on /forklift_map/markers
[INFO] [1784522568.299549710]: [forklift_map] Published 387 markers on /forklift_map/markers

[INFO] [1784522659.775584771]: [path_catalog] midpoint check: B60(1.0012,0.1250), B61(1.4988,0.1250) -> A2(1.2500,0.1250)
[INFO] [1784522659.776353456]: [path_catalog] A1 virtual slot: id=101 row=0 dock=(1.2500,4.3750) pre=(1.2500,4.1000) yaw=90.0deg
[INFO] [1784522659.776466454]: [path_catalog] A2 virtual slot: id=102 row=7 dock=(1.2500,0.1250) pre=(1.2500,0.4000) yaw=-90.0deg
[WARN] [1784522659.776574651]: [path_catalog] selected depot=A1 direction=from_depot target_row=-1 targets (66): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65]
[WARN] [1784522659.776804247]: [planner][row1-debug] route=A1_TO_B src=101 tgt=60 src_corr=1 tgt_corr=4 target=(1.001,0.400 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.584 far_gap=0.304 horiz=0.374 min_x=0.676
[WARN] [1784522659.776934044]: [planner][row1-debug] initial_reverse tgt=60 same_corr=0 start_to_right=0 start_lane_y=3.791 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.610 actual_end=(1.610,3.791) curve_start=(1.250,4.129) curve_pts=56 direct_ok=1
[WARN] [1784522659.777064042]: [planner][row1-debug] center detour tgt=60 aux=(1.708,0.709) stage=(0.563,0.709) heading=0.0deg turn_pts=71
[WARN] [1784522659.777177640]: [planner][row1-debug] skeleton tgt=60 points=8 terminal_reverse=0 goal_lane_y=0.709 terminal_stop_y=0.400 final_ref_x=1.375
[WARN] [1784522659.777254038]: [planner][row1-debug] skeleton[0]=(1.610,3.791)
[WARN] [1784522659.777372936]: [planner][row1-debug] skeleton[1]=(0.145,3.791)
[WARN] [1784522659.777432235]: [planner][row1-debug] skeleton[2]=(0.145,2.950)
[WARN] [1784522659.777507233]: [planner][row1-debug] skeleton[3]=(1.250,2.950)
[WARN] [1784522659.777580832]: [planner][row1-debug] skeleton[4]=(1.250,1.550)
[WARN] [1784522659.777691630]: [planner][row1-debug] skeleton[5]=(1.375,1.550)
[WARN] [1784522659.777766228]: [planner][row1-debug] skeleton[6]=(1.375,0.709)
[WARN] [1784522659.777828727]: [planner][row1-debug] skeleton[7]=(1.708,0.709)
[WARN] [1784522659.777936825]: [planner][row1-debug] lane_shift rejected tgt=60 j=2 terminal=0 lateral=1.105 lead_in=0.472 lead_out=1.025
[WARN] [1784522659.778000023]: [planner][row1-debug] lane_shift accepted tgt=60 j=4 terminal=0 lateral=0.125 lead_in=0.252 lead_out=0.252
[WARN] [1784522659.778894906]: [planner][row1-debug] clothoid infeasible tgt=60 j=6 p=(1.375, 0.709) prev=(1.375, 1.550) next=(1.708, 0.709) prev_len=0.840 next_len=0.333 limit=0.330
[WARN] [1784522659.779016203]: [planner] slot 60: using local arc fallback only at infeasible turns; accepted lane shifts and clothoids are preserved
[WARN] [1784522659.779153601]: [planner][row1-debug] local arc fallback tgt=60 j=6 radius=0.150 max_radius=0.150 pts=41
[WARN] [1784522659.779569293]: [path_catalog][debug-layers] A1_to_B60 layers=6 arc=1
[WARN] [1784522659.779658691]: [path_catalog][debug-layers] layer[0] type=skeleton label=route_skeleton pts=8
[WARN] [1784522659.779744389]: [path_catalog][debug-layers] S0=(1.610,3.791)
[WARN] [1784522659.779812088]: [path_catalog][debug-layers] S1=(0.145,3.791)
[WARN] [1784522659.779901886]: [path_catalog][debug-layers] S2=(0.145,2.950)
[WARN] [1784522659.779959285]: [path_catalog][debug-layers] S3=(1.250,2.950)
[WARN] [1784522659.780034884]: [path_catalog][debug-layers] S4=(1.250,1.550)
[WARN] [1784522659.780111082]: [path_catalog][debug-layers] S5=(1.375,1.550)
[WARN] [1784522659.780190081]: [path_catalog][debug-layers] S6=(1.375,0.709)
[WARN] [1784522659.780247579]: [path_catalog][debug-layers] S7=(1.708,0.709)
[WARN] [1784522659.780316078]: [path_catalog][debug-layers] layer[1] type=clothoid label=clothoid pts=69
[WARN] [1784522659.780387077]: [path_catalog][debug-layers] layer[2] type=clothoid label=clothoid pts=69
[WARN] [1784522659.780453475]: [path_catalog][debug-layers] layer[3] type=clothoid label=clothoid pts=87
[WARN] [1784522659.780528674]: [path_catalog][debug-layers] layer[4] type=lane_shift label=lane_shift pts=52
[WARN] [1784522659.780604872]: [path_catalog][debug-layers] layer[5] type=arc_fallback label=local_arc_fallback pts=41
[WARN] [1784522659.780750369]: [path_catalog] single path A1_to_B60 row=7 col=4 wpts=594 len=8.143 arc=1 animate=1
[WARN] [1784522659.781122862]: [path_catalog] published single path markers on /forklift_map/markers
[INFO] [1784522660.162081173]: [forklift_map] Published 387 markers on /forklift_map/markers

