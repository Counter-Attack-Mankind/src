[INFO] [1784609615.664164811]: [path_catalog] A1 virtual slot: id=101 row=0 dock=(1.2500,4.3750) pre=(1.2500,4.1000) yaw=90.0deg
[INFO] [1784609615.664271708]: [path_catalog] A2 virtual slot: id=102 row=7 dock=(1.2500,0.1250) pre=(1.2500,0.4000) yaw=-90.0deg
[WARN] [1784609615.664371606]: [path_catalog] selected depot=A1 direction=to_depot target_row=1 targets (5): [10, 12, 14, 16, 18]
[WARN] [1784609615.664640899]: [planner][row1-debug] route=B_TO_A1 debug_rev=row1_aux_lane_to_a1_v2 src=10 tgt=101 src_corr=1 tgt_corr=1 target=(1.250,4.100 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.584 far_gap=0.304 horiz=0.000 min_x=0.676
[WARN] [1784609615.664759196]: [planner][row1-debug] row1-upper first lane to A1 src=10 lower=3.791 upper=4.071 start_lane_y=4.071 source_rear_y=3.579
[WARN] [1784609615.664926092]: [planner][row1-debug] initial_reverse tgt=101 same_corr=1 start_to_right=1 start_lane_y=4.071 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.706 actual_end=(0.353,4.071) reverse_origin=(0.706,3.579) curve_start=(0.706,3.717) curve_pts=52 direct_ok=0
[WARN] [1784609615.664985391]: [planner][row1-debug] skeleton tgt=101 points=3 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=4.100 final_ref_x=1.250
[WARN] [1784609615.665084188]: [planner][row1-debug] skeleton[0]=(0.353,4.071)
[WARN] [1784609615.665198585]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1784609615.665246584]: [planner][row1-debug] skeleton[2]=(1.250,4.301)
[WARN] [1784609615.665405880]: [planner][row1-debug] exact A1 lane using terminal-constrained G2 fallback src=10 din=-1 t_in=0.300 t_out=0.300 mt_port=0.300 available_out=0.300 rear_stop_y=4.301 max_safe_rear_y=4.311
[WARN] [1784609615.665549777]: [planner][row1-debug] row1-upper to A1 road lane_start=(0.353,4.071) first=(1.865,4.071) second=(1.865,3.791) curve_start=(1.550,3.791) r=0.126 len=(1.512,0.280,0.315)
[WARN] [1784609615.665687473]: [planner][row1-debug] exact A1 lane src=10 din=-1 lane_start=(0.353,4.071) curve_start=(1.550,3.791) curve_end=(1.250,4.091) rear_stop_y=4.301 max_safe_rear_y=4.311 mt_port=0.300 available_out=0.300 finish_mode=forward_extend pts=267
[WARN] [1784609615.665789071]: [planner][row1-debug] prepend enter src=10 row1_upper=1 raw_pts=66 a_rev=(0.706,3.579) b=(0.353,4.071) path_front=(0.353,4.071 0.0deg type=0)
[WARN] [1784609615.665908068]: [planner][row1-debug] chord reverse prefix src=10 pts=66 first=(0.706,3.579 -90.0deg) last=(0.353,4.071 0.0deg)
[WARN] [1784609615.666013465]: [planner][row1-debug] prefix[0]=(0.706,3.579 -90.0deg type=1)
[WARN] [1784609615.666068864]: [planner][row1-debug] prefix[1]=(0.706,3.589 -90.0deg type=1)
[WARN] [1784609615.666172061]: [planner][row1-debug] prefix[2]=(0.706,3.599 -90.0deg type=1)
[WARN] [1784609615.666242960]: [planner][row1-debug] prefix[3]=(0.706,3.609 -90.0deg type=1)
[WARN] [1784609615.666330957]: [planner][row1-debug] prefix[18]=(0.706,3.782 -89.6deg type=1)
[WARN] [1784609615.666402055]: [planner][row1-debug] prefix[19]=(0.706,3.798 -89.3deg type=1)
[WARN] [1784609615.666490253]: [planner][row1-debug] prefix[20]=(0.706,3.813 -89.0deg type=1)
[WARN] [1784609615.666565251]: [path_catalog][debug-layers] B10_to_A1 layers=1 arc=0
[WARN] [1784609615.666644650]: [path_catalog][debug-layers] layer[0] type=skeleton label=route_skeleton pts=3
[WARN] [1784609615.666716448]: [path_catalog][debug-layers] S0=(0.353,4.071)
[WARN] [1784609615.666794845]: [path_catalog][debug-layers] S1=(1.250,4.071)
[WARN] [1784609615.666840345]: [path_catalog][debug-layers] S2=(1.250,4.301)
[WARN] [1784609615.666990540]: [path_catalog] single path B10_to_A1 row=1 col=0 wpts=333 len=3.519 arc=0 animate=1
[WARN] [1784609615.667191136]: [path_catalog] published single path markers on /forklift_map/markers
[INFO] [1784609616.063428415]: [forklift_map] Published 387 markers on /forklift_map/markers

[INFO] [1784610679.089843131]: [path_catalog] A1 virtual slot: id=101 row=0 dock=(1.2500,4.3750) pre=(1.2500,4.1000) yaw=90.0deg
[INFO] [1784610679.089951028]: [path_catalog] A2 virtual slot: id=102 row=7 dock=(1.2500,0.1250) pre=(1.2500,0.4000) yaw=-90.0deg
[WARN] [1784610679.090003427]: [path_catalog] selected depot=A1 direction=from_depot target_row=1 targets (5): [10, 12, 14, 16, 18]
[WARN] [1784610679.090250721]: [planner][row1-debug] route=A1_TO_B src=101 tgt=10 src_corr=1 tgt_corr=1 target=(0.706,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.491 far_gap=0.211 horiz=0.544 min_x=0.676
[WARN] [1784610679.090369518]: [planner][row1-debug] row1-upper direct lane tgt=10 lower=3.791 upper=4.071 connector_mid_y=3.370
[WARN] [1784610679.090435517]: [planner][row1-debug] initial_reverse tgt=10 same_corr=1 start_to_right=0 start_lane_y=3.370 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.865 actual_end=(1.865,3.370) curve_start=(1.250,3.709) curve_pts=56 direct_ok=1
[WARN] [1784610679.090545214]: [planner][row1-debug] skeleton tgt=10 points=4 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=3.761 final_ref_x=1.250
[WARN] [1784610679.090601613]: [planner][row1-debug] skeleton[0]=(1.865,3.370)
[WARN] [1784610679.090682110]: [planner][row1-debug] skeleton[1]=(1.865,4.071)
[WARN] [1784610679.090737809]: [planner][row1-debug] skeleton[2]=(0.706,4.071)
[WARN] [1784610679.090801408]: [planner][row1-debug] skeleton[3]=(0.706,3.579)
[WARN] [1784610679.091061902]: [planner][row1-debug] row1-upper reverse road a_rev=(1.250,4.301) second=(1.250,3.791)->(1.865,3.791) s0=(1.865,3.370) r=0.189 len=(0.511,0.615,0.420) pts=142 front_theta=90.0deg
[WARN] [1784610679.091189698]: [path_catalog][debug-layers] A1_to_B10 layers=3 arc=0
[WARN] [1784610679.091306496]: [path_catalog][debug-layers] layer[0] type=skeleton label=route_skeleton pts=4
[WARN] [1784610679.091362494]: [path_catalog][debug-layers] S0=(1.865,3.370)
[WARN] [1784610679.091457492]: [path_catalog][debug-layers] S1=(1.865,4.071)
[WARN] [1784610679.091513991]: [path_catalog][debug-layers] S2=(0.706,4.071)
[WARN] [1784610679.091561190]: [path_catalog][debug-layers] S3=(0.706,3.579)
[WARN] [1784610679.091607689]: [path_catalog][debug-layers] layer[1] type=clothoid label=clothoid pts=87
[WARN] [1784610679.091704586]: [path_catalog][debug-layers] layer[2] type=clothoid label=clothoid pts=81
[WARN] [1784610679.091792384]: [path_catalog] single path A1_to_B10 row=1 col=0 wpts=312 len=3.406 arc=0 animate=1
[WARN] [1784610679.092225974]: [path_catalog] published single path markers on /forklift_map/markers
[INFO] [1784610679.475216373]: [forklift_map] Published 387 markers on /forklift_map/markers

