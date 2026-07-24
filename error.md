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

[multi_patrol] coordination log started
vehicle_count=2 one_shot=0 use_a1_cycle=1
[multi_patrol][state] tick=1 sim_t=0.10 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=0 slot=0->47 s=0.002/11.728 rem=11.726 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=1 sim_t=0.10 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=0 slot=8->48 s=0.002/9.570 rem=9.568 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=23 sim_t=2.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.318/9.570 rem=9.252 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] tick=26 sim_t=2.60 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=0 slot=8->48 s=0.334/9.570 rem=9.236 speed=0.022 wait=0.40 dwell=0.00
[multi_patrol][state] tick=28 sim_t=2.80 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=0 slot=8->48 s=0.336/9.570 rem=9.234 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] tick=29 sim_t=2.90 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=0 slot=8->48 s=0.340/9.570 rem=9.230 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] tick=30 sim_t=3.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.341/9.570 rem=9.229 speed=0.010 wait=0.80 dwell=0.00
[multi_patrol][state] tick=31 sim_t=3.10 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=0 slot=8->48 s=0.341/9.570 rem=9.229 speed=0.000 wait=0.90 dwell=0.00
[multi_patrol][state] tick=35 sim_t=3.50 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=0 slot=8->48 s=0.343/9.570 rem=9.227 speed=0.020 wait=1.30 dwell=0.00
[multi_patrol][state] tick=36 sim_t=3.60 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=0 slot=8->48 s=0.347/9.570 rem=9.223 speed=0.040 wait=1.40 dwell=0.00
[multi_patrol][state] tick=37 sim_t=3.70 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.010 wait=1.50 dwell=0.00
[multi_patrol][state] tick=57 sim_t=5.70 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=3.50 dwell=0.00
[multi_patrol][state] tick=78 sim_t=7.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=5.60 dwell=0.00
[multi_patrol][state] tick=99 sim_t=9.90 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=7.70 dwell=0.00
[multi_patrol][state] tick=119 sim_t=11.90 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=9.70 dwell=0.00
[multi_patrol][state] tick=139 sim_t=13.90 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=11.70 dwell=0.00
[multi_patrol][state] tick=160 sim_t=16.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=13.80 dwell=0.00
[multi_patrol][state] tick=181 sim_t=18.10 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=15.90 dwell=0.00
[multi_patrol][state] tick=202 sim_t=20.20 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=18.00 dwell=0.00
[multi_patrol][state] tick=223 sim_t=22.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=20.10 dwell=0.00
[multi_patrol][state] tick=243 sim_t=24.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=22.10 dwell=0.00
[multi_patrol][state] tick=264 sim_t=26.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=0 slot=8->48 s=0.348/9.570 rem=9.222 speed=0.000 wait=24.20 dwell=0.00
[multi_patrol][state] tick=278 sim_t=27.80 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=0 slot=8->48 s=0.350/9.570 rem=9.220 speed=0.020 wait=25.60 dwell=0.00
[multi_patrol][state] tick=279 sim_t=27.90 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=0 slot=8->48 s=0.354/9.570 rem=9.216 speed=0.040 wait=25.70 dwell=0.00
[multi_patrol][state] tick=283 sim_t=28.30 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=0 slot=8->48 s=0.376/9.570 rem=9.194 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=668 sim_t=66.80 V0 mode=DWELL action=STOP reason=dwell blocker=-1 task=1 slot=47->47 s=11.728/11.728 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=669 sim_t=66.90 V0 mode=DWELL action=STOP reason=not_active blocker=-1 task=1 slot=47->47 s=11.728/11.728 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=718 sim_t=71.80 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=1 slot=47->60 s=0.002/15.025 rem=15.023 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=814 sim_t=81.40 V1 mode=DWELL action=STOP reason=dwell blocker=-1 task=1 slot=48->48 s=9.570/9.570 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=815 sim_t=81.50 V1 mode=DWELL action=STOP reason=not_active blocker=-1 task=1 slot=48->48 s=9.570/9.570 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=864 sim_t=86.40 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=1 slot=48->50 s=0.002/11.821 rem=11.819 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=884 sim_t=88.40 V1 mode=ACTIVE action=STOP reason=clear_block_V0 blocker=0 task=1 slot=48->50 s=0.284/11.821 rem=11.537 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] tick=886 sim_t=88.60 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=1 slot=48->50 s=0.298/11.821 rem=11.523 speed=0.052 wait=0.30 dwell=0.00
[multi_patrol][state] tick=889 sim_t=88.90 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=1 slot=48->50 s=0.302/11.821 rem=11.519 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] tick=890 sim_t=89.00 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=1 slot=48->50 s=0.306/11.821 rem=11.515 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] tick=894 sim_t=89.40 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=1 slot=48->50 s=0.328/11.821 rem=11.493 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=896 sim_t=89.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.343/11.821 rem=11.478 speed=0.060 wait=0.10 dwell=0.00
[multi_patrol][state] tick=899 sim_t=89.90 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=1 slot=48->50 s=0.346/11.821 rem=11.475 speed=0.000 wait=0.40 dwell=0.00
[multi_patrol][state] tick=901 sim_t=90.10 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] tick=902 sim_t=90.20 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=0.70 dwell=0.00
[multi_patrol][state] tick=923 sim_t=92.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=2.80 dwell=0.00
[multi_patrol][state] tick=944 sim_t=94.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=4.90 dwell=0.00
[multi_patrol][state] tick=965 sim_t=96.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=7.00 dwell=0.00
[multi_patrol][state] tick=986 sim_t=98.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=9.10 dwell=0.00
[multi_patrol][state] tick=1007 sim_t=100.70 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=11.20 dwell=0.00
[multi_patrol][state] tick=1028 sim_t=102.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=13.30 dwell=0.00
[multi_patrol][state] tick=1048 sim_t=104.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=15.30 dwell=0.00
[multi_patrol][state] tick=1069 sim_t=106.90 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=17.40 dwell=0.00
[multi_patrol][state] tick=1090 sim_t=109.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=19.50 dwell=0.00
[multi_patrol][state] tick=1110 sim_t=111.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=21.50 dwell=0.00
[multi_patrol][state] tick=1130 sim_t=113.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=23.50 dwell=0.00
[multi_patrol][state] tick=1150 sim_t=115.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=25.50 dwell=0.00
[multi_patrol][state] tick=1171 sim_t=117.10 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=27.60 dwell=0.00
[multi_patrol][state] tick=1191 sim_t=119.10 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=29.60 dwell=0.00
[multi_patrol][state] tick=1211 sim_t=121.10 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=31.60 dwell=0.00
[multi_patrol][state] tick=1232 sim_t=123.20 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=33.70 dwell=0.00
[multi_patrol][state] tick=1253 sim_t=125.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=35.80 dwell=0.00
[multi_patrol][state] tick=1273 sim_t=127.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=37.80 dwell=0.00
[multi_patrol][state] tick=1294 sim_t=129.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=39.90 dwell=0.00
[multi_patrol][state] tick=1314 sim_t=131.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=41.90 dwell=0.00
[multi_patrol][state] tick=1335 sim_t=133.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=44.00 dwell=0.00
[multi_patrol][state] tick=1356 sim_t=135.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=46.10 dwell=0.00
[multi_patrol][state] tick=1376 sim_t=137.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=1 slot=48->50 s=0.348/11.821 rem=11.473 speed=0.000 wait=48.10 dwell=0.00
[multi_patrol][state] tick=1384 sim_t=138.40 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=1 slot=48->50 s=0.350/11.821 rem=11.471 speed=0.020 wait=48.90 dwell=0.00
[multi_patrol][state] tick=1385 sim_t=138.50 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=1 slot=48->50 s=0.354/11.821 rem=11.467 speed=0.040 wait=49.00 dwell=0.00
[multi_patrol][state] tick=1389 sim_t=138.90 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=1 slot=48->50 s=0.376/11.821 rem=11.445 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=1589 sim_t=158.90 V0 mode=DWELL action=STOP reason=dwell blocker=-1 task=2 slot=60->60 s=15.025/15.025 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=1590 sim_t=159.00 V0 mode=DWELL action=STOP reason=not_active blocker=-1 task=2 slot=60->60 s=15.025/15.025 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=1639 sim_t=163.90 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=2 slot=60->8 s=0.002/8.403 rem=8.401 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=1763 sim_t=176.30 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.008/8.403 rem=6.395 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] tick=1765 sim_t=176.50 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=2 slot=60->8 s=2.033/8.403 rem=6.370 speed=0.110 wait=0.30 dwell=0.00
[multi_patrol][state] tick=1768 sim_t=176.80 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=2 slot=60->8 s=2.051/8.403 rem=6.352 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] tick=1769 sim_t=176.90 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=2 slot=60->8 s=2.056/8.403 rem=6.347 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=1772 sim_t=177.20 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.068/8.403 rem=6.335 speed=0.020 wait=1.00 dwell=0.00
[multi_patrol][state] tick=1773 sim_t=177.30 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=2 slot=60->8 s=2.068/8.403 rem=6.335 speed=0.000 wait=1.10 dwell=0.00
[multi_patrol][state] tick=1777 sim_t=177.70 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=2 slot=60->8 s=2.070/8.403 rem=6.333 speed=0.020 wait=1.50 dwell=0.00
[multi_patrol][state] tick=1778 sim_t=177.80 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.070/8.403 rem=6.333 speed=0.000 wait=1.60 dwell=0.00
[multi_patrol][state] tick=1779 sim_t=177.90 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=2 slot=60->8 s=2.070/8.403 rem=6.333 speed=0.000 wait=1.70 dwell=0.00
[multi_patrol][state] tick=1783 sim_t=178.30 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.020 wait=2.10 dwell=0.00
[multi_patrol][state] tick=1784 sim_t=178.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] tick=1805 sim_t=180.50 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=4.30 dwell=0.00
[multi_patrol][state] tick=1825 sim_t=182.50 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=6.30 dwell=0.00
[multi_patrol][state] tick=1845 sim_t=184.50 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=8.30 dwell=0.00
[multi_patrol][state] tick=1866 sim_t=186.60 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=10.40 dwell=0.00
[multi_patrol][state] tick=1886 sim_t=188.60 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=12.40 dwell=0.00
[multi_patrol][state] tick=1907 sim_t=190.70 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=14.50 dwell=0.00
[multi_patrol][state] tick=1928 sim_t=192.80 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=16.60 dwell=0.00
[multi_patrol][state] tick=1949 sim_t=194.90 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=18.70 dwell=0.00
[multi_patrol][state] tick=1970 sim_t=197.00 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=20.80 dwell=0.00
[multi_patrol][state] tick=1990 sim_t=199.00 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=2 slot=60->8 s=2.072/8.403 rem=6.331 speed=0.000 wait=22.80 dwell=0.00
[multi_patrol][state] tick=2001 sim_t=200.10 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=2 slot=60->8 s=2.074/8.403 rem=6.329 speed=0.020 wait=23.90 dwell=0.00
[multi_patrol][state] tick=2002 sim_t=200.20 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=2 slot=60->8 s=2.078/8.403 rem=6.325 speed=0.040 wait=24.00 dwell=0.00
[multi_patrol][state] tick=2006 sim_t=200.60 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=2 slot=60->8 s=2.100/8.403 rem=6.303 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=2079 sim_t=207.90 V1 mode=DWELL action=STOP reason=dwell blocker=-1 task=2 slot=50->50 s=11.821/11.821 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=2080 sim_t=208.00 V1 mode=DWELL action=STOP reason=not_active blocker=-1 task=2 slot=50->50 s=11.821/11.821 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=2129 sim_t=212.90 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=2 slot=50->55 s=0.002/12.020 rem=12.018 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=2358 sim_t=235.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=2 slot=50->55 s=3.681/12.020 rem=8.339 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] tick=2361 sim_t=236.10 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=2 slot=50->55 s=3.714/12.020 rem=8.306 speed=0.080 wait=0.40 dwell=0.00
[multi_patrol][state] tick=2363 sim_t=236.30 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=2 slot=50->55 s=3.724/12.020 rem=8.296 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] tick=2364 sim_t=236.40 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=2 slot=50->55 s=3.729/12.020 rem=8.291 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=2368 sim_t=236.80 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=2 slot=50->55 s=3.751/12.020 rem=8.269 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=2382 sim_t=238.20 V0 mode=DWELL action=STOP reason=dwell blocker=-1 task=3 slot=8->8 s=8.403/8.403 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=2383 sim_t=238.30 V0 mode=DWELL action=STOP reason=not_active blocker=-1 task=3 slot=8->8 s=8.403/8.403 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=2432 sim_t=243.20 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=3 slot=8->23 s=0.002/8.972 rem=8.970 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=2444 sim_t=244.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=3 slot=8->23 s=0.167/8.972 rem=8.805 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] tick=2448 sim_t=244.80 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=3 slot=8->23 s=0.205/8.972 rem=8.767 speed=0.050 wait=0.50 dwell=0.00
[multi_patrol][state] tick=2449 sim_t=244.90 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=3 slot=8->23 s=0.210/8.972 rem=8.762 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] tick=2450 sim_t=245.00 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=3 slot=8->23 s=0.215/8.972 rem=8.757 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=2451 sim_t=245.10 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=3 slot=8->23 s=0.217/8.972 rem=8.755 speed=0.020 wait=0.80 dwell=0.00
[multi_patrol][state] tick=2452 sim_t=245.20 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=3 slot=8->23 s=0.217/8.972 rem=8.755 speed=0.000 wait=0.90 dwell=0.00
[multi_patrol][state] tick=2456 sim_t=245.60 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=3 slot=8->23 s=0.219/8.972 rem=8.753 speed=0.020 wait=1.30 dwell=0.00
[multi_patrol][state] tick=2457 sim_t=245.70 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=3 slot=8->23 s=0.219/8.972 rem=8.753 speed=0.000 wait=1.40 dwell=0.00
[multi_patrol][state] tick=2458 sim_t=245.80 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=3 slot=8->23 s=0.219/8.972 rem=8.753 speed=0.000 wait=1.50 dwell=0.00
[multi_patrol][state] tick=2462 sim_t=246.20 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=3 slot=8->23 s=0.221/8.972 rem=8.751 speed=0.020 wait=1.90 dwell=0.00
[multi_patrol][state] tick=2463 sim_t=246.30 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=3 slot=8->23 s=0.221/8.972 rem=8.751 speed=0.000 wait=2.00 dwell=0.00
[multi_patrol][state] tick=2484 sim_t=248.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=3 slot=8->23 s=0.221/8.972 rem=8.751 speed=0.000 wait=4.10 dwell=0.00
[multi_patrol][state] tick=2505 sim_t=250.50 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=3 slot=8->23 s=0.221/8.972 rem=8.751 speed=0.000 wait=6.20 dwell=0.00
[multi_patrol][state] tick=2525 sim_t=252.50 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=3 slot=8->23 s=0.221/8.972 rem=8.751 speed=0.000 wait=8.20 dwell=0.00
[multi_patrol][state] tick=2546 sim_t=254.60 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=3 slot=8->23 s=0.221/8.972 rem=8.751 speed=0.000 wait=10.30 dwell=0.00
[multi_patrol][state] tick=2566 sim_t=256.60 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=3 slot=8->23 s=0.221/8.972 rem=8.751 speed=0.000 wait=12.30 dwell=0.00
[multi_patrol][state] tick=2578 sim_t=257.80 V0 mode=ACTIVE action=CREEP reason=following_V1 blocker=1 task=3 slot=8->23 s=0.223/8.972 rem=8.749 speed=0.020 wait=13.50 dwell=0.00
[multi_patrol][state] tick=2583 sim_t=258.30 V0 mode=ACTIVE action=YIELD reason=following_V1 blocker=1 task=3 slot=8->23 s=0.249/8.972 rem=8.723 speed=0.070 wait=14.00 dwell=0.00
[multi_patrol][state] tick=2589 sim_t=258.90 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=3 slot=8->23 s=0.310/8.972 rem=8.662 speed=0.120 wait=0.00 dwell=0.00
[multi_patrol][state] tick=2864 sim_t=286.40 V1 mode=DWELL action=STOP reason=dwell blocker=-1 task=3 slot=55->55 s=12.020/12.020 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=2865 sim_t=286.50 V1 mode=DWELL action=STOP reason=not_active blocker=-1 task=3 slot=55->55 s=12.020/12.020 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=2914 sim_t=291.40 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=3 slot=55->24 s=0.002/10.550 rem=10.548 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=3094 sim_t=309.40 V0 mode=DWELL action=STOP reason=dwell blocker=-1 task=4 slot=23->23 s=8.972/8.972 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=3095 sim_t=309.50 V0 mode=DWELL action=STOP reason=not_active blocker=-1 task=4 slot=23->23 s=8.972/8.972 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=3144 sim_t=314.40 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=4 slot=23->36 s=0.002/10.377 rem=10.375 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=3313 sim_t=331.30 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.932/10.377 rem=7.445 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] tick=3315 sim_t=331.50 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=4 slot=23->36 s=2.957/10.377 rem=7.420 speed=0.110 wait=0.30 dwell=0.00
[multi_patrol][state] tick=3318 sim_t=331.80 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=23->36 s=2.975/10.377 rem=7.402 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] tick=3319 sim_t=331.90 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=4 slot=23->36 s=2.980/10.377 rem=7.397 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=3322 sim_t=332.20 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.992/10.377 rem=7.385 speed=0.020 wait=1.00 dwell=0.00
[multi_patrol][state] tick=3323 sim_t=332.30 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=4 slot=23->36 s=2.992/10.377 rem=7.385 speed=0.000 wait=1.10 dwell=0.00
[multi_patrol][state] tick=3327 sim_t=332.70 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=23->36 s=2.994/10.377 rem=7.383 speed=0.020 wait=1.50 dwell=0.00
[multi_patrol][state] tick=3328 sim_t=332.80 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.994/10.377 rem=7.383 speed=0.000 wait=1.60 dwell=0.00
[multi_patrol][state] tick=3329 sim_t=332.90 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=4 slot=23->36 s=2.994/10.377 rem=7.383 speed=0.000 wait=1.70 dwell=0.00
[multi_patrol][state] tick=3333 sim_t=333.30 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=23->36 s=2.996/10.377 rem=7.381 speed=0.020 wait=2.10 dwell=0.00
[multi_patrol][state] tick=3334 sim_t=333.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.996/10.377 rem=7.381 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] tick=3335 sim_t=333.50 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=4 slot=23->36 s=2.996/10.377 rem=7.381 speed=0.000 wait=2.30 dwell=0.00
[multi_patrol][state] tick=3339 sim_t=333.90 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.020 wait=2.70 dwell=0.00
[multi_patrol][state] tick=3340 sim_t=334.00 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=2.80 dwell=0.00
[multi_patrol][state] tick=3361 sim_t=336.10 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=4.90 dwell=0.00
[multi_patrol][state] tick=3382 sim_t=338.20 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=7.00 dwell=0.00
[multi_patrol][state] tick=3403 sim_t=340.30 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=9.10 dwell=0.00
[multi_patrol][state] tick=3424 sim_t=342.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=11.20 dwell=0.00
[multi_patrol][state] tick=3444 sim_t=344.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=13.20 dwell=0.00
[multi_patrol][state] tick=3464 sim_t=346.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=15.20 dwell=0.00
[multi_patrol][state] tick=3485 sim_t=348.50 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=17.30 dwell=0.00
[multi_patrol][state] tick=3506 sim_t=350.60 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=19.40 dwell=0.00
[multi_patrol][state] tick=3526 sim_t=352.60 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=4 slot=23->36 s=2.998/10.377 rem=7.379 speed=0.000 wait=21.40 dwell=0.00
[multi_patrol][state] tick=3535 sim_t=353.50 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=23->36 s=3.000/10.377 rem=7.377 speed=0.020 wait=22.30 dwell=0.00
[multi_patrol][state] tick=3536 sim_t=353.60 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=4 slot=23->36 s=3.004/10.377 rem=7.373 speed=0.040 wait=22.40 dwell=0.00
[multi_patrol][state] tick=3540 sim_t=354.00 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=4 slot=23->36 s=3.026/10.377 rem=7.351 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=3550 sim_t=355.00 V1 mode=DWELL action=STOP reason=dwell blocker=-1 task=4 slot=24->24 s=10.550/10.550 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=3551 sim_t=355.10 V1 mode=DWELL action=STOP reason=not_active blocker=-1 task=4 slot=24->24 s=10.550/10.550 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=3600 sim_t=360.00 V1 mode=ACTIVE action=STOP reason=clear_block_V0 blocker=0 task=4 slot=24->26 s=0.000/7.129 rem=7.129 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] tick=3604 sim_t=360.40 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=4 slot=24->26 s=0.000/7.129 rem=7.129 speed=0.000 wait=0.50 dwell=0.00
[multi_patrol][state] tick=3605 sim_t=360.50 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=24->26 s=0.002/7.129 rem=7.127 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] tick=3606 sim_t=360.60 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=4 slot=24->26 s=0.006/7.129 rem=7.123 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] tick=3610 sim_t=361.00 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=4 slot=24->26 s=0.028/7.129 rem=7.101 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=3616 sim_t=361.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.107/7.129 rem=7.022 speed=0.140 wait=0.10 dwell=0.00
[multi_patrol][state] tick=3619 sim_t=361.90 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=4 slot=24->26 s=0.131/7.129 rem=6.998 speed=0.050 wait=0.40 dwell=0.00
[multi_patrol][state] tick=3621 sim_t=362.10 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=24->26 s=0.137/7.129 rem=6.992 speed=0.040 wait=0.60 dwell=0.00
[multi_patrol][state] tick=3622 sim_t=362.20 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=4 slot=24->26 s=0.142/7.129 rem=6.987 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=3623 sim_t=362.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.144/7.129 rem=6.985 speed=0.020 wait=0.80 dwell=0.00
[multi_patrol][state] tick=3625 sim_t=362.50 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=4 slot=24->26 s=0.144/7.129 rem=6.985 speed=0.000 wait=1.00 dwell=0.00
[multi_patrol][state] tick=3628 sim_t=362.80 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.020 wait=1.30 dwell=0.00
[multi_patrol][state] tick=3629 sim_t=362.90 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=1.40 dwell=0.00
[multi_patrol][state] tick=3650 sim_t=365.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=3.50 dwell=0.00
[multi_patrol][state] tick=3670 sim_t=367.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=5.50 dwell=0.00
[multi_patrol][state] tick=3691 sim_t=369.10 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=7.60 dwell=0.00
[multi_patrol][state] tick=3711 sim_t=371.10 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=9.60 dwell=0.00
[multi_patrol][state] tick=3731 sim_t=373.10 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=11.60 dwell=0.00
[multi_patrol][state] tick=3752 sim_t=375.20 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=13.70 dwell=0.00
[multi_patrol][state] tick=3773 sim_t=377.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=15.80 dwell=0.00
[multi_patrol][state] tick=3794 sim_t=379.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=17.90 dwell=0.00
[multi_patrol][state] tick=3815 sim_t=381.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=20.00 dwell=0.00
[multi_patrol][state] tick=3836 sim_t=383.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=22.10 dwell=0.00
[multi_patrol][state] tick=3857 sim_t=385.70 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=24.20 dwell=0.00
[multi_patrol][state] tick=3878 sim_t=387.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=26.30 dwell=0.00
[multi_patrol][state] tick=3898 sim_t=389.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=0.146/7.129 rem=6.983 speed=0.000 wait=28.30 dwell=0.00
[multi_patrol][state] tick=3908 sim_t=390.80 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=24->26 s=0.148/7.129 rem=6.981 speed=0.020 wait=29.30 dwell=0.00
[multi_patrol][state] tick=3909 sim_t=390.90 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=4 slot=24->26 s=0.152/7.129 rem=6.977 speed=0.040 wait=29.40 dwell=0.00
[multi_patrol][state] tick=3913 sim_t=391.30 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=4 slot=24->26 s=0.174/7.129 rem=6.955 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=3987 sim_t=398.70 V0 mode=DWELL action=STOP reason=dwell blocker=-1 task=5 slot=36->36 s=10.377/10.377 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=3988 sim_t=398.80 V0 mode=DWELL action=STOP reason=not_active blocker=-1 task=5 slot=36->36 s=10.377/10.377 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=4037 sim_t=403.70 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=5 slot=36->14 s=0.002/6.161 rem=6.159 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=4058 sim_t=405.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.441/7.129 rem=4.688 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] tick=4062 sim_t=406.20 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=4 slot=24->26 s=2.479/7.129 rem=4.650 speed=0.050 wait=0.50 dwell=0.00
[multi_patrol][state] tick=4063 sim_t=406.30 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=24->26 s=2.484/7.129 rem=4.645 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] tick=4064 sim_t=406.40 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=4 slot=24->26 s=2.489/7.129 rem=4.640 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=4065 sim_t=406.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.491/7.129 rem=4.638 speed=0.020 wait=0.80 dwell=0.00
[multi_patrol][state] tick=4066 sim_t=406.60 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=4 slot=24->26 s=2.491/7.129 rem=4.638 speed=0.000 wait=0.90 dwell=0.00
[multi_patrol][state] tick=4070 sim_t=407.00 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=24->26 s=2.493/7.129 rem=4.636 speed=0.020 wait=1.30 dwell=0.00
[multi_patrol][state] tick=4071 sim_t=407.10 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=4 slot=24->26 s=2.497/7.129 rem=4.632 speed=0.040 wait=1.40 dwell=0.00
[multi_patrol][state] tick=4072 sim_t=407.20 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.010 wait=1.50 dwell=0.00
[multi_patrol][state] tick=4093 sim_t=409.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=3.60 dwell=0.00
[multi_patrol][state] tick=4113 sim_t=411.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=5.60 dwell=0.00
[multi_patrol][state] tick=4134 sim_t=413.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=7.70 dwell=0.00
[multi_patrol][state] tick=4154 sim_t=415.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=9.70 dwell=0.00
[multi_patrol][state] tick=4174 sim_t=417.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=11.70 dwell=0.00
[multi_patrol][state] tick=4195 sim_t=419.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=13.80 dwell=0.00
[multi_patrol][state] tick=4215 sim_t=421.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=15.80 dwell=0.00
[multi_patrol][state] tick=4236 sim_t=423.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=17.90 dwell=0.00
[multi_patrol][state] tick=4256 sim_t=425.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=19.90 dwell=0.00
[multi_patrol][state] tick=4276 sim_t=427.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=21.90 dwell=0.00
[multi_patrol][state] tick=4297 sim_t=429.70 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=24.00 dwell=0.00
[multi_patrol][state] tick=4318 sim_t=431.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=26.10 dwell=0.00
[multi_patrol][state] tick=4338 sim_t=433.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=28.10 dwell=0.00
[multi_patrol][state] tick=4358 sim_t=435.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=30.10 dwell=0.00
[multi_patrol][state] tick=4378 sim_t=437.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=32.10 dwell=0.00
[multi_patrol][state] tick=4399 sim_t=439.90 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=34.20 dwell=0.00
[multi_patrol][state] tick=4419 sim_t=441.90 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=4 slot=24->26 s=2.498/7.129 rem=4.631 speed=0.000 wait=36.20 dwell=0.00
[multi_patrol][state] tick=4431 sim_t=443.10 V0 mode=DWELL action=STOP reason=dwell blocker=-1 task=6 slot=14->14 s=6.161/6.161 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=4431 sim_t=443.10 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=4 slot=24->26 s=2.500/7.129 rem=4.629 speed=0.020 wait=37.40 dwell=0.00
[multi_patrol][state] tick=4432 sim_t=443.20 V0 mode=DWELL action=STOP reason=not_active blocker=-1 task=6 slot=14->14 s=6.161/6.161 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=4432 sim_t=443.20 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=4 slot=24->26 s=2.504/7.129 rem=4.625 speed=0.040 wait=37.50 dwell=0.00
[multi_patrol][state] tick=4436 sim_t=443.60 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=4 slot=24->26 s=2.526/7.129 rem=4.603 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=4481 sim_t=448.10 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=6 slot=14->39 s=0.000/9.053 rem=9.053 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] tick=4502 sim_t=450.20 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=6 slot=14->39 s=0.000/9.053 rem=9.053 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] tick=4523 sim_t=452.30 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=6 slot=14->39 s=0.000/9.053 rem=9.053 speed=0.000 wait=4.30 dwell=0.00
[multi_patrol][state] tick=4544 sim_t=454.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=6 slot=14->39 s=0.000/9.053 rem=9.053 speed=0.000 wait=6.40 dwell=0.00
[multi_patrol][state] tick=4565 sim_t=456.50 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=6 slot=14->39 s=0.000/9.053 rem=9.053 speed=0.000 wait=8.50 dwell=0.00
[multi_patrol][state] tick=4566 sim_t=456.60 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=6 slot=14->39 s=0.002/9.053 rem=9.051 speed=0.020 wait=8.60 dwell=0.00
[multi_patrol][state] tick=4567 sim_t=456.70 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=6 slot=14->39 s=0.006/9.053 rem=9.047 speed=0.040 wait=8.70 dwell=0.00
[multi_patrol][state] tick=4571 sim_t=457.10 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=6 slot=14->39 s=0.028/9.053 rem=9.025 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=4715 sim_t=471.50 V1 mode=DWELL action=STOP reason=dwell blocker=-1 task=5 slot=26->26 s=7.129/7.129 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=4716 sim_t=471.60 V1 mode=DWELL action=STOP reason=not_active blocker=-1 task=5 slot=26->26 s=7.129/7.129 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=4765 sim_t=476.50 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=5 slot=26->0 s=0.002/6.249 rem=6.247 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=4800 sim_t=480.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=5 slot=26->0 s=0.498/6.249 rem=5.751 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] tick=4804 sim_t=480.40 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=5 slot=26->0 s=0.514/6.249 rem=5.736 speed=0.000 wait=0.50 dwell=0.00
[multi_patrol][state] tick=4805 sim_t=480.50 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=5 slot=26->0 s=0.516/6.249 rem=5.734 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] tick=4806 sim_t=480.60 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=5 slot=26->0 s=0.520/6.249 rem=5.730 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] tick=4807 sim_t=480.70 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=5 slot=26->0 s=0.521/6.249 rem=5.729 speed=0.010 wait=0.80 dwell=0.00
[multi_patrol][state] tick=4809 sim_t=480.90 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=5 slot=26->0 s=0.521/6.249 rem=5.729 speed=0.000 wait=1.00 dwell=0.00
[multi_patrol][state] tick=4812 sim_t=481.20 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=5 slot=26->0 s=0.523/6.249 rem=5.727 speed=0.020 wait=1.30 dwell=0.00
[multi_patrol][state] tick=4813 sim_t=481.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=5 slot=26->0 s=0.523/6.249 rem=5.727 speed=0.000 wait=1.40 dwell=0.00
[multi_patrol][state] tick=4834 sim_t=483.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=5 slot=26->0 s=0.523/6.249 rem=5.727 speed=0.000 wait=3.50 dwell=0.00
[multi_patrol][state] tick=4854 sim_t=485.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=5 slot=26->0 s=0.523/6.249 rem=5.727 speed=0.000 wait=5.50 dwell=0.00
[multi_patrol][state] tick=4875 sim_t=487.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=5 slot=26->0 s=0.523/6.249 rem=5.727 speed=0.000 wait=7.60 dwell=0.00
[multi_patrol][state] tick=4887 sim_t=488.70 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=5 slot=26->0 s=0.525/6.249 rem=5.725 speed=0.020 wait=8.80 dwell=0.00
[multi_patrol][state] tick=4888 sim_t=488.80 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=5 slot=26->0 s=0.529/6.249 rem=5.721 speed=0.040 wait=8.90 dwell=0.00
[multi_patrol][state] tick=4891 sim_t=489.10 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=5 slot=26->0 s=0.541/6.249 rem=5.709 speed=0.020 wait=9.20 dwell=0.00
[multi_patrol][state] tick=4892 sim_t=489.20 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=5 slot=26->0 s=0.541/6.249 rem=5.709 speed=0.000 wait=9.30 dwell=0.00
[multi_patrol][state] tick=4896 sim_t=489.60 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=5 slot=26->0 s=0.543/6.249 rem=5.707 speed=0.020 wait=9.70 dwell=0.00
[multi_patrol][state] tick=4897 sim_t=489.70 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=5 slot=26->0 s=0.547/6.249 rem=5.703 speed=0.040 wait=9.80 dwell=0.00
[multi_patrol][state] tick=4901 sim_t=490.10 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=5 slot=26->0 s=0.569/6.249 rem=5.681 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=4905 sim_t=490.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=5 slot=26->0 s=0.612/6.249 rem=5.638 speed=0.100 wait=0.10 dwell=0.00
[multi_patrol][state] tick=4911 sim_t=491.10 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=5 slot=26->0 s=0.626/6.249 rem=5.624 speed=0.020 wait=0.70 dwell=0.00
[multi_patrol][state] tick=4912 sim_t=491.20 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=5 slot=26->0 s=0.630/6.249 rem=5.620 speed=0.040 wait=0.80 dwell=0.00
[multi_patrol][state] tick=4916 sim_t=491.60 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=5 slot=26->0 s=0.652/6.249 rem=5.598 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=4919 sim_t=491.90 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=5 slot=26->0 s=0.680/6.249 rem=5.570 speed=0.080 wait=0.10 dwell=0.00
[multi_patrol][state] tick=4923 sim_t=492.30 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=5 slot=26->0 s=0.687/6.249 rem=5.563 speed=0.000 wait=0.50 dwell=0.00
[multi_patrol][state] tick=4924 sim_t=492.40 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=5 slot=26->0 s=0.689/6.249 rem=5.561 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] tick=4925 sim_t=492.50 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=5 slot=26->0 s=0.693/6.249 rem=5.557 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] tick=4929 sim_t=492.90 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=5 slot=26->0 s=0.714/6.249 rem=5.535 speed=0.065 wait=0.00 dwell=0.00
[multi_patrol][state] tick=5128 sim_t=512.80 V0 mode=DWELL action=STOP reason=dwell blocker=-1 task=7 slot=39->39 s=9.053/9.053 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=5129 sim_t=512.90 V0 mode=DWELL action=STOP reason=not_active blocker=-1 task=7 slot=39->39 s=9.053/9.053 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=5178 sim_t=517.80 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=7 slot=39->51 s=0.002/14.629 rem=14.627 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=5255 sim_t=525.50 V1 mode=DWELL action=STOP reason=dwell blocker=-1 task=6 slot=0->0 s=6.249/6.249 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=5256 sim_t=525.60 V1 mode=DWELL action=STOP reason=not_active blocker=-1 task=6 slot=0->0 s=6.249/6.249 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=5425 sim_t=542.50 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=6 slot=0->60 s=0.002/11.837 rem=11.835 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=5452 sim_t=545.20 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->60 s=0.390/11.837 rem=11.448 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] tick=5454 sim_t=545.40 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=6 slot=0->60 s=0.403/11.837 rem=11.434 speed=0.052 wait=0.30 dwell=0.00
[multi_patrol][state] tick=5457 sim_t=545.70 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=6 slot=0->60 s=0.407/11.837 rem=11.430 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] tick=5458 sim_t=545.80 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=6 slot=0->60 s=0.411/11.837 rem=11.426 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] tick=5460 sim_t=546.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->60 s=0.418/11.837 rem=11.419 speed=0.020 wait=0.90 dwell=0.00
[multi_patrol][state] tick=5461 sim_t=546.10 V1 mode=ACTIVE action=STOP reason=clear_block_V0 blocker=0 task=6 slot=0->60 s=0.418/11.837 rem=11.419 speed=0.000 wait=1.00 dwell=0.00
[multi_patrol][state] tick=5481 sim_t=548.10 V1 mode=ACTIVE action=STOP reason=clear_block_V0 blocker=0 task=6 slot=0->60 s=0.418/11.837 rem=11.419 speed=0.000 wait=3.00 dwell=0.00
[multi_patrol][state] tick=5486 sim_t=548.60 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=6 slot=0->60 s=0.420/11.837 rem=11.417 speed=0.020 wait=3.50 dwell=0.00
[multi_patrol][state] tick=5487 sim_t=548.70 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->60 s=0.420/11.837 rem=11.417 speed=0.000 wait=3.60 dwell=0.00
[multi_patrol][state] tick=5488 sim_t=548.80 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=6 slot=0->60 s=0.420/11.837 rem=11.417 speed=0.000 wait=3.70 dwell=0.00
[multi_patrol][state] tick=5492 sim_t=549.20 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=6 slot=0->60 s=0.422/11.837 rem=11.415 speed=0.020 wait=4.10 dwell=0.00
[multi_patrol][state] tick=5493 sim_t=549.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->60 s=0.422/11.837 rem=11.415 speed=0.000 wait=4.20 dwell=0.00
[multi_patrol][state] tick=5513 sim_t=551.30 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->60 s=0.422/11.837 rem=11.415 speed=0.000 wait=6.20 dwell=0.00
[multi_patrol][state] tick=5528 sim_t=552.80 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=6 slot=0->60 s=0.424/11.837 rem=11.413 speed=0.020 wait=7.70 dwell=0.00
[multi_patrol][state] tick=5529 sim_t=552.90 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=6 slot=0->60 s=0.428/11.837 rem=11.409 speed=0.040 wait=7.80 dwell=0.00
[multi_patrol][state] tick=5533 sim_t=553.30 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=6 slot=0->60 s=0.450/11.837 rem=11.387 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=5557 sim_t=555.70 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->60 s=0.813/11.837 rem=11.024 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] tick=5560 sim_t=556.00 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=6 slot=0->60 s=0.846/11.837 rem=10.991 speed=0.080 wait=0.40 dwell=0.00
[multi_patrol][state] tick=5562 sim_t=556.20 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=6 slot=0->60 s=0.856/11.837 rem=10.981 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] tick=5563 sim_t=556.30 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=6 slot=0->60 s=0.861/11.837 rem=10.976 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=5567 sim_t=556.70 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=6 slot=0->60 s=0.883/11.837 rem=10.954 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=5568 sim_t=556.80 V1 mode=ACTIVE action=YIELD reason=following_V0 blocker=0 task=6 slot=0->60 s=0.892/11.837 rem=10.945 speed=0.090 wait=0.10 dwell=0.00
[multi_patrol][state] tick=5572 sim_t=557.20 V1 mode=ACTIVE action=STOP reason=clear_block_V0 blocker=0 task=6 slot=0->60 s=0.929/11.837 rem=10.908 speed=0.070 wait=0.50 dwell=0.00
[multi_patrol][state] tick=5576 sim_t=557.60 V0 mode=ACTIVE action=CREEP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.621/14.629 rem=8.008 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] tick=5589 sim_t=558.90 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.020 wait=1.40 dwell=0.00
[multi_patrol][state] tick=5589 sim_t=558.90 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] tick=5609 sim_t=560.90 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=3.40 dwell=0.00
[multi_patrol][state] tick=5609 sim_t=560.90 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=4.20 dwell=0.00
[multi_patrol][state] tick=5630 sim_t=563.00 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=5.50 dwell=0.00
[multi_patrol][state] tick=5630 sim_t=563.00 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=6.30 dwell=0.00
[multi_patrol][state] tick=5651 sim_t=565.10 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=7.60 dwell=0.00
[multi_patrol][state] tick=5651 sim_t=565.10 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=8.40 dwell=0.00
[multi_patrol][state] tick=5672 sim_t=567.20 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=9.70 dwell=0.00
[multi_patrol][state] tick=5672 sim_t=567.20 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=10.50 dwell=0.00
[multi_patrol][state] tick=5692 sim_t=569.20 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=11.70 dwell=0.00
[multi_patrol][state] tick=5692 sim_t=569.20 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=12.50 dwell=0.00
[multi_patrol][state] tick=5713 sim_t=571.30 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=13.80 dwell=0.00
[multi_patrol][state] tick=5713 sim_t=571.30 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=14.60 dwell=0.00
[multi_patrol][state] tick=5734 sim_t=573.40 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=15.90 dwell=0.00
[multi_patrol][state] tick=5734 sim_t=573.40 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=16.70 dwell=0.00
[multi_patrol][state] tick=5755 sim_t=575.50 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=18.00 dwell=0.00
[multi_patrol][state] tick=5755 sim_t=575.50 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=18.80 dwell=0.00
[multi_patrol][state] tick=5776 sim_t=577.60 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=20.10 dwell=0.00
[multi_patrol][state] tick=5776 sim_t=577.60 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=20.90 dwell=0.00
[multi_patrol][state] tick=5796 sim_t=579.60 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=22.10 dwell=0.00
[multi_patrol][state] tick=5796 sim_t=579.60 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=22.90 dwell=0.00
[multi_patrol][state] tick=5816 sim_t=581.60 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=24.10 dwell=0.00
[multi_patrol][state] tick=5816 sim_t=581.60 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->60 s=0.934/11.837 rem=10.903 speed=0.000 wait=24.90 dwell=0.00
[multi_patrol][state] tick=5825 sim_t=582.50 V1 mode=ACTIVE action=NOMINAL reason=deadlock_replan blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=0.00 dwell=0.00
[multi_patrol][state] tick=5826 sim_t=582.60 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] tick=5837 sim_t=583.70 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=26.20 dwell=0.00
[multi_patrol][state] tick=5847 sim_t=584.70 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] tick=5858 sim_t=585.80 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=28.30 dwell=0.00
[multi_patrol][state] tick=5867 sim_t=586.70 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=4.20 dwell=0.00
[multi_patrol][state] tick=5879 sim_t=587.90 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=30.40 dwell=0.00
[multi_patrol][state] tick=5888 sim_t=588.80 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=6.30 dwell=0.00
[multi_patrol][state] tick=5899 sim_t=589.90 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=32.40 dwell=0.00
[multi_patrol][state] tick=5908 sim_t=590.80 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=8.30 dwell=0.00
[multi_patrol][state] tick=5920 sim_t=592.00 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=34.50 dwell=0.00
[multi_patrol][state] tick=5929 sim_t=592.90 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=10.40 dwell=0.00
[multi_patrol][state] tick=5941 sim_t=594.10 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=36.60 dwell=0.00
[multi_patrol][state] tick=5949 sim_t=594.90 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=12.40 dwell=0.00
[multi_patrol][state] tick=5961 sim_t=596.10 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=38.60 dwell=0.00
[multi_patrol][state] tick=5969 sim_t=596.90 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=14.40 dwell=0.00
[multi_patrol][state] tick=5982 sim_t=598.20 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=40.70 dwell=0.00
[multi_patrol][state] tick=5990 sim_t=599.00 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=16.50 dwell=0.00
[multi_patrol][state] tick=6002 sim_t=600.20 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=42.70 dwell=0.00
[multi_patrol][state] tick=6011 sim_t=601.10 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=18.60 dwell=0.00
[multi_patrol][state] tick=6023 sim_t=602.30 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=44.80 dwell=0.00
[multi_patrol][state] tick=6031 sim_t=603.10 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=20.60 dwell=0.00
[multi_patrol][state] tick=6044 sim_t=604.40 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=46.90 dwell=0.00
[multi_patrol][state] tick=6052 sim_t=605.20 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=22.70 dwell=0.00
[multi_patrol][state] tick=6064 sim_t=606.40 V0 mode=ACTIVE action=STOP reason=following_V1 blocker=1 task=7 slot=39->51 s=6.686/14.629 rem=7.943 speed=0.000 wait=48.90 dwell=0.00
[multi_patrol][state] tick=6072 sim_t=607.20 V1 mode=ACTIVE action=STOP reason=following_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=24.70 dwell=0.00
[multi_patrol][state] tick=6075 sim_t=607.50 V0 mode=ACTIVE action=NOMINAL reason=deadlock_replan blocker=1 task=7 slot=39->32 s=0.000/2.130 rem=2.130 speed=0.000 wait=0.00 dwell=0.00
[multi_patrol][state] tick=6076 sim_t=607.60 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=7 slot=39->32 s=0.002/2.130 rem=2.128 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=6082 sim_t=608.20 V1 mode=ACTIVE action=STOP reason=clear_block_V0 blocker=0 task=6 slot=0->28 s=0.000/2.768 rem=2.768 speed=0.000 wait=25.70 dwell=0.00
[multi_patrol][state] tick=6101 sim_t=610.10 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=6 slot=0->28 s=0.002/2.768 rem=2.766 speed=0.020 wait=27.60 dwell=0.00
[multi_patrol][state] tick=6102 sim_t=610.20 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=6 slot=0->28 s=0.006/2.768 rem=2.762 speed=0.040 wait=27.70 dwell=0.00
[multi_patrol][state] tick=6106 sim_t=610.60 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=6 slot=0->28 s=0.028/2.768 rem=2.740 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=6131 sim_t=613.10 V1 mode=ACTIVE action=STOP reason=clear_block_V0 blocker=0 task=6 slot=0->28 s=0.489/2.768 rem=2.279 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] tick=6135 sim_t=613.50 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=6 slot=0->28 s=0.527/2.768 rem=2.241 speed=0.050 wait=0.50 dwell=0.00
[multi_patrol][state] tick=6136 sim_t=613.60 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=6 slot=0->28 s=0.532/2.768 rem=2.236 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] tick=6137 sim_t=613.70 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=6 slot=0->28 s=0.537/2.768 rem=2.231 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=6139 sim_t=613.90 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->28 s=0.544/2.768 rem=2.224 speed=0.020 wait=0.90 dwell=0.00
[multi_patrol][state] tick=6160 sim_t=616.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->28 s=0.544/2.768 rem=2.224 speed=0.000 wait=3.00 dwell=0.00
[multi_patrol][state] tick=6180 sim_t=618.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->28 s=0.544/2.768 rem=2.224 speed=0.000 wait=5.00 dwell=0.00
[multi_patrol][state] tick=6200 sim_t=620.00 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=6 slot=0->28 s=0.544/2.768 rem=2.224 speed=0.000 wait=7.00 dwell=0.00
[multi_patrol][state] tick=6204 sim_t=620.40 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=6 slot=0->28 s=0.546/2.768 rem=2.222 speed=0.020 wait=7.40 dwell=0.00
[multi_patrol][state] tick=6205 sim_t=620.50 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=6 slot=0->28 s=0.550/2.768 rem=2.218 speed=0.040 wait=7.50 dwell=0.00
[multi_patrol][state] tick=6209 sim_t=620.90 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=6 slot=0->28 s=0.572/2.768 rem=2.196 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=6222 sim_t=622.20 V0 mode=DWELL action=STOP reason=dwell blocker=-1 task=8 slot=32->32 s=2.130/2.130 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=6223 sim_t=622.30 V0 mode=DWELL action=STOP reason=not_active blocker=-1 task=8 slot=32->32 s=2.130/2.130 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=6272 sim_t=627.20 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=8 slot=32->40 s=0.002/9.361 rem=9.359 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=6284 sim_t=628.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=0.167/9.361 rem=9.194 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] tick=6288 sim_t=628.80 V0 mode=ACTIVE action=STOP reason=clear_block_V1 blocker=1 task=8 slot=32->40 s=0.205/9.361 rem=9.156 speed=0.050 wait=0.50 dwell=0.00
[multi_patrol][state] tick=6308 sim_t=630.80 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=32->40 s=0.209/9.361 rem=9.152 speed=0.020 wait=2.50 dwell=0.00
[multi_patrol][state] tick=6309 sim_t=630.90 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=8 slot=32->40 s=0.213/9.361 rem=9.148 speed=0.040 wait=2.60 dwell=0.00
[multi_patrol][state] tick=6311 sim_t=631.10 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=0.220/9.361 rem=9.141 speed=0.020 wait=2.80 dwell=0.00
[multi_patrol][state] tick=6313 sim_t=631.30 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=8 slot=32->40 s=0.220/9.361 rem=9.141 speed=0.000 wait=3.00 dwell=0.00
[multi_patrol][state] tick=6316 sim_t=631.60 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=32->40 s=0.222/9.361 rem=9.139 speed=0.020 wait=3.30 dwell=0.00
[multi_patrol][state] tick=6317 sim_t=631.70 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=8 slot=32->40 s=0.226/9.361 rem=9.135 speed=0.040 wait=3.40 dwell=0.00
[multi_patrol][state] tick=6321 sim_t=632.10 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=8 slot=32->40 s=0.248/9.361 rem=9.113 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=6342 sim_t=634.20 V1 mode=DWELL action=STOP reason=dwell blocker=-1 task=7 slot=28->28 s=2.768/2.768 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=6343 sim_t=634.30 V1 mode=DWELL action=STOP reason=not_active blocker=-1 task=7 slot=28->28 s=2.768/2.768 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=6392 sim_t=639.20 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=7 slot=28->7 s=0.002/6.030 rem=6.028 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=6418 sim_t=641.80 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.813/9.361 rem=7.548 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] tick=6421 sim_t=642.10 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=8 slot=32->40 s=1.846/9.361 rem=7.515 speed=0.080 wait=0.40 dwell=0.00
[multi_patrol][state] tick=6423 sim_t=642.30 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=32->40 s=1.856/9.361 rem=7.505 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] tick=6424 sim_t=642.40 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=8 slot=32->40 s=1.861/9.361 rem=7.500 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=6426 sim_t=642.60 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.868/9.361 rem=7.493 speed=0.020 wait=0.90 dwell=0.00
[multi_patrol][state] tick=6427 sim_t=642.70 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=8 slot=32->40 s=1.868/9.361 rem=7.493 speed=0.000 wait=1.00 dwell=0.00
[multi_patrol][state] tick=6431 sim_t=643.10 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=32->40 s=1.870/9.361 rem=7.491 speed=0.020 wait=1.40 dwell=0.00
[multi_patrol][state] tick=6432 sim_t=643.20 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.870/9.361 rem=7.491 speed=0.000 wait=1.50 dwell=0.00
[multi_patrol][state] tick=6433 sim_t=643.30 V0 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=8 slot=32->40 s=1.870/9.361 rem=7.491 speed=0.000 wait=1.60 dwell=0.00
[multi_patrol][state] tick=6437 sim_t=643.70 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.020 wait=2.00 dwell=0.00
[multi_patrol][state] tick=6438 sim_t=643.80 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=2.10 dwell=0.00
[multi_patrol][state] tick=6459 sim_t=645.90 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=4.20 dwell=0.00
[multi_patrol][state] tick=6480 sim_t=648.00 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=6.30 dwell=0.00
[multi_patrol][state] tick=6501 sim_t=650.10 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=8.40 dwell=0.00
[multi_patrol][state] tick=6522 sim_t=652.20 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=10.50 dwell=0.00
[multi_patrol][state] tick=6542 sim_t=654.20 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=12.50 dwell=0.00
[multi_patrol][state] tick=6562 sim_t=656.20 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=14.50 dwell=0.00
[multi_patrol][state] tick=6583 sim_t=658.30 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=16.60 dwell=0.00
[multi_patrol][state] tick=6603 sim_t=660.30 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=18.60 dwell=0.00
[multi_patrol][state] tick=6624 sim_t=662.40 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=20.70 dwell=0.00
[multi_patrol][state] tick=6645 sim_t=664.50 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=22.80 dwell=0.00
[multi_patrol][state] tick=6666 sim_t=666.60 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=24.90 dwell=0.00
[multi_patrol][state] tick=6687 sim_t=668.70 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=27.00 dwell=0.00
[multi_patrol][state] tick=6708 sim_t=670.80 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=29.10 dwell=0.00
[multi_patrol][state] tick=6729 sim_t=672.90 V0 mode=ACTIVE action=STOP reason=brake_V1 blocker=1 task=8 slot=32->40 s=1.872/9.361 rem=7.489 speed=0.000 wait=31.20 dwell=0.00
[multi_patrol][state] tick=6743 sim_t=674.30 V0 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=32->40 s=1.874/9.361 rem=7.487 speed=0.020 wait=32.60 dwell=0.00
[multi_patrol][state] tick=6744 sim_t=674.40 V0 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=8 slot=32->40 s=1.878/9.361 rem=7.483 speed=0.040 wait=32.70 dwell=0.00
[multi_patrol][state] tick=6748 sim_t=674.80 V0 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=8 slot=32->40 s=1.900/9.361 rem=7.461 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] tick=6757 sim_t=675.70 V1 mode=DWELL action=STOP reason=dwell blocker=-1 task=8 slot=7->7 s=6.030/6.030 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[multi_patrol][state] tick=6758 sim_t=675.80 V1 mode=DWELL action=STOP reason=not_active blocker=-1 task=8 slot=7->7 s=6.030/6.030 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] tick=6807 sim_t=680.70 V1 mode=ACTIVE action=NOMINAL reason=clear blocker=-1 task=8 slot=7->65 s=0.002/9.072 rem=9.070 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] tick=6815 sim_t=681.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=8 slot=7->65 s=0.085/9.072 rem=8.987 speed=0.130 wait=0.10 dwell=0.00
[multi_patrol][state] tick=6818 sim_t=681.80 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=8 slot=7->65 s=0.106/9.072 rem=8.966 speed=0.040 wait=0.40 dwell=0.00
[multi_patrol][state] tick=6820 sim_t=682.00 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=7->65 s=0.110/9.072 rem=8.962 speed=0.030 wait=0.60 dwell=0.00
[multi_patrol][state] tick=6821 sim_t=682.10 V1 mode=ACTIVE action=CREEP reason=action_hold blocker=-1 task=8 slot=7->65 s=0.115/9.072 rem=8.957 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] tick=6822 sim_t=682.20 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=8 slot=7->65 s=0.117/9.072 rem=8.955 speed=0.020 wait=0.80 dwell=0.00
[multi_patrol][state] tick=6823 sim_t=682.30 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=8 slot=7->65 s=0.117/9.072 rem=8.955 speed=0.000 wait=0.90 dwell=0.00
[multi_patrol][state] tick=6827 sim_t=682.70 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=7->65 s=0.119/9.072 rem=8.953 speed=0.020 wait=1.30 dwell=0.00
[multi_patrol][state] tick=6828 sim_t=682.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=8 slot=7->65 s=0.119/9.072 rem=8.953 speed=0.000 wait=1.40 dwell=0.00
[multi_patrol][state] tick=6829 sim_t=682.90 V1 mode=ACTIVE action=STOP reason=action_hold blocker=-1 task=8 slot=7->65 s=0.119/9.072 rem=8.953 speed=0.000 wait=1.50 dwell=0.00
[multi_patrol][state] tick=6833 sim_t=683.30 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=7->65 s=0.121/9.072 rem=8.951 speed=0.020 wait=1.90 dwell=0.00
[multi_patrol][state] tick=6834 sim_t=683.40 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=8 slot=7->65 s=0.121/9.072 rem=8.951 speed=0.000 wait=2.00 dwell=0.00
[multi_patrol][state] tick=6855 sim_t=685.50 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=8 slot=7->65 s=0.121/9.072 rem=8.951 speed=0.000 wait=4.10 dwell=0.00
[multi_patrol][state] tick=6876 sim_t=687.60 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=8 slot=7->65 s=0.121/9.072 rem=8.951 speed=0.000 wait=6.20 dwell=0.00
[multi_patrol][state] tick=6897 sim_t=689.70 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=8 slot=7->65 s=0.121/9.072 rem=8.951 speed=0.000 wait=8.30 dwell=0.00
[multi_patrol][state] tick=6918 sim_t=691.80 V1 mode=ACTIVE action=STOP reason=brake_V0 blocker=0 task=8 slot=7->65 s=0.121/9.072 rem=8.951 speed=0.000 wait=10.40 dwell=0.00
[multi_patrol][state] tick=6929 sim_t=692.90 V1 mode=ACTIVE action=CREEP reason=clear blocker=-1 task=8 slot=7->65 s=0.123/9.072 rem=8.949 speed=0.020 wait=11.50 dwell=0.00