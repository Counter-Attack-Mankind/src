[INFO] [1783309668.826146802]: [path_catalog] midpoint check: B4(1.0012,4.3750), B5(1.4988,4.3750) -> A1(1.2500,4.3750)
[INFO] [1783309668.827654000]: [path_catalog] midpoint check: B60(1.0012,0.1250), B61(1.4988,0.1250) -> A2(1.2500,0.1250)
[INFO] [1783309668.828432600]: [path_catalog] A1 virtual slot: id=101 row=0 dock=(1.2500,4.3750) pre=(1.2500,4.1000) yaw=90.0deg
[INFO] [1783309668.828535300]: [path_catalog] A2 virtual slot: id=102 row=7 dock=(1.2500,0.1250) pre=(1.2500,0.4000) yaw=-90.0deg
[WARN] [1783309668.828613000]: [path_catalog] selected depot=A1 direction=from_depot targets B0..B65 (66): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65]
[INFO] [1783309668.829112199]: [path_catalog] A1_to_B0 row=0 col=0 wpts=162 len=2.555 arc=0
[INFO] [1783309668.829343799]: [path_catalog] A1_to_B1 row=0 col=1 wpts=162 len=2.336 arc=0
[INFO] [1783309668.829566199]: [path_catalog] A1_to_B2 row=0 col=2 wpts=162 len=2.116 arc=0
[INFO] [1783309668.829795299]: [path_catalog] A1_to_B3 row=0 col=3 wpts=162 len=1.901 arc=0
[INFO] [1783309668.830041798]: [path_catalog] A1_to_B4 row=0 col=4 wpts=162 len=1.681 arc=0
[INFO] [1783309668.830280698]: [path_catalog] A1_to_B5 row=0 col=5 wpts=162 len=1.681 arc=0
[INFO] [1783309668.830517298]: [path_catalog] A1_to_B6 row=0 col=6 wpts=162 len=1.901 arc=0
[INFO] [1783309668.830738798]: [path_catalog] A1_to_B7 row=0 col=7 wpts=162 len=2.116 arc=0
[INFO] [1783309668.830951297]: [path_catalog] A1_to_B8 row=0 col=8 wpts=162 len=2.336 arc=0
[INFO] [1783309668.831174197]: [path_catalog] A1_to_B9 row=0 col=9 wpts=162 len=2.555 arc=0
[WARN] [1783309668.831307597]: [planner][row1-debug] route=A1_TO_B src=101 tgt=10 src_corr=1 tgt_corr=1 target=(0.706,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.491 far_gap=0.211 horiz=0.544 min_x=0.676
[WARN] [1783309668.831423197]: [planner][row1-debug] row1-upper aux lane tgt=10 lower=3.791 upper=4.071 aux_y=3.968 terminal_rear_y=3.579
[WARN] [1783309668.831496997]: [planner][row1-debug] initial_reverse tgt=10 same_corr=1 start_to_right=0 start_lane_y=3.968 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.610 actual_end=(1.610,3.968) curve_start=(1.250,4.306) curve_pts=56 direct_ok=1
[WARN] [1783309668.831597397]: [planner][row1-debug] skeleton tgt=10 points=3 terminal_reverse=0 goal_lane_y=3.968 terminal_stop_y=3.761 final_ref_x=1.250
[WARN] [1783309668.831650397]: [planner][row1-debug] skeleton[0]=(1.610,3.968)
[WARN] [1783309668.831749997]: [planner][row1-debug] skeleton[1]=(0.706,3.968)
[WARN] [1783309668.831825297]: [planner][row1-debug] skeleton[2]=(0.706,3.579)
[WARN] [1783309668.831998097]: [planner] reverse segment heading mismatch at i=0 pose=(1.250, 4.301) theta=1.571 motion=1.571 err=0.000
[WARN] [1783309668.832075797]: [path_catalog] kink geometry at segment 1 turn=179.2deg prev_type=1 type=1 prev_vec=(0.000, 0.004) vec=(0.000, -0.010)
[ERROR] [1783309668.832242396]: [path_catalog] A1_to_B10 rejected: kink
[WARN] [1783309668.832327096]: [path_catalog][reject-detail] A1_to_B10 B10 row=1 col=0 kink sharp_turn=179.2deg prev=(1.250,4.301,90.0deg,REVERSE) mid=(1.250,4.306,90.0deg,REVERSE) next=(1.250,4.296,90.8deg,REVERSE)
[INFO] [1783309668.832735696]: [path_catalog] A1_to_B11 row=2 col=0 wpts=333 len=4.555 arc=0
[WARN] [1783309668.832867896]: [planner][row1-debug] route=A1_TO_B src=101 tgt=12 src_corr=1 tgt_corr=1 target=(0.928,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.491 far_gap=0.211 horiz=0.358 min_x=0.676
[WARN] [1783309668.832929696]: [planner][row1-debug] row1-upper aux lane tgt=12 lower=3.791 upper=4.071 aux_y=3.968 terminal_rear_y=3.579
[WARN] [1783309668.833017896]: [planner][row1-debug] initial_reverse tgt=12 same_corr=1 start_to_right=0 start_lane_y=3.968 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.610 actual_end=(1.610,3.968) curve_start=(1.250,4.306) curve_pts=56 direct_ok=1
[WARN] [1783309668.833089095]: [planner][row1-debug] skeleton tgt=12 points=3 terminal_reverse=0 goal_lane_y=3.968 terminal_stop_y=3.761 final_ref_x=1.286
[WARN] [1783309668.833174296]: [planner][row1-debug] skeleton[0]=(1.610,3.968)
[WARN] [1783309668.833256395]: [planner][row1-debug] skeleton[1]=(0.928,3.968)
[WARN] [1783309668.833333695]: [planner][row1-debug] skeleton[2]=(0.928,3.579)
[WARN] [1783309668.833491295]: [planner] reverse segment heading mismatch at i=0 pose=(1.250, 4.301) theta=1.571 motion=1.571 err=0.000
[WARN] [1783309668.833631595]: [path_catalog] kink geometry at segment 1 turn=179.2deg prev_type=1 type=1 prev_vec=(0.000, 0.004) vec=(0.000, -0.010)
[ERROR] [1783309668.833678095]: [path_catalog] A1_to_B12 rejected: kink
[WARN] [1783309668.833817195]: [path_catalog][reject-detail] A1_to_B12 B12 row=1 col=1 kink sharp_turn=179.2deg prev=(1.250,4.301,90.0deg,REVERSE) mid=(1.250,4.306,90.0deg,REVERSE) next=(1.250,4.296,90.8deg,REVERSE)
[INFO] [1783309668.834148095]: [path_catalog] A1_to_B13 row=2 col=1 wpts=333 len=4.333 arc=0
[WARN] [1783309668.834283494]: [planner][row1-debug] route=A1_TO_B src=101 tgt=14 src_corr=1 tgt_corr=1 target=(1.150,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.491 far_gap=0.211 horiz=0.358 min_x=0.676
[WARN] [1783309668.834328394]: [planner][row1-debug] row1-upper aux lane tgt=14 lower=3.791 upper=4.071 aux_y=3.968 terminal_rear_y=3.579
[WARN] [1783309668.834425894]: [planner][row1-debug] initial_reverse tgt=14 same_corr=1 start_to_right=0 start_lane_y=3.968 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.687 actual_end=(1.687,3.968) curve_start=(1.250,4.306) curve_pts=56 direct_ok=1
[WARN] [1783309668.834481094]: [planner][row1-debug] skeleton tgt=14 points=3 terminal_reverse=0 goal_lane_y=3.968 terminal_stop_y=3.761 final_ref_x=1.508
[WARN] [1783309668.834557994]: [planner][row1-debug] skeleton[0]=(1.687,3.968)
[WARN] [1783309668.834627394]: [planner][row1-debug] skeleton[1]=(1.150,3.968)
[WARN] [1783309668.834697193]: [planner][row1-debug] skeleton[2]=(1.150,3.579)
[WARN] [1783309668.834814093]: [planner] reverse segment heading mismatch at i=0 pose=(1.250, 4.301) theta=1.571 motion=1.571 err=0.000
[WARN] [1783309668.834917893]: [path_catalog] kink geometry at segment 1 turn=179.2deg prev_type=1 type=1 prev_vec=(0.000, 0.004) vec=(0.000, -0.010)
[ERROR] [1783309668.834985993]: [path_catalog] A1_to_B14 rejected: kink
[WARN] [1783309668.835074393]: [path_catalog][reject-detail] A1_to_B14 B14 row=1 col=2 kink sharp_turn=179.2deg prev=(1.250,4.301,90.0deg,REVERSE) mid=(1.250,4.306,90.0deg,REVERSE) next=(1.250,4.296,90.8deg,REVERSE)
[INFO] [1783309668.835386193]: [path_catalog] A1_to_B15 row=2 col=2 wpts=333 len=4.311 arc=0
[WARN] [1783309668.835516393]: [planner][row1-debug] route=A1_TO_B src=101 tgt=16 src_corr=1 tgt_corr=1 target=(1.372,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.211 far_gap=0.491 horiz=0.358 min_x=0.676
[WARN] [1783309668.835622493]: [planner][row1-debug] row1-upper aux lane tgt=16 lower=3.791 upper=4.071 aux_y=3.968 terminal_rear_y=3.579
[WARN] [1783309668.835670992]: [planner][row1-debug] initial_reverse tgt=16 same_corr=1 start_to_right=1 start_lane_y=3.968 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.834 actual_end=(0.834,3.968) curve_start=(1.250,4.306) curve_pts=56 direct_ok=1
[WARN] [1783309668.835810192]: [planner][row1-debug] skeleton tgt=16 points=3 terminal_reverse=0 goal_lane_y=3.968 terminal_stop_y=3.761 final_ref_x=1.014
[WARN] [1783309668.835885992]: [planner][row1-debug] skeleton[0]=(0.834,3.968)
[WARN] [1783309668.835954693]: [planner][row1-debug] skeleton[1]=(1.372,3.968)
[WARN] [1783309668.836036992]: [planner][row1-debug] skeleton[2]=(1.372,3.579)
[WARN] [1783309668.836350592]: [planner] reverse segment heading mismatch at i=0 pose=(1.250, 4.301) theta=1.571 motion=1.571 err=0.000
[WARN] [1783309668.836455592]: [path_catalog] kink geometry at segment 1 turn=179.2deg prev_type=1 type=1 prev_vec=(0.000, 0.004) vec=(-0.000, -0.010)
[ERROR] [1783309668.836499492]: [path_catalog] A1_to_B16 rejected: kink
[WARN] [1783309668.836564092]: [path_catalog][reject-detail] A1_to_B16 B16 row=1 col=3 kink sharp_turn=179.2deg prev=(1.250,4.301,90.0deg,REVERSE) mid=(1.250,4.306,90.0deg,REVERSE) next=(1.250,4.296,89.2deg,REVERSE)
[INFO] [1783309668.836990491]: [path_catalog] A1_to_B17 row=2 col=3 wpts=333 len=4.533 arc=0
[WARN] [1783309668.837094891]: [planner][row1-debug] route=A1_TO_B src=101 tgt=18 src_corr=1 tgt_corr=1 target=(1.594,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.211 far_gap=0.491 horiz=0.358 min_x=0.676
[WARN] [1783309668.837139691]: [planner][row1-debug] row1-upper aux lane tgt=18 lower=3.791 upper=4.071 aux_y=3.968 terminal_rear_y=3.579
[WARN] [1783309668.837187091]: [planner][row1-debug] initial_reverse tgt=18 same_corr=1 start_to_right=1 start_lane_y=3.968 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.890 actual_end=(0.890,3.968) curve_start=(1.250,4.306) curve_pts=56 direct_ok=1
[WARN] [1783309668.837249191]: [planner][row1-debug] skeleton tgt=18 points=3 terminal_reverse=0 goal_lane_y=3.968 terminal_stop_y=3.761 final_ref_x=1.235
[WARN] [1783309668.837323791]: [planner][row1-debug] skeleton[0]=(0.890,3.968)
[WARN] [1783309668.837407991]: [planner][row1-debug] skeleton[1]=(1.594,3.968)
[WARN] [1783309668.837451391]: [planner][row1-debug] skeleton[2]=(1.594,3.579)
[WARN] [1783309668.837610491]: [planner] reverse segment heading mismatch at i=0 pose=(1.250, 4.301) theta=1.571 motion=1.571 err=0.000
[WARN] [1783309668.837714791]: [path_catalog] kink geometry at segment 1 turn=179.2deg prev_type=1 type=1 prev_vec=(0.000, 0.004) vec=(-0.000, -0.010)
[ERROR] [1783309668.837758590]: [path_catalog] A1_to_B18 rejected: kink
[WARN] [1783309668.837847391]: [path_catalog][reject-detail] A1_to_B18 B18 row=1 col=4 kink sharp_turn=179.2deg prev=(1.250,4.301,90.0deg,REVERSE) mid=(1.250,4.306,90.0deg,REVERSE) next=(1.250,4.296,89.2deg,REVERSE)
