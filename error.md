etting /run_id to 61f9e12c-783f-11f1-92dd-574a8d0867f8
process[rosout-1]: started with pid [177136]
started core service [/rosout]
process[forklift_map-2]: started with pid [177143]
process[path_catalog_debug_node-3]: started with pid [177144]
process[rviz-4]: started with pid [177150]
[INFO] [1783234905.921386310]: [path_catalog] midpoint check: B4(1.0012,4.3750), B5(1.4988,4.3750) -> A1(1.2500,4.3750)
[INFO] [1783234905.922780334]: [path_catalog] midpoint check: B60(1.0012,0.1250), B61(1.4988,0.1250) -> A2(1.2500,0.1250)
[INFO] [1783234905.923477396]: [path_catalog] A1 virtual slot: id=101 row=0 dock=(1.2500,4.3750) pre=(1.2500,4.1000) yaw=90.0deg
[INFO] [1783234905.923578005]: [path_catalog] A2 virtual slot: id=102 row=7 dock=(1.2500,0.1250) pre=(1.2500,0.4000) yaw=-90.0deg
[WARN] [1783234905.923691415]: [path_catalog] selected depot=A1 targets B0..B65 (66): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65]
[WARN] [1783234905.923933336]: [planner][row1-debug] src=101 tgt=43 src_corr=1 tgt_corr=4 target=(0.381,0.738 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.211 far_gap=0.491 horiz=0.744 min_x=0.676
[WARN] [1783234905.924050647]: [planner][row1-debug] initial_reverse tgt=43 same_corr=0 start_to_right=0 start_lane_y=3.791 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.610 actual_end=(1.610,3.791) curve_start=(1.250,4.129) curve_pts=56 direct_ok=1
[WARN] [1783234905.924117853]: [planner][row1-debug] skeleton tgt=43 points=9 terminal_reverse=0 goal_lane_y=0.429 terminal_stop_y=0.738 final_ref_x=1.125
[WARN] [1783234905.924201660]: [planner][row1-debug] skeleton[0]=(1.610,3.791)
[WARN] [1783234905.924269966]: [planner][row1-debug] skeleton[1]=(0.145,3.791)
[WARN] [1783234905.924344573]: [planner][row1-debug] skeleton[2]=(0.145,2.950)
[WARN] [1783234905.924411179]: [planner][row1-debug] skeleton[3]=(1.250,2.950)
[WARN] [1783234905.924485385]: [planner][row1-debug] skeleton[4]=(1.250,1.550)
[WARN] [1783234905.924552091]: [planner][row1-debug] skeleton[5]=(1.125,1.550)
[WARN] [1783234905.924623998]: [planner][row1-debug] skeleton[6]=(1.125,0.429)
[WARN] [1783234905.924687503]: [planner][row1-debug] skeleton[7]=(0.381,0.429)
[WARN] [1783234905.924781612]: [planner][row1-debug] skeleton[8]=(0.381,0.920)
[WARN] [1783234905.924829116]: [planner][row1-debug] lane_shift rejected tgt=43 j=2 terminal=0 lateral=1.105 lead_in=0.472 lead_out=1.025
[WARN] [1783234905.924916824]: [planner][row1-debug] lane_shift accepted tgt=43 j=4 terminal=0 lateral=0.125 lead_in=0.252 lead_out=0.252
[WARN] [1783234905.926203138]: [planner][row1-debug] clothoid infeasible tgt=43 j=6 p=(1.125, 0.429) prev=(1.125, 1.550) next=(0.381, 0.429) prev_len=1.120 next_len=0.744 limit=0.345
[WARN] [1783234905.926313048]: [planner] slot 43: using arc fallback; curvature continuity is not satisfied
[ERROR] [1783234905.926533768]: [path_catalog] A1_to_B43 rejected by task-style validation: curvature_discontinuity
[INFO] [1783234906.327278518]: [forklift_map] Published 387 markers on /forklift_map/markers
