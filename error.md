process[path_catalog_debug_node-3]: started with pid [49732]
process[rviz-4]: started with pid [49738]
[INFO] [1783863878.692154887]: [path_catalog] midpoint check: B4(1.0012,4.3750), B5(1.4988,4.3750) -> A1(1.2500,4.3750)
[INFO] [1783863878.693416470]: [path_catalog] midpoint check: B60(1.0012,0.1250), B61(1.4988,0.1250) -> A2(1.2500,0.1250)
[INFO] [1783863878.694192060]: [path_catalog] A1 virtual slot: id=101 row=0 dock=(1.2500,4.3750) pre=(1.2500,4.1000) yaw=90.0deg
[INFO] [1783863878.694295358]: [path_catalog] A2 virtual slot: id=102 row=7 dock=(1.2500,0.1250) pre=(1.2500,0.4000) yaw=-90.0deg
[WARN] [1783863878.694413757]: [path_catalog] selected depot=A1 direction=to_depot target_row=1 targets (5): [10, 12, 14, 16, 18]
[WARN] [1783863878.694677453]: [planner][row1-debug] route=B_TO_A1 debug_rev=row1_exact_a1_lane_v1 src=10 tgt=101 src_corr=1 tgt_corr=1 target=(1.250,4.100 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.584 far_gap=0.304 horiz=0.000 min_x=0.676
[WARN] [1783863878.694827151]: [planner][row1-debug] initial_reverse tgt=101 same_corr=1 start_to_right=1 start_lane_y=4.071 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.346 actual_end=(0.346,4.071) reverse_origin=(0.706,3.579) curve_start=(0.706,3.717) curve_pts=52 direct_ok=1
[WARN] [1783863878.694920350]: [planner][row1-debug] skeleton tgt=101 points=3 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=4.100 final_ref_x=1.250
[WARN] [1783863878.695021049]: [planner][row1-debug] skeleton[0]=(0.346,4.071)
[WARN] [1783863878.695075448]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783863878.695174547]: [planner][row1-debug] skeleton[2]=(1.250,4.301)
[WARN] [1783863878.695284345]: [planner][row1-debug] exact A1 lane using terminal-constrained G2 fallback src=10 din=1 t_in=0.231 t_out=0.231 mt_port=0.231 available_out=0.231
[WARN] [1783863878.695387644]: [planner][row1-debug] exact A1 lane src=10 din=1 lane_start=(0.346,4.071) curve_start=(1.019,4.071) curve_end=(1.250,4.301) stop_y=4.301 rear_stop_y=4.301 mt_port=0.231 available_out=0.231 pts=102
[WARN] [1783863878.695480942]: [planner][row1-debug] prepend enter src=10 row1_upper=1 raw_pts=67 a_rev=(0.706,3.579) b=(0.346,4.071) path_front=(0.346,4.071 0.0deg type=0)
[WARN] [1783863878.695569541]: [planner][row1-debug] chord reverse prefix src=10 pts=67 first=(0.706,3.579 -90.0deg) last=(0.346,4.071 0.0deg)
[WARN] [1783863878.695738939]: [planner][row1-debug] prefix[0]=(0.706,3.579 -90.0deg type=1)
[WARN] [1783863878.695829938]: [planner][row1-debug] prefix[1]=(0.706,3.589 -90.0deg type=1)
[WARN] [1783863878.695947536]: [planner][row1-debug] prefix[2]=(0.706,3.599 -90.0deg type=1)
[WARN] [1783863878.696036735]: [planner][row1-debug] prefix[3]=(0.706,3.609 -90.0deg type=1)
[WARN] [1783863878.696125734]: [planner][row1-debug] prefix[18]=(0.706,3.782 -89.6deg type=1)
[WARN] [1783863878.696230732]: [planner][row1-debug] prefix[19]=(0.706,3.798 -89.3deg type=1)
[WARN] [1783863878.696286432]: [planner][row1-debug] prefix[20]=(0.706,3.813 -89.0deg type=1)
[WARN] [1783863878.696480829]: [planner][row1-debug] route=B_TO_A1 debug_rev=row1_exact_a1_lane_v1 src=12 tgt=101 src_corr=1 tgt_corr=1 target=(1.250,4.100 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.584 far_gap=0.304 horiz=0.000 min_x=0.676
[WARN] [1783863878.696590027]: [planner][row1-debug] initial_reverse tgt=101 same_corr=1 start_to_right=1 start_lane_y=4.071 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.568 actual_end=(0.568,4.071) reverse_origin=(0.928,3.579) curve_start=(0.928,3.717) curve_pts=52 direct_ok=1
[WARN] [1783863878.696638727]: [planner][row1-debug] skeleton tgt=101 points=3 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=4.100 final_ref_x=1.250
[WARN] [1783863878.696740425]: [planner][row1-debug] skeleton[0]=(0.568,4.071)
[WARN] [1783863878.696798025]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783863878.696844024]: [planner][row1-debug] skeleton[2]=(1.250,4.301)
[WARN] [1783863878.696974122]: [planner][row1-debug] exact A1 lane using terminal-constrained G2 fallback src=12 din=1 t_in=0.231 t_out=0.231 mt_port=0.231 available_out=0.231
[WARN] [1783863878.697068821]: [planner][row1-debug] exact A1 lane src=12 din=1 lane_start=(0.568,4.071) curve_start=(1.019,4.071) curve_end=(1.250,4.301) stop_y=4.301 rear_stop_y=4.301 mt_port=0.231 available_out=0.231 pts=80
[WARN] [1783863878.697158820]: [planner][row1-debug] prepend enter src=12 row1_upper=1 raw_pts=67 a_rev=(0.928,3.579) b=(0.568,4.071) path_front=(0.568,4.071 0.0deg type=0)
[WARN] [1783863878.697267318]: [planner][row1-debug] chord reverse prefix src=12 pts=67 first=(0.928,3.579 -90.0deg) last=(0.568,4.071 0.0deg)
[WARN] [1783863878.697342118]: [planner][row1-debug] prefix[0]=(0.928,3.579 -90.0deg type=1)
[WARN] [1783863878.697390017]: [planner][row1-debug] prefix[1]=(0.928,3.589 -90.0deg type=1)
[WARN] [1783863878.697482915]: [planner][row1-debug] prefix[2]=(0.928,3.599 -90.0deg type=1)
[WARN] [1783863878.697539815]: [planner][row1-debug] prefix[3]=(0.928,3.609 -90.0deg type=1)
[WARN] [1783863878.697587414]: [planner][row1-debug] prefix[18]=(0.928,3.782 -89.6deg type=1)
[WARN] [1783863878.697691112]: [planner][row1-debug] prefix[19]=(0.928,3.798 -89.3deg type=1)
[WARN] [1783863878.697750612]: [planner][row1-debug] prefix[20]=(0.928,3.813 -89.0deg type=1)
[WARN] [1783863878.697896110]: [planner][row1-debug] route=B_TO_A1 debug_rev=row1_exact_a1_lane_v1 src=14 tgt=101 src_corr=1 tgt_corr=1 target=(1.250,4.100 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.584 far_gap=0.304 horiz=0.000 min_x=0.676
[WARN] [1783863878.698005408]: [planner][row1-debug] initial_reverse tgt=101 same_corr=1 start_to_right=1 start_lane_y=4.071 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.790 actual_end=(0.790,4.071) reverse_origin=(1.150,3.579) curve_start=(1.150,3.717) curve_pts=52 direct_ok=1
[WARN] [1783863878.698053607]: [planner][row1-debug] skeleton tgt=101 points=3 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=4.100 final_ref_x=1.250
[WARN] [1783863878.698138807]: [planner][row1-debug] skeleton[0]=(0.790,4.071)
[WARN] [1783863878.698193506]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783863878.698256705]: [planner][row1-debug] skeleton[2]=(1.250,4.301)
[WARN] [1783863878.698370903]: [planner][row1-debug] exact A1 lane using terminal-constrained G2 fallback src=14 din=1 t_in=0.231 t_out=0.231 mt_port=0.231 available_out=0.231
[WARN] [1783863878.698458902]: [planner][row1-debug] exact A1 lane src=14 din=1 lane_start=(0.790,4.071) curve_start=(1.019,4.071) curve_end=(1.250,4.301) stop_y=4.301 rear_stop_y=4.301 mt_port=0.231 available_out=0.231 pts=57
[WARN] [1783863878.698533401]: [planner][row1-debug] prepend enter src=14 row1_upper=1 raw_pts=67 a_rev=(1.150,3.579) b=(0.790,4.071) path_front=(0.790,4.071 0.0deg type=0)
[WARN] [1783863878.698640700]: [planner][row1-debug] chord reverse prefix src=14 pts=67 first=(1.150,3.579 -90.0deg) last=(0.790,4.071 0.0deg)
[WARN] [1783863878.698712499]: [planner][row1-debug] prefix[0]=(1.150,3.579 -90.0deg type=1)
[WARN] [1783863878.698798197]: [planner][row1-debug] prefix[1]=(1.150,3.589 -90.0deg type=1)
[WARN] [1783863878.698853297]: [planner][row1-debug] prefix[2]=(1.150,3.599 -90.0deg type=1)
[WARN] [1783863878.698957095]: [planner][row1-debug] prefix[3]=(1.150,3.609 -90.0deg type=1)
[WARN] [1783863878.699012394]: [planner][row1-debug] prefix[18]=(1.150,3.782 -89.6deg type=1)
[WARN] [1783863878.699092593]: [planner][row1-debug] prefix[19]=(1.150,3.798 -89.3deg type=1)
[WARN] [1783863878.699163592]: [planner][row1-debug] prefix[20]=(1.150,3.813 -89.0deg type=1)
[WARN] [1783863878.699289591]: [planner][row1-debug] route=B_TO_A1 debug_rev=row1_exact_a1_lane_v1 src=16 tgt=101 src_corr=1 tgt_corr=1 target=(1.250,4.100 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.584 far_gap=0.304 horiz=0.000 min_x=0.676
[WARN] [1783863878.699527288]: [planner][row1-debug] initial_reverse tgt=101 same_corr=1 start_to_right=0 start_lane_y=4.071 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.732 actual_end=(1.732,4.071) reverse_origin=(1.372,3.579) curve_start=(1.372,3.717) curve_pts=52 direct_ok=1
[WARN] [1783863878.699618687]: [planner][row1-debug] skeleton tgt=101 points=3 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=4.100 final_ref_x=1.250
[WARN] [1783863878.699664686]: [planner][row1-debug] skeleton[0]=(1.732,4.071)
[WARN] [1783863878.699768284]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783863878.699850483]: [planner][row1-debug] skeleton[2]=(1.250,4.301)
[WARN] [1783863878.699996681]: [planner][row1-debug] exact A1 lane using terminal-constrained G2 fallback src=16 din=-1 t_in=0.231 t_out=0.231 mt_port=0.231 available_out=0.231
[WARN] [1783863878.700100380]: [planner][row1-debug] exact A1 lane src=16 din=-1 lane_start=(1.732,4.071) curve_start=(1.481,4.071) curve_end=(1.250,4.301) stop_y=4.301 rear_stop_y=4.301 mt_port=0.231 available_out=0.231 pts=60
[WARN] [1783863878.700206879]: [planner][row1-debug] prepend enter src=16 row1_upper=1 raw_pts=67 a_rev=(1.372,3.579) b=(1.732,4.071) path_front=(1.732,4.071 180.0deg type=0)
[WARN] [1783863878.700298777]: [planner][row1-debug] chord reverse prefix src=16 pts=67 first=(1.372,3.579 -90.0deg) last=(1.732,4.071 180.0deg)
[WARN] [1783863878.700403176]: [planner][row1-debug] prefix[0]=(1.372,3.579 -90.0deg type=1)
[WARN] [1783863878.700458275]: [planner][row1-debug] prefix[1]=(1.372,3.589 -90.0deg type=1)
[WARN] [1783863878.700559174]: [planner][row1-debug] prefix[2]=(1.372,3.599 -90.0deg type=1)
[WARN] [1783863878.700613873]: [planner][row1-debug] prefix[3]=(1.372,3.609 -90.0deg type=1)
[WARN] [1783863878.700699572]: [planner][row1-debug] prefix[18]=(1.372,3.782 -90.4deg type=1)
[WARN] [1783863878.700792371]: [planner][row1-debug] prefix[19]=(1.372,3.798 -90.7deg type=1)
[WARN] [1783863878.700895669]: [planner][row1-debug] prefix[20]=(1.372,3.813 -91.0deg type=1)
[WARN] [1783863878.701050568]: [planner][row1-debug] route=B_TO_A1 debug_rev=row1_exact_a1_lane_v1 src=18 tgt=101 src_corr=1 tgt_corr=1 target=(1.250,4.100 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.584 far_gap=0.304 horiz=0.000 min_x=0.676
[WARN] [1783863878.701160766]: [planner][row1-debug] initial_reverse tgt=101 same_corr=1 start_to_right=0 start_lane_y=4.071 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.954 actual_end=(1.954,4.071) reverse_origin=(1.594,3.579) curve_start=(1.594,3.717) curve_pts=52 direct_ok=1
[WARN] [1783863878.701234765]: [planner][row1-debug] skeleton tgt=101 points=3 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=4.100 final_ref_x=1.250
[WARN] [1783863878.701339363]: [planner][row1-debug] skeleton[0]=(1.954,4.071)
[WARN] [1783863878.701394462]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783863878.701457662]: [planner][row1-debug] skeleton[2]=(1.250,4.301)
[WARN] [1783863878.701589260]: [planner][row1-debug] exact A1 lane using terminal-constrained G2 fallback src=18 din=-1 t_in=0.231 t_out=0.231 mt_port=0.231 available_out=0.231
[WARN] [1783863878.701682059]: [planner][row1-debug] exact A1 lane src=18 din=-1 lane_start=(1.954,4.071) curve_start=(1.481,4.071) curve_end=(1.250,4.301) stop_y=4.301 rear_stop_y=4.301 mt_port=0.231 available_out=0.231 pts=82
[WARN] [1783863878.701790958]: [planner][row1-debug] prepend enter src=18 row1_upper=1 raw_pts=67 a_rev=(1.594,3.579) b=(1.954,4.071) path_front=(1.954,4.071 180.0deg type=0)
[WARN] [1783863878.701872256]: [planner][row1-debug] chord reverse prefix src=18 pts=67 first=(1.594,3.579 -90.0deg) last=(1.954,4.071 180.0deg)
[WARN] [1783863878.701977855]: [planner][row1-debug] prefix[0]=(1.594,3.579 -90.0deg type=1)
[WARN] [1783863878.702049754]: [planner][row1-debug] prefix[1]=(1.594,3.589 -90.0deg type=1)
[WARN] [1783863878.702127053]: [planner][row1-debug] prefix[2]=(1.594,3.599 -90.0deg type=1)
[WARN] [1783863878.702199052]: [planner][row1-debug] prefix[3]=(1.594,3.609 -90.0deg type=1)
[WARN] [1783863878.702279251]: [planner][row1-debug] prefix[18]=(1.594,3.782 -90.4deg type=1)
[WARN] [1783863878.702347950]: [planner][row1-debug] prefix[19]=(1.594,3.798 -90.7deg type=1)
[WARN] [1783863878.702425249]: [planner][row1-debug] prefix[20]=(1.594,3.813 -91.0deg type=1)
[WARN] [1783863878.702563447]: [path_catalog] A1 generated 5/5 paths, failed=0
[WARN] [1783863878.702834443]: [path_catalog] published 13 markers on /forklift_map/markers
[INFO] [1783863879.088758255]: [forklift_map] Published 387 markers on /forklift_map/markers

