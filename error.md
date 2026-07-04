tarted core service [/rosout]
process[forklift_map-2]: started with pid [147541]
process[path_catalog_debug_node-3]: started with pid [147542]
process[rviz-4]: started with pid [147548]
[INFO] [1783155325.929227578]: [path_catalog] midpoint check: B4(1.0012,4.3750), B5(1.4988,4.3750) -> A1(1.2500,4.3750)
[INFO] [1783155325.930338258]: [path_catalog] midpoint check: B60(1.0012,0.1250), B61(1.4988,0.1250) -> A2(1.2500,0.1250)
[INFO] [1783155325.931083444]: [path_catalog] A1 virtual slot: id=101 row=0 dock=(1.2500,4.3750) pre=(1.2500,4.1000) yaw=90.0deg
[INFO] [1783155325.931187843]: [path_catalog] A2 virtual slot: id=102 row=7 dock=(1.2500,0.1250) pre=(1.2500,0.4000) yaw=-90.0deg
[WARN] [1783155325.931274141]: [path_catalog] selected depot=A1 targets B0..B65 (66): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65]
[INFO] [1783155325.931784532]: [path_catalog] A1 -> B0 row=0 col=0 wpts=162 len=2.555 arc=0
[INFO] [1783155325.932010828]: [path_catalog] A1 -> B1 row=0 col=1 wpts=162 len=2.336 arc=0
[INFO] [1783155325.932537719]: [path_catalog] A1 -> B2 row=0 col=2 wpts=162 len=2.116 arc=0
[INFO] [1783155325.932775415]: [path_catalog] A1 -> B3 row=0 col=3 wpts=162 len=1.901 arc=0
[INFO] [1783155325.932999910]: [path_catalog] A1 -> B4 row=0 col=4 wpts=162 len=1.681 arc=0
[INFO] [1783155325.933226706]: [path_catalog] A1 -> B5 row=0 col=5 wpts=162 len=1.681 arc=0
[INFO] [1783155325.933435803]: [path_catalog] A1 -> B6 row=0 col=6 wpts=162 len=1.901 arc=0
[INFO] [1783155325.933664999]: [path_catalog] A1 -> B7 row=0 col=7 wpts=162 len=2.116 arc=0
[INFO] [1783155325.933907795]: [path_catalog] A1 -> B8 row=0 col=8 wpts=162 len=2.336 arc=0
[INFO] [1783155325.934088591]: [path_catalog] A1 -> B9 row=0 col=9 wpts=162 len=2.555 arc=0
[WARN] [1783155325.934219589]: [planner][row1-debug] src=101 tgt=10 src_corr=1 tgt_corr=1 target=(0.706,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.491 far_gap=0.211 horiz=0.544 min_x=0.676
[WARN] [1783155325.934407286]: [planner][row1-debug] initial_reverse tgt=10 same_corr=1 start_to_right=0 start_lane_y=4.071 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.610 actual_end=(1.250,3.791) curve_start=(1.250,4.100) curve_pts=0 direct_ok=0
[WARN] [1783155325.934507484]: [planner][row1-debug] skeleton tgt=10 points=4 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=3.761 final_ref_x=1.250
[WARN] [1783155325.934552583]: [planner][row1-debug] skeleton[0]=(1.250,3.791)
[WARN] [1783155325.934596282]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783155325.934639882]: [planner][row1-debug] skeleton[2]=(0.706,4.071)
[WARN] [1783155325.934744180]: [planner][row1-debug] skeleton[3]=(0.706,3.579)
[WARN] [1783155325.934858478]: [planner] slot 10: clothoid turn infeasible at skeleton point 1; p=(1.250, 4.071), prev_len=0.280, next_len=0.544, limit=0.194, route skeleton needs adjustment
[WARN] [1783155325.934965676]: [planner] slot 10: using arc fallback; curvature continuity is not satisfied
[ERROR] [1783155325.935061074]: [path_catalog] A1 -> B10 rejected: curvature_discontinuity
[WARN] [1783155325.935191772]: [planner][row1-debug] src=101 tgt=11 src_corr=1 tgt_corr=2 target=(0.706,2.979 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.211 far_gap=0.491 horiz=0.544 min_x=0.676
[WARN] [1783155325.935278270]: [planner][row1-debug] initial_reverse tgt=11 same_corr=0 start_to_right=0 start_lane_y=3.791 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.610 actual_end=(1.610,3.791) curve_start=(1.250,4.129) curve_pts=56 direct_ok=1
[WARN] [1783155325.935381169]: [planner][row1-debug] skeleton tgt=11 points=5 terminal_reverse=0 goal_lane_y=2.670 terminal_stop_y=2.979 final_ref_x=1.250
[WARN] [1783155325.935452467]: [planner][row1-debug] skeleton[0]=(1.610,3.791)
[WARN] [1783155325.935527966]: [planner][row1-debug] skeleton[1]=(0.145,3.791)
[WARN] [1783155325.935572165]: [planner][row1-debug] skeleton[2]=(0.145,2.670)
[WARN] [1783155325.935686163]: [planner][row1-debug] skeleton[3]=(0.706,2.670)
[WARN] [1783155325.935750762]: [planner][row1-debug] skeleton[4]=(0.706,3.161)
[ERROR] [1783155325.935938859]: [path_catalog] A1 -> B11 rejected: curvature_discontinuity
[WARN] [1783155325.936047157]: [planner][row1-debug] src=101 tgt=12 src_corr=1 tgt_corr=1 target=(0.928,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.491 far_gap=0.211 horiz=0.358 min_x=0.676
[WARN] [1783155325.936330152]: [planner][row1-debug] initial_reverse tgt=12 same_corr=1 start_to_right=0 start_lane_y=4.071 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.610 actual_end=(1.250,3.791) curve_start=(1.250,4.100) curve_pts=0 direct_ok=0
[WARN] [1783155325.936432750]: [planner][row1-debug] skeleton tgt=12 points=4 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=3.761 final_ref_x=1.286
[WARN] [1783155325.936477049]: [planner][row1-debug] skeleton[0]=(1.250,3.791)
[WARN] [1783155325.936539648]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783155325.936631746]: [planner][row1-debug] skeleton[2]=(0.928,4.071)
[WARN] [1783155325.936684145]: [planner][row1-debug] skeleton[3]=(0.928,3.579)
[ERROR] [1783155325.936883242]: [path_catalog] A1 -> B12 rejected: curvature_discontinuity
[WARN] [1783155325.937013039]: [planner][row1-debug] src=101 tgt=13 src_corr=1 tgt_corr=2 target=(0.928,2.979 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.211 far_gap=0.491 horiz=0.358 min_x=0.676
[WARN] [1783155325.937086738]: [planner][row1-debug] initial_reverse tgt=13 same_corr=0 start_to_right=1 start_lane_y=3.791 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.890 actual_end=(0.890,3.791) curve_start=(1.250,4.129) curve_pts=56 direct_ok=1
[WARN] [1783155325.937173337]: [planner][row1-debug] skeleton tgt=13 points=5 terminal_reverse=0 goal_lane_y=2.670 terminal_stop_y=2.979 final_ref_x=1.286
[WARN] [1783155325.937217636]: [planner][row1-debug] skeleton[0]=(0.890,3.791)
[WARN] [1783155325.937261135]: [planner][row1-debug] skeleton[1]=(1.865,3.791)
[WARN] [1783155325.937342033]: [planner][row1-debug] skeleton[2]=(1.865,2.670)
[WARN] [1783155325.937410432]: [planner][row1-debug] skeleton[3]=(0.928,2.670)
[WARN] [1783155325.937454332]: [planner][row1-debug] skeleton[4]=(0.928,3.161)
[ERROR] [1783155325.937846225]: [path_catalog] A1 -> B13 rejected: shelf_collision
[WARN] [1783155325.937976422]: [planner][row1-debug] src=101 tgt=14 src_corr=1 tgt_corr=1 target=(1.150,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.491 far_gap=0.211 horiz=0.358 min_x=0.676
[WARN] [1783155325.938175919]: [planner][row1-debug] initial_reverse tgt=14 same_corr=1 start_to_right=0 start_lane_y=4.071 forward_heading=180.0deg reverse_end_dir=(0.0deg) desired_end_x=1.610 actual_end=(1.250,3.791) curve_start=(1.250,4.100) curve_pts=0 direct_ok=0
[WARN] [1783155325.938277417]: [planner][row1-debug] skeleton tgt=14 points=4 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=3.761 final_ref_x=1.508
[WARN] [1783155325.938346016]: [planner][row1-debug] skeleton[0]=(1.250,3.791)
[WARN] [1783155325.938444914]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783155325.938497213]: [planner][row1-debug] skeleton[2]=(1.150,4.071)
[WARN] [1783155325.938557212]: [planner][row1-debug] skeleton[3]=(1.150,3.579)
[ERROR] [1783155325.938756708]: [path_catalog] A1 -> B14 rejected: curvature_discontinuity
[WARN] [1783155325.938886606]: [planner][row1-debug] src=101 tgt=15 src_corr=1 tgt_corr=2 target=(1.150,2.979 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.211 far_gap=0.491 horiz=0.358 min_x=0.676
[WARN] [1783155325.938977005]: [planner][row1-debug] initial_reverse tgt=15 same_corr=0 start_to_right=1 start_lane_y=3.791 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.890 actual_end=(0.890,3.791) curve_start=(1.250,4.129) curve_pts=56 direct_ok=1
[WARN] [1783155325.939047103]: [planner][row1-debug] skeleton tgt=15 points=5 terminal_reverse=0 goal_lane_y=2.670 terminal_stop_y=2.979 final_ref_x=1.508
[WARN] [1783155325.939123602]: [planner][row1-debug] skeleton[0]=(0.890,3.791)
[WARN] [1783155325.939191201]: [planner][row1-debug] skeleton[1]=(1.865,3.791)
[WARN] [1783155325.939251499]: [planner][row1-debug] skeleton[2]=(1.865,2.670)
[WARN] [1783155325.939319799]: [planner][row1-debug] skeleton[3]=(1.150,2.670)
[WARN] [1783155325.939375698]: [planner][row1-debug] skeleton[4]=(1.150,3.161)
[ERROR] [1783155325.939608194]: [path_catalog] A1 -> B15 rejected: curvature_discontinuity
[WARN] [1783155325.939738391]: [planner][row1-debug] src=101 tgt=16 src_corr=1 tgt_corr=1 target=(1.372,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.211 far_gap=0.491 horiz=0.358 min_x=0.676
[WARN] [1783155325.939908288]: [planner][row1-debug] initial_reverse tgt=16 same_corr=1 start_to_right=1 start_lane_y=4.071 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.890 actual_end=(1.250,3.791) curve_start=(1.250,4.100) curve_pts=0 direct_ok=0
[WARN] [1783155325.939996987]: [planner][row1-debug] skeleton tgt=16 points=4 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=3.761 final_ref_x=1.014
[WARN] [1783155325.940041186]: [planner][row1-debug] skeleton[0]=(1.250,3.791)
[WARN] [1783155325.940117584]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783155325.940194583]: [planner][row1-debug] skeleton[2]=(1.372,4.071)
[WARN] [1783155325.940378580]: [planner][row1-debug] skeleton[3]=(1.372,3.579)
[ERROR] [1783155325.940561877]: [path_catalog] A1 -> B16 rejected: curvature_discontinuity
[WARN] [1783155325.940692874]: [planner][row1-debug] src=101 tgt=17 src_corr=1 tgt_corr=2 target=(1.372,2.979 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.491 far_gap=0.211 horiz=0.358 min_x=0.676
[WARN] [1783155325.940803872]: [planner][row1-debug] initial_reverse tgt=17 same_corr=0 start_to_right=1 start_lane_y=3.791 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.890 actual_end=(0.890,3.791) curve_start=(1.250,4.129) curve_pts=56 direct_ok=1
[WARN] [1783155325.940867871]: [planner][row1-debug] skeleton tgt=17 points=5 terminal_reverse=0 goal_lane_y=2.670 terminal_stop_y=2.979 final_ref_x=1.014
[WARN] [1783155325.940962169]: [planner][row1-debug] skeleton[0]=(0.890,3.791)
[WARN] [1783155325.941030369]: [planner][row1-debug] skeleton[1]=(1.865,3.791)
[WARN] [1783155325.941103367]: [planner][row1-debug] skeleton[2]=(1.865,2.670)
[WARN] [1783155325.941171266]: [planner][row1-debug] skeleton[3]=(1.372,2.670)
[WARN] [1783155325.941247365]: [planner][row1-debug] skeleton[4]=(1.372,3.161)
[ERROR] [1783155325.941435061]: [path_catalog] A1 -> B17 rejected: curvature_discontinuity
[WARN] [1783155325.941565459]: [planner][row1-debug] src=101 tgt=18 src_corr=1 tgt_corr=1 target=(1.594,3.761 th=-90.0deg) mode=auto terminal_reverse=0 near_gap=0.211 far_gap=0.491 horiz=0.358 min_x=0.676
[WARN] [1783155325.941707957]: [planner][row1-debug] initial_reverse tgt=18 same_corr=1 start_to_right=1 start_lane_y=4.071 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.890 actual_end=(1.250,3.791) curve_start=(1.250,4.100) curve_pts=0 direct_ok=0
[WARN] [1783155325.941794855]: [planner][row1-debug] skeleton tgt=18 points=4 terminal_reverse=0 goal_lane_y=4.071 terminal_stop_y=3.761 final_ref_x=1.235
[WARN] [1783155325.941863253]: [planner][row1-debug] skeleton[0]=(1.250,3.791)
[WARN] [1783155325.941936252]: [planner][row1-debug] skeleton[1]=(1.250,4.071)
[WARN] [1783155325.942001551]: [planner][row1-debug] skeleton[2]=(1.594,4.071)
[WARN] [1783155325.942076250]: [planner][row1-debug] skeleton[3]=(1.594,3.579)
[ERROR] [1783155325.942236947]: [path_catalog] A1 -> B18 rejected: curvature_discontinuity
[WARN] [1783155325.942367045]: [planner][row1-debug] src=101 tgt=19 src_corr=1 tgt_corr=2 target=(1.594,2.979 th=90.0deg) mode=auto terminal_reverse=0 near_gap=0.491 far_gap=0.211 horiz=0.358 min_x=0.676
[WARN] [1783155325.942428244]: [planner][row1-debug] initial_reverse tgt=19 same_corr=0 start_to_right=1 start_lane_y=3.791 forward_heading=0.0deg reverse_end_dir=(180.0deg) desired_end_x=0.890 actual_end=(0.890,3.791) curve_start=(1.250,4.129) curve_pts=56 direct_ok=1
[WARN] [1783155325.942513942]: [planner][row1-debug] skeleton tgt=19 points=5 terminal_reverse=0 goal_lane_y=2.670 terminal_stop_y=2.979 final_ref_x=1.235
[WARN] [1783155325.942566641]: [planner][row1-debug] skeleton[0]=(0.890,3.791)
[WARN] [1783155325.942665539]: [planner][row1-debug] skeleton[1]=(1.865,3.791)
[WARN] [1783155325.942738038]: [planner][row1-debug] skeleton[2]=(1.865,2.670)
[WARN] [1783155325.942837536]: [planner][row1-debug] skeleton[3]=(1.594,2.670)
[WARN] [1783155325.942905735]: [planner][row1-debug] skeleton[4]=(1.594,3.161)
[ERROR] [1783155325.943120131]: [path_catalog] A1 -> B19 rejected: curvature_discontinuity
[INFO] [1783155325.943370527]: [path_catalog] A1 -> B20 row=3 col=0 wpts=187 len=3.356 arc=0
[ERROR] [1783155325.943725821]: [path_catalog] A1 -> B21 rejected: curvature_discontinuity
[INFO] [1783155325.944105014]: [path_catalog] A1 -> B22 row=3 col=1 wpts=230 len=3.396 arc=0
[ERROR] [1783155325.944642104]: [path_catalog] A1 -> B23 rejected: curvature_discontinuity
[ERROR] [1783155325.944879200]: [path_catalog] A1 -> B24 rejected: curvature_discontinuity
[ERROR] [1783155325.945306192]: [path_catalog] A1 -> B25 rejected: curvature_discontinuity
[ERROR] [1783155325.945633787]: [path_catalog] A1 -> B26 rejected: shelf_collision
[ERROR] [1783155325.946035879]: [path_catalog] A1 -> B27 rejected: curvature_discontinuity
[ERROR] [1783155325.946396973]: [path_catalog] A1 -> B28 rejected: curvature_discontinuity
[ERROR] [1783155325.946835765]: [path_catalog] A1 -> B29 rejected: curvature_discontinuity
[ERROR] [1783155325.947099160]: [path_catalog] A1 -> B30 rejected: curvature_discontinuity
[ERROR] [1783155325.947511853]: [path_catalog] A1 -> B31 rejected: curvature_discontinuity
[ERROR] [1783155325.947706749]: [path_catalog] A1 -> B32 rejected: shelf_collision
[ERROR] [1783155325.948243040]: [path_catalog] A1 -> B33 rejected: curvature_discontinuity
[ERROR] [1783155325.948440136]: [path_catalog] A1 -> B34 rejected: shelf_collision
[ERROR] [1783155325.948793330]: [path_catalog] A1 -> B35 rejected: curvature_discontinuity
[ERROR] [1783155325.949022726]: [path_catalog] A1 -> B36 rejected: curvature_discontinuity
[ERROR] [1783155325.949439418]: [path_catalog] A1 -> B37 rejected: curvature_discontinuity
[ERROR] [1783155325.949666214]: [path_catalog] A1 -> B38 rejected: curvature_discontinuity
[ERROR] [1783155325.949957309]: [path_catalog] A1 -> B39 rejected: curvature_discontinuity
[ERROR] [1783155325.950312603]: [path_catalog] A1 -> B40 rejected: curvature_discontinuity
[ERROR] [1783155325.951340385]: [path_catalog] A1 -> B41 rejected: curvature_discontinuity
[ERROR] [1783155325.951754478]: [path_catalog] A1 -> B42 rejected: curvature_discontinuity
[ERROR] [1783155325.952310768]: [path_catalog] A1 -> B43 rejected: curvature_discontinuity
[ERROR] [1783155325.952749960]: [path_catalog] A1 -> B44 rejected: curvature_discontinuity
[ERROR] [1783155325.953194452]: [path_catalog] A1 -> B45 rejected: curvature_discontinuity
[ERROR] [1783155325.953603345]: [path_catalog] A1 -> B46 rejected: curvature_discontinuity
[ERROR] [1783155325.954056437]: [path_catalog] A1 -> B47 rejected: curvature_discontinuity
[ERROR] [1783155325.954472929]: [path_catalog] A1 -> B48 rejected: curvature_discontinuity
[ERROR] [1783155325.954857622]: [path_catalog] A1 -> B49 rejected: curvature_discontinuity
[ERROR] [1783155325.955212916]: [path_catalog] A1 -> B50 rejected: curvature_discontinuity
[ERROR] [1783155325.955580009]: [path_catalog] A1 -> B51 rejected: curvature_discontinuity
[ERROR] [1783155325.955940104]: [path_catalog] A1 -> B52 rejected: curvature_discontinuity
[ERROR] [1783155325.957017684]: [path_catalog] A1 -> B53 rejected: curvature_discontinuity
[ERROR] [1783155325.957297679]: [path_catalog] A1 -> B54 rejected: curvature_discontinuity
[ERROR] [1783155325.957597274]: [path_catalog] A1 -> B55 rejected: curvature_discontinuity
[ERROR] [1783155325.958205563]: [path_catalog] A1 -> B56 rejected: curvature_discontinuity
[ERROR] [1783155325.958634956]: [path_catalog] A1 -> B57 rejected: curvature_discontinuity
[ERROR] [1783155325.959060148]: [path_catalog] A1 -> B58 rejected: curvature_discontinuity
[ERROR] [1783155325.959478541]: [path_catalog] A1 -> B59 rejected: curvature_discontinuity
[ERROR] [1783155325.959737636]: [path_catalog] A1 -> B60 rejected: curvature_discontinuity
[ERROR] [1783155325.960273726]: [path_catalog] A1 -> B61 rejected: curvature_discontinuity
[ERROR] [1783155325.960596620]: [path_catalog] A1 -> B62 rejected: curvature_discontinuity
[ERROR] [1783155325.960929514]: [path_catalog] A1 -> B63 rejected: curvature_discontinuity
[ERROR] [1783155325.961202810]: [path_catalog] A1 -> B64 rejected: curvature_discontinuity
[ERROR] [1783155325.961498704]: [path_catalog] A1 -> B65 rejected: curvature_discontinuity
[WARN] [1783155325.961602702]: [path_catalog] A1 generated 12/66 paths, failed=54
[WARN] [1783155325.962067094]: [path_catalog] published 81 markers on /forklift_map/markers
[INFO] [1783155326.340575069]: [forklift_map] Published 387 markers on /forklift_map/markers

