setting /run_id to 18f70f8a-7dc2-11f1-af60-47f16ede03e4
process[rosout-1]: started with pid [32105]
started core service [/rosout]
process[forklift_map-2]: started with pid [32112]
process[path_catalog_debug_node-3]: started with pid [32113]
process[rviz-4]: started with pid [32119]
[INFO] [1783840803.395480985]: [path_catalog] midpoint check: B4(1.0012,4.3750), B5(1.4988,4.3750) -> A1(1.2500,4.3750)
[INFO] [1783840803.396544465]: [path_catalog] midpoint check: B60(1.0012,0.1250), B61(1.4988,0.1250) -> A2(1.2500,0.1250)
[INFO] [1783840803.397144953]: [path_catalog] A1 virtual slot: id=101 row=0 dock=(1.2500,4.3750) pre=(1.2500,4.1000) yaw=90.0deg
[INFO] [1783840803.397255051]: [path_catalog] A2 virtual slot: id=102 row=7 dock=(1.2500,0.1250) pre=(1.2500,0.4000) yaw=-90.0deg
[WARN] [1783840803.397469746]: [path_catalog] selected depot=A1 direction=to_depot targets B0..B65 (66): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65]
[WARN] [1783840803.400134694]: [planner] slot 101: clothoid turn infeasible at skeleton point 7; p=(0.435, 3.791), prev_len=0.840, next_len=0.815, limit=0.332, route skeleton needs adjustment
[WARN] [1783840803.400364689]: [planner] slot 101: using arc fallback; curvature continuity is not satisfied
[WARN] [1783840803.400600885]: [path_catalog] kink pose jump at i=153 pose=(1.543, 0.429) prev_theta=180.0deg theta=90.0deg
[ERROR] [1783840803.400705483]: [path_catalog] B43_to_A1 rejected by task-style validation: kink
[WARN] [1783840803.400789881]: [path_catalog][reject-detail] B43_to_A1 B43 row=6 col=1 kink sharp_turn=90.0deg prev=(1.533,0.429,180.0deg,REVERSE) mid=(1.543,0.429,90.0deg,FORWARD) next=(1.543,0.583,91.1deg,FORWARD)
[INFO] [1783840803.793643640]: [forklift_map] Published 387 markers on /forklift_map/markers

