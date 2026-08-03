[multi_patrol] coordination log started
vehicle_count=3 one_shot=0 use_a1_cycle=1 a1_request=1.5 prediction_horizon=10 prediction_step=0.05
[A1_STATE] state=WAITING owner=V-1 candidate=V-1 blocker=V-1 queue=[V2,V1] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_A1,s=0/3.36835,reason=clear;V1:TO_A1,s=0/3.3914,reason=clear;V2:TO_A1,s=0/3.14901,reason=brake_V0}
[multi_patrol][state] sim_t=0.10s V0 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=0 slot=38->-1 pending_B=9 s=0.002/3.368 rem=3.366 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=0.10s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=0 slot=20->-1 pending_B=61 s=0.002/3.391 rem=3.389 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=0.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.000/3.149 rem=3.149 speed=0.000 wait=0.10 dwell=0.00
[A1_STATE] state=ADMITTED owner=V0 candidate=V0 blocker=V-1 queue=[V0,V2,V1] tx_valid=1 tx_target=B9 entry_s=1.585 release_s=1.245 vehicles={V0:TO_A1,s=0.002/3.36835,reason=clear;V1:TO_A1,s=0.002/3.3914,reason=clear;V2:TO_A1,s=0/3.14901,reason=brake_V0}
[A1_STATE] state=ADMITTED owner=V0 candidate=V-1 blocker=V-1 queue=[V0,V2,V1] tx_valid=1 tx_target=B9 entry_s=1.585 release_s=1.245 vehicles={V0:TO_A1,s=0.006/3.36835,reason=clear;V1:TO_A1,s=0.006/3.3914,reason=clear;V2:TO_A1,s=0/3.14901,reason=brake_V0}
[multi_patrol][state] sim_t=0.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=following_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.000/3.149 rem=3.149 speed=0.000 wait=0.40 dwell=0.00
[multi_patrol][state] sim_t=2.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.000/3.149 rem=3.149 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=3.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.000/3.149 rem=3.149 speed=0.000 wait=3.20 dwell=0.00
[multi_patrol][state] sim_t=5.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.000/3.149 rem=3.149 speed=0.000 wait=5.20 dwell=0.00
[multi_patrol][state] sim_t=7.00s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.002/3.149 rem=3.147 speed=0.020 wait=7.00 dwell=0.00
[multi_patrol][state] sim_t=7.10s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.006/3.149 rem=3.143 speed=0.040 wait=7.10 dwell=0.00
[multi_patrol][state] sim_t=7.50s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.028/3.149 rem=3.121 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=7.80s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.056/3.149 rem=3.093 speed=0.080 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=9.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.374/3.391 rem=2.017 speed=0.086 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=9.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.380/3.391 rem=2.012 speed=0.056 wait=0.20 dwell=0.00
[multi_patrol][state] sim_t=9.50s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.065/3.149 rem=3.084 speed=0.020 wait=1.80 dwell=0.00
[multi_patrol][state] sim_t=9.60s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.384/3.391 rem=2.007 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=9.60s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.069/3.149 rem=3.080 speed=0.040 wait=1.90 dwell=0.00
[multi_patrol][state] sim_t=9.70s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.388/3.391 rem=2.003 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=10.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.400/3.391 rem=1.991 speed=0.020 wait=1.00 dwell=0.00
[multi_patrol][state] sim_t=10.00s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.091/3.149 rem=3.058 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=10.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.400/3.391 rem=1.991 speed=0.000 wait=1.10 dwell=0.00
[multi_patrol][state] sim_t=10.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.402/3.391 rem=1.989 speed=0.020 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=10.60s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.406/3.391 rem=1.985 speed=0.040 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=10.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.407/3.391 rem=1.984 speed=0.010 wait=1.70 dwell=0.00
[multi_patrol][state] sim_t=10.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.407/3.391 rem=1.984 speed=0.000 wait=1.80 dwell=0.00
[multi_patrol][state] sim_t=11.20s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.409/3.391 rem=1.982 speed=0.020 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=11.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.409/3.391 rem=1.982 speed=0.000 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=11.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.409/3.391 rem=1.982 speed=0.000 wait=2.40 dwell=0.00
[multi_patrol][state] sim_t=11.80s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.020 wait=2.80 dwell=0.00
[multi_patrol][state] sim_t=11.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=2.90 dwell=0.00
[multi_patrol][state] sim_t=14.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=5.00 dwell=0.00
[multi_patrol][state] sim_t=14.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.658/3.149 rem=2.491 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=14.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.672/3.149 rem=2.477 speed=0.052 wait=0.30 dwell=0.00
[multi_patrol][state] sim_t=14.50s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.676/3.149 rem=2.473 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=14.60s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.680/3.149 rem=2.469 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=14.80s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.687/3.149 rem=2.462 speed=0.020 wait=0.90 dwell=0.00
[multi_patrol][state] sim_t=14.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.687/3.149 rem=2.462 speed=0.000 wait=1.00 dwell=0.00
[multi_patrol][state] sim_t=15.30s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.689/3.149 rem=2.460 speed=0.020 wait=1.40 dwell=0.00
[multi_patrol][state] sim_t=15.40s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.693/3.149 rem=2.456 speed=0.040 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=15.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.694/3.149 rem=2.455 speed=0.010 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=15.70s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.694/3.149 rem=2.455 speed=0.000 wait=1.80 dwell=0.00
[multi_patrol][state] sim_t=16.00s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.696/3.149 rem=2.453 speed=0.020 wait=2.10 dwell=0.00
[multi_patrol][state] sim_t=16.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=7.10 dwell=0.00
[multi_patrol][state] sim_t=16.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.696/3.149 rem=2.453 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=18.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=9.20 dwell=0.00
[multi_patrol][state] sim_t=18.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.696/3.149 rem=2.453 speed=0.000 wait=4.30 dwell=0.00
[multi_patrol][state] sim_t=20.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=11.20 dwell=0.00
[multi_patrol][state] sim_t=20.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=0.696/3.149 rem=2.453 speed=0.000 wait=6.30 dwell=0.00
[multi_patrol][state] sim_t=21.10s V0 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=0 slot=38->-1 pending_B=9 s=3.368/3.368 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V0 candidate=V-1 blocker=V-1 queue=[V2,V1] tx_valid=1 tx_target=B9 entry_s=1.585 release_s=1.245 vehicles={V0:PICKUP_DWELL,s=3.36835/3.36835,reason=not_active;V1:TO_A1,s=1.41136/3.3914,reason=wait_a1_turn_V0;V2:TO_A1,s=0.696084/3.14901,reason=clear}
[multi_patrol][state] sim_t=21.20s V0 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=0 slot=38->-1 pending_B=9 s=3.368/3.368 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=21.20s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.698/3.149 rem=2.451 speed=0.020 wait=7.30 dwell=0.00
[multi_patrol][state] sim_t=21.30s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.702/3.149 rem=2.447 speed=0.040 wait=7.40 dwell=0.00
[multi_patrol][state] sim_t=21.70s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=0.724/3.149 rem=2.425 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=22.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=13.30 dwell=0.00
[multi_patrol][state] sim_t=24.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=15.40 dwell=0.00
[multi_patrol][state] sim_t=25.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=1.271/3.149 rem=1.878 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=25.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.279/3.149 rem=1.870 speed=0.082 wait=0.20 dwell=0.00
[multi_patrol][state] sim_t=25.60s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.289/3.149 rem=1.860 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=25.70s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.293/3.149 rem=1.856 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=26.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=1.305/3.149 rem=1.844 speed=0.020 wait=1.00 dwell=0.00
[A1_STATE] state=EXITING owner=V0 candidate=V-1 blocker=V-1 queue=[V2,V1] tx_valid=1 tx_target=B9 entry_s=1.585 release_s=1.245 vehicles={V0:TO_B,s=0/2.55515,reason=clear;V1:TO_A1,s=1.41136/3.3914,reason=wait_a1_turn_V0;V2:TO_A1,s=1.30462/3.14901,reason=action_hold}
[multi_patrol][state] sim_t=26.10s V0 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=0 slot=38->9 pending_B=-1 s=0.002/2.555 rem=2.553 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=26.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.305/3.149 rem=1.844 speed=0.000 wait=1.10 dwell=0.00
[multi_patrol][state] sim_t=26.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=17.30 dwell=0.00
[multi_patrol][state] sim_t=26.50s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.307/3.149 rem=1.842 speed=0.020 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=26.60s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.311/3.149 rem=1.838 speed=0.040 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=26.70s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=1.312/3.149 rem=1.837 speed=0.010 wait=1.70 dwell=0.00
[multi_patrol][state] sim_t=26.80s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.312/3.149 rem=1.837 speed=0.000 wait=1.80 dwell=0.00
[multi_patrol][state] sim_t=27.20s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.314/3.149 rem=1.835 speed=0.020 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=27.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=1.314/3.149 rem=1.835 speed=0.000 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=27.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.314/3.149 rem=1.835 speed=0.000 wait=2.40 dwell=0.00
[multi_patrol][state] sim_t=27.80s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.020 wait=2.80 dwell=0.00
[multi_patrol][state] sim_t=27.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=2.90 dwell=0.00
[multi_patrol][state] sim_t=28.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=19.40 dwell=0.00
[multi_patrol][state] sim_t=30.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=5.00 dwell=0.00
[multi_patrol][state] sim_t=30.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=21.30 dwell=0.00
[multi_patrol][state] sim_t=32.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=7.00 dwell=0.00
[multi_patrol][state] sim_t=32.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=23.30 dwell=0.00
[A1_STATE] state=WAITING owner=V-1 candidate=V1 blocker=V0 queue=[V1,V2] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_B,s=1.26441/2.55515,reason=clear;V1:TO_A1,s=1.41136/3.3914,reason=wait_a1_admission_clear;V2:TO_A1,s=1.31562/3.14901,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=34.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=25.10 dwell=0.00
[multi_patrol][state] sim_t=34.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=9.10 dwell=0.00
[multi_patrol][state] sim_t=36.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=0 slot=20->-1 pending_B=61 s=1.411/3.391 rem=1.980 speed=0.000 wait=27.20 dwell=0.00
[multi_patrol][state] sim_t=36.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=11.20 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V1 blocker=V-1 queue=[V1,V2] tx_valid=1 tx_target=B4 entry_s=1.46 release_s=1.435 vehicles={V0:TO_B,s=2.05717/2.55515,reason=clear;V1:TO_A1,s=1.41136/3.3914,reason=clear;V2:TO_A1,s=1.31562/3.14901,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=38.30s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=20->-1 pending_B=4 s=1.413/3.391 rem=1.978 speed=0.020 wait=29.30 dwell=0.00
[multi_patrol][state] sim_t=38.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=13.30 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V-1 blocker=V-1 queue=[V1,V2] tx_valid=1 tx_target=B4 entry_s=1.46 release_s=1.435 vehicles={V0:TO_B,s=2.0714/2.55515,reason=clear;V1:TO_A1,s=1.41336/3.3914,reason=action_hold;V2:TO_A1,s=1.31562/3.14901,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=38.40s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=20->-1 pending_B=4 s=1.417/3.391 rem=1.974 speed=0.040 wait=29.40 dwell=0.00
[multi_patrol][state] sim_t=38.80s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=0 slot=20->-1 pending_B=4 s=1.439/3.391 rem=1.952 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=40.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=15.40 dwell=0.00
[multi_patrol][state] sim_t=41.30s V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=0 slot=9->9 pending_B=-1 s=2.555/2.555 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V-1 blocker=V-1 queue=[V1,V2] tx_valid=1 tx_target=B4 entry_s=1.46 release_s=1.435 vehicles={V0:UNLOAD_DWELL,s=2.55515/2.55515,reason=not_active;V1:TO_A1,s=1.87734/3.3914,reason=clear;V2:TO_A1,s=1.31562/3.14901,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=41.40s V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=0 slot=9->9 pending_B=-1 s=2.555/2.555 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[multi_patrol][state] sim_t=42.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=17.40 dwell=0.00
[multi_patrol][state] sim_t=44.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=19.50 dwell=0.00
[multi_patrol][state] sim_t=46.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=21.50 dwell=0.00
[multi_patrol][state] sim_t=48.60s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=23.60 dwell=0.00
[multi_patrol][state] sim_t=50.40s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=0 slot=20->-1 pending_B=4 s=3.391/3.391 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V1 candidate=V-1 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B4 entry_s=1.46 release_s=1.435 vehicles={V0:UNLOAD_DWELL,s=2.55515/2.55515,reason=not_active;V1:PICKUP_DWELL,s=3.3914/3.3914,reason=not_active;V2:TO_A1,s=1.31562/3.14901,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=50.50s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=0 slot=20->-1 pending_B=4 s=3.391/3.391 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=50.60s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=25.60 dwell=0.00
[A1_STATE] state=LOADING owner=V1 candidate=V-1 blocker=V-1 queue=[V2,V0] tx_valid=1 tx_target=B4 entry_s=1.46 release_s=1.435 vehicles={V0:TO_A1,s=0/3.03446,reason=wait_a1_turn_V1;V1:PICKUP_DWELL,s=3.3914/3.3914,reason=not_active;V2:TO_A1,s=1.31562/3.14901,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=51.30s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=52.70s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=27.70 dwell=0.00
[multi_patrol][state] sim_t=53.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=54.70s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=29.70 dwell=0.00
[A1_STATE] state=EXITING owner=V1 candidate=V-1 blocker=V-1 queue=[V2,V0] tx_valid=1 tx_target=B4 entry_s=1.46 release_s=1.435 vehicles={V0:TO_A1,s=0/3.03446,reason=wait_a1_turn_V1;V1:TO_B,s=0/1.68118,reason=clear;V2:TO_A1,s=1.31562/3.14901,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=55.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=4.20 dwell=0.00
[multi_patrol][state] sim_t=55.40s V1 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=0 slot=20->4 pending_B=-1 s=0.002/1.681 rem=1.679 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=56.80s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=31.80 dwell=0.00
[multi_patrol][state] sim_t=57.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=6.30 dwell=0.00
[multi_patrol][state] sim_t=58.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=33.90 dwell=0.00
[multi_patrol][state] sim_t=59.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=8.30 dwell=0.00
[multi_patrol][state] sim_t=60.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=35.90 dwell=0.00
[multi_patrol][state] sim_t=61.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=10.30 dwell=0.00
[multi_patrol][state] sim_t=63.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=38.00 dwell=0.00
[multi_patrol][state] sim_t=63.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=12.30 dwell=0.00
[A1_STATE] state=WAITING owner=V-1 candidate=V2 blocker=V1 queue=[V2,V0] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_A1,s=0/3.03446,reason=wait_a1_turn_V2;V1:TO_B,s=1.44131/1.68118,reason=clear;V2:TO_A1,s=1.31562/3.14901,reason=wait_a1_admission_clear}
[multi_patrol][state] sim_t=65.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=13.80 dwell=0.00
[multi_patrol][state] sim_t=65.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=0 slot=36->-1 pending_B=8 s=1.316/3.149 rem=1.833 speed=0.000 wait=40.00 dwell=0.00
[multi_patrol][state] sim_t=66.20s V1 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=0 slot=4->4 pending_B=-1 s=1.681/1.681 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V2 blocker=V-1 queue=[V2,V0] tx_valid=1 tx_target=B5 entry_s=1.365 release_s=1.435 vehicles={V0:TO_A1,s=0/3.03446,reason=wait_a1_turn_V2;V1:UNLOAD_DWELL,s=1.68118/1.68118,reason=not_active;V2:TO_A1,s=1.31562/3.14901,reason=clear}
[multi_patrol][state] sim_t=66.30s V1 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=0 slot=4->4 pending_B=-1 s=1.681/1.681 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[multi_patrol][state] sim_t=66.30s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=0 slot=36->-1 pending_B=5 s=1.318/3.149 rem=1.831 speed=0.020 wait=41.30 dwell=0.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V-1 blocker=V-1 queue=[V2,V0] tx_valid=1 tx_target=B5 entry_s=1.365 release_s=1.435 vehicles={V0:TO_A1,s=0/3.03446,reason=wait_a1_turn_V2;V1:UNLOAD_DWELL,s=1.68118/1.68118,reason=not_active;V2:TO_A1,s=1.31762/3.14901,reason=action_hold}
[multi_patrol][state] sim_t=66.40s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=0 slot=36->-1 pending_B=5 s=1.322/3.149 rem=1.827 speed=0.040 wait=41.40 dwell=0.00
[multi_patrol][state] sim_t=66.80s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=0 slot=36->-1 pending_B=5 s=1.344/3.149 rem=1.805 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=67.10s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=15.90 dwell=0.00
[multi_patrol][state] sim_t=69.10s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=17.90 dwell=0.00
[multi_patrol][state] sim_t=71.10s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=19.90 dwell=0.00
[multi_patrol][state] sim_t=73.20s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=22.00 dwell=0.00
[multi_patrol][state] sim_t=75.30s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=24.10 dwell=0.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V-1 blocker=V-1 queue=[V2,V0,V1] tx_valid=1 tx_target=B5 entry_s=1.365 release_s=1.435 vehicles={V0:TO_A1,s=0/3.03446,reason=wait_a1_turn_V2;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V2;V2:TO_A1,s=2.81295/3.14901,reason=clear}
[multi_patrol][state] sim_t=76.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=77.30s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=26.10 dwell=0.00
[multi_patrol][state] sim_t=78.10s V2 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=0 slot=36->-1 pending_B=5 s=3.149/3.149 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V2 candidate=V-1 blocker=V-1 queue=[V0,V1] tx_valid=1 tx_target=B5 entry_s=1.365 release_s=1.435 vehicles={V0:TO_A1,s=0/3.03446,reason=wait_a1_turn_V2;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V2;V2:PICKUP_DWELL,s=3.14901/3.14901,reason=not_active}
[multi_patrol][state] sim_t=78.20s V2 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=0 slot=36->-1 pending_B=5 s=3.149/3.149 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=78.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=79.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=28.20 dwell=0.00
[multi_patrol][state] sim_t=80.50s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=4.40 dwell=0.00
[multi_patrol][state] sim_t=81.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=30.30 dwell=0.00
[multi_patrol][state] sim_t=82.50s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=6.40 dwell=0.00
[A1_STATE] state=EXITING owner=V2 candidate=V-1 blocker=V-1 queue=[V0,V1] tx_valid=1 tx_target=B5 entry_s=1.365 release_s=1.435 vehicles={V0:TO_A1,s=0/3.03446,reason=wait_a1_turn_V2;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V2;V2:TO_B,s=0/1.68118,reason=clear}
[multi_patrol][state] sim_t=83.10s V2 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=0 slot=36->5 pending_B=-1 s=0.002/1.681 rem=1.679 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=83.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=32.30 dwell=0.00
[multi_patrol][state] sim_t=84.60s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=8.50 dwell=0.00
[multi_patrol][state] sim_t=85.60s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=34.40 dwell=0.00
[multi_patrol][state] sim_t=86.60s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=10.50 dwell=0.00
[multi_patrol][state] sim_t=87.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=36.50 dwell=0.00
[multi_patrol][state] sim_t=88.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=12.60 dwell=0.00
[multi_patrol][state] sim_t=89.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=38.60 dwell=0.00
[multi_patrol][state] sim_t=90.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=14.70 dwell=0.00
[multi_patrol][state] sim_t=91.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=40.60 dwell=0.00
[A1_STATE] state=WAITING owner=V-1 candidate=V0 blocker=V2 queue=[V0,V1] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_A1,s=0/3.03446,reason=wait_a1_admission_clear;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V0;V2:TO_B,s=1.44131/1.68118,reason=clear}
[multi_patrol][state] sim_t=92.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=2 task=1 slot=9->-1 pending_B=7 s=0.000/3.034 rem=3.034 speed=0.000 wait=41.50 dwell=0.00
[multi_patrol][state] sim_t=92.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=16.60 dwell=0.00
[multi_patrol][state] sim_t=93.90s V2 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=0 slot=5->5 pending_B=-1 s=1.681/1.681 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=ADMITTED owner=V0 candidate=V0 blocker=V-1 queue=[V0,V1] tx_valid=1 tx_target=B7 entry_s=0 release_s=1.245 vehicles={V0:TO_A1,s=0/3.03446,reason=clear;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V0;V2:UNLOAD_DWELL,s=1.68118/1.68118,reason=not_active}
[multi_patrol][state] sim_t=94.00s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=1 slot=9->-1 pending_B=7 s=0.002/3.034 rem=3.032 speed=0.020 wait=42.80 dwell=0.00
[multi_patrol][state] sim_t=94.00s V2 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=0 slot=5->5 pending_B=-1 s=1.681/1.681 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[A1_STATE] state=ADMITTED owner=V0 candidate=V-1 blocker=V-1 queue=[V0,V1] tx_valid=1 tx_target=B7 entry_s=0 release_s=1.245 vehicles={V0:TO_A1,s=0.002/3.03446,reason=action_hold;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V0;V2:UNLOAD_DWELL,s=1.68118/1.68118,reason=not_active}
[multi_patrol][state] sim_t=94.10s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=1 slot=9->-1 pending_B=7 s=0.006/3.034 rem=3.028 speed=0.040 wait=42.90 dwell=0.00
[multi_patrol][state] sim_t=94.50s V0 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=1 slot=9->-1 pending_B=7 s=0.028/3.034 rem=3.006 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=94.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=18.70 dwell=0.00
[multi_patrol][state] sim_t=96.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=20.70 dwell=0.00
[multi_patrol][state] sim_t=98.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=22.80 dwell=0.00
[multi_patrol][state] sim_t=101.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=24.90 dwell=0.00
[multi_patrol][state] sim_t=103.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=26.90 dwell=0.00
[A1_STATE] state=ADMITTED owner=V0 candidate=V-1 blocker=V-1 queue=[V0,V1,V2] tx_valid=1 tx_target=B7 entry_s=0 release_s=1.245 vehicles={V0:TO_A1,s=1.65164/3.03446,reason=clear;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V0;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V0}
[multi_patrol][state] sim_t=103.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=104.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=28.80 dwell=0.00
[multi_patrol][state] sim_t=106.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=106.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=30.80 dwell=0.00
[multi_patrol][state] sim_t=108.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=4.40 dwell=0.00
[multi_patrol][state] sim_t=109.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=32.90 dwell=0.00
[multi_patrol][state] sim_t=110.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=6.50 dwell=0.00
[multi_patrol][state] sim_t=111.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=34.90 dwell=0.00
[multi_patrol][state] sim_t=111.50s V0 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=1 slot=9->-1 pending_B=7 s=3.034/3.034 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V0 candidate=V-1 blocker=V-1 queue=[V1,V2] tx_valid=1 tx_target=B7 entry_s=0 release_s=1.245 vehicles={V0:PICKUP_DWELL,s=3.03446/3.03446,reason=not_active;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V0;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V0}
[multi_patrol][state] sim_t=111.60s V0 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=1 slot=9->-1 pending_B=7 s=3.034/3.034 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=112.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=8.50 dwell=0.00
[multi_patrol][state] sim_t=113.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=37.00 dwell=0.00
[multi_patrol][state] sim_t=114.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=10.60 dwell=0.00
[multi_patrol][state] sim_t=115.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=39.10 dwell=0.00
[multi_patrol][state] sim_t=116.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=12.60 dwell=0.00
[A1_STATE] state=EXITING owner=V0 candidate=V-1 blocker=V-1 queue=[V1,V2] tx_valid=1 tx_target=B7 entry_s=0 release_s=1.245 vehicles={V0:TO_B,s=0/2.11648,reason=clear;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V0;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V0}
[multi_patrol][state] sim_t=116.50s V0 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=1 slot=9->7 pending_B=-1 s=0.002/2.116 rem=2.114 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=117.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=41.00 dwell=0.00
[multi_patrol][state] sim_t=118.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=following_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=14.30 dwell=0.00
[multi_patrol][state] sim_t=118.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=14.40 dwell=0.00
[multi_patrol][state] sim_t=119.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=42.90 dwell=0.00
[multi_patrol][state] sim_t=120.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=16.20 dwell=0.00
[multi_patrol][state] sim_t=120.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=44.80 dwell=0.00
[multi_patrol][state] sim_t=122.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=18.20 dwell=0.00
[multi_patrol][state] sim_t=122.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=46.70 dwell=0.00
[multi_patrol][state] sim_t=124.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V0 blocker=0 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=20.20 dwell=0.00
[A1_STATE] state=WAITING owner=V-1 candidate=V1 blocker=V0 queue=[V1,V2] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_B,s=1.26441/2.11648,reason=clear;V1:TO_A1,s=0/1.6805,reason=wait_a1_admission_clear;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=124.50s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=1 slot=4->-1 pending_B=21 s=0.000/1.680 rem=1.680 speed=0.000 wait=48.40 dwell=0.00
[multi_patrol][state] sim_t=124.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=20.70 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V1 blocker=V-1 queue=[V1,V2] tx_valid=1 tx_target=B21 entry_s=0 release_s=2.5 vehicles={V0:TO_B,s=1.61757/2.11648,reason=clear;V1:TO_A1,s=0/1.6805,reason=clear;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=126.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=1 slot=4->-1 pending_B=21 s=0.002/1.680 rem=1.678 speed=0.020 wait=50.40 dwell=0.00
[multi_patrol][state] sim_t=126.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=22.70 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V-1 blocker=V-1 queue=[V1,V2] tx_valid=1 tx_target=B21 entry_s=0 release_s=2.5 vehicles={V0:TO_B,s=1.63182/2.11648,reason=clear;V1:TO_A1,s=0.002/1.6805,reason=action_hold;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=126.60s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=1 slot=4->-1 pending_B=21 s=0.006/1.680 rem=1.674 speed=0.040 wait=50.50 dwell=0.00
[multi_patrol][state] sim_t=127.00s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=1 slot=4->-1 pending_B=21 s=0.028/1.680 rem=1.652 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=128.60s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=24.80 dwell=0.00
[multi_patrol][state] sim_t=129.50s V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=1 slot=7->7 pending_B=-1 s=2.116/2.116 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V-1 blocker=V-1 queue=[V1,V2] tx_valid=1 tx_target=B21 entry_s=0 release_s=2.5 vehicles={V0:UNLOAD_DWELL,s=2.11648/2.11648,reason=not_active;V1:TO_A1,s=0.408743/1.6805,reason=clear;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=129.60s V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=1 slot=7->7 pending_B=-1 s=2.116/2.116 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[multi_patrol][state] sim_t=130.60s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=26.80 dwell=0.00
[multi_patrol][state] sim_t=132.60s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=28.80 dwell=0.00
[multi_patrol][state] sim_t=134.60s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=30.80 dwell=0.00
[multi_patrol][state] sim_t=136.70s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=32.90 dwell=0.00
[multi_patrol][state] sim_t=137.50s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=1 slot=4->-1 pending_B=21 s=1.680/1.680 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V1 candidate=V-1 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B21 entry_s=0 release_s=2.5 vehicles={V0:UNLOAD_DWELL,s=2.11648/2.11648,reason=not_active;V1:PICKUP_DWELL,s=1.6805/1.6805,reason=not_active;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=137.60s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=1 slot=4->-1 pending_B=21 s=1.680/1.680 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=138.80s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=35.00 dwell=0.00
[A1_STATE] state=LOADING owner=V1 candidate=V-1 blocker=V-1 queue=[V2,V0] tx_valid=1 tx_target=B21 entry_s=0 release_s=2.5 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V1;V1:PICKUP_DWELL,s=1.6805/1.6805,reason=not_active;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=139.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=140.80s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=37.00 dwell=0.00
[multi_patrol][state] sim_t=141.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=2.30 dwell=0.00
[A1_STATE] state=EXITING owner=V1 candidate=V-1 blocker=V-1 queue=[V2,V0] tx_valid=1 tx_target=B21 entry_s=0 release_s=2.5 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V1;V1:TO_B,s=0/6.35838,reason=clear;V2:TO_A1,s=0/1.6805,reason=wait_a1_turn_V1}
[multi_patrol][state] sim_t=142.50s V1 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=1 slot=4->21 pending_B=-1 s=0.002/6.358 rem=6.356 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=142.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=39.10 dwell=0.00
[multi_patrol][state] sim_t=143.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=4.30 dwell=0.00
[multi_patrol][state] sim_t=145.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=41.20 dwell=0.00
[multi_patrol][state] sim_t=145.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=6.40 dwell=0.00
[multi_patrol][state] sim_t=147.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=43.30 dwell=0.00
[multi_patrol][state] sim_t=147.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=8.40 dwell=0.00
[multi_patrol][state] sim_t=149.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=45.40 dwell=0.00
[multi_patrol][state] sim_t=149.90s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=10.50 dwell=0.00
[multi_patrol][state] sim_t=151.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=47.40 dwell=0.00
[multi_patrol][state] sim_t=151.90s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=12.50 dwell=0.00
[multi_patrol][state] sim_t=153.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=49.50 dwell=0.00
[multi_patrol][state] sim_t=154.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=14.60 dwell=0.00
[multi_patrol][state] sim_t=155.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=51.50 dwell=0.00
[multi_patrol][state] sim_t=156.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=16.60 dwell=0.00
[multi_patrol][state] sim_t=157.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=1 slot=5->-1 pending_B=33 s=0.000/1.680 rem=1.680 speed=0.000 wait=53.60 dwell=0.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V2 blocker=V-1 queue=[V2,V0] tx_valid=1 tx_target=B33 entry_s=0 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V2;V1:TO_B,s=2.50266/6.35838,reason=clear;V2:TO_A1,s=0/1.6805,reason=clear}
[multi_patrol][state] sim_t=157.60s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=18.20 dwell=0.00
[multi_patrol][state] sim_t=157.60s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=1 slot=5->-1 pending_B=33 s=0.002/1.680 rem=1.678 speed=0.020 wait=53.80 dwell=0.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V-1 blocker=V-1 queue=[V2,V0] tx_valid=1 tx_target=B33 entry_s=0 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V2;V1:TO_B,s=2.52266/6.35838,reason=clear;V2:TO_A1,s=0.002/1.6805,reason=action_hold}
[multi_patrol][state] sim_t=157.70s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=1 slot=5->-1 pending_B=33 s=0.006/1.680 rem=1.674 speed=0.040 wait=53.90 dwell=0.00
[multi_patrol][state] sim_t=158.10s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=1 slot=5->-1 pending_B=33 s=0.028/1.680 rem=1.652 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=159.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=20.30 dwell=0.00
[multi_patrol][state] sim_t=161.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=22.30 dwell=0.00
[multi_patrol][state] sim_t=163.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=24.40 dwell=0.00
[multi_patrol][state] sim_t=165.90s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=26.50 dwell=0.00
[multi_patrol][state] sim_t=167.90s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=28.50 dwell=0.00
[multi_patrol][state] sim_t=168.60s V2 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=1 slot=5->-1 pending_B=33 s=1.680/1.680 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V2 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B33 entry_s=0 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V2;V1:TO_B,s=4.38588/6.35838,reason=clear;V2:PICKUP_DWELL,s=1.6805/1.6805,reason=not_active}
[multi_patrol][state] sim_t=168.70s V2 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=1 slot=5->-1 pending_B=33 s=1.680/1.680 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=170.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=30.60 dwell=0.00
[multi_patrol][state] sim_t=172.10s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=32.70 dwell=0.00
[A1_STATE] state=EXITING owner=V2 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B33 entry_s=0 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V2;V1:TO_B,s=5.21672/6.35838,reason=clear;V2:TO_B,s=0/7.11056,reason=clear}
[multi_patrol][state] sim_t=173.60s V2 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=1 slot=5->33 pending_B=-1 s=0.002/7.111 rem=7.109 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=174.20s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=34.80 dwell=0.00
[multi_patrol][state] sim_t=176.20s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=36.80 dwell=0.00
[multi_patrol][state] sim_t=178.30s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=38.90 dwell=0.00
[multi_patrol][state] sim_t=180.10s V1 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=1 slot=21->21 pending_B=-1 s=6.358/6.358 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=EXITING owner=V2 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B33 entry_s=0 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V2;V1:UNLOAD_DWELL,s=6.35838/6.35838,reason=not_active;V2:TO_B,s=0.987751/7.11056,reason=clear}
[multi_patrol][state] sim_t=180.20s V1 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=1 slot=21->21 pending_B=-1 s=6.358/6.358 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[multi_patrol][state] sim_t=180.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=41.00 dwell=0.00
[multi_patrol][state] sim_t=182.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=43.00 dwell=0.00
[multi_patrol][state] sim_t=184.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=45.00 dwell=0.00
[multi_patrol][state] sim_t=186.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=2 slot=7->-1 pending_B=56 s=0.000/2.116 rem=2.116 speed=0.000 wait=47.10 dwell=0.00
[A1_STATE] state=ADMITTED owner=V0 candidate=V0 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B56 entry_s=0 release_s=2.5 vehicles={V0:TO_A1,s=0/2.1158,reason=clear;V1:UNLOAD_DWELL,s=6.35838/6.35838,reason=not_active;V2:TO_B,s=2.30288/7.11056,reason=clear}
[multi_patrol][state] sim_t=187.60s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=7->-1 pending_B=56 s=0.002/2.116 rem=2.114 speed=0.020 wait=48.20 dwell=0.00
[A1_STATE] state=ADMITTED owner=V0 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B56 entry_s=0 release_s=2.5 vehicles={V0:TO_A1,s=0.002/2.1158,reason=action_hold;V1:UNLOAD_DWELL,s=6.35838/6.35838,reason=not_active;V2:TO_B,s=2.32288/7.11056,reason=clear}
[multi_patrol][state] sim_t=187.70s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=7->-1 pending_B=56 s=0.006/2.116 rem=2.110 speed=0.040 wait=48.30 dwell=0.00
[multi_patrol][state] sim_t=188.10s V0 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=2 slot=7->-1 pending_B=56 s=0.028/2.116 rem=2.088 speed=0.070 wait=0.00 dwell=0.00
[A1_STATE] state=ADMITTED owner=V0 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B56 entry_s=0 release_s=2.5 vehicles={V0:TO_A1,s=0.323261/2.1158,reason=clear;V1:TO_A1,s=0/6.10198,reason=clear;V2:TO_B,s=2.6753/7.11056,reason=clear}
[multi_patrol][state] sim_t=190.10s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.002/6.102 rem=6.100 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=193.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=2 slot=21->-1 pending_B=25 s=0.541/6.102 rem=5.561 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=194.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.549/6.102 rem=5.553 speed=0.082 wait=0.20 dwell=0.00
[multi_patrol][state] sim_t=194.40s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.558/6.102 rem=5.544 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=194.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.562/6.102 rem=5.540 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=194.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=2 slot=21->-1 pending_B=25 s=0.569/6.102 rem=5.533 speed=0.020 wait=0.90 dwell=0.00
[multi_patrol][state] sim_t=194.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.569/6.102 rem=5.533 speed=0.000 wait=1.00 dwell=0.00
[multi_patrol][state] sim_t=195.20s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.571/6.102 rem=5.531 speed=0.020 wait=1.40 dwell=0.00
[multi_patrol][state] sim_t=195.30s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.575/6.102 rem=5.527 speed=0.040 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=195.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=2 slot=21->-1 pending_B=25 s=0.576/6.102 rem=5.526 speed=0.010 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=197.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=2 slot=21->-1 pending_B=25 s=0.576/6.102 rem=5.526 speed=0.000 wait=3.60 dwell=0.00
[multi_patrol][state] sim_t=199.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=2 slot=21->-1 pending_B=25 s=0.576/6.102 rem=5.526 speed=0.000 wait=5.60 dwell=0.00
[multi_patrol][state] sim_t=200.80s V0 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=2 slot=7->-1 pending_B=56 s=2.116/2.116 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V0 candidate=V-1 blocker=V-1 queue=[] tx_valid=1 tx_target=B56 entry_s=0 release_s=2.5 vehicles={V0:PICKUP_DWELL,s=2.1158/2.1158,reason=not_active;V1:TO_A1,s=0.576307/6.10198,reason=brake_V2;V2:TO_B,s=4.54379/7.11056,reason=clear}
[multi_patrol][state] sim_t=200.90s V0 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=2 slot=7->-1 pending_B=56 s=2.116/2.116 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=201.50s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=2 slot=21->-1 pending_B=25 s=0.576/6.102 rem=5.526 speed=0.000 wait=7.70 dwell=0.00
[multi_patrol][state] sim_t=203.60s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=2 slot=21->-1 pending_B=25 s=0.576/6.102 rem=5.526 speed=0.000 wait=9.80 dwell=0.00
[multi_patrol][state] sim_t=205.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=2 slot=21->-1 pending_B=25 s=0.576/6.102 rem=5.526 speed=0.000 wait=11.90 dwell=0.00
[A1_STATE] state=EXITING owner=V0 candidate=V-1 blocker=V-1 queue=[] tx_valid=1 tx_target=B56 entry_s=0 release_s=2.5 vehicles={V0:TO_B,s=0/7.36119,reason=clear;V1:TO_A1,s=0.576307/6.10198,reason=brake_V2;V2:TO_B,s=5.41591/7.11056,reason=clear}
[multi_patrol][state] sim_t=205.80s V0 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.002/7.361 rem=7.359 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=207.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=2 slot=21->-1 pending_B=25 s=0.576/6.102 rem=5.526 speed=0.000 wait=14.00 dwell=0.00
[multi_patrol][state] sim_t=209.70s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.578/6.102 rem=5.524 speed=0.020 wait=15.90 dwell=0.00
[multi_patrol][state] sim_t=209.80s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.582/6.102 rem=5.520 speed=0.040 wait=16.00 dwell=0.00
[multi_patrol][state] sim_t=210.20s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=0.604/6.102 rem=5.498 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=215.10s V2 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=1 slot=33->33 pending_B=-1 s=7.111/7.111 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=EXITING owner=V0 candidate=V-1 blocker=V-1 queue=[] tx_valid=1 tx_target=B56 entry_s=0 release_s=2.5 vehicles={V0:TO_B,s=1.54775/7.36119,reason=clear;V1:TO_A1,s=1.54035/6.10198,reason=clear;V2:UNLOAD_DWELL,s=7.11056/7.11056,reason=not_active}
[multi_patrol][state] sim_t=215.20s V2 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=1 slot=33->33 pending_B=-1 s=7.111/7.111 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[multi_patrol][state] sim_t=215.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=2 slot=21->-1 pending_B=25 s=1.697/6.102 rem=4.405 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=216.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.730/6.102 rem=4.372 speed=0.080 wait=0.40 dwell=0.00
[multi_patrol][state] sim_t=216.40s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.740/6.102 rem=4.362 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=216.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.745/6.102 rem=4.357 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=216.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=2 slot=21->-1 pending_B=25 s=1.762/6.102 rem=4.340 speed=0.020 wait=1.10 dwell=0.00
[multi_patrol][state] sim_t=217.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.762/6.102 rem=4.340 speed=0.000 wait=1.20 dwell=0.00
[multi_patrol][state] sim_t=217.40s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.764/6.102 rem=4.338 speed=0.020 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=217.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.768/6.102 rem=4.334 speed=0.040 wait=1.70 dwell=0.00
[multi_patrol][state] sim_t=217.60s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=2 slot=21->-1 pending_B=25 s=1.769/6.102 rem=4.333 speed=0.010 wait=1.80 dwell=0.00
[multi_patrol][state] sim_t=217.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.769/6.102 rem=4.333 speed=0.000 wait=2.00 dwell=0.00
[multi_patrol][state] sim_t=218.10s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.771/6.102 rem=4.331 speed=0.020 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=218.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=2 slot=21->-1 pending_B=25 s=1.771/6.102 rem=4.331 speed=0.000 wait=2.40 dwell=0.00
[multi_patrol][state] sim_t=220.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V0 blocker=0 task=2 slot=21->-1 pending_B=25 s=1.771/6.102 rem=4.331 speed=0.000 wait=4.50 dwell=0.00
[A1_STATE] state=FREE owner=V-1 candidate=V-1 blocker=V-1 queue=[] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_B,s=2.50266/7.36119,reason=brake_V1;V1:TO_A1,s=1.77135/6.10198,reason=clear;V2:UNLOAD_DWELL,s=7.11056/7.11056,reason=not_active}
[multi_patrol][state] sim_t=220.90s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.520/7.361 rem=4.842 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=220.90s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.773/6.102 rem=4.329 speed=0.020 wait=5.10 dwell=0.00
[multi_patrol][state] sim_t=221.00s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.777/6.102 rem=4.325 speed=0.040 wait=5.20 dwell=0.00
[multi_patrol][state] sim_t=221.40s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=1.799/6.102 rem=4.303 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=223.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=225.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=4.20 dwell=0.00
[A1_STATE] state=FREE owner=V-1 candidate=V-1 blocker=V-1 queue=[] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_B,s=2.55966/7.36119,reason=brake_V1;V1:TO_A1,s=2.39738/6.10198,reason=clear;V2:TO_A1,s=0/5.44398,reason=clear}
[multi_patrol][state] sim_t=225.10s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.002/5.444 rem=5.442 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=225.80s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V1 blocker=1 task=2 slot=33->-1 pending_B=50 s=0.067/5.444 rem=5.377 speed=0.110 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=225.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.075/5.444 rem=5.369 speed=0.080 wait=0.20 dwell=0.00
[multi_patrol][state] sim_t=226.30s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.084/5.444 rem=5.360 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=226.40s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.088/5.444 rem=5.356 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=226.60s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=5.80 dwell=0.00
[multi_patrol][state] sim_t=226.80s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.110/5.444 rem=5.334 speed=0.070 wait=0.00 dwell=0.00
[A1_STATE] state=WAITING owner=V-1 candidate=V-1 blocker=V-1 queue=[V1] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_B,s=2.55966/7.36119,reason=brake_V1;V1:TO_A1,s=2.90921/6.10198,reason=clear;V2:TO_A1,s=0.276279/5.44398,reason=clear}
[A1_STATE] state=WAITING owner=V-1 candidate=V1 blocker=V0 queue=[V1] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_B,s=2.55966/7.36119,reason=brake_V1;V1:TO_A1,s=2.92921/6.10198,reason=clear;V2:TO_A1,s=0.290526/5.44398,reason=clear}
[multi_patrol][state] sim_t=228.40s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=7.60 dwell=0.00
[multi_patrol][state] sim_t=228.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.344/5.444 rem=5.099 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=228.70s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.358/5.444 rem=5.086 speed=0.052 wait=0.30 dwell=0.00
[multi_patrol][state] sim_t=229.00s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.362/5.444 rem=5.082 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=229.10s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.366/5.444 rem=5.078 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=229.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.373/5.444 rem=5.071 speed=0.020 wait=0.90 dwell=0.00
[multi_patrol][state] sim_t=229.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.373/5.444 rem=5.071 speed=0.000 wait=1.00 dwell=0.00
[multi_patrol][state] sim_t=229.80s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.375/5.444 rem=5.069 speed=0.020 wait=1.40 dwell=0.00
[multi_patrol][state] sim_t=229.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.375/5.444 rem=5.069 speed=0.000 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=230.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.375/5.444 rem=5.069 speed=0.000 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=230.20s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=9.40 dwell=0.00
[multi_patrol][state] sim_t=230.40s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.020 wait=2.00 dwell=0.00
[multi_patrol][state] sim_t=230.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=2.10 dwell=0.00
[multi_patrol][state] sim_t=232.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=11.20 dwell=0.00
[multi_patrol][state] sim_t=232.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=3.90 dwell=0.00
[multi_patrol][state] sim_t=234.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=13.20 dwell=0.00
[multi_patrol][state] sim_t=234.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=5.60 dwell=0.00
[multi_patrol][state] sim_t=236.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=15.20 dwell=0.00
[multi_patrol][state] sim_t=236.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=7.60 dwell=0.00
[multi_patrol][state] sim_t=236.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.295/6.102 rem=1.807 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=236.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.304/6.102 rem=1.798 speed=0.082 wait=0.20 dwell=0.00
[multi_patrol][state] sim_t=236.80s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.313/6.102 rem=1.789 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=236.90s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.317/6.102 rem=1.785 speed=0.040 wait=0.70 dwell=0.00
[coord_diag][cycle] tick=2373 sim_t=237.30 ring=V0->V1->V0 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V0 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V1 blk=1 brkr=0 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 spd=0.000 wait=16.5 gen=6
[coord_diag][vehicle]  V1 mode=1 phase=TO_A1 loaded=0 act=0 reason=wait_a1_admission_clear blk=0 brkr=0 task=2 slot=21->-1 pending_B=25 s=4.334/6.102 rem=1.768 spd=0.020 wait=1.1 gen=5
[coord_diag][a1_gate] owner=V1 waiter=V1 stop_s=4.395 xy=(0.480,3.122) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V0<->V1 a1_owner=V-1 reservation=V1 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.560/7.361 gear=F act=0 blk=V1 gen=6 pending_B=-1 pending_gen=3 | B phase=TO_A1 s=4.334/6.102 gear=F act=0 blk=V0 gen=5 pending_B=25 pending_gen=4
[coord_diag][envelope] A[2.650,5.250] committed=0 inside_real=1 | B[2.050,4.450] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.413,1.500) | A[2.650,5.250] stop=2.471 gap=-0.089 inside=1 t=[0.000,14.112] | B[2.050,4.450] stop=1.871 gap=-2.463 inside=1 t=[0.000,1.144] overlap=1
[multi_patrol][state] sim_t=237.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.334/6.102 rem=1.768 speed=0.020 wait=1.10 dwell=0.00
[multi_patrol][state] sim_t=237.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.334/6.102 rem=1.768 speed=0.000 wait=1.20 dwell=0.00
[multi_patrol][state] sim_t=237.80s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.336/6.102 rem=1.766 speed=0.020 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=237.90s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.340/6.102 rem=1.762 speed=0.040 wait=1.70 dwell=0.00
[coord_diag][cycle] tick=2380 sim_t=238.00 ring=V0->V1->V0 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V0 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V1 blk=1 brkr=0 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 spd=0.000 wait=17.2 gen=6
[coord_diag][vehicle]  V1 mode=1 phase=TO_A1 loaded=0 act=0 reason=wait_a1_admission_clear blk=0 brkr=0 task=2 slot=21->-1 pending_B=25 s=4.341/6.102 rem=1.761 spd=0.010 wait=1.8 gen=5
[coord_diag][a1_gate] owner=V1 waiter=V1 stop_s=4.395 xy=(0.480,3.122) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V0<->V1 a1_owner=V-1 reservation=V1 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.560/7.361 gear=F act=0 blk=V1 gen=6 pending_B=-1 pending_gen=3 | B phase=TO_A1 s=4.341/6.102 gear=F act=0 blk=V0 gen=5 pending_B=25 pending_gen=4
[coord_diag][envelope] A[2.650,5.250] committed=0 inside_real=1 | B[2.050,4.450] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.413,1.500) | A[2.650,5.250] stop=2.471 gap=-0.089 inside=1 t=[0.000,14.112] | B[2.050,4.450] stop=1.871 gap=-2.470 inside=1 t=[0.000,1.156] overlap=1
[multi_patrol][state] sim_t=238.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=17.20 dwell=0.00
[multi_patrol][state] sim_t=238.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.341/6.102 rem=1.761 speed=0.010 wait=1.80 dwell=0.00
[multi_patrol][state] sim_t=238.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=9.60 dwell=0.00
[multi_patrol][state] sim_t=238.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.341/6.102 rem=1.761 speed=0.000 wait=1.90 dwell=0.00
[multi_patrol][state] sim_t=238.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.343/6.102 rem=1.759 speed=0.020 wait=2.30 dwell=0.00
[coord_diag][cycle] tick=2386 sim_t=238.60 ring=V0->V1->V0 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V0 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V1 blk=1 brkr=0 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 spd=0.000 wait=17.8 gen=6
[coord_diag][vehicle]  V1 mode=1 phase=TO_A1 loaded=0 act=0 reason=wait_a1_admission_clear blk=0 brkr=0 task=2 slot=21->-1 pending_B=25 s=4.343/6.102 rem=1.759 spd=0.000 wait=2.4 gen=5
[coord_diag][a1_gate] owner=V1 waiter=V1 stop_s=4.395 xy=(0.480,3.122) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V0<->V1 a1_owner=V-1 reservation=V1 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.560/7.361 gear=F act=0 blk=V1 gen=6 pending_B=-1 pending_gen=3 | B phase=TO_A1 s=4.343/6.102 gear=F act=0 blk=V0 gen=5 pending_B=25 pending_gen=4
[coord_diag][envelope] A[2.650,5.250] committed=0 inside_real=1 | B[2.050,4.450] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.413,1.500) | A[2.650,5.250] stop=2.471 gap=-0.089 inside=1 t=[0.000,14.112] | B[2.050,4.450] stop=1.871 gap=-2.472 inside=1 t=[0.000,1.194] overlap=1
[multi_patrol][state] sim_t=238.60s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.343/6.102 rem=1.759 speed=0.000 wait=2.40 dwell=0.00
[multi_patrol][state] sim_t=238.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.343/6.102 rem=1.759 speed=0.000 wait=2.50 dwell=0.00
[multi_patrol][state] sim_t=239.10s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.020 wait=2.90 dwell=0.00
[coord_diag][cycle] tick=2392 sim_t=239.20 ring=V0->V1->V0 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V0 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V1 blk=1 brkr=0 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 spd=0.000 wait=18.4 gen=6
[coord_diag][vehicle]  V1 mode=1 phase=TO_A1 loaded=0 act=0 reason=wait_a1_admission_clear blk=0 brkr=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 spd=0.000 wait=3.0 gen=5
[coord_diag][a1_gate] owner=V1 waiter=V1 stop_s=4.395 xy=(0.480,3.122) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V0<->V1 a1_owner=V-1 reservation=V1 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.560/7.361 gear=F act=0 blk=V1 gen=6 pending_B=-1 pending_gen=3 | B phase=TO_A1 s=4.345/6.102 gear=F act=0 blk=V0 gen=5 pending_B=25 pending_gen=4
[coord_diag][envelope] A[2.650,5.250] committed=0 inside_real=1 | B[2.050,4.450] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.413,1.500) | A[2.650,5.250] stop=2.471 gap=-0.089 inside=1 t=[0.000,14.112] | B[2.050,4.450] stop=1.871 gap=-2.474 inside=1 t=[0.000,1.184] overlap=1
[multi_patrol][state] sim_t=239.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=3.00 dwell=0.00
[multi_patrol][state] sim_t=240.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=19.20 dwell=0.00
[multi_patrol][state] sim_t=240.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=11.60 dwell=0.00
[multi_patrol][state] sim_t=241.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=4.80 dwell=0.00
[multi_patrol][state] sim_t=242.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=21.20 dwell=0.00
[multi_patrol][state] sim_t=242.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=13.60 dwell=0.00
[multi_patrol][state] sim_t=242.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=6.60 dwell=0.00
[multi_patrol][state] sim_t=244.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=23.20 dwell=0.00
[multi_patrol][state] sim_t=244.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=15.60 dwell=0.00
[coord_diag][cycle] tick=2443 sim_t=244.30 ring=V0->V1->V0 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V0 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V1 blk=1 brkr=0 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 spd=0.000 wait=23.5 gen=6
[coord_diag][vehicle]  V1 mode=1 phase=TO_A1 loaded=0 act=0 reason=wait_a1_admission_clear blk=0 brkr=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 spd=0.000 wait=8.1 gen=5
[coord_diag][a1_gate] owner=V1 waiter=V1 stop_s=4.395 xy=(0.480,3.122) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V0<->V1 a1_owner=V-1 reservation=V1 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.560/7.361 gear=F act=0 blk=V1 gen=6 pending_B=-1 pending_gen=3 | B phase=TO_A1 s=4.345/6.102 gear=F act=0 blk=V0 gen=5 pending_B=25 pending_gen=4
[coord_diag][envelope] A[2.650,5.250] committed=0 inside_real=1 | B[2.050,4.450] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.413,1.500) | A[2.650,5.250] stop=2.471 gap=-0.089 inside=1 t=[0.000,14.112] | B[2.050,4.450] stop=1.871 gap=-2.474 inside=1 t=[0.000,1.184] overlap=1
[multi_patrol][state] sim_t=244.60s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=8.40 dwell=0.00
[multi_patrol][state] sim_t=246.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=25.20 dwell=0.00
[multi_patrol][state] sim_t=246.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=17.60 dwell=0.00
[multi_patrol][state] sim_t=246.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=10.20 dwell=0.00
[multi_patrol][state] sim_t=248.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=27.20 dwell=0.00
[multi_patrol][state] sim_t=248.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=19.60 dwell=0.00
[multi_patrol][state] sim_t=248.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=12.00 dwell=0.00
[coord_diag][cycle] tick=2494 sim_t=249.40 ring=V0->V1->V0 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V0 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V1 blk=1 brkr=0 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 spd=0.000 wait=28.6 gen=6
[coord_diag][vehicle]  V1 mode=1 phase=TO_A1 loaded=0 act=0 reason=wait_a1_admission_clear blk=0 brkr=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 spd=0.000 wait=13.2 gen=5
[coord_diag][a1_gate] owner=V1 waiter=V1 stop_s=4.395 xy=(0.480,3.122) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V0<->V1 a1_owner=V-1 reservation=V1 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.560/7.361 gear=F act=0 blk=V1 gen=6 pending_B=-1 pending_gen=3 | B phase=TO_A1 s=4.345/6.102 gear=F act=0 blk=V0 gen=5 pending_B=25 pending_gen=4
[coord_diag][envelope] A[2.650,5.250] committed=0 inside_real=1 | B[2.050,4.450] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.413,1.500) | A[2.650,5.250] stop=2.471 gap=-0.089 inside=1 t=[0.000,14.112] | B[2.050,4.450] stop=1.871 gap=-2.474 inside=1 t=[0.000,1.184] overlap=1
[multi_patrol][state] sim_t=250.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=29.20 dwell=0.00
[multi_patrol][state] sim_t=250.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=13.80 dwell=0.00
[multi_patrol][state] sim_t=250.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=21.60 dwell=0.00
[multi_patrol][state] sim_t=252.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=31.20 dwell=0.00
[multi_patrol][state] sim_t=252.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=15.80 dwell=0.00
[multi_patrol][state] sim_t=252.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=23.60 dwell=0.00
[multi_patrol][state] sim_t=254.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=33.20 dwell=0.00
[multi_patrol][state] sim_t=254.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=17.80 dwell=0.00
[multi_patrol][state] sim_t=254.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=25.60 dwell=0.00
[coord_diag][cycle] tick=2545 sim_t=254.50 ring=V0->V1->V0 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V0 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V1 blk=1 brkr=0 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 spd=0.000 wait=33.7 gen=6
[coord_diag][vehicle]  V1 mode=1 phase=TO_A1 loaded=0 act=0 reason=wait_a1_admission_clear blk=0 brkr=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 spd=0.000 wait=18.3 gen=5
[coord_diag][a1_gate] owner=V1 waiter=V1 stop_s=4.395 xy=(0.480,3.122) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V0<->V1 a1_owner=V-1 reservation=V1 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.560/7.361 gear=F act=0 blk=V1 gen=6 pending_B=-1 pending_gen=3 | B phase=TO_A1 s=4.345/6.102 gear=F act=0 blk=V0 gen=5 pending_B=25 pending_gen=4
[coord_diag][envelope] A[2.650,5.250] committed=0 inside_real=1 | B[2.050,4.450] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.413,1.500) | A[2.650,5.250] stop=2.471 gap=-0.089 inside=1 t=[0.000,14.112] | B[2.050,4.450] stop=1.871 gap=-2.474 inside=1 t=[0.000,1.184] overlap=1
[multi_patrol][state] sim_t=256.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=35.20 dwell=0.00
[multi_patrol][state] sim_t=256.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=19.80 dwell=0.00
[multi_patrol][state] sim_t=256.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=27.60 dwell=0.00
[multi_patrol][state] sim_t=258.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=37.20 dwell=0.00
[multi_patrol][state] sim_t=258.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=21.80 dwell=0.00
[multi_patrol][state] sim_t=258.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=29.60 dwell=0.00
[coord_diag][cycle] tick=2595 sim_t=259.50 ring=V0->V1->V0 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V0 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V1 blk=1 brkr=0 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 spd=0.000 wait=38.7 gen=6
[coord_diag][vehicle]  V1 mode=1 phase=TO_A1 loaded=0 act=0 reason=wait_a1_admission_clear blk=0 brkr=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 spd=0.000 wait=23.3 gen=5
[coord_diag][a1_gate] owner=V1 waiter=V1 stop_s=4.395 xy=(0.480,3.122) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V0<->V1 a1_owner=V-1 reservation=V1 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.560/7.361 gear=F act=0 blk=V1 gen=6 pending_B=-1 pending_gen=3 | B phase=TO_A1 s=4.345/6.102 gear=F act=0 blk=V0 gen=5 pending_B=25 pending_gen=4
[coord_diag][envelope] A[2.650,5.250] committed=0 inside_real=1 | B[2.050,4.450] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.413,1.500) | A[2.650,5.250] stop=2.471 gap=-0.089 inside=1 t=[0.000,14.112] | B[2.050,4.450] stop=1.871 gap=-2.474 inside=1 t=[0.000,1.184] overlap=1
[multi_patrol][state] sim_t=260.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V1 blocker=1 task=2 slot=7->56 pending_B=-1 s=2.560/7.361 rem=4.802 speed=0.000 wait=39.20 dwell=0.00
[multi_patrol][state] sim_t=260.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=23.80 dwell=0.00
[multi_patrol][state] sim_t=260.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V0 blocker=0 task=2 slot=33->-1 pending_B=50 s=0.377/5.444 rem=5.067 speed=0.000 wait=31.60 dwell=0.00
[multi_patrol][DEADLOCK] tick=2615 sim_t=261.500000 members=[V0(brake_V1->V1) V1(wait_a1_admission_clear->V0)]
[multi_patrol][state] sim_t=261.50s V0 mode=ACTIVE phase=TO_B action=NOMINAL reason=deadlock_replan blocker=1 task=2 slot=7->56 pending_B=-1 s=0.000/4.372 rem=4.372 speed=0.000 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=261.60s V0 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.002/4.372 rem=4.370 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=261.60s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.379/5.444 rem=5.065 speed=0.020 wait=33.20 dwell=0.00
[multi_patrol][state] sim_t=261.70s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.383/5.444 rem=5.061 speed=0.040 wait=33.30 dwell=0.00
[multi_patrol][state] sim_t=262.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=25.80 dwell=0.00
[multi_patrol][state] sim_t=262.10s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=0.405/5.444 rem=5.039 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=264.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=2 slot=21->-1 pending_B=25 s=4.345/6.102 rem=1.757 speed=0.000 wait=27.80 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V1 blocker=V-1 queue=[V1] tx_valid=1 tx_target=B4 entry_s=4.395 release_s=1.435 vehicles={V0:TO_B,s=0.517636/4.37175,reason=clear;V1:TO_A1,s=4.34513/6.10198,reason=clear;V2:TO_A1,s=0.840836/5.44398,reason=clear}
[multi_patrol][state] sim_t=265.00s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=21->-1 pending_B=4 s=4.347/6.102 rem=1.755 speed=0.020 wait=28.80 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V-1 blocker=V-1 queue=[V1] tx_valid=1 tx_target=B4 entry_s=4.395 release_s=1.435 vehicles={V0:TO_B,s=0.531882/4.37175,reason=clear;V1:TO_A1,s=4.34713/6.10198,reason=action_hold;V2:TO_A1,s=0.860836/5.44398,reason=clear}
[multi_patrol][state] sim_t=265.10s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=21->-1 pending_B=4 s=4.351/6.102 rem=1.751 speed=0.040 wait=28.90 dwell=0.00
[multi_patrol][state] sim_t=265.50s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=2 slot=21->-1 pending_B=4 s=4.373/6.102 rem=1.729 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=267.20s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=2 slot=7->56 pending_B=-1 s=0.895/4.372 rem=3.476 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=267.60s V0 mode=ACTIVE phase=TO_B action=STOP reason=action_hold blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.933/4.372 rem=3.438 speed=0.050 wait=0.50 dwell=0.00
[multi_patrol][state] sim_t=267.70s V0 mode=ACTIVE phase=TO_B action=CREEP reason=clear blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.938/4.372 rem=3.433 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=267.80s V0 mode=ACTIVE phase=TO_B action=CREEP reason=action_hold blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.943/4.372 rem=3.428 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=267.90s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=2 slot=7->56 pending_B=-1 s=0.945/4.372 rem=3.426 speed=0.020 wait=0.80 dwell=0.00
[multi_patrol][state] sim_t=268.00s V0 mode=ACTIVE phase=TO_B action=STOP reason=action_hold blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.945/4.372 rem=3.426 speed=0.000 wait=0.90 dwell=0.00
[multi_patrol][state] sim_t=268.40s V0 mode=ACTIVE phase=TO_B action=CREEP reason=clear blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.947/4.372 rem=3.424 speed=0.020 wait=1.30 dwell=0.00
[multi_patrol][state] sim_t=268.50s V0 mode=ACTIVE phase=TO_B action=CREEP reason=action_hold blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.951/4.372 rem=3.420 speed=0.040 wait=1.40 dwell=0.00
[multi_patrol][state] sim_t=268.60s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=2 slot=7->56 pending_B=-1 s=0.952/4.372 rem=3.419 speed=0.010 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=270.70s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=2 slot=7->56 pending_B=-1 s=0.952/4.372 rem=3.419 speed=0.000 wait=3.60 dwell=0.00
[multi_patrol][state] sim_t=272.70s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=2 slot=7->56 pending_B=-1 s=0.952/4.372 rem=3.419 speed=0.000 wait=5.60 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V-1 blocker=V-1 queue=[V1,V2] tx_valid=1 tx_target=B4 entry_s=4.395 release_s=1.435 vehicles={V0:TO_B,s=0.952367/4.37175,reason=brake_V2;V1:TO_A1,s=5.51422/6.10198,reason=clear;V2:TO_A1,s=2.24664/5.44398,reason=clear}
[multi_patrol][state] sim_t=274.70s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=2 slot=7->56 pending_B=-1 s=0.952/4.372 rem=3.419 speed=0.000 wait=7.60 dwell=0.00
[multi_patrol][state] sim_t=276.60s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=2 slot=21->-1 pending_B=4 s=6.102/6.102 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V1 candidate=V-1 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B4 entry_s=4.395 release_s=1.435 vehicles={V0:TO_B,s=0.952367/4.37175,reason=brake_V2;V1:PICKUP_DWELL,s=6.10198/6.10198,reason=not_active;V2:TO_A1,s=2.93826/5.44398,reason=clear}
[multi_patrol][state] sim_t=276.70s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=2 slot=21->-1 pending_B=4 s=6.102/6.102 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=276.80s V0 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=2 slot=7->56 pending_B=-1 s=0.952/4.372 rem=3.419 speed=0.000 wait=9.70 dwell=0.00
[multi_patrol][state] sim_t=277.80s V0 mode=ACTIVE phase=TO_B action=CREEP reason=clear blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.954/4.372 rem=3.417 speed=0.020 wait=10.70 dwell=0.00
[multi_patrol][state] sim_t=277.90s V0 mode=ACTIVE phase=TO_B action=CREEP reason=action_hold blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.958/4.372 rem=3.413 speed=0.040 wait=10.80 dwell=0.00
[multi_patrol][state] sim_t=278.30s V0 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=2 slot=7->56 pending_B=-1 s=0.980/4.372 rem=3.391 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=281.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=33->-1 pending_B=50 s=3.648/5.444 rem=1.796 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=281.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.662/5.444 rem=1.782 speed=0.052 wait=0.30 dwell=0.00
[A1_STATE] state=EXITING owner=V1 candidate=V-1 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B4 entry_s=4.395 release_s=1.435 vehicles={V0:TO_B,s=1.49484/4.37175,reason=clear;V1:TO_B,s=0/1.68118,reason=clear;V2:TO_A1,s=3.6641/5.44398,reason=action_hold}
[multi_patrol][state] sim_t=281.60s V1 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=2 slot=21->4 pending_B=-1 s=0.002/1.681 rem=1.679 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=281.70s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.666/5.444 rem=1.778 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=281.80s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.670/5.444 rem=1.774 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=282.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=33->-1 pending_B=50 s=3.682/5.444 rem=1.762 speed=0.020 wait=1.00 dwell=0.00
[multi_patrol][state] sim_t=282.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.682/5.444 rem=1.762 speed=0.000 wait=1.10 dwell=0.00
[multi_patrol][state] sim_t=282.60s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.684/5.444 rem=1.760 speed=0.020 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=282.70s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.688/5.444 rem=1.756 speed=0.040 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=282.80s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=33->-1 pending_B=50 s=3.689/5.444 rem=1.755 speed=0.010 wait=1.70 dwell=0.00
[multi_patrol][state] sim_t=283.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.689/5.444 rem=1.755 speed=0.000 wait=1.90 dwell=0.00
[multi_patrol][state] sim_t=283.30s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.691/5.444 rem=1.753 speed=0.020 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=283.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=33->-1 pending_B=50 s=3.691/5.444 rem=1.753 speed=0.000 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=285.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=33->-1 pending_B=50 s=3.691/5.444 rem=1.753 speed=0.000 wait=4.40 dwell=0.00
[multi_patrol][state] sim_t=287.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=33->-1 pending_B=50 s=3.691/5.444 rem=1.753 speed=0.000 wait=6.40 dwell=0.00
[multi_patrol][state] sim_t=289.60s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=2 slot=33->-1 pending_B=50 s=3.691/5.444 rem=1.753 speed=0.000 wait=8.50 dwell=0.00
[A1_STATE] state=WAITING owner=V-1 candidate=V2 blocker=V1 queue=[V2] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_B,s=3.17785/4.37175,reason=clear;V1:TO_B,s=1.44131/1.68118,reason=clear;V2:TO_A1,s=3.6911/5.44398,reason=wait_a1_admission_clear}
[multi_patrol][state] sim_t=291.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=2 slot=33->-1 pending_B=50 s=3.691/5.444 rem=1.753 speed=0.000 wait=10.10 dwell=0.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V2 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B50 entry_s=3.74 release_s=2.29 vehicles={V0:TO_B,s=3.29725/4.37175,reason=clear;V1:TO_B,s=1.55043/1.68118,reason=clear;V2:TO_A1,s=3.6911/5.44398,reason=clear}
[multi_patrol][state] sim_t=291.80s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.693/5.444 rem=1.751 speed=0.020 wait=10.70 dwell=0.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V-1 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B50 entry_s=3.74 release_s=2.29 vehicles={V0:TO_B,s=3.31725/4.37175,reason=clear;V1:TO_B,s=1.57043/1.68118,reason=clear;V2:TO_A1,s=3.6931/5.44398,reason=action_hold}
[multi_patrol][state] sim_t=291.90s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.697/5.444 rem=1.747 speed=0.040 wait=10.80 dwell=0.00
[multi_patrol][state] sim_t=292.30s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=2 slot=33->-1 pending_B=50 s=3.719/5.444 rem=1.725 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=292.40s V1 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=2 slot=4->4 pending_B=-1 s=1.681/1.681 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V-1 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B50 entry_s=3.74 release_s=2.29 vehicles={V0:TO_B,s=3.43725/4.37175,reason=clear;V1:UNLOAD_DWELL,s=1.68118/1.68118,reason=not_active;V2:TO_A1,s=3.7281/5.44398,reason=clear}
[multi_patrol][state] sim_t=292.50s V1 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=2 slot=4->4 pending_B=-1 s=1.681/1.681 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[multi_patrol][state] sim_t=297.70s V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=2 slot=56->56 pending_B=-1 s=4.372/4.372 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V-1 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B50 entry_s=3.74 release_s=2.29 vehicles={V0:UNLOAD_DWELL,s=4.37175/4.37175,reason=not_active;V1:UNLOAD_DWELL,s=1.68118/1.68118,reason=not_active;V2:TO_A1,s=4.53679/5.44398,reason=clear}
[multi_patrol][state] sim_t=297.80s V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=2 slot=56->56 pending_B=-1 s=4.372/4.372 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[A1_STATE] state=ADMITTED owner=V2 candidate=V-1 blocker=V-1 queue=[V2,V1] tx_valid=1 tx_target=B50 entry_s=3.74 release_s=2.29 vehicles={V0:UNLOAD_DWELL,s=4.37175/4.37175,reason=not_active;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V2;V2:TO_A1,s=5.24931/5.44398,reason=clear}
[multi_patrol][state] sim_t=302.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=303.30s V2 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=2 slot=33->-1 pending_B=50 s=5.444/5.444 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V2 candidate=V-1 blocker=V-1 queue=[V1] tx_valid=1 tx_target=B50 entry_s=3.74 release_s=2.29 vehicles={V0:UNLOAD_DWELL,s=4.37175/4.37175,reason=not_active;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V2;V2:PICKUP_DWELL,s=5.44398/5.44398,reason=not_active}
[multi_patrol][state] sim_t=303.40s V2 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=2 slot=33->-1 pending_B=50 s=5.444/5.444 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=304.50s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=306.50s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=4.20 dwell=0.00
[A1_STATE] state=LOADING owner=V2 candidate=V-1 blocker=V-1 queue=[V1] tx_valid=1 tx_target=B50 entry_s=3.74 release_s=2.29 vehicles={V0:TO_A1,s=0/6.94457,reason=clear;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V2;V2:PICKUP_DWELL,s=5.44398/5.44398,reason=not_active}
[multi_patrol][state] sim_t=307.70s V0 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=3 slot=56->-1 pending_B=2 s=0.002/6.945 rem=6.943 speed=0.020 wait=0.00 dwell=0.00
[A1_STATE] state=EXITING owner=V2 candidate=V-1 blocker=V-1 queue=[V1] tx_valid=1 tx_target=B50 entry_s=3.74 release_s=2.29 vehicles={V0:TO_A1,s=0.042/6.94457,reason=clear;V1:TO_A1,s=0/1.6805,reason=wait_a1_turn_V2;V2:TO_B,s=0/6.99329,reason=clear}
[multi_patrol][state] sim_t=308.30s V2 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=2 slot=33->50 pending_B=-1 s=0.002/6.993 rem=6.991 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=308.50s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=6.20 dwell=0.00
[multi_patrol][state] sim_t=310.50s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=8.20 dwell=0.00
[multi_patrol][state] sim_t=312.60s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=10.30 dwell=0.00
[multi_patrol][state] sim_t=314.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=12.40 dwell=0.00
[multi_patrol][state] sim_t=316.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=14.50 dwell=0.00
[multi_patrol][state] sim_t=318.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=16.50 dwell=0.00
[multi_patrol][state] sim_t=320.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=3 slot=4->-1 pending_B=17 s=0.000/1.680 rem=1.680 speed=0.000 wait=18.60 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V1 blocker=V-1 queue=[V1] tx_valid=1 tx_target=B17 entry_s=0 release_s=2.48 vehicles={V0:TO_A1,s=2.48954/6.94457,reason=clear;V1:TO_A1,s=0/1.6805,reason=clear;V2:TO_B,s=2.30288/6.99329,reason=clear}
[multi_patrol][state] sim_t=322.30s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=3 slot=4->-1 pending_B=17 s=0.002/1.680 rem=1.678 speed=0.020 wait=20.00 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V-1 blocker=V-1 queue=[V1] tx_valid=1 tx_target=B17 entry_s=0 release_s=2.48 vehicles={V0:TO_A1,s=2.50379/6.94457,reason=clear;V1:TO_A1,s=0.002/1.6805,reason=action_hold;V2:TO_B,s=2.32288/6.99329,reason=clear}
[multi_patrol][state] sim_t=322.40s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=3 slot=4->-1 pending_B=17 s=0.006/1.680 rem=1.674 speed=0.040 wait=20.10 dwell=0.00
[multi_patrol][state] sim_t=322.80s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=3 slot=4->-1 pending_B=17 s=0.028/1.680 rem=1.652 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=324.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.868/6.945 rem=4.077 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=324.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.901/6.945 rem=4.044 speed=0.080 wait=0.40 dwell=0.00
[multi_patrol][state] sim_t=324.90s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.911/6.945 rem=4.034 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=325.00s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.916/6.945 rem=4.029 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=325.20s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.923/6.945 rem=4.022 speed=0.020 wait=0.90 dwell=0.00
[multi_patrol][state] sim_t=325.30s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.923/6.945 rem=4.022 speed=0.000 wait=1.00 dwell=0.00
[multi_patrol][state] sim_t=325.70s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.925/6.945 rem=4.020 speed=0.020 wait=1.40 dwell=0.00
[multi_patrol][state] sim_t=325.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.925/6.945 rem=4.020 speed=0.000 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=325.90s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.925/6.945 rem=4.020 speed=0.000 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=326.30s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.927/6.945 rem=4.018 speed=0.020 wait=2.00 dwell=0.00
[multi_patrol][state] sim_t=326.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.927/6.945 rem=4.018 speed=0.000 wait=2.10 dwell=0.00
[multi_patrol][state] sim_t=328.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.927/6.945 rem=4.018 speed=0.000 wait=4.20 dwell=0.00
[multi_patrol][state] sim_t=330.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.927/6.945 rem=4.018 speed=0.000 wait=6.20 dwell=0.00
[multi_patrol][state] sim_t=332.60s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.927/6.945 rem=4.018 speed=0.000 wait=8.30 dwell=0.00
[multi_patrol][state] sim_t=333.30s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=3 slot=4->-1 pending_B=17 s=1.680/1.680 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V1 candidate=V-1 blocker=V-1 queue=[] tx_valid=1 tx_target=B17 entry_s=0 release_s=2.48 vehicles={V0:TO_A1,s=2.92699/6.94457,reason=brake_V2;V1:PICKUP_DWELL,s=1.6805/1.6805,reason=not_active;V2:TO_B,s=4.1496/6.99329,reason=clear}
[multi_patrol][state] sim_t=333.40s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=3 slot=4->-1 pending_B=17 s=1.680/1.680 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=334.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.927/6.945 rem=4.018 speed=0.000 wait=10.40 dwell=0.00
[multi_patrol][state] sim_t=336.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.927/6.945 rem=4.018 speed=0.000 wait=12.50 dwell=0.00
[A1_STATE] state=EXITING owner=V1 candidate=V-1 blocker=V-1 queue=[] tx_valid=1 tx_target=B17 entry_s=0 release_s=2.48 vehicles={V0:TO_A1,s=2.92699/6.94457,reason=brake_V2;V1:TO_B,s=0/4.5331,reason=clear;V2:TO_B,s=4.97591/6.99329,reason=clear}
[multi_patrol][state] sim_t=338.30s V1 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=3 slot=4->17 pending_B=-1 s=0.002/4.533 rem=4.531 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=338.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.927/6.945 rem=4.018 speed=0.000 wait=14.50 dwell=0.00
[multi_patrol][state] sim_t=340.90s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=3 slot=56->-1 pending_B=2 s=2.927/6.945 rem=4.018 speed=0.000 wait=16.60 dwell=0.00
[multi_patrol][state] sim_t=341.40s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.929/6.945 rem=4.016 speed=0.020 wait=17.10 dwell=0.00
[multi_patrol][state] sim_t=341.50s V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.933/6.945 rem=4.012 speed=0.040 wait=17.20 dwell=0.00
[multi_patrol][state] sim_t=341.90s V0 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=3 slot=56->-1 pending_B=2 s=2.955/6.945 rem=3.990 speed=0.070 wait=0.00 dwell=0.00
[A1_STATE] state=EXITING owner=V1 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B17 entry_s=0 release_s=2.48 vehicles={V0:TO_A1,s=3.75899/6.94457,reason=clear;V1:TO_B,s=1.24775/4.5331,reason=clear;V2:TO_B,s=6.49759/6.99329,reason=clear}
[multi_patrol][state] sim_t=349.30s V2 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=2 slot=50->50 pending_B=-1 s=6.993/6.993 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=EXITING owner=V1 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B17 entry_s=0 release_s=2.48 vehicles={V0:TO_A1,s=4.36186/6.94457,reason=clear;V1:TO_B,s=1.88615/4.5331,reason=clear;V2:UNLOAD_DWELL,s=6.99329/6.99329,reason=not_active}
[multi_patrol][state] sim_t=349.40s V2 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=2 slot=50->50 pending_B=-1 s=6.993/6.993 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[A1_STATE] state=ADMITTED owner=V0 candidate=V0 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B2 entry_s=5.24 release_s=1.245 vehicles={V0:TO_A1,s=4.96215/6.94457,reason=clear;V1:TO_B,s=2.49605/4.5331,reason=clear;V2:UNLOAD_DWELL,s=6.99329/6.99329,reason=not_active}
[A1_STATE] state=ADMITTED owner=V0 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B2 entry_s=5.24 release_s=1.245 vehicles={V0:TO_A1,s=4.9764/6.94457,reason=clear;V1:TO_B,s=2.51605/4.5331,reason=clear;V2:UNLOAD_DWELL,s=6.99329/6.99329,reason=not_active}
[A1_STATE] state=ADMITTED owner=V0 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B2 entry_s=5.24 release_s=1.245 vehicles={V0:TO_A1,s=5.88822/6.94457,reason=clear;V1:TO_B,s=3.56458/4.5331,reason=clear;V2:TO_A1,s=0/5.06576,reason=clear}
[multi_patrol][state] sim_t=359.30s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=3 slot=50->-1 pending_B=34 s=0.002/5.066 rem=5.064 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=364.90s V1 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=3 slot=17->17 pending_B=-1 s=4.533/4.533 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=ADMITTED owner=V0 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B2 entry_s=5.24 release_s=1.245 vehicles={V0:TO_A1,s=6.75893/6.94457,reason=clear;V1:UNLOAD_DWELL,s=4.5331/4.5331,reason=not_active;V2:TO_A1,s=0.848801/5.06576,reason=clear}
[multi_patrol][state] sim_t=365.00s V1 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=3 slot=17->17 pending_B=-1 s=4.533/4.533 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[multi_patrol][state] sim_t=365.90s V0 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=3 slot=56->-1 pending_B=2 s=6.945/6.945 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V0 candidate=V-1 blocker=V-1 queue=[] tx_valid=1 tx_target=B2 entry_s=5.24 release_s=1.245 vehicles={V0:PICKUP_DWELL,s=6.94457/6.94457,reason=not_active;V1:UNLOAD_DWELL,s=4.5331/4.5331,reason=not_active;V2:TO_A1,s=1.0488/5.06576,reason=clear}
[multi_patrol][state] sim_t=366.00s V0 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=3 slot=56->-1 pending_B=2 s=6.945/6.945 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[A1_STATE] state=EXITING owner=V0 candidate=V-1 blocker=V-1 queue=[] tx_valid=1 tx_target=B2 entry_s=5.24 release_s=1.245 vehicles={V0:TO_B,s=0/2.11648,reason=clear;V1:UNLOAD_DWELL,s=4.5331/4.5331,reason=not_active;V2:TO_A1,s=1.93115/5.06576,reason=clear}
[multi_patrol][state] sim_t=370.90s V0 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=3 slot=56->2 pending_B=-1 s=0.002/2.116 rem=2.114 speed=0.020 wait=0.00 dwell=0.00
[A1_STATE] state=EXITING owner=V0 candidate=V-1 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B2 entry_s=5.24 release_s=1.245 vehicles={V0:TO_B,s=0.03/2.11648,reason=clear;V1:UNLOAD_DWELL,s=4.5331/4.5331,reason=not_active;V2:TO_A1,s=2.00237/5.06576,reason=clear}
[A1_STATE] state=EXITING owner=V0 candidate=V-1 blocker=V-1 queue=[V2] tx_valid=1 tx_target=B2 entry_s=5.24 release_s=1.245 vehicles={V0:TO_B,s=0.563529/2.11648,reason=clear;V1:TO_A1,s=0/3.4202,reason=brake_V2;V2:TO_A1,s=2.6547/5.06576,reason=clear}
[multi_patrol][state] sim_t=374.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=0.000/3.420 rem=3.420 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=377.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=0.000/3.420 rem=3.420 speed=0.000 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=378.60s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=0.002/3.420 rem=3.418 speed=0.020 wait=3.80 dwell=0.00
[multi_patrol][state] sim_t=378.70s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=0.006/3.420 rem=3.414 speed=0.040 wait=3.90 dwell=0.00
[A1_STATE] state=WAITING owner=V-1 candidate=V2 blocker=V0 queue=[V2] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_B,s=1.26441/2.11648,reason=clear;V1:TO_A1,s=0.011/3.4202,reason=action_hold;V2:TO_A1,s=3.21369/5.06576,reason=clear}
[multi_patrol][state] sim_t=379.10s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=0.028/3.420 rem=3.392 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=380.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=3 slot=50->-1 pending_B=34 s=3.404/5.066 rem=1.662 speed=0.099 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=380.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.411/5.066 rem=1.655 speed=0.069 wait=0.20 dwell=0.00
[multi_patrol][state] sim_t=380.80s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.418/5.066 rem=1.648 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=380.90s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.422/5.066 rem=1.644 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=381.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=3 slot=50->-1 pending_B=34 s=3.434/5.066 rem=1.632 speed=0.020 wait=1.00 dwell=0.00
[multi_patrol][state] sim_t=381.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.434/5.066 rem=1.632 speed=0.000 wait=1.10 dwell=0.00
[A1_STATE] state=WAITING owner=V-1 candidate=V2 blocker=V0 queue=[V2,V1] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_B,s=1.70305/2.11648,reason=clear;V1:TO_A1,s=0.374518/3.4202,reason=clear;V2:TO_A1,s=3.43364/5.06576,reason=action_hold}
[multi_patrol][state] sim_t=381.70s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.436/5.066 rem=1.630 speed=0.020 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=381.80s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.440/5.066 rem=1.626 speed=0.040 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=381.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=0 task=3 slot=50->-1 pending_B=34 s=3.441/5.066 rem=1.625 speed=0.010 wait=1.70 dwell=0.00
[multi_patrol][state] sim_t=382.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.441/5.066 rem=1.625 speed=0.000 wait=1.80 dwell=0.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V2 blocker=V-1 queue=[V2,V1] tx_valid=1 tx_target=B34 entry_s=3.495 release_s=2.21 vehicles={V0:TO_B,s=1.8172/2.11648,reason=clear;V1:TO_A1,s=0.488483/3.4202,reason=clear;V2:TO_A1,s=3.44064/5.06576,reason=action_hold}
[A1_STATE] state=ADMITTED owner=V2 candidate=V-1 blocker=V-1 queue=[V2,V1] tx_valid=1 tx_target=B34 entry_s=3.495 release_s=2.21 vehicles={V0:TO_B,s=1.83179/2.11648,reason=clear;V1:TO_A1,s=0.50273/3.4202,reason=clear;V2:TO_A1,s=3.44064/5.06576,reason=clear}
[multi_patrol][state] sim_t=382.40s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.443/5.066 rem=1.623 speed=0.020 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=382.50s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.447/5.066 rem=1.619 speed=0.040 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=382.90s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=3 slot=50->-1 pending_B=34 s=3.469/5.066 rem=1.597 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=383.90s V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=3 slot=2->2 pending_B=-1 s=2.116/2.116 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=ADMITTED owner=V2 candidate=V-1 blocker=V-1 queue=[V2,V1] tx_valid=1 tx_target=B34 entry_s=3.495 release_s=2.21 vehicles={V0:UNLOAD_DWELL,s=2.11648/2.11648,reason=not_active;V1:TO_A1,s=0.723659/3.4202,reason=clear;V2:TO_A1,s=3.59469/5.06576,reason=clear}
[multi_patrol][state] sim_t=384.00s V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=3 slot=2->2 pending_B=-1 s=2.116/2.116 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[multi_patrol][state] sim_t=387.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.418/3.420 rem=2.002 speed=0.113 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=387.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.426/3.420 rem=1.994 speed=0.083 wait=0.20 dwell=0.00
[multi_patrol][state] sim_t=388.20s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.436/3.420 rem=1.984 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=388.30s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.440/3.420 rem=1.980 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=388.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.457/3.420 rem=1.963 speed=0.020 wait=1.10 dwell=0.00
[multi_patrol][state] sim_t=388.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.457/3.420 rem=1.963 speed=0.000 wait=1.20 dwell=0.00
[multi_patrol][state] sim_t=389.20s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.459/3.420 rem=1.961 speed=0.020 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=389.30s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.463/3.420 rem=1.957 speed=0.040 wait=1.70 dwell=0.00
[multi_patrol][state] sim_t=389.40s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.464/3.420 rem=1.956 speed=0.010 wait=1.80 dwell=0.00
[multi_patrol][state] sim_t=389.50s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.464/3.420 rem=1.956 speed=0.000 wait=1.90 dwell=0.00
[multi_patrol][state] sim_t=389.90s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.466/3.420 rem=1.954 speed=0.020 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=390.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.466/3.420 rem=1.954 speed=0.000 wait=2.40 dwell=0.00
[multi_patrol][state] sim_t=390.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.466/3.420 rem=1.954 speed=0.000 wait=2.50 dwell=0.00
[multi_patrol][state] sim_t=390.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.468/3.420 rem=1.952 speed=0.020 wait=2.90 dwell=0.00
[multi_patrol][state] sim_t=390.60s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.468/3.420 rem=1.952 speed=0.000 wait=3.00 dwell=0.00
[multi_patrol][state] sim_t=390.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.468/3.420 rem=1.952 speed=0.000 wait=3.10 dwell=0.00
[multi_patrol][state] sim_t=391.10s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.470/3.420 rem=1.950 speed=0.020 wait=3.50 dwell=0.00
[multi_patrol][state] sim_t=391.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.470/3.420 rem=1.950 speed=0.000 wait=3.60 dwell=0.00
[multi_patrol][state] sim_t=393.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.470/3.420 rem=1.950 speed=0.000 wait=5.60 dwell=0.00
[multi_patrol][state] sim_t=393.70s V2 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=3 slot=50->-1 pending_B=34 s=5.066/5.066 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V2 candidate=V-1 blocker=V-1 queue=[V1] tx_valid=1 tx_target=B34 entry_s=3.495 release_s=2.21 vehicles={V0:UNLOAD_DWELL,s=2.11648/2.11648,reason=not_active;V1:TO_A1,s=1.46986/3.4202,reason=clear;V2:PICKUP_DWELL,s=5.06576/5.06576,reason=not_active}
[multi_patrol][state] sim_t=393.80s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.472/3.420 rem=1.948 speed=0.020 wait=6.20 dwell=0.00
[multi_patrol][state] sim_t=393.80s V2 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=3 slot=50->-1 pending_B=34 s=5.066/5.066 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[A1_STATE] state=LOADING owner=V2 candidate=V-1 blocker=V-1 queue=[V1,V0] tx_valid=1 tx_target=B34 entry_s=3.495 release_s=2.21 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V2;V1:TO_A1,s=1.47186/3.4202,reason=action_hold;V2:PICKUP_DWELL,s=5.06576/5.06576,reason=not_active}
[multi_patrol][state] sim_t=393.90s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=393.90s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.476/3.420 rem=1.944 speed=0.040 wait=6.30 dwell=0.00
[multi_patrol][state] sim_t=394.30s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.498/3.420 rem=1.922 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=396.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=396.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.750/3.420 rem=1.670 speed=0.157 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=396.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.779/3.420 rem=1.641 speed=0.067 wait=0.40 dwell=0.00
[multi_patrol][state] sim_t=396.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.788/3.420 rem=1.632 speed=0.050 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=396.60s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.793/3.420 rem=1.627 speed=0.050 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=396.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.805/3.420 rem=1.615 speed=0.020 wait=1.00 dwell=0.00
[multi_patrol][state] sim_t=397.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.805/3.420 rem=1.615 speed=0.000 wait=1.10 dwell=0.00
[multi_patrol][state] sim_t=397.40s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.807/3.420 rem=1.613 speed=0.020 wait=1.50 dwell=0.00
[multi_patrol][state] sim_t=397.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.811/3.420 rem=1.609 speed=0.040 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=397.60s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.812/3.420 rem=1.608 speed=0.010 wait=1.70 dwell=0.00
[multi_patrol][state] sim_t=397.70s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.812/3.420 rem=1.608 speed=0.000 wait=1.80 dwell=0.00
[multi_patrol][state] sim_t=398.10s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=4.30 dwell=0.00
[multi_patrol][state] sim_t=398.10s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.814/3.420 rem=1.606 speed=0.020 wait=2.20 dwell=0.00
[multi_patrol][state] sim_t=398.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.814/3.420 rem=1.606 speed=0.000 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=398.30s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.814/3.420 rem=1.606 speed=0.000 wait=2.40 dwell=0.00
[A1_STATE] state=EXITING owner=V2 candidate=V-1 blocker=V-1 queue=[V1,V0] tx_valid=1 tx_target=B34 entry_s=3.495 release_s=2.21 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V2;V1:TO_A1,s=1.81388/3.4202,reason=clear;V2:TO_B,s=0/4.27481,reason=clear}
[multi_patrol][state] sim_t=398.70s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.816/3.420 rem=1.604 speed=0.020 wait=2.80 dwell=0.00
[multi_patrol][state] sim_t=398.70s V2 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=3 slot=50->34 pending_B=-1 s=0.002/4.275 rem=4.273 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=398.80s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.816/3.420 rem=1.604 speed=0.000 wait=2.90 dwell=0.00
[multi_patrol][state] sim_t=400.20s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=6.40 dwell=0.00
[multi_patrol][state] sim_t=400.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.816/3.420 rem=1.604 speed=0.000 wait=5.00 dwell=0.00
[multi_patrol][state] sim_t=402.30s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=8.50 dwell=0.00
[multi_patrol][state] sim_t=402.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.816/3.420 rem=1.604 speed=0.000 wait=7.00 dwell=0.00
[multi_patrol][state] sim_t=404.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=10.60 dwell=0.00
[multi_patrol][state] sim_t=404.90s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.816/3.420 rem=1.604 speed=0.000 wait=9.00 dwell=0.00
[multi_patrol][state] sim_t=406.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=12.70 dwell=0.00
[multi_patrol][state] sim_t=407.00s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.816/3.420 rem=1.604 speed=0.000 wait=11.10 dwell=0.00
[multi_patrol][state] sim_t=408.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=14.70 dwell=0.00
[multi_patrol][state] sim_t=409.10s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.816/3.420 rem=1.604 speed=0.000 wait=13.20 dwell=0.00
[multi_patrol][state] sim_t=410.60s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=16.80 dwell=0.00
[multi_patrol][state] sim_t=411.20s V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V2 blocker=2 task=4 slot=17->-1 pending_B=55 s=1.816/3.420 rem=1.604 speed=0.000 wait=15.30 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V1 blocker=V-1 queue=[V1,V0] tx_valid=1 tx_target=B55 entry_s=1.865 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V1;V1:TO_A1,s=1.81588/3.4202,reason=clear;V2:TO_B,s=2.22232/4.27481,reason=clear}
[multi_patrol][state] sim_t=412.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=18.60 dwell=0.00
[multi_patrol][state] sim_t=412.40s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.818/3.420 rem=1.602 speed=0.020 wait=16.50 dwell=0.00
[A1_STATE] state=ADMITTED owner=V1 candidate=V-1 blocker=V-1 queue=[V1,V0] tx_valid=1 tx_target=B55 entry_s=1.865 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V1;V1:TO_A1,s=1.81788/3.4202,reason=action_hold;V2:TO_B,s=2.24232/4.27481,reason=clear}
[multi_patrol][state] sim_t=412.50s V1 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.822/3.420 rem=1.598 speed=0.040 wait=16.60 dwell=0.00
[multi_patrol][state] sim_t=412.90s V1 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=4 slot=17->-1 pending_B=55 s=1.844/3.420 rem=1.576 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=414.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=20.70 dwell=0.00
[multi_patrol][state] sim_t=416.50s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=22.70 dwell=0.00
[multi_patrol][state] sim_t=418.60s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=24.80 dwell=0.00
[multi_patrol][state] sim_t=420.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=26.90 dwell=0.00
[multi_patrol][state] sim_t=422.70s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=28.90 dwell=0.00
[multi_patrol][state] sim_t=423.30s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=pickup_dwell blocker=-1 task=4 slot=17->-1 pending_B=55 s=3.420/3.420 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[A1_STATE] state=LOADING owner=V1 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B55 entry_s=1.865 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V1;V1:PICKUP_DWELL,s=3.4202/3.4202,reason=not_active;V2:TO_B,s=4.0797/4.27481,reason=clear}
[multi_patrol][state] sim_t=423.40s V1 mode=DWELL phase=PICKUP_DWELL action=STOP reason=not_active blocker=-1 task=4 slot=17->-1 pending_B=55 s=3.420/3.420 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[multi_patrol][state] sim_t=424.40s V2 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=3 slot=34->34 pending_B=-1 s=4.275/4.275 rem=0.000 speed=0.000 wait=0.00 dwell=10.00
[A1_STATE] state=LOADING owner=V1 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B55 entry_s=1.865 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V1;V1:PICKUP_DWELL,s=3.4202/3.4202,reason=not_active;V2:UNLOAD_DWELL,s=4.27481/4.27481,reason=not_active}
[multi_patrol][state] sim_t=424.50s V2 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=3 slot=34->34 pending_B=-1 s=4.275/4.275 rem=0.000 speed=0.000 wait=0.00 dwell=9.90
[multi_patrol][state] sim_t=424.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=31.00 dwell=0.00
[multi_patrol][state] sim_t=426.80s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=33.00 dwell=0.00
[A1_STATE] state=EXITING owner=V1 candidate=V-1 blocker=V-1 queue=[V0] tx_valid=1 tx_target=B55 entry_s=1.865 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V1;V1:TO_B,s=0/6.95421,reason=clear;V2:UNLOAD_DWELL,s=4.27481/4.27481,reason=not_active}
[multi_patrol][state] sim_t=428.30s V1 mode=ACTIVE phase=TO_B action=NOMINAL reason=clear blocker=-1 task=4 slot=17->55 pending_B=-1 s=0.002/6.954 rem=6.952 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=428.90s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=35.10 dwell=0.00
[multi_patrol][state] sim_t=431.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=37.20 dwell=0.00
[multi_patrol][state] sim_t=433.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=39.20 dwell=0.00
[A1_STATE] state=EXITING owner=V1 candidate=V-1 blocker=V-1 queue=[V0,V2] tx_valid=1 tx_target=B55 entry_s=1.865 release_s=2.29 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_turn_V1;V1:TO_B,s=0.887751/6.95421,reason=clear;V2:TO_A1,s=0/3.08771,reason=clear}
[multi_patrol][state] sim_t=434.40s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.002/3.088 rem=3.086 speed=0.020 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=435.10s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=41.30 dwell=0.00
[multi_patrol][state] sim_t=437.20s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=43.40 dwell=0.00
[multi_patrol][state] sim_t=437.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.398/3.088 rem=2.690 speed=0.112 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=437.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.406/3.088 rem=2.681 speed=0.082 wait=0.20 dwell=0.00
[multi_patrol][state] sim_t=437.70s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.416/3.088 rem=2.672 speed=0.020 wait=0.60 dwell=0.00
[multi_patrol][state] sim_t=437.80s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.420/3.088 rem=2.668 speed=0.040 wait=0.70 dwell=0.00
[multi_patrol][state] sim_t=438.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.437/3.088 rem=2.651 speed=0.020 wait=1.10 dwell=0.00
[multi_patrol][state] sim_t=438.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.437/3.088 rem=2.651 speed=0.000 wait=1.20 dwell=0.00
[multi_patrol][state] sim_t=438.70s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.439/3.088 rem=2.649 speed=0.020 wait=1.60 dwell=0.00
[multi_patrol][state] sim_t=438.80s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.443/3.088 rem=2.645 speed=0.040 wait=1.70 dwell=0.00
[multi_patrol][state] sim_t=438.90s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.444/3.088 rem=2.644 speed=0.010 wait=1.80 dwell=0.00
[multi_patrol][state] sim_t=439.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.444/3.088 rem=2.644 speed=0.000 wait=2.00 dwell=0.00
[multi_patrol][state] sim_t=439.30s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=45.50 dwell=0.00
[multi_patrol][state] sim_t=439.40s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.446/3.088 rem=2.642 speed=0.020 wait=2.30 dwell=0.00
[multi_patrol][state] sim_t=439.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.446/3.088 rem=2.642 speed=0.000 wait=2.40 dwell=0.00
[multi_patrol][state] sim_t=439.60s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.446/3.088 rem=2.642 speed=0.000 wait=2.50 dwell=0.00
[multi_patrol][state] sim_t=440.00s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.448/3.088 rem=2.640 speed=0.020 wait=2.90 dwell=0.00
[multi_patrol][state] sim_t=440.10s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.448/3.088 rem=2.640 speed=0.000 wait=3.00 dwell=0.00
[multi_patrol][state] sim_t=441.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_turn_V1 blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=47.60 dwell=0.00
[multi_patrol][state] sim_t=442.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.260/6.954 rem=4.694 speed=0.170 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=442.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_local_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.448/3.088 rem=2.640 speed=0.000 wait=4.90 dwell=0.00
[multi_patrol][state] sim_t=442.10s V1 mode=ACTIVE phase=TO_B action=STOP reason=action_hold blocker=-1 task=4 slot=17->55 pending_B=-1 s=2.274/6.954 rem=4.680 speed=0.140 wait=0.20 dwell=0.00
[A1_STATE] state=WAITING owner=V-1 candidate=V0 blocker=V1 queue=[V0,V2] tx_valid=0 tx_target=B-1 entry_s=0 release_s=0 vehicles={V0:TO_A1,s=0/2.1158,reason=wait_a1_admission_clear;V1:TO_B,s=2.29288/6.95421,reason=brake_V2;V2:TO_A1,s=0.447855/3.08771,reason=clear}
[multi_patrol][state] sim_t=442.40s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=48.60 dwell=0.00
[multi_patrol][state] sim_t=442.40s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.298/6.954 rem=4.656 speed=0.050 wait=0.50 dwell=0.00
[multi_patrol][state] sim_t=442.40s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.450/3.088 rem=2.638 speed=0.020 wait=5.30 dwell=0.00
[multi_patrol][state] sim_t=442.50s V2 mode=ACTIVE phase=TO_A1 action=CREEP reason=action_hold blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.454/3.088 rem=2.634 speed=0.040 wait=5.40 dwell=0.00
[multi_patrol][state] sim_t=442.90s V2 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=4 slot=34->-1 pending_B=3 s=0.476/3.088 rem=2.612 speed=0.070 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=444.30s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=50.50 dwell=0.00
[multi_patrol][state] sim_t=444.30s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=2.40 dwell=0.00
[multi_patrol][state] sim_t=444.60s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.711/3.088 rem=2.376 speed=0.137 wait=0.10 dwell=0.00
[coord_diag][cycle] tick=4456 sim_t=445.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 spd=0.000 wait=3.7 gen=10
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=1.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.300/6.954 gear=F act=0 blk=V2 gen=10 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.800,3.225] committed=1 inside_real=1 | B[0.725,2.775] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.546,2.933) | A[0.800,3.225] stop=0.621 gap=-1.679 inside=1 t=[0.000,5.286] | B[0.725,2.775] stop=0.546 gap=-0.190 inside=1 t=[0.000,10.854] overlap=1
[multi_patrol][state] sim_t=446.20s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=52.40 dwell=0.00
[multi_patrol][state] sim_t=446.20s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=4.30 dwell=0.00
[multi_patrol][state] sim_t=446.50s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=2.00 dwell=0.00
[multi_patrol][state] sim_t=448.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=54.20 dwell=0.00
[multi_patrol][state] sim_t=448.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=6.10 dwell=0.00
[multi_patrol][state] sim_t=448.40s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=3.90 dwell=0.00
[multi_patrol][state] sim_t=450.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=56.20 dwell=0.00
[multi_patrol][state] sim_t=450.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=8.10 dwell=0.00
[multi_patrol][state] sim_t=450.30s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=5.80 dwell=0.00
[coord_diag][cycle] tick=4506 sim_t=450.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 spd=0.000 wait=8.7 gen=10
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=6.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.300/6.954 gear=F act=0 blk=V2 gen=10 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.800,3.225] committed=1 inside_real=1 | B[0.725,2.775] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.546,2.933) | A[0.800,3.225] stop=0.621 gap=-1.679 inside=1 t=[0.000,5.286] | B[0.725,2.775] stop=0.546 gap=-0.190 inside=1 t=[0.000,10.854] overlap=1
[multi_patrol][state] sim_t=452.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=58.20 dwell=0.00
[multi_patrol][state] sim_t=452.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=10.10 dwell=0.00
[multi_patrol][state] sim_t=452.20s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=7.70 dwell=0.00
[multi_patrol][state] sim_t=454.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=60.20 dwell=0.00
[multi_patrol][state] sim_t=454.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=12.10 dwell=0.00
[multi_patrol][state] sim_t=454.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=9.50 dwell=0.00
[coord_diag][cycle] tick=4556 sim_t=455.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 spd=0.000 wait=13.7 gen=10
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=11.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.300/6.954 gear=F act=0 blk=V2 gen=10 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.800,3.225] committed=1 inside_real=1 | B[0.725,2.775] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.546,2.933) | A[0.800,3.225] stop=0.621 gap=-1.679 inside=1 t=[0.000,5.286] | B[0.725,2.775] stop=0.546 gap=-0.190 inside=1 t=[0.000,10.854] overlap=1
[multi_patrol][state] sim_t=456.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=62.20 dwell=0.00
[multi_patrol][state] sim_t=456.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=14.10 dwell=0.00
[multi_patrol][state] sim_t=456.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=11.50 dwell=0.00
[multi_patrol][state] sim_t=458.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=64.20 dwell=0.00
[multi_patrol][state] sim_t=458.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=16.10 dwell=0.00
[multi_patrol][state] sim_t=458.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=13.50 dwell=0.00
[multi_patrol][state] sim_t=460.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=66.20 dwell=0.00
[multi_patrol][state] sim_t=460.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=18.10 dwell=0.00
[multi_patrol][state] sim_t=460.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=15.50 dwell=0.00
[coord_diag][cycle] tick=4606 sim_t=460.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 spd=0.000 wait=18.7 gen=10
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=16.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.300/6.954 gear=F act=0 blk=V2 gen=10 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.800,3.225] committed=1 inside_real=1 | B[0.725,2.775] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.546,2.933) | A[0.800,3.225] stop=0.621 gap=-1.679 inside=1 t=[0.000,5.286] | B[0.725,2.775] stop=0.546 gap=-0.190 inside=1 t=[0.000,10.854] overlap=1
[multi_patrol][state] sim_t=462.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=68.20 dwell=0.00
[multi_patrol][state] sim_t=462.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=20.10 dwell=0.00
[multi_patrol][state] sim_t=462.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=17.50 dwell=0.00
[multi_patrol][state] sim_t=464.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=70.20 dwell=0.00
[multi_patrol][state] sim_t=464.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=22.10 dwell=0.00
[multi_patrol][state] sim_t=464.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=19.50 dwell=0.00
[coord_diag][cycle] tick=4656 sim_t=465.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 spd=0.000 wait=23.7 gen=10
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=21.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=2.300/6.954 gear=F act=0 blk=V2 gen=10 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.800,3.225] committed=1 inside_real=1 | B[0.725,2.775] committed=1 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.546,2.933) | A[0.800,3.225] stop=0.621 gap=-1.679 inside=1 t=[0.000,5.286] | B[0.725,2.775] stop=0.546 gap=-0.190 inside=1 t=[0.000,10.854] overlap=1
[multi_patrol][state] sim_t=466.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=72.20 dwell=0.00
[multi_patrol][state] sim_t=466.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=24.10 dwell=0.00
[multi_patrol][state] sim_t=466.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=21.50 dwell=0.00
[multi_patrol][state] sim_t=468.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=74.20 dwell=0.00
[multi_patrol][state] sim_t=468.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=2.300/6.954 rem=4.654 speed=0.000 wait=26.10 dwell=0.00
[multi_patrol][state] sim_t=468.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=23.50 dwell=0.00
[multi_patrol][DEADLOCK] tick=4695 sim_t=469.500000 members=[V1(brake_V2->V2) V2(clear_block_V1->V1)]
[multi_patrol][state] sim_t=469.50s V1 mode=ACTIVE phase=TO_B action=NOMINAL reason=deadlock_replan blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=469.60s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=0.10 dwell=0.00
[multi_patrol][state] sim_t=470.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=76.20 dwell=0.00
[multi_patrol][state] sim_t=470.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=25.50 dwell=0.00
[coord_diag][cycle] tick=4706 sim_t=470.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 spd=0.000 wait=1.1 gen=11
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=26.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=0.000/4.653 gear=F act=0 blk=V2 gen=11 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.000,0.675] committed=0 inside_real=1 | B[0.900,1.600] committed=0 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.905,2.841) | A[0.000,0.675] stop=-0.179 gap=-0.179 inside=1 t=[0.000,4.035] | B[0.900,1.600] stop=0.721 gap=-0.015 inside=1 t=[0.000,4.979] overlap=1
[multi_patrol][state] sim_t=471.50s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=2.00 dwell=0.00
[multi_patrol][state] sim_t=472.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=78.20 dwell=0.00
[multi_patrol][state] sim_t=472.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=27.50 dwell=0.00
[multi_patrol][state] sim_t=473.40s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=3.90 dwell=0.00
[multi_patrol][state] sim_t=474.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=80.20 dwell=0.00
[multi_patrol][state] sim_t=474.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=29.50 dwell=0.00
[multi_patrol][state] sim_t=475.30s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=5.80 dwell=0.00
[coord_diag][cycle] tick=4756 sim_t=475.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 spd=0.000 wait=6.1 gen=11
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=31.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=0.000/4.653 gear=F act=0 blk=V2 gen=11 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.000,0.675] committed=0 inside_real=1 | B[0.900,1.600] committed=0 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.905,2.841) | A[0.000,0.675] stop=-0.179 gap=-0.179 inside=1 t=[0.000,4.035] | B[0.900,1.600] stop=0.721 gap=-0.015 inside=1 t=[0.000,4.979] overlap=1
[multi_patrol][state] sim_t=476.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=82.20 dwell=0.00
[multi_patrol][state] sim_t=476.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=31.50 dwell=0.00
[multi_patrol][state] sim_t=477.20s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=7.70 dwell=0.00
[multi_patrol][state] sim_t=478.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=84.20 dwell=0.00
[multi_patrol][state] sim_t=478.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=33.50 dwell=0.00
[multi_patrol][state] sim_t=479.10s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=9.60 dwell=0.00
[multi_patrol][state] sim_t=480.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=86.20 dwell=0.00
[multi_patrol][state] sim_t=480.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=35.50 dwell=0.00
[coord_diag][cycle] tick=4806 sim_t=480.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 spd=0.000 wait=11.1 gen=11
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=36.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=0.000/4.653 gear=F act=0 blk=V2 gen=11 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.000,0.675] committed=0 inside_real=1 | B[0.900,1.600] committed=0 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.905,2.841) | A[0.000,0.675] stop=-0.179 gap=-0.179 inside=1 t=[0.000,4.035] | B[0.900,1.600] stop=0.721 gap=-0.015 inside=1 t=[0.000,4.979] overlap=1
[multi_patrol][state] sim_t=481.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=11.50 dwell=0.00
[multi_patrol][state] sim_t=482.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=88.20 dwell=0.00
[multi_patrol][state] sim_t=482.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=37.50 dwell=0.00
[multi_patrol][state] sim_t=482.90s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=13.40 dwell=0.00
[multi_patrol][state] sim_t=484.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=90.20 dwell=0.00
[multi_patrol][state] sim_t=484.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=39.50 dwell=0.00
[multi_patrol][state] sim_t=484.80s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=15.30 dwell=0.00
[coord_diag][cycle] tick=4856 sim_t=485.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 spd=0.000 wait=16.1 gen=11
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=41.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=0.000/4.653 gear=F act=0 blk=V2 gen=11 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.000,0.675] committed=0 inside_real=1 | B[0.900,1.600] committed=0 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.905,2.841) | A[0.000,0.675] stop=-0.179 gap=-0.179 inside=1 t=[0.000,4.035] | B[0.900,1.600] stop=0.721 gap=-0.015 inside=1 t=[0.000,4.979] overlap=1
[multi_patrol][state] sim_t=486.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=92.20 dwell=0.00
[multi_patrol][state] sim_t=486.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=41.50 dwell=0.00
[multi_patrol][state] sim_t=486.70s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=17.20 dwell=0.00
[multi_patrol][state] sim_t=488.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=94.20 dwell=0.00
[multi_patrol][state] sim_t=488.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=43.50 dwell=0.00
[multi_patrol][state] sim_t=488.60s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=19.10 dwell=0.00
[multi_patrol][state] sim_t=490.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=96.20 dwell=0.00
[multi_patrol][state] sim_t=490.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=45.50 dwell=0.00
[multi_patrol][state] sim_t=490.50s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=21.00 dwell=0.00
[coord_diag][cycle] tick=4906 sim_t=490.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 spd=0.000 wait=21.1 gen=11
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=46.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=0.000/4.653 gear=F act=0 blk=V2 gen=11 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.000,0.675] committed=0 inside_real=1 | B[0.900,1.600] committed=0 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.905,2.841) | A[0.000,0.675] stop=-0.179 gap=-0.179 inside=1 t=[0.000,4.035] | B[0.900,1.600] stop=0.721 gap=-0.015 inside=1 t=[0.000,4.979] overlap=1
[multi_patrol][state] sim_t=492.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=98.20 dwell=0.00
[multi_patrol][state] sim_t=492.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=47.50 dwell=0.00
[multi_patrol][state] sim_t=492.40s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=22.90 dwell=0.00
[multi_patrol][state] sim_t=494.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=100.20 dwell=0.00
[multi_patrol][state] sim_t=494.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=49.50 dwell=0.00
[multi_patrol][state] sim_t=494.30s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=24.80 dwell=0.00
[multi_patrol][DEADLOCK] tick=4945 sim_t=494.500000 members=[V1(brake_V2->V2) V2(clear_block_V1->V1)]
[multi_patrol][state] sim_t=494.50s V1 mode=ACTIVE phase=TO_B action=NOMINAL reason=deadlock_replan blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=0.00 dwell=0.00
[multi_patrol][state] sim_t=494.60s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=0.10 dwell=0.00
[coord_diag][cycle] tick=4956 sim_t=495.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 spd=0.000 wait=1.1 gen=12
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=51.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=0.000/4.653 gear=F act=0 blk=V2 gen=12 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.000,0.675] committed=0 inside_real=1 | B[0.900,1.600] committed=0 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.905,2.841) | A[0.000,0.675] stop=-0.179 gap=-0.179 inside=1 t=[0.000,4.035] | B[0.900,1.600] stop=0.721 gap=-0.015 inside=1 t=[0.000,4.979] overlap=1
[multi_patrol][state] sim_t=496.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=102.20 dwell=0.00
[multi_patrol][state] sim_t=496.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=51.50 dwell=0.00
[multi_patrol][state] sim_t=496.50s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=2.00 dwell=0.00
[multi_patrol][state] sim_t=498.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=104.20 dwell=0.00
[multi_patrol][state] sim_t=498.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=53.50 dwell=0.00
[multi_patrol][state] sim_t=498.40s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=3.90 dwell=0.00
[multi_patrol][state] sim_t=500.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=106.20 dwell=0.00
[multi_patrol][state] sim_t=500.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=55.50 dwell=0.00
[multi_patrol][state] sim_t=500.30s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=5.80 dwell=0.00
[coord_diag][cycle] tick=5006 sim_t=500.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 spd=0.000 wait=6.1 gen=12
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=56.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=0.000/4.653 gear=F act=0 blk=V2 gen=12 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.000,0.675] committed=0 inside_real=1 | B[0.900,1.600] committed=0 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.905,2.841) | A[0.000,0.675] stop=-0.179 gap=-0.179 inside=1 t=[0.000,4.035] | B[0.900,1.600] stop=0.721 gap=-0.015 inside=1 t=[0.000,4.979] overlap=1
[multi_patrol][state] sim_t=502.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=108.20 dwell=0.00
[multi_patrol][state] sim_t=502.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=57.50 dwell=0.00
[multi_patrol][state] sim_t=502.20s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=7.70 dwell=0.00
[multi_patrol][state] sim_t=504.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=110.20 dwell=0.00
[multi_patrol][state] sim_t=504.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=9.50 dwell=0.00
[multi_patrol][state] sim_t=504.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=59.50 dwell=0.00
[coord_diag][cycle] tick=5056 sim_t=505.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 spd=0.000 wait=11.1 gen=12
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=61.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=0.000/4.653 gear=F act=0 blk=V2 gen=12 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.000,0.675] committed=0 inside_real=1 | B[0.900,1.600] committed=0 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.905,2.841) | A[0.000,0.675] stop=-0.179 gap=-0.179 inside=1 t=[0.000,4.035] | B[0.900,1.600] stop=0.721 gap=-0.015 inside=1 t=[0.000,4.979] overlap=1
[multi_patrol][state] sim_t=506.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=112.20 dwell=0.00
[multi_patrol][state] sim_t=506.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=11.50 dwell=0.00
[multi_patrol][state] sim_t=506.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=61.50 dwell=0.00
[multi_patrol][state] sim_t=508.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=114.20 dwell=0.00
[multi_patrol][state] sim_t=508.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=13.50 dwell=0.00
[multi_patrol][state] sim_t=508.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=63.50 dwell=0.00
[multi_patrol][state] sim_t=510.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=116.20 dwell=0.00
[multi_patrol][state] sim_t=510.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=15.50 dwell=0.00
[multi_patrol][state] sim_t=510.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=65.50 dwell=0.00
[coord_diag][cycle] tick=5106 sim_t=510.60 ring=V1->V2->V1 a1_state=WAITING a1_owner=V-1
[coord_diag][vehicle]  V1 mode=1 phase=TO_B loaded=1 act=0 reason=brake_V2 blk=2 brkr=0 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 spd=0.000 wait=16.1 gen=12
[coord_diag][vehicle]  V2 mode=1 phase=TO_A1 loaded=0 act=0 reason=clear_block_V1 blk=1 brkr=0 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 spd=0.000 wait=66.1 gen=9
[coord_diag][a1_gate] owner=V0 waiter=V2 stop_s=1.300 xy=(2.108,3.125) source=turn approach_zones=0 departure_zones=0 late=0
[coord_diag][pair] V1<->V2 a1_owner=V-1 reservation=V2 following=0 following_leader=V-1 zones=1 all_same_dir=0 nominal_time_overlap=1 | A phase=TO_B s=0.000/4.653 gear=F act=0 blk=V2 gen=12 pending_B=-1 pending_gen=7 | B phase=TO_A1 s=0.736/3.088 gear=F act=0 blk=V1 gen=9 pending_B=3 pending_gen=6
[coord_diag][envelope] A[0.000,0.675] committed=0 inside_real=1 | B[0.900,1.600] committed=0 inside_real=1 both_inside_same_zone=1
[coord_diag][zone 0] same_dir=0 phase=F/F xy=(1.905,2.841) | A[0.000,0.675] stop=-0.179 gap=-0.179 inside=1 t=[0.000,4.035] | B[0.900,1.600] stop=0.721 gap=-0.015 inside=1 t=[0.000,4.979] overlap=1
[multi_patrol][state] sim_t=512.00s V0 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_admission_clear blocker=1 task=4 slot=2->-1 pending_B=52 s=0.000/2.116 rem=2.116 speed=0.000 wait=118.20 dwell=0.00
[multi_patrol][state] sim_t=512.00s V1 mode=ACTIVE phase=TO_B action=STOP reason=brake_V2 blocker=2 task=4 slot=17->55 pending_B=-1 s=0.000/4.653 rem=4.653 speed=0.000 wait=17.50 dwell=0.00
[multi_patrol][state] sim_t=512.00s V2 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=4 slot=34->-1 pending_B=3 s=0.736/3.088 rem=2.352 speed=0.000 wait=67.50 dwell=0.00