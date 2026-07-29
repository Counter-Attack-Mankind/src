INFO] [1785248931.757163302]: [multi_patrol][A1 activate] owner=V1 egress_ready=1 target_B=64
[INFO] [1785248932.956316846]: [multi_patrol] tick=47112 sim_t=4711.20 V0 arrived B30; dwell 5.00s; load=loaded phase=5
[INFO] [1785248932.956506247]: [multi_patrol][state] tick=47112 sim_t=4711.20 V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=unload_dwell blocker=-1 task=54 slot=30->30 pending_B=-1 s=3.835/3.835 rem=0.000 speed=0.000 wait=0.00 dwell=5.00
[INFO] [1785248933.056167582]: [multi_patrol][state] tick=47113 sim_t=4711.30 V0 mode=DWELL phase=UNLOAD_DWELL action=STOP reason=not_active blocker=-1 task=54 slot=30->30 pending_B=-1 s=3.835/3.835 rem=0.000 speed=0.000 wait=0.00 dwell=4.90
[INFO] [1785248933.756750934]: [multi_patrol][A1 reserve only] reserved=V1 remaining=2.468 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[INFO] [1785248933.757574738]: [multi_patrol][A1 atomic admission] owner=V1 remaining=1.499 boundary=1.500 requesters=1 orange_handoff=0 pending_B=64
[INFO] [1785248933.757684438]: [multi_patrol][A1 activate] owner=V1 egress_ready=1 target_B=64
[INFO] [1785248935.756481710]: [multi_patrol][A1 activate] owner=V1 egress_ready=1 target_B=64
[INFO] [1785248935.855933440]: [multi_patrol][A1 reserve only] reserved=V1 remaining=2.161 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[INFO] [1785248937.756373630]: [multi_patrol][A1 atomic admission] owner=V1 remaining=1.499 boundary=1.500 requesters=1 orange_handoff=0 pending_B=64
[INFO] [1785248937.756564831]: [multi_patrol][A1 activate] owner=V1 egress_ready=1 target_B=64
[INFO] [1785248937.855979459]: [multi_patrol][A1 reserve only] reserved=V1 remaining=1.871 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[WARN] [1785248937.956404090]: 
********
[multi_patrol] V0 feasible target slots from slot 30 row=3 col=5: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 65
[multi_patrol] rejected target slots:
  30 (slot 30 row=3 col=5): same_slot
  64 (slot 64 row=7 col=8): reserved_by_other
************
[INFO] [1785248937.956541791]: [multi_patrol][task_select] mode=deterministic seed=20260728 V0 task_index=55 source=B30 selected=B19 feasible=64 ranked=B19,B42,B31,B5,B43,B16,B41,B54,B6,B46,B32,B35,B36,B39,B7,B34,B14,B52,B28,B38,B45,B60,B3,B12,B33,B29,B17,B10,B26,B58,B11,B61,B63,B21,B40,B37,B15,B47,B59,B27,B18,B44,B56,B55,B9,B13,B8,B49,B1,B2,B23,B50,B24,B25,B57,B62,B51,B53,B0,B65,B48,B4,B22,B20
[INFO] [1785248937.956663591]: [multi_patrol][A1] V0 reserved future dropoff: A1 -> B19  wpts=333 len=4.755 pending_gen=56
[INFO] [1785248937.956762992]: [multi_patrol][A1] V0 pickup leg: B30 -> A1  reserved_dropoff=B19 pending_wpts=333 wpts=286 len=3.524
[WARN] [1785248937.965967332]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=departure_not_stoppable gate_gap=-0.244 required_stop=0.112; candidate remains hint-only and orange arbitration stays authoritative
[INFO] [1785248937.969513947]: [multi_patrol][state] tick=47162 sim_t=4716.20 V0 mode=ACTIVE phase=TO_A1 action=NOMINAL reason=clear blocker=-1 task=55 slot=30->-1 pending_B=19 s=0.002/3.524 rem=3.522 speed=0.020 wait=0.00 dwell=0.00
[ERROR] [1785248939.758805105]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[INFO] [1785248939.856185521]: [multi_patrol][A1 reserve only] reserved=V0 remaining=3.265 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[WARN] [1785248940.056598976]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=departure_not_stoppable gate_gap=-0.287 required_stop=0.112; candidate remains hint-only and orange arbitration stays authoritative
[ERROR] [1785248941.758859013]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[INFO] [1785248941.856294827]: [multi_patrol][A1 reserve only] reserved=V0 remaining=2.980 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[WARN] [1785248942.156167798]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=departure_not_stoppable gate_gap=-0.586 required_stop=0.112; candidate remains hint-only and orange arbitration stays authoritative
[ERROR] [1785248943.759013067]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[INFO] [1785248943.956032897]: [multi_patrol][A1 reserve only] reserved=V0 remaining=2.658 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[WARN] [1785248944.156175739]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=departure_not_stoppable gate_gap=-0.905 required_stop=0.157; candidate remains hint-only and orange arbitration stays authoritative
[ERROR] [1785248945.759094163]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[INFO] [1785248945.956068688]: [multi_patrol][A1 reserve only] reserved=V0 remaining=2.258 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[WARN] [1785248946.256459143]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=departure_not_stoppable gate_gap=-1.325 required_stop=0.157; candidate remains hint-only and orange arbitration stays authoritative
[ERROR] [1785248947.759300406]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[INFO] [1785248947.956119325]: [multi_patrol][A1 reserve only] reserved=V0 remaining=1.918 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[WARN] [1785248948.256877772]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=departure_not_stoppable gate_gap=-1.649 required_stop=0.112; candidate remains hint-only and orange arbitration stays authoritative
[INFO] [1785248949.055931785]: [multi_patrol] tick=47273 sim_t=4727.30 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248949.056088786]: [multi_patrol][state] tick=47273 sim_t=4727.30 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248949.156083399]: [multi_patrol] tick=47274 sim_t=4727.40 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248949.257167017]: [multi_patrol][state] tick=47275 sim_t=4727.50 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248949.656362266]: [multi_patrol] tick=47279 sim_t=4727.90 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248949.656512467]: [multi_patrol][state] tick=47279 sim_t=4727.90 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[ERROR] [1785248949.759318991]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[INFO] [1785248949.760580997]: [multi_patrol][state] tick=47280 sim_t=4728.00 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248949.956204205]: [multi_patrol][A1 reserve only] reserved=V0 remaining=1.610 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[INFO] [1785248950.156019530]: [multi_patrol] tick=47284 sim_t=4728.40 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248950.156428331]: [multi_patrol][state] tick=47284 sim_t=4728.40 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248950.256180142]: [multi_patrol][state] tick=47285 sim_t=4728.50 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[WARN] [1785248950.356162254]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=departure_not_stoppable gate_gap=-1.994 required_stop=0.157; candidate remains hint-only and orange arbitration stays authoritative
[INFO] [1785248950.656184790]: [multi_patrol] tick=47289 sim_t=4728.90 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248950.656330290]: [multi_patrol][state] tick=47289 sim_t=4728.90 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248950.755936201]: [multi_patrol][state] tick=47290 sim_t=4729.00 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248951.155891346]: [multi_patrol] tick=47294 sim_t=4729.40 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248951.156038046]: [multi_patrol][state] tick=47294 sim_t=4729.40 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248951.256235358]: [multi_patrol][state] tick=47295 sim_t=4729.50 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248951.656179500]: [multi_patrol] tick=47299 sim_t=4729.90 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248951.656384402]: [multi_patrol][state] tick=47299 sim_t=4729.90 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[ERROR] [1785248951.759408024]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[INFO] [1785248951.760420229]: [multi_patrol][state] tick=47300 sim_t=4730.00 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248951.957069135]: [multi_patrol][A1 reserve only] reserved=V0 remaining=1.276 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[INFO] [1785248952.156095451]: [multi_patrol] tick=47304 sim_t=4730.40 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248952.156245451]: [multi_patrol][state] tick=47304 sim_t=4730.40 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248952.256045359]: [multi_patrol][state] tick=47305 sim_t=4730.50 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[WARN] [1785248952.356569870]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=departure_not_stoppable gate_gap=-2.303 required_stop=0.108; candidate remains hint-only and orange arbitration stays authoritative
[INFO] [1785248952.656125196]: [multi_patrol] tick=47309 sim_t=4730.90 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248952.656310296]: [multi_patrol][state] tick=47309 sim_t=4730.90 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248952.756183205]: [multi_patrol][state] tick=47310 sim_t=4731.00 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248953.156391341]: [multi_patrol] tick=47314 sim_t=4731.40 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248953.156534541]: [multi_patrol][state] tick=47314 sim_t=4731.40 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248953.256388449]: [multi_patrol][state] tick=47315 sim_t=4731.50 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248953.656285079]: [multi_patrol] tick=47319 sim_t=4731.90 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248953.656431479]: [multi_patrol][state] tick=47319 sim_t=4731.90 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[ERROR] [1785248953.759659100]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[INFO] [1785248953.760245203]: [multi_patrol][state] tick=47320 sim_t=4732.00 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248954.057523415]: [multi_patrol][A1 reserve only] reserved=V0 remaining=0.989 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[INFO] [1785248954.156288616]: [multi_patrol] tick=47324 sim_t=4732.40 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248954.156445516]: [multi_patrol][state] tick=47324 sim_t=4732.40 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248954.256297222]: [multi_patrol][state] tick=47325 sim_t=4732.50 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[WARN] [1785248954.357155533]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=departure_not_stoppable gate_gap=-2.581 required_stop=0.130; candidate remains hint-only and orange arbitration stays authoritative
[INFO] [1785248954.656039247]: [multi_patrol] tick=47329 sim_t=4732.90 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248954.656182947]: [multi_patrol][state] tick=47329 sim_t=4732.90 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248954.756161354]: [multi_patrol][state] tick=47330 sim_t=4733.00 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248955.156164677]: [multi_patrol] tick=47334 sim_t=4733.40 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248955.156309778]: [multi_patrol][state] tick=47334 sim_t=4733.40 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248955.255950781]: [multi_patrol][state] tick=47335 sim_t=4733.50 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248955.656169402]: [multi_patrol] tick=47339 sim_t=4733.90 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248955.656591104]: [multi_patrol][state] tick=47339 sim_t=4733.90 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[WARN] [1785248955.759222320]: [DIAG wedge] V0(s=2.913,rem=0.610,wait=8.1,act=0) vs V1(s=3.319,rem=0.000,wait=0.1,act=0) owner=V1 nzones=1 | A region[se=3.125 sx=3.500] stopline=2.946 committed=0 | B region[se=2.900 sx=3.300] stopline=2.721 committed=1 | front_ext=0.179
[WARN] [1785248955.759350021]: [DIAG wedge]   zone0 A[3.125,3.500] B[2.900,3.300] @(1.25,4.26)
[ERROR] [1785248955.759782523]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[INFO] [1785248955.760559126]: [multi_patrol][state] tick=47340 sim_t=4734.00 V0 mode=ACTIVE phase=TO_A1 action=STOP reason=brake_V1 blocker=1 task=55 slot=30->-1 pending_B=19 s=2.874/3.524 rem=0.650 speed=0.157 wait=0.10 dwell=0.00
[INFO] [1785248955.761025928]: [multi_patrol][state] tick=47340 sim_t=4734.00 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248955.956273018]: [multi_patrol][state] tick=47342 sim_t=4734.20 V0 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=55 slot=30->-1 pending_B=19 s=2.896/3.524 rem=0.628 speed=0.097 wait=0.30 dwell=0.00
[INFO] [1785248956.156456726]: [multi_patrol][A1 reserve only] reserved=V0 remaining=0.621 boundary=1.500 directional=0 (ordinary arbitration remains authoritative)
[INFO] [1785248956.156619927]: [multi_patrol] tick=47344 sim_t=4734.40 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248956.156697227]: [multi_patrol][state] tick=47344 sim_t=4734.40 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248956.256150129]: [multi_patrol][state] tick=47345 sim_t=4734.50 V0 mode=ACTIVE phase=TO_A1 action=CREEP reason=clear blocker=-1 task=55 slot=30->-1 pending_B=19 s=2.911/3.524 rem=0.612 speed=0.050 wait=0.60 dwell=0.00
[INFO] [1785248956.256321529]: [multi_patrol][state] tick=47345 sim_t=4734.50 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248956.355895531]: [multi_patrol][state] tick=47346 sim_t=4734.60 V0 mode=ACTIVE phase=TO_A1 action=STOP reason=clear_block_V1 blocker=1 task=55 slot=30->-1 pending_B=19 s=2.913/3.524 rem=0.610 speed=0.020 wait=0.70 dwell=0.00
[WARN] [1785248956.456096936]: [multi_patrol][A1 admission deferred] candidate=V1 blocker=V0 cause=approach_not_stoppable gate_gap=-0.217 required_stop=0.055; candidate remains hint-only and orange arbitration stays authoritative
[INFO] [1785248956.655868242]: [multi_patrol] tick=47349 sim_t=4734.90 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248956.656022543]: [multi_patrol][state] tick=47349 sim_t=4734.90 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248956.756264948]: [multi_patrol][state] tick=47350 sim_t=4735.00 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248957.156185060]: [multi_patrol] tick=47354 sim_t=4735.40 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248957.156330260]: [multi_patrol][state] tick=47354 sim_t=4735.40 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[INFO] [1785248957.256145562]: [multi_patrol][state] tick=47355 sim_t=4735.50 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=action_hold blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.10 dwell=0.00
[INFO] [1785248957.656461273]: [multi_patrol] tick=47359 sim_t=4735.90 V1 arrived A1; dwell 0.00s; load=empty phase=1
[INFO] [1785248957.656662174]: [multi_patrol][state] tick=47359 sim_t=4735.90 V1 mode=ACTIVE phase=TO_A1 action=STOP reason=wait_a1_transaction_commit blocker=-1 task=53 slot=11->-1 pending_B=64 s=3.319/3.319 rem=0.000 speed=0.000 wait=0.00 dwell=0.00
[ERROR] [1785248957.760223090]: [multi_patrol][A1 arrival blocked] V1 reached A1 without committed service ownership; egress_owner=V-1
[WARN] [1785248957.761549696]: [DIAG wedge] V0(s=2.913,rem=0.610,wait=11.1,act=0) vs V1(s=3.319,rem=0.000,wait=0.1,act=0) owner=V