-module(main_loop).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Main robot control loop — orchestrator.
%%%
%%% Owns:
%%%   • Pose (#pose{}) — updated every tick via odometry:integrate/3.
%%%   • Trajectory lifecycle state machine (idle/running/paused).
%%%   • Button edge/hold detection and cooldown timers.
%%%   • LED display.
%%%   • Velocity mux (trajectory vs manual).
%%%
%%% Lifecycle state machine (owns here):
%%%
%%% FB_Tap = F+B released after a *short* press (held < 500 ms).
%%% FB_Hold ≥ 500 ms always wins on the same gesture — the tap is suppressed
%%% when the press crosses the hold threshold, so a single long press cannot
%%% chain reset → tap action.
%%%
%%% Every → idle transition (hold, fall, traj-done, cancel-drains, CLEAR_ALL)
%%% wipes pose to {0,0,0}, clears wps_remaining AND wps_traversed, and stamps
%%% the 1 s reset cooldown. Idle is the single terminal state — there is no
%%% sticky "finished" state. Pre-flight queueing (ADD while idle) is allowed
%%% and survives a fall in idle.
%%%
%%%   idle      ──(FB_Tap, reset_cooldown_ok)──▶ running
%%%   idle      ──(FB_Hold ≥ 500 ms)──▶ idle (safety wipe + cooldown)
%%%   running   ──(FB_Tap, 500 ms cooldown)──▶ paused
%%%   running   ──(FB_Hold ≥ 500 ms)──▶ idle  (wipe + cooldown)
%%%   running   ──(Robot_Up false)──▶ idle    (wipe + cooldown, fall reset)
%%%   running   ──(traj_planner done)──▶ idle (wipe + cooldown)
%%%   running   ──(cancel drains wps_remaining)──▶ idle (wipe + cooldown)
%%%   paused    ──(FB_Tap, 500 ms cooldown)──▶ running
%%%   paused    ──(FB_Hold ≥ 500 ms)──▶ idle  (wipe + cooldown)
%%%   paused    ──(Robot_Up false)──▶ idle    (wipe + cooldown, fall reset)
%%%   paused    ──(cancel drains wps_remaining)──▶ idle (wipe + cooldown)
%%%   *         ──(CLEAR_ALL)──▶ idle         (wipe + cooldown)
%%%
%%% Non-draining cancel (wps_remaining still non-empty after cancel_last /
%%% cancel_n) does NOT transition — rebuild_path/1 runs, robot keeps cruising.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([robot_init/1, modify_frequency/1]).
-include("robot_types.hrl").

%% ─── Feature flags ───────────────────────────────────────────────────────────
-define(ENABLE_TRAJECTORY, true).    %% trajectory layer
-define(USE_KALMAN,        true).    %% true=Kalman tilt filter, false=complementary

%% ─── Angles & velocity limits ────────────────────────────────────────────────
-define(RAD_TO_DEG, 180.0/math:pi()).
-define(DEG_TO_RAD, math:pi()/180.0).
-define(ADV_V_MAX,   30.0).          %% cm/s — manual forward cap
-define(TURN_V_MAX,  80.0).          %% cm/s — manual turn cap
-define(COEF_FILTER, 0.667).

%% ─── Button timing (ms) ──────────────────────────────────────────────────────
-define(TRAJ_PAUSE_COOLDOWN_MS,  500).   %% between pause/resume taps
-define(TRAJ_HOLD_MS,            500).   %% FB held this long → reset
-define(TRAJ_RESET_COOLDOWN_MS, 1000).   %% after reset, FB ignored for this long

%% ─── Legacy logging ──────────────────────────────────────────────────────────
-define(LOG_DURATION, 15000).

%% wp injection fsm - phase 2 just prints + leds, no traj stuff yet
-record(cmd_rx, {
    state               = idle :: idle
                                | want_x_hi  | want_x_hi_to_lo
                                | want_x_lo  | want_x_to_y
                                | want_y_hi  | want_y_hi_to_lo
                                | want_y_lo  | want_commit
                                | want_n     | want_n_commit,
    x_acc               = 0    :: 0..1023,
    y_acc               = 0    :: 0..1023,
    n_acc               = 0    :: 0..31,
    last_byte           = -1   :: integer(),   %% raw byt last seen
    stable_count        = 0    :: non_neg_integer(),
    last_committed_byte = -1   :: integer(),   %% byt we already accepted, ignore til it changes
    led_pulse_kind      = none :: none | cmd_frame_ok | cmd_committed | cmd_error,
    led_pulse_left      = 0    :: non_neg_integer()
}).

%% ─── Main state record ────────────────────────────────────────────────────────
-record(rstate, {
    pose                = #pose{}   :: #pose{},
    lifecycle           = idle      :: idle | running | paused,
    lifecycle_entered_ms = 0        :: non_neg_integer(),

    fb_held_ms          = 0         :: non_neg_integer(),
    lr_held_ms          = 0         :: non_neg_integer(),

    prev_fb_combo       = false     :: boolean(),
    prev_lr_combo       = false     :: boolean(),

    last_traj_action_ms = 0         :: non_neg_integer(),
    last_reset_ms       = 0         :: non_neg_integer(),

    traj_wps_left       = 0         :: non_neg_integer(),
    traj_substate       = idle      :: atom(),
    traj_counter        = 0         :: non_neg_integer(),

    traj                = undefined :: term(),

    robot_state         = rest      :: atom(),
    robot_up            = false     :: boolean(),

    %% Latched on every PROTO=0 (drive) byte, replayed during PROTO=1 bursts so
    %% protocol payloads cannot ghost-press Get_Up / F / B / L / R.
    last_drive_byte     = 0         :: 0..255,

    cmd_rx              = #cmd_rx{} :: #cmd_rx{}
}).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Initialization
%%% ═══════════════════════════════════════════════════════════════════════════

robot_init(Hera_pid) ->
    process_flag(priority, max),
    T0 = erlang:system_time() / 1.0e6,

    ets:new(variables, [set, public, named_table]),
    ets:insert(variables, {"Freq_Goal", 300.0}),

    io:format("[Robot] Calibrating... Do not move the pmod_nav!~n"),
    led_control:accel_calibrating(),
    [Gx0, Gy0, _Gz0] = calibrate(),
    io:format("[Robot] Gyro bias: Gx0=~.4f Gy0=~.4f dps~n", [Gx0, Gy0]),
    io:format("[Robot] Done calibrating~n"),
    led_control:idle(),

    X0     = mat:matrix([[0], [0]]),
    P0     = mat:matrix([[0.1, 0], [0, 0.1]]),
    I2Cbus = grisp_i2c:open(i2c1),

    Pid_Speed     = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),
    io:format("[Robot] Pid speed=~p  stability=~p~n", [Pid_Speed, Pid_Stability]),
    io:format("[Robot] Starting robot.~n"),

    %% init traj at boot so the wire-protocol decoder always has a live
    %% #traj_state{} to mutate, laptop can queue waypoints even while idle.
    %% lifecycle_step no longer wipes this on transitions, only Clear_All over
    %% the wire empties it (FB_Hold reset preserves the array now)
    RS = #rstate{traj = traj_planner:init()},

    robot_main(T0, Hera_pid, {T0, X0, P0}, I2Cbus,
               {0, T0, []},
               {Gy0, Gx0, 0.0, 0.0},
               {Pid_Speed, Pid_Stability},
               {0.0, 0.0},
               {0, 0, 200.0, T0},
               RS).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Main loop
%%% ═══════════════════════════════════════════════════════════════════════════

robot_main(Start_Time, Hera_pid,
           {T0, X0, P0}, I2Cbus,
           {Logging, Log_End, Log_List},
           {Gy0, Gx0, Angle_Complem, Angle_Rate},
           {Pid_Speed, Pid_Stability},
           {Adv_V_Ref, Turn_V_Ref},
           {N, Freq, Mean_Freq, T_End},
           RS) ->

    %%─── 1. Time ─────────────────────────────────────────────────────────────
    T1    = erlang:system_time() / 1.0e6,
    Dt    = (T1 - T0) / 1000.0,
    Dt_ms = max(1, round(T1 - T0)),
    Now   = erlang:system_time(millisecond),

    %%─── 2. IMU ──────────────────────────────────────────────────────────────
    [Gy, Ax, Az] = pmod_nav:read(acc,
        [out_y_g, out_x_xl, out_z_xl], #{g_unit => dps}),

    %%─── 3. ESP32 ────────────────────────────────────────────────────────────
    [<<SL1,SL2,SR1,SR2,CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
    [Speed_L, Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
    Speed = (Speed_L + Speed_R) / 2.0,

    %% Drive-bit gating during proto bursts:
    %%   bit 7 (Arm_Ready)        - always live (ESP owns it)
    %%   bit 4 (Get_Up)           - latched from last drive byte (preserve stand)
    %%   bits 3..0 (F/B/L/R)      - forced zero (Python suppresses these on tx)
    %%   bits 6,5 (Switch/Test)   - decoded but ignored (filter is ?USE_KALMAN,
    %%                              Hera log trigger removed)
    Proto_Bit = (CtrlByte band ?CMD_PROTO_MASK) =/= 0,
    Drive_Byte = if Proto_Bit ->
                        (CtrlByte band 16#80)                          %% live Arm_Ready
                        bor (RS#rstate.last_drive_byte band 16#10);    %% latched Get_Up
                    true ->
                        CtrlByte
                 end,
    [Arm_Ready, _Switch_unused, _Test_unused, Get_Up, Forward, Backward, Left, Right] =
        hera_com:get_bits(Drive_Byte),
    Last_Drive_Byte_New = if Proto_Bit -> RS#rstate.last_drive_byte;
                             true      -> CtrlByte
                          end,

    %%─── 3b. wp inject fsm + traj-array mutation (phases 2+5) ───────────────
    %% b6=1 -> protocol byt, else its just normal drive
    %% runs regardless of lifecycle so op can q waypoints anytime
    %% phase 5, dispatch produces a CmdAction, we apply it to the live traj
    %% below. phase 6 (deferred), spline rebuild on each mutation
    HadRemainingBefore = traj_planner:has_remaining(RS#rstate.traj),
    {CmdAction, CmdRx1} = cmd_rx_step(CtrlByte, RS#rstate.cmd_rx),
    {Traj_Raw, Pose_Raw, LC_FromCmd} =
        apply_cmd_action(CmdAction, RS#rstate.traj,
                         RS#rstate.pose, RS#rstate.lifecycle),
    HasRemainingAfter = traj_planner:has_remaining(Traj_Raw),

    %% Wire-driven lifecycle overrides — every reset destination is idle now:
    %%   CLEAR_ALL (any state)                  → idle, full wipe + cooldown
    %%   running/paused + cancel drained list   → idle, full wipe + cooldown
    %% Anything else passes through untouched. Non-draining cancel and ADD
    %% rebuild the spline inside apply_cmd_action but don't transition.
    {LC_Override, Traj0, Pose0, ResetStamp} =
        case CmdAction of
            clear_all ->
                traj_planner:print_transition(RS#rstate.lifecycle, idle, Traj_Raw),
                {idle, Traj_Raw, #pose{}, Now};
            _ when HadRemainingBefore, not HasRemainingAfter,
                   (LC_FromCmd =:= running orelse LC_FromCmd =:= paused) ->
                Traj_Wiped = traj_planner:clear_remaining(Traj_Raw),
                traj_planner:print_transition(LC_FromCmd, idle, Traj_Wiped),
                {idle, Traj_Wiped, #pose{}, Now};
            _ ->
                {LC_FromCmd, Traj_Raw, Pose_Raw, keep}
        end,
    Last_Reset_Override =
        case ResetStamp of
            keep -> RS#rstate.last_reset_ms;
            T    -> T
        end,

    %%─── 4. Tilt — only the filter selected by ?USE_KALMAN runs ──────────────
    %% Selection is now a compile-time flag, not a remote-toggled bit. The
    %% original `Switch` wire-bit doubled as PROTO and made the filter flip
    %% mid-waypoint upload; that's gone.
    Angle_Acc_Val = math:atan(Az /(-Ax)) * ?RAD_TO_DEG,
    {X1, P1, Angle_Kalman, Angle_Complem_New, Angle_Rate_New, Angle} =
        case ?USE_KALMAN of
            true ->
                {Xk, Pk} = kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0),
                [Th_K, _W] = mat:to_array(Xk),
                Ak = Th_K * ?RAD_TO_DEG,
                {Xk, Pk, Ak, Angle_Complem, Angle_Rate, Ak};
            false ->
                K = 1.25 / (1.25 + (1.0 / Mean_Freq)),
                {Acn, Arn} =
                    complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}),
                {X0, P0, 0.0, Acn, Arn, Angle_Complem}
        end,

    %%─── 5. Robot_Up ─────────────────────────────────────────────────────────
    Robot_Up = RS#rstate.robot_up,
    Robot_Up_New =
        if      Robot_Up andalso (abs(Angle) > 20) -> false;
           not  Robot_Up andalso (abs(Angle) < 18) -> true;
           true                                     -> Robot_Up
        end,

    %%─── 6. Button combos: edges + hold accumulation ─────────────────────────
    FB_Combo = Forward andalso Backward,
    LR_Combo = Left    andalso Right,
    Prev_FB  = RS#rstate.prev_fb_combo,
    Prev_LR  = RS#rstate.prev_lr_combo,

    FB_Falling = input_combo:falling_edge(FB_Combo, Prev_FB),

    FB_Base  = input_combo:hold_check(FB_Combo, Prev_FB, RS#rstate.fb_held_ms),
    FB_Held  = if FB_Combo -> FB_Base + Dt_ms; true -> 0 end,
    LR_Base  = input_combo:hold_check(LR_Combo, Prev_LR, RS#rstate.lr_held_ms),
    LR_Held  = if LR_Combo -> LR_Base + Dt_ms; true -> 0 end,

    FB_Hold_500 = FB_Held >= ?TRAJ_HOLD_MS,
    %% Tap = release within the hold threshold. fb_held_ms holds the duration
    %% RIGHT BEFORE the falling-edge tick (hold_check returns 0 on release, so
    %% FB_Held is already 0 here — we need the stored value). A release after
    %% FB_Hold_500 has fired will have fb_held_ms >= ?TRAJ_HOLD_MS and is
    %% suppressed, so the long-hold reset cannot chain into a tap.
    FB_Tap = FB_Falling andalso (RS#rstate.fb_held_ms < ?TRAJ_HOLD_MS),

    %%─── 7. Odometry (running only) ──────────────────────────────────────────
    %% pose output is only consumed by traj_planner and csv_logger, both gated
    %% on lifecycle=running. lifecycle_step resets pose to zero on idle→running,
    %% so skipping integration outside running is safe.
    %% Pose0 came from apply_cmd_action, normally same as RS.pose but Clear_All
    %% zeroes it (doesnt drive home, just clears the values)
    Pose1 = case RS#rstate.lifecycle of
        running -> odometry:integrate({Speed_L, Speed_R}, Dt, Pose0);
        _       -> Pose0
    end,

    %%─── 8. Lifecycle state machine ──────────────────────────────────────────
    RS1 = RS#rstate{
        pose          = Pose1,
        robot_up      = Robot_Up_New,
        fb_held_ms    = FB_Held,
        lr_held_ms    = LR_Held,
        prev_fb_combo = FB_Combo,
        prev_lr_combo = LR_Combo,
        cmd_rx        = CmdRx1,
        traj          = Traj0,
        lifecycle     = LC_Override,
        last_reset_ms = Last_Reset_Override
    },
    {LC_New, TrajState1, Pose2, RS2} =
        lifecycle_step(RS1, FB_Tap, FB_Hold_500, Now),

    %%─── 10. Trajectory controller (only when running) ───────────────────────
    {Adv_V_Traj, Turn_V_Traj, TrajFinished, WpsLeft, TrajSS, TrajCnt, TrajDistWP, TrajState2} =
        case LC_New of
            running when ?ENABLE_TRAJECTORY, TrajState1 =/= undefined ->
                Sensors = #{speed => Speed, dt => Dt},
                {TOut, TS2} = traj_planner:step(Sensors, Pose2, TrajState1),
                #{adv_v    := AV,
                  turn_v   := TV,
                  finished := Fin,
                  wps_left := WL,
                  substate := SS,
                  debug    := #{counter := Cnt, dist_wp := DWP}} = TOut,
                {AV, TV, Fin, WL, SS, Cnt, DWP, TS2};
            _ ->
                {0.0, 0.0, false, 0, idle, 0, 0.0, TrajState1}
        end,

    %% Traj completion lands directly in idle (no sticky finished state).
    %% Full wipe + cooldown keeps idle's invariant: pose=0, traj cleared.
    {LC_Final, RS2_Final, TrajState_Final} =
        if TrajFinished andalso LC_New =:= running ->
               io:format("===LOG_END_TRAJ===~n"),
               traj_planner:print_transition(running, idle, TrajState2),
               RS_Idle = go_idle(RS2, TrajState2, Now),
               {idle, RS_Idle, RS_Idle#rstate.traj};
           true ->
               {LC_New, RS2, TrajState2}
        end,

    %%─── 11. Velocity mux ────────────────────────────────────────────────────
    Man_Adv  = speed_ref(Forward, Backward),
    Man_Turn = turn_ref(Left, Right),
    {Adv_V_Goal, Turn_V_Goal} =
        case LC_Final of
            running -> {Adv_V_Traj + Man_Adv, Turn_V_Traj + Man_Turn};
            _       -> {Man_Adv, Man_Turn}
        end,

    %%─── 12. Stability controller ────────────────────────────────────────────
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} =
        stability_engine:controller(
            {Dt, Angle, Speed},
            {Pid_Speed, Pid_Stability},
            {Adv_V_Goal, Adv_V_Ref},
            {Turn_V_Goal, Turn_V_Ref}),

    %%─── 13. Robot_State machine ─────────────────────────────────────────────
    Next_Robot_State =
        next_robot_state(RS2_Final#rstate.robot_state, Robot_Up_New, Get_Up,
                         Angle, Arm_Ready),
    {Power, Freeze, Extend, Robot_Up_Bit} = state_outputs(Next_Robot_State),

    %%─── 14. ESP32 output ────────────────────────────────────────────────────
    F_B = if Angle > 0.0 -> 1; true -> 0 end,
    Output_Byte = get_byte([Power, Freeze, Extend, Robot_Up_Bit, F_B, 0, 0, 0]),
    [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),
    grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]),

    %%─── 15. LED ─────────────────────────────────────────────────────────────
    update_leds(LC_Final, RS2_Final#rstate.last_reset_ms, Now, RS2_Final#rstate.cmd_rx),

    %%─── 16. CSV logging ─────────────────────────────────────────────────────
    if LC_Final =:= running ->
           case csv_logger:append() of
               logged ->
                   #pose{x = PX, y = PY, theta = PT} = Pose2,
                   csv_logger:emit_serial(#{t_ms       => Now,
                                            lifecycle  => LC_Final,
                                            substate   => TrajSS,
                                            x          => PX,
                                            y          => PY,
                                            speed      => Speed,
                                            adv_v      => Adv_V_Goal,
                                            turn_v     => Turn_V_Goal,
                                            dist_wp    => TrajDistWP,
                                            wps_left   => WpsLeft,
                                            yaw_odo    => PT});
               skipped -> ok
           end;
       true -> ok
    end,

    %%─── 17. Frequency + shell-query plumbing ────────────────────────────────
    %% The legacy Hera log path (Test bit -> 15s buffered log) is removed.
    %% csv_logger covers run logging. The Log_* triple is kept as a no-op so
    %% the recursive signature is unchanged.
    {N_New, Freq_New, Mean_Freq_New} = frequency_computation(Dt, N, Freq, Mean_Freq),
    Log_End_New  = Log_End,
    Logging_New  = Logging,
    Log_List_New = Log_List,
    handle_messages(T1, Start_Time, Dt, Gy, Acc, CtrlByte,
                    Angle_Acc_Val, Angle_Kalman, Angle_Complem,
                    Adv_V_Ref, Adv_V_Ref_New, Turn_V_Ref_New, Speed,
                    Log_List),

    %%─── 18. Rate limiter ────────────────────────────────────────────────────
    T2 = erlang:system_time() / 1.0e6,
    [{_, Freq_Goal}] = ets:lookup(variables, "Freq_Goal"),
    Delay_Goal = 1.0 / Freq_Goal * 1000.0,
    if T2 - T_End < Delay_Goal -> wait(Delay_Goal - (T2 - T1)); true -> ok end,
    T_End_New = erlang:system_time() / 1.0e6,

    %%─── 19. Recurse ─────────────────────────────────────────────────────────
    %% RS2_Final carries the go_idle wipe (pose=0, traj cleared, cooldown
    %% stamped) when traj-done fired this tick, otherwise it equals RS2.
    RS_Next = RS2_Final#rstate{
        lifecycle       = LC_Final,
        traj            = TrajState_Final,
        robot_state     = Next_Robot_State,
        robot_up        = Robot_Up_New,
        traj_wps_left   = WpsLeft,
        traj_substate   = TrajSS,
        traj_counter    = TrajCnt,
        last_drive_byte = Last_Drive_Byte_New
    },
    robot_main(Start_Time, Hera_pid,
               {T1, X1, P1}, I2Cbus,
               {Logging_New, Log_End_New, Log_List_New},
               {Gy0, Gx0, Angle_Complem_New, Angle_Rate_New},
               {Pid_Speed, Pid_Stability},
               {Adv_V_Ref_New, Turn_V_Ref_New},
               {N_New, Freq_New, Mean_Freq_New, T_End_New},
               RS_Next).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Lifecycle state machine
%%% ═══════════════════════════════════════════════════════════════════════════

lifecycle_step(RS, FB_Tap, FB_Hold_500, Now) ->
    LC    = RS#rstate.lifecycle,
    RobUp = RS#rstate.robot_up,
    TS    = RS#rstate.traj,

    Traj_OK   = (Now - RS#rstate.last_traj_action_ms) >= ?TRAJ_PAUSE_COOLDOWN_MS,
    Reset_OK  = (Now - RS#rstate.last_reset_ms)       >= ?TRAJ_RESET_COOLDOWN_MS,

    case LC of
        %% Note: idle intentionally has no fall (¬RobUp) handler. Pre-flight
        %% queued WPs (ADD before first FB_Tap) survive being knocked over.
        idle ->
            if FB_Tap andalso Reset_OK ->
                   %% Idle is started by tap only — no instant-on-press. Waypoints
                   %% are preserved (only Clear_All / hold-reset wipes them).
                   csv_logger:reset(),
                   traj_planner:print_transition(idle, running, TS),
                   RS2 = RS#rstate{lifecycle            = running,
                                   lifecycle_entered_ms = Now,
                                   last_traj_action_ms  = Now,
                                   pose                 = #pose{}},
                   {running, TS, RS2#rstate.pose, RS2};
               FB_Hold_500 andalso Reset_OK ->
                   %% Safety wipe — Reset_OK gates against repeat-firing while
                   %% the operator keeps holding past 500 ms.
                   traj_planner:print_transition(idle, idle, TS),
                   RS2 = go_idle(RS, TS, Now),
                   {idle, RS2#rstate.traj, RS2#rstate.pose, RS2};
               true ->
                   {idle, TS, RS#rstate.pose, RS}
            end;

        running ->
            if not RobUp ->
                   traj_planner:print_transition(running, idle, TS),
                   RS2 = go_idle(RS, TS, Now),
                   {idle, RS2#rstate.traj, RS2#rstate.pose, RS2};
               FB_Hold_500 ->
                   traj_planner:print_transition(running, idle, TS),
                   RS2 = go_idle(RS, TS, Now),
                   {idle, RS2#rstate.traj, RS2#rstate.pose, RS2};
               FB_Tap andalso Traj_OK ->
                   traj_planner:print_transition(running, paused, TS),
                   RS2 = RS#rstate{lifecycle            = paused,
                                   lifecycle_entered_ms = Now,
                                   last_traj_action_ms  = Now},
                   {paused, TS, RS2#rstate.pose, RS2};
               true ->
                   {running, TS, RS#rstate.pose, RS}
            end;

        paused ->
            if not RobUp ->
                   traj_planner:print_transition(paused, idle, TS),
                   RS2 = go_idle(RS, TS, Now),
                   {idle, RS2#rstate.traj, RS2#rstate.pose, RS2};
               FB_Hold_500 ->
                   traj_planner:print_transition(paused, idle, TS),
                   RS2 = go_idle(RS, TS, Now),
                   {idle, RS2#rstate.traj, RS2#rstate.pose, RS2};
               FB_Tap andalso Traj_OK ->
                   traj_planner:print_transition(paused, running, TS),
                   RS2 = RS#rstate{lifecycle            = running,
                                   lifecycle_entered_ms = Now,
                                   last_traj_action_ms  = Now},
                   {running, TS, RS2#rstate.pose, RS2};
               true ->
                   {paused, TS, RS#rstate.pose, RS}
            end
    end.

%% Single-source-of-truth →idle helper. Every reset destination (hold, fall,
%% traj-done, cancel-drain, CLEAR_ALL) routes through here: pose to origin,
%% traj fully wiped (including last_pose — see traj_planner:clear_remaining/1),
%% 1 s reset cooldown stamped so the LED + Reset_OK gate stay consistent.
go_idle(RS, TS, Now) ->
    TS2 = traj_planner:clear_remaining(TS),
    RS#rstate{lifecycle     = idle,
              last_reset_ms = Now,
              pose          = #pose{},
              traj          = TS2}.

%%% ═══════════════════════════════════════════════════════════════════════════
%%% LED display
%%% ═══════════════════════════════════════════════════════════════════════════

update_leds(Lifecycle, Last_Reset_Ms, Now, CmdRx) ->
    In_Cooldown = (Now - Last_Reset_Ms) < ?TRAJ_RESET_COOLDOWN_MS,
    case Lifecycle of
        idle when In_Cooldown -> led_control:traj_reset_cooldown();
        idle     -> led_control:idle();
        running  -> led_control:traj_running();
        paused   -> led_control:traj_paused()
    end,
    %% override led1 if cmd-rx fsm has somthing to show
    %% led2 stays w/ lifecycle so we can see both at the same time
    case cmd_led_override(CmdRx) of
        none           -> ok;
        cmd_active     -> led_control:cmd_active();
        cmd_frame_ok   -> led_control:cmd_frame_ok();
        cmd_committed  -> led_control:cmd_committed();
        cmd_error      -> led_control:cmd_error()
    end.

cmd_led_override(#cmd_rx{led_pulse_left = N, led_pulse_kind = K}) when N > 0, K =/= none -> K;
cmd_led_override(#cmd_rx{state = idle}) -> none;
cmd_led_override(#cmd_rx{}) -> cmd_active.

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Robot state machine (unchanged from original)
%%% ═══════════════════════════════════════════════════════════════════════════

next_robot_state(Robot_State, Robot_Up, Get_Up, Angle, Arm_Ready) ->
    case Robot_State of
        rest ->
            if Get_Up -> raising; true -> rest end;
        raising ->
            if Robot_Up  -> stand_up;
               not Get_Up -> soft_fall;
               true       -> raising
            end;
        stand_up ->
            if not Get_Up -> wait_for_extend;
               not Robot_Up -> rest;
               true        -> stand_up
            end;
        wait_for_extend ->
            prepare_arms;
        prepare_arms ->
            if Arm_Ready  -> free_fall;
               Get_Up     -> stand_up;
               not Robot_Up -> rest;
               true        -> prepare_arms
            end;
        free_fall ->
            if abs(Angle) > 10 -> wait_for_retract; true -> free_fall end;
        wait_for_retract ->
            soft_fall;
        soft_fall ->
            if Arm_Ready -> rest;
               Get_Up    -> raising;
               true      -> soft_fall
            end
    end.

state_outputs(State) ->
    case State of
        rest             -> {0, 0, 0, 0};
        raising          -> {1, 0, 1, 0};
        stand_up         -> {1, 0, 0, 1};
        wait_for_extend  -> {1, 0, 1, 1};
        prepare_arms     -> {1, 0, 1, 1};
        free_fall        -> {1, 1, 1, 1};
        wait_for_retract -> {1, 0, 0, 0};
        soft_fall        -> {1, 0, 0, 0}
    end.

%%% 
%%% wp ota inject fsm  (rx + print + led only)
%%% 
%%%
%% each tick feeds CtrlByte to cmd_rx_step/2:
%%   1. tick down any in-flight led pulse counter
%%   2. count consecutive same-byt reads (debounce)
%%   3. if fresh stable PROTO=1 byt -> run thru fsm (advance/commit/err)
%%   4. if stable PROTO=0 byt -> drop commit gate so repeated commands
%%      (eg cancel_last twice) get re-accepted insted of dedup'd
%%
%% effects rn = io:format prints + led pulse state on #cmd_rx{}
%% no traj mutation yet, comes phases 4-5

%% Returns {Action, NewCR}. Action is consumed by apply_cmd_action/3 below.
%%   Action ::= none
%%            | {append, DX, DY}
%%            | cancel_last
%%            | {cancel_n, N}
%%            | clear_all
cmd_rx_step(CtrlByte, CR = #cmd_rx{}) ->
    CR1 = tick_led_pulse(CR),
    Stable_New =
        if CtrlByte =:= CR1#cmd_rx.last_byte -> CR1#cmd_rx.stable_count + 1;
           true                              -> 1
        end,
    CR2 = CR1#cmd_rx{last_byte = CtrlByte, stable_count = Stable_New},
    Proto    = (CtrlByte band ?CMD_PROTO_MASK) =/= 0,
    Stable   = Stable_New >= 2,
    Changed  = CtrlByte =/= CR2#cmd_rx.last_committed_byte,
    case {Proto, Stable, Changed} of
        {true, true, true} ->
            CR3     = CR2#cmd_rx{last_committed_byte = CtrlByte},
            Kind    = (CtrlByte band ?CMD_KIND_MASK) =/= 0,
            Payload = CtrlByte band ?CMD_PAYLOAD_MASK,
            dispatch_frame(Kind, Payload, CR3);
        {false, true, _} ->
            %% stable drive byt - drop the gate so next proto byt counts fresh
            {none, CR2#cmd_rx{last_committed_byte = -1}};
        _ ->
            {none, CR2}
    end.

%% ABORT in idle = silent noop, else reset w/ err blink
dispatch_frame(false, ?CMD_CTRL_ABORT, CR = #cmd_rx{state = idle}) ->
    {none, CR};
dispatch_frame(false, ?CMD_CTRL_ABORT, CR) ->
    {none, pulse(reset_cmd_rx(CR), cmd_error, ?CMD_LED_PULSE_ERROR)};

%% from idle a ctrl frame kicks off a transaction
dispatch_frame(false, ?CMD_CTRL_ADD_HEADER, CR = #cmd_rx{state = idle}) ->
    {none, pulse(CR#cmd_rx{state = want_x_hi}, cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};
dispatch_frame(false, ?CMD_CTRL_CANCEL_LAST, CR = #cmd_rx{state = idle}) ->
    io:format("RX_CANCEL_LAST~n"),
    {cancel_last,
     pulse(reset_cmd_rx(CR), cmd_committed, ?CMD_LED_PULSE_COMMITTED)};
dispatch_frame(false, ?CMD_CTRL_CLEAR_ALL, CR = #cmd_rx{state = idle}) ->
    io:format("RX_CLEAR_ALL~n"),
    {clear_all,
     pulse(reset_cmd_rx(CR), cmd_committed, ?CMD_LED_PULSE_COMMITTED)};
dispatch_frame(false, ?CMD_CTRL_CANCEL_N_HDR, CR = #cmd_rx{state = idle}) ->
    {none, pulse(CR#cmd_rx{state = want_n}, cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};

%% ADD seq: data HI_TO_LO data X_TO_Y data HI_TO_LO data ADD_COMMIT
dispatch_frame(true, Payload, CR = #cmd_rx{state = want_x_hi}) ->
    {none, pulse(CR#cmd_rx{state = want_x_hi_to_lo, x_acc = Payload bsl 5},
                 cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};
dispatch_frame(false, ?CMD_CTRL_HI_TO_LO, CR = #cmd_rx{state = want_x_hi_to_lo}) ->
    {none, pulse(CR#cmd_rx{state = want_x_lo}, cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};
dispatch_frame(true, Payload, CR = #cmd_rx{state = want_x_lo}) ->
    {none, pulse(CR#cmd_rx{state = want_x_to_y, x_acc = CR#cmd_rx.x_acc bor Payload},
                 cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};
dispatch_frame(false, ?CMD_CTRL_X_TO_Y, CR = #cmd_rx{state = want_x_to_y}) ->
    {none, pulse(CR#cmd_rx{state = want_y_hi}, cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};
dispatch_frame(true, Payload, CR = #cmd_rx{state = want_y_hi}) ->
    {none, pulse(CR#cmd_rx{state = want_y_hi_to_lo, y_acc = Payload bsl 5},
                 cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};
dispatch_frame(false, ?CMD_CTRL_HI_TO_LO, CR = #cmd_rx{state = want_y_hi_to_lo}) ->
    {none, pulse(CR#cmd_rx{state = want_y_lo}, cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};
dispatch_frame(true, Payload, CR = #cmd_rx{state = want_y_lo}) ->
    {none, pulse(CR#cmd_rx{state = want_commit, y_acc = CR#cmd_rx.y_acc bor Payload},
                 cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};
dispatch_frame(false, ?CMD_CTRL_ADD_COMMIT, CR = #cmd_rx{state = want_commit}) ->
    DX = CR#cmd_rx.x_acc - ?CMD_SIGN_OFFSET,
    DY = CR#cmd_rx.y_acc - ?CMD_SIGN_OFFSET,
    case in_offset_range(DX) andalso in_offset_range(DY) of
        true ->
            io:format("RX_WP: dx=~p dy=~p~n", [DX, DY]),
            {{append, DX, DY},
             pulse(reset_cmd_rx(CR), cmd_committed, ?CMD_LED_PULSE_COMMITTED)};
        false ->
            io:format("RX_WP_ERR: dx=~p dy=~p (out of range)~n", [DX, DY]),
            {none, pulse(reset_cmd_rx(CR), cmd_error, ?CMD_LED_PULSE_ERROR)}
    end;

%% CANCEL_N seq: data, CANCEL_N_COMMIT
dispatch_frame(true, Payload, CR = #cmd_rx{state = want_n}) ->
    {none, pulse(CR#cmd_rx{state = want_n_commit, n_acc = Payload},
                 cmd_frame_ok, ?CMD_LED_PULSE_FRAME_OK)};
dispatch_frame(false, ?CMD_CTRL_CANCEL_N_COMMIT, CR = #cmd_rx{state = want_n_commit}) ->
    N = CR#cmd_rx.n_acc,
    io:format("RX_CANCEL_N: ~p~n", [N]),
    {{cancel_n, N},
     pulse(reset_cmd_rx(CR), cmd_committed, ?CMD_LED_PULSE_COMMITTED)};

%% anything else in any state = proto err, reset + err blink
dispatch_frame(_Kind, _Payload, CR) ->
    {none, pulse(reset_cmd_rx(CR), cmd_error, ?CMD_LED_PULSE_ERROR)}.

%% Apply a decoded action to the live #traj_state{} and print the resulting
%% array. Returns {NewTS, NewPose, NewLC} so the caller can also pick up a
%% pose reset (Clear_All zeroes pose values). Spline rebuild now happens
%% inside traj_planner's mutators (phase 6), and the finished→running
%% reactivation is owned by the lifecycle FSM in lifecycle_step/4 — so no
%% lifecycle hints are emitted from here anymore.
%%
%% Reference point for ADD is the last queued waypoint, or current pose if
%% the array is empty.
apply_cmd_action(none, TS, Pose, LC) -> {TS, Pose, LC};

apply_cmd_action({append, DX, DY}, TS, Pose, LC) ->
    {LastX, LastY} =
        case traj_planner:last_waypoint_abs(TS) of
            empty    -> {Pose#pose.x, Pose#pose.y};
            {LX, LY} -> {LX, LY}
        end,
    AbsX = LastX + float(DX),
    AbsY = LastY + float(DY),
    TS2 = traj_planner:append_waypoint(AbsX, AbsY, TS),
    traj_planner:print_waypoints(TS2),
    {TS2, Pose, LC};

apply_cmd_action(cancel_last, TS, Pose, LC) ->
    TS2 = traj_planner:cancel_last(TS),
    traj_planner:print_waypoints(TS2),
    {TS2, Pose, LC};

apply_cmd_action({cancel_n, N}, TS, Pose, LC) ->
    TS2 = traj_planner:cancel_last_n(N, TS),
    traj_planner:print_waypoints(TS2),
    {TS2, Pose, LC};

apply_cmd_action(clear_all, TS, _Pose, LC) ->
    TS2 = traj_planner:clear_remaining(TS),
    traj_planner:print_waypoints(TS2),
    %% Clear_All also zeroes the pose values (no driving back to 0,0,0,
    %% just clears the numbers so future ADD references start fresh)
    {TS2, #pose{}, LC}.

reset_cmd_rx(CR) ->
    %% keep debounce fields, just wipe fsm state + accs
    CR#cmd_rx{state = idle, x_acc = 0, y_acc = 0, n_acc = 0}.

pulse(CR, Kind, Ticks) ->
    CR#cmd_rx{led_pulse_kind = Kind, led_pulse_left = Ticks}.

tick_led_pulse(CR = #cmd_rx{led_pulse_left = 0}) -> CR;
tick_led_pulse(CR = #cmd_rx{led_pulse_left = 1}) ->
    CR#cmd_rx{led_pulse_left = 0, led_pulse_kind = none};
tick_led_pulse(CR = #cmd_rx{led_pulse_left = N}) ->
    CR#cmd_rx{led_pulse_left = N - 1}.

in_offset_range(V) ->
    V >= -?CMD_MAX_OFFSET andalso V =< ?CMD_MAX_OFFSET.

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Internal helpers
%%% ═══════════════════════════════════════════════════════════════════════════

calibrate() ->
    N = 500,
    Data = [list_to_tuple(pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g]))
            || _ <- lists:seq(1, N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N].

kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0) ->
    R  = mat:matrix([[3.0, 0.0], [0, 3.0e-6]]),
    Q  = mat:matrix([[3.0e-5, 0.0], [0.0, 10.0]]),
    F  = fun(X) -> [Th,W] = mat:to_array(X), mat:matrix([[Th+Dt*W],[W]]) end,
    Jf = fun(X) -> [_,_]  = mat:to_array(X), mat:matrix([[1,Dt],[0,1]]) end,
    H  = fun(X) -> [Th,W] = mat:to_array(X), mat:matrix([[Th],[W]])     end,
    Jh = fun(X) -> [_,_]  = mat:to_array(X), mat:matrix([[1,0],[0,1]])  end,
    Z  = mat:matrix([[math:atan(Az / (-Ax))], [(Gy - Gy0) * ?DEG_TO_RAD]]),
    kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z).

complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}) ->
    Rate_New  = (Gy - Gy0) * ?COEF_FILTER + Angle_Rate * (1 - ?COEF_FILTER),
    Delta_Gyr = Rate_New * Dt,
    Angle_Acc = math:atan(Az / (-Ax)) * 180 / math:pi(),
    {(Angle_Complem + Delta_Gyr) * K + Angle_Acc * (1 - K), Rate_New}.

speed_ref(Forward, Backward) ->
    if Forward andalso Backward -> 0.0;   %% combo gesture — not a direction
       Forward                  -> ?ADV_V_MAX;
       Backward                 -> -?ADV_V_MAX;
       true                     -> 0.0
    end.

turn_ref(Left, Right) ->
    if Left andalso Right -> 0.0;         %% combo gesture — not a direction
       Right              -> ?TURN_V_MAX;
       Left               -> -?TURN_V_MAX;
       true               -> 0.0
    end.

frequency_computation(Dt, N, Freq, Mean_Freq) ->
    if N =:= 100 -> {0, 0, Freq};
       true      -> {N+1, ((Freq*N) + (1/Dt)) / (N+1), Mean_Freq}
    end.

handle_messages(T1, Start_Time, Dt, Gy, Acc, CtrlByte,
                Angle_Acc, Angle_Kalman, Angle_Complem,
                Adv_V_Ref, Adv_V_Ref_New, Turn_V_Ref_New, Speed,
                Log_List) ->
    receive
        {From, log_values}  -> From ! {self(), log, Log_List};
        {From1, get_all_data} ->
            From1 ! {self(), data,
                     [T1-Start_Time, 1/Dt, Gy, Acc, CtrlByte,
                      -Angle_Acc, -Angle_Kalman, -Angle_Complem,
                      Adv_V_Ref, Adv_V_Ref_New, Turn_V_Ref_New, Speed]};
        {From1, freq} -> From1 ! {self(), 1/Dt};
        {From2, acc}  -> From2 ! {self(), Acc};
        {_, Msg}      -> io:format("[Robot] Unknown msg: ~p~n", [Msg])
    after 0 -> ok
    end.

wait(T) ->
    Tnow = erlang:system_time() / 1.0e6,
    wait_loop(Tnow, Tnow + T).
wait_loop(Tnow, Tend) when Tnow >= Tend -> ok;
wait_loop(_, Tend) -> wait_loop(erlang:system_time() / 1.0e6, Tend).

get_byte([A, B, C, D, E, F, G, H]) ->
    A*128 + B*64 + C*32 + D*16 + E*8 + F*4 + G*2 + H.

modify_frequency(Freq) ->
    ets:insert(variables, {"Freq_Goal", Freq}),
    ok.
