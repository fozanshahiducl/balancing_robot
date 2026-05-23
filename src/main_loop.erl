-module(main_loop).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Main robot control loop — orchestrator.
%%%
%%% Owns:
%%%   • Pose (#pose{}) — updated every tick via odometry:integrate/3.
%%%   • Trajectory lifecycle state machine (idle/running/paused/finished).
%%%   • Button edge/hold detection and cooldown timers.
%%%   • LED display.
%%%   • Velocity mux (trajectory vs manual).
%%%
%%% Lifecycle state machine (owns here):
%%%
%%%   idle      ──(FB_Edge, reset_cooldown_ok)──▶ running
%%%   running   ──(FB_Edge, 500 ms cooldown)──▶ paused
%%%   running   ──(FB_Hold ≥ 500 ms)──▶ idle  (1 s reset cooldown starts)
%%%   running   ──(Robot_Up false)──▶ idle    (fall reset)
%%%   running   ──(traj_planner finished)──▶ finished
%%%   paused    ──(FB_Edge, 500 ms cooldown)──▶ running
%%%   paused    ──(FB_Hold ≥ 500 ms)──▶ idle  (1 s reset cooldown)
%%%   paused    ──(Robot_Up false)──▶ idle    (fall reset)
%%%   finished  ──(FB_Edge)──▶ running        (re-init traj_planner)
%%%   finished  ──(FB_Hold ≥ 500 ms)──▶ idle
%%% ═══════════════════════════════════════════════════════════════════════════

-export([robot_init/1, modify_frequency/1]).
-include("robot_types.hrl").

%% ─── Feature flags ───────────────────────────────────────────────────────────
-define(ENABLE_TRAJECTORY, true).    %% trajectory layer

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

%% ─── Main state record ────────────────────────────────────────────────────────
-record(rstate, {
    pose                = #pose{}   :: #pose{},
    lifecycle           = idle      :: idle | running | paused | finished,
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
    robot_up            = false     :: boolean()
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
    [_Gx0, Gy0, _Gz0] = calibrate(),
    io:format("[Robot] Done calibrating~n"),
    led_control:idle(),

    X0     = mat:matrix([[0], [0]]),
    P0     = mat:matrix([[0.1, 0], [0, 0.1]]),
    I2Cbus = grisp_i2c:open(i2c1),

    Pid_Speed     = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),
    io:format("[Robot] Pid speed=~p  stability=~p~n", [Pid_Speed, Pid_Stability]),
    io:format("[Robot] Starting robot.~n"),

    RS = #rstate{},

    robot_main(T0, Hera_pid, {T0, X0, P0}, I2Cbus,
               {0, T0, []},
               {Gy0, 0.0, 0.0},
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
           {Gy0, Angle_Complem, Angle_Rate},
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
    [Arm_Ready, Switch, Test, Get_Up, Forward, Backward, Left, Right] =
        hera_com:get_bits(CtrlByte),

    %%─── 4. Tilt — only the filter selected by Switch runs ───────────────────
    %% Original code ran both Kalman and complementary every tick and discarded
    %% one via select_angle. Each Kalman pass allocates ~7 mat:matrix structs
    %% for the EKF; skipping the unused branch removes that GC pressure.
    %% Behavior preserved: select_angle(Switch, ...) used Angle_Kalman (just
    %% computed) when Switch=true, and Angle_Complem (previous tick, 1-tick lag)
    %% when Switch=false. We replicate exactly that.
    Angle_Acc_Val = math:atan(Az /(-Ax)) * ?RAD_TO_DEG,
    {X1, P1, Angle_Kalman, Angle_Complem_New, Angle_Rate_New, Angle} =
        case Switch of
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

    FB_Edge  = input_combo:rising_edge(FB_Combo, Prev_FB),

    FB_Base  = input_combo:hold_check(FB_Combo, Prev_FB, RS#rstate.fb_held_ms),
    FB_Held  = if FB_Combo -> FB_Base + Dt_ms; true -> 0 end,
    LR_Base  = input_combo:hold_check(LR_Combo, Prev_LR, RS#rstate.lr_held_ms),
    LR_Held  = if LR_Combo -> LR_Base + Dt_ms; true -> 0 end,

    FB_Hold_500 = FB_Held >= ?TRAJ_HOLD_MS,

    %%─── 7. Odometry (running only) ──────────────────────────────────────────
    %% Pose output is only consumed by traj_planner and csv_logger, both gated
    %% on lifecycle=running. lifecycle_step resets pose to zero on idle→running,
    %% so skipping integration outside running is safe.
    Pose1 = case RS#rstate.lifecycle of
        running -> odometry:integrate({Speed_L, Speed_R}, Dt, RS#rstate.pose);
        _       -> RS#rstate.pose
    end,

    %%─── 8. Lifecycle state machine ──────────────────────────────────────────
    RS1 = RS#rstate{
        pose          = Pose1,
        robot_up      = Robot_Up_New,
        fb_held_ms    = FB_Held,
        lr_held_ms    = LR_Held,
        prev_fb_combo = FB_Combo,
        prev_lr_combo = LR_Combo
    },
    {LC_New, TrajState1, Pose2, RS2} =
        lifecycle_step(RS1, FB_Edge, FB_Hold_500, Now),

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

    %% Transition to finished if traj_planner signals completion.
    LC_Final =
        if TrajFinished andalso LC_New =:= running ->
               io:format("===LOG_END_TRAJ===~n"),
               finished;
           true ->
               LC_New
        end,

    %%─── 11. Velocity mux ────────────────────────────────────────────────────
    Man_Adv  = speed_ref(Forward, Backward),
    Man_Turn = turn_ref(Left, Right),
    {Adv_V_Goal, Turn_V_Goal} =
        case LC_Final of
            running -> {Adv_V_Traj + Man_Adv, Turn_V_Traj + Man_Turn};
            paused  -> {Man_Adv,               Man_Turn};
            _       -> {Man_Adv,               Man_Turn}
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
        next_robot_state(RS2#rstate.robot_state, Robot_Up_New, Get_Up,
                         Angle, Arm_Ready),
    {Power, Freeze, Extend, Robot_Up_Bit} = state_outputs(Next_Robot_State),

    %%─── 14. ESP32 output ────────────────────────────────────────────────────
    F_B = if Angle > 0.0 -> 1; true -> 0 end,
    Output_Byte = get_byte([Power, Freeze, Extend, Robot_Up_Bit, F_B, 0, 0, 0]),
    [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),
    grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]),

    %%─── 15. LED ─────────────────────────────────────────────────────────────
    update_leds(LC_Final, RS2#rstate.last_reset_ms, Now),

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

    %%─── 17. Frequency + legacy hera logging ─────────────────────────────────
    {N_New, Freq_New, Mean_Freq_New} = frequency_computation(Dt, N, Freq, Mean_Freq),
    Log_End_New =
        if Test -> erlang:system_time() / 1.0e6 + ?LOG_DURATION; true -> Log_End end,
    Logging_New = erlang:system_time() / 1.0e6 < Log_End_New,
    handle_hera_logging(Hera_pid, Logging, Logging_New),
    Log_List_New =
        if Logging_New ->
               [[T1 - Start_Time, 1/Dt, Gy, Acc, CtrlByte,
                 -Angle_Acc_Val, -Angle_Kalman, -Angle_Complem,
                 Adv_V_Ref, Switch, Adv_V_Ref_New, Turn_V_Ref_New, Speed] | Log_List];
           true -> Log_List
        end,
    handle_messages(T1, Start_Time, Dt, Gy, Acc, CtrlByte,
                    Angle_Acc_Val, Angle_Kalman, Angle_Complem,
                    Adv_V_Ref, Switch, Adv_V_Ref_New, Turn_V_Ref_New, Speed,
                    Log_List),

    %%─── 18. Rate limiter ────────────────────────────────────────────────────
    T2 = erlang:system_time() / 1.0e6,
    [{_, Freq_Goal}] = ets:lookup(variables, "Freq_Goal"),
    Delay_Goal = 1.0 / Freq_Goal * 1000.0,
    if T2 - T_End < Delay_Goal -> wait(Delay_Goal - (T2 - T1)); true -> ok end,
    T_End_New = erlang:system_time() / 1.0e6,

    %%─── 19. Recurse ─────────────────────────────────────────────────────────
    RS_Next = RS2#rstate{
        pose         = Pose2,
        lifecycle    = LC_Final,
        traj         = TrajState2,
        robot_state  = Next_Robot_State,
        robot_up     = Robot_Up_New,
        traj_wps_left = WpsLeft,
        traj_substate = TrajSS,
        traj_counter  = TrajCnt
    },
    robot_main(Start_Time, Hera_pid,
               {T1, X1, P1}, I2Cbus,
               {Logging_New, Log_End_New, Log_List_New},
               {Gy0, Angle_Complem_New, Angle_Rate_New},
               {Pid_Speed, Pid_Stability},
               {Adv_V_Ref_New, Turn_V_Ref_New},
               {N_New, Freq_New, Mean_Freq_New, T_End_New},
               RS_Next).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Lifecycle state machine
%%% ═══════════════════════════════════════════════════════════════════════════

lifecycle_step(RS, FB_Edge, FB_Hold_500, Now) ->
    LC    = RS#rstate.lifecycle,
    RobUp = RS#rstate.robot_up,
    TS    = RS#rstate.traj,

    Traj_OK   = (Now - RS#rstate.last_traj_action_ms) >= ?TRAJ_PAUSE_COOLDOWN_MS,
    Reset_OK  = (Now - RS#rstate.last_reset_ms)       >= ?TRAJ_RESET_COOLDOWN_MS,

    case LC of
        idle ->
            if FB_Edge andalso Reset_OK ->
                   TS2 = traj_planner:init(),
                   csv_logger:reset(),
                   RS2 = RS#rstate{lifecycle            = running,
                                   lifecycle_entered_ms = Now,
                                   last_traj_action_ms  = Now,
                                   traj = TS2, pose = #pose{}},
                   {running, TS2, RS2#rstate.pose, RS2};
               true ->
                   {idle, TS, RS#rstate.pose, RS}
            end;

        running ->
            if not RobUp ->
                   RS2 = RS#rstate{lifecycle = idle, traj = undefined,
                                   last_reset_ms = Now},
                   {idle, undefined, RS2#rstate.pose, RS2};
               FB_Hold_500 ->
                   RS2 = RS#rstate{lifecycle = idle, traj = undefined,
                                   last_reset_ms = Now},
                   {idle, undefined, RS2#rstate.pose, RS2};
               FB_Edge andalso Traj_OK ->
                   RS2 = RS#rstate{lifecycle            = paused,
                                   lifecycle_entered_ms = Now,
                                   last_traj_action_ms  = Now},
                   {paused, TS, RS2#rstate.pose, RS2};
               true ->
                   {running, TS, RS#rstate.pose, RS}
            end;

        paused ->
            if not RobUp ->
                   RS2 = RS#rstate{lifecycle = idle, traj = undefined,
                                   last_reset_ms = Now},
                   {idle, undefined, RS2#rstate.pose, RS2};
               FB_Hold_500 ->
                   RS2 = RS#rstate{lifecycle = idle, traj = undefined,
                                   last_reset_ms = Now},
                   {idle, undefined, RS2#rstate.pose, RS2};
               FB_Edge andalso Traj_OK ->
                   RS2 = RS#rstate{lifecycle            = running,
                                   lifecycle_entered_ms = Now,
                                   last_traj_action_ms  = Now},
                   {running, TS, RS2#rstate.pose, RS2};
               true ->
                   {paused, TS, RS#rstate.pose, RS}
            end;

        finished ->
            if FB_Hold_500 ->
                   RS2 = RS#rstate{lifecycle = idle, traj = undefined,
                                   last_reset_ms = Now},
                   {idle, undefined, RS2#rstate.pose, RS2};
               FB_Edge ->
                   TS2 = traj_planner:init(),
                   csv_logger:reset(),
                   RS2 = RS#rstate{lifecycle            = running,
                                   lifecycle_entered_ms = Now,
                                   last_traj_action_ms  = Now,
                                   traj = TS2, pose = #pose{}},
                   {running, TS2, RS2#rstate.pose, RS2};
               true ->
                   {finished, TS, RS#rstate.pose, RS}
            end
    end.

%%% ═══════════════════════════════════════════════════════════════════════════
%%% LED display
%%% ═══════════════════════════════════════════════════════════════════════════

update_leds(Lifecycle, Last_Reset_Ms, Now) ->
    In_Cooldown = (Now - Last_Reset_Ms) < ?TRAJ_RESET_COOLDOWN_MS,
    case Lifecycle of
        idle when In_Cooldown -> led_control:traj_reset_cooldown();
        idle     -> led_control:idle();
        running  -> led_control:traj_running();
        paused   -> led_control:traj_paused();
        finished -> led_control:traj_finished()
    end.

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

handle_hera_logging(Hera_pid, Logging, Logging_New) ->
    if not Logging andalso Logging_New ->
           Hera_pid ! {self(), start_log};
       Logging andalso not Logging_New ->
           led_control:idle(),
           Hera_pid ! {self(), stop_log};
       true -> ok
    end.

handle_messages(T1, Start_Time, Dt, Gy, Acc, CtrlByte,
                Angle_Acc, Angle_Kalman, Angle_Complem,
                Adv_V_Ref, Switch, Adv_V_Ref_New, Turn_V_Ref_New, Speed,
                Log_List) ->
    receive
        {From, log_values}  -> From ! {self(), log, Log_List};
        {From1, get_all_data} ->
            From1 ! {self(), data,
                     [T1-Start_Time, 1/Dt, Gy, Acc, CtrlByte,
                      -Angle_Acc, -Angle_Kalman, -Angle_Complem,
                      Adv_V_Ref, Switch, Adv_V_Ref_New, Turn_V_Ref_New, Speed]};
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
