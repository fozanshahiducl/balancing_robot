-module(trajectory_layer2).
-export([init/0, step/2, status/1]).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Waypoint-stop trajectory layer.
%%%
%%% Drives a Catmull-Rom spline through the WAYPOINTS list, dwelling at each
%%% user-defined waypoint. Pure Pursuit handles steering between waypoints;
%%% forward speed is scaled down on tight curvature so the robot doesn't carry
%%% momentum into corners. Acceleration is slow; braking at each waypoint is
%%% crisp via asymmetric ramps.
%%%
%%% Sub-state machine inside `running`:
%%%   cruise   – pursue the spline toward next waypoint, scale speed by κ
%%%   braking  – within BRAKE_DIST of next waypoint, command zero
%%%   dwelling – stopped at waypoint, count down DWELL_TICKS, then advance
%%% ═══════════════════════════════════════════════════════════════════════════

%% ─── Physical ────────────────────────────────────────────────────────────────
-define(WHEEL_BASE, 19.3).         % cm

%% ─── Speed ───────────────────────────────────────────────────────────────────
-define(MAX_SPEED,        5.0).    % cm/s — straight-line cruise
-define(MIN_CORNER_SPEED, 1.0).    % cm/s — floor when κ is high
-define(MAX_TURN_V,       18.0).    % cm/s — cap on differential turn velocity

%% ─── Ramps (the asymmetric "slow accel, crisp brake") ───────────────────────
%% Acceleration is gentle so balance has time to adapt.
%% Braking is fast so the robot stops cleanly at each waypoint.
-define(ACCEL_RATE,        4.0).   % cm/s² (slow)
-define(BRAKE_RATE,       15.0).   % cm/s² (crisp)
-define(TURN_ACCEL_RATE,   15.0).   % cm/s² differential
-define(TURN_BRAKE_RATE,  25.0).   % cm/s² differential

%% ─── Pure Pursuit ────────────────────────────────────────────────────────────
-define(LOOKAHEAD,  8.0).         % cm
-define(SPLINE_RES, 30).           % subpoints per segment

%% ─── Waypoint stop logic ────────────────────────────────────────────────────
-define(BRAKE_DIST,       6.0).   % cm — start braking this far before waypoint
-define(WAYPOINT_REACHED,  4.0).   % cm — close enough to declare arrival
-define(SETTLE_VEL,        1.0).   % cm/s — considered stopped below this
-define(DWELL_TICKS,      600).    % at 300Hz ≈ 2 seconds dwell per waypoint

%% ─── Final orientation ──────────────────────────────────────────────────────
%% After the last waypoint, pivot in place until heading is within tolerance of 0°.
-define(FINAL_HEADING_TARGET, 0.0).  % degrees
-define(FINAL_HEADING_TOL,    5.0).  % degrees

%% ─── Logging ────────────────────────────────────────────────────────────────
%% Emit a tagged CSV row every LOG_RATE_DIV ticks. At 300Hz loop and DIV=10,
%% that's ~30 Hz logging → ~3 KB/s @ 115200 baud → fits comfortably.
-define(LOG_RATE_DIV, 10).

%% ─── Waypoints ───────────────────────────────────────────────────────────────
%% First point MUST be the robot's launch position (zeroed at idle→running).
-define(WAYPOINTS, [
    {  0.0,   0.0},
    { 50.0,  0.0},
    { 50.0, 50.0},
    { 0.0,  50.0},
    { 0.0, 0.0}
]).

%% ─── State tuple ─────────────────────────────────────────────────────────────
%% {Status, X, Y, Theta, Path, WPs_Rem, SubState, Counter, Prev_Adv_V, Prev_Turn_V}
%%   Status        – idle | running | paused | finished
%%   X, Y, Theta   – pose estimate (cm, cm, deg)
%%   Path          – remaining Catmull-Rom spline subpoints
%%   WPs_Rem       – user waypoints not yet stopped at (head = current target)
%%   SubState      – cruise | braking | dwelling
%%   Counter       – dwell countdown when in `dwelling`, otherwise unused
%%   Prev_Adv_V    – previous ramped forward output (for ramp continuity)
%%   Prev_Turn_V   – previous ramped turn output (for ramp continuity)

init() ->
    Path = build_path(?WAYPOINTS),
    [_Start | Rest_WPs] = ?WAYPOINTS,   % we're already at the start
    {idle, 0.0, 0.0, 0.0, Path, Rest_WPs, cruise, 0, 0.0, 0.0}.

%% Convenience: let main_loop read the trajectory status atom without knowing
%% the state tuple layout.
status(State) -> element(1, State).

step(State, {Dt, Speed_L, Speed_R, Forward, Robot_Up}) ->
    {Status, X, Y, Theta, Path, WPs_Rem, SubState, Counter, Prev_Adv_V, Prev_Turn_V} = State,
    Speed = (Speed_L + Speed_R) / 2.0,

    %% 1. ODOMETRY ─────────────────────────────────────────────────────────────
    D_Dist        = Speed * Dt,
    D_Theta_Rad   = ((Speed_R - Speed_L) / ?WHEEL_BASE) * Dt,
    D_Theta_Deg   = D_Theta_Rad * 180.0 / math:pi(),
    Theta_New     = norm_angle(Theta + D_Theta_Deg),
    Theta_Mid_Rad = (Theta + D_Theta_Deg / 2.0) * math:pi() / 180.0,
    X_New = X + D_Dist * math:cos(Theta_Mid_Rad),
    Y_New = Y + D_Dist * math:sin(Theta_Mid_Rad),

    %% 2. TOP-LEVEL STATE MACHINE ──────────────────────────────────────────────
    New_Status = case Status of
        idle     -> if Forward and Robot_Up  -> running; true -> idle     end;
        running  -> if not Robot_Up          -> paused;  true -> running  end;
        paused   -> if Robot_Up and Forward  -> running; true -> paused   end;
        finished -> finished
    end,

    %% 3. COORDINATE RESET on idle→running ─────────────────────────────────────
    {X_T, Y_T, Theta_T} = if
        (Status =:= idle) and (New_Status =:= running) -> {0.0, 0.0, 0.0};
        true -> {X_New, Y_New, Theta_New}
    end,
    Theta_Rad = Theta_T * math:pi() / 180.0,

    %% 4. SUBSTATE LOGIC (only meaningful when running) ────────────────────────
    {Adv_V_Tgt, Turn_V_Tgt, Path2, WPs_Rem2, SubState2, Counter2, Final_Status} =
        case New_Status of
            running ->
                substep(X_T, Y_T, Theta_Rad, Speed, Path, WPs_Rem, SubState, Counter);
            _ ->
                {0.0, 0.0, Path, WPs_Rem, SubState, Counter, New_Status}
        end,

    %% 5. RAMPS – slow accel, crisp brake ─────────────────────────────────────
    DV = Adv_V_Tgt - Prev_Adv_V,
    Adv_V_Out = if
        DV > 0.0 -> Prev_Adv_V + min(DV,  ?ACCEL_RATE * Dt);
        DV < 0.0 -> Prev_Adv_V + max(DV, -?BRAKE_RATE * Dt);
        true     -> Prev_Adv_V
    end,

    DTurn = Turn_V_Tgt - Prev_Turn_V,
    Turn_V_Out = if
        DTurn > 0.0 -> Prev_Turn_V + min(DTurn,  ?TURN_ACCEL_RATE * Dt);
        DTurn < 0.0 -> Prev_Turn_V + max(DTurn, -?TURN_BRAKE_RATE * Dt);
        true        -> Prev_Turn_V
    end,

    %% 6. SERIAL LOGGING ───────────────────────────────────────────────────────
    %% Emits TLOG-prefixed CSV rows for the Python capture script. Boundary
    %% sentinels mark start/end so the script can ignore other shell chatter.
    log_step(Status, Final_Status, SubState2, X_T, Y_T, Theta_T, Speed,
             Adv_V_Out, Turn_V_Out, WPs_Rem2, Counter2),

    State_New = {Final_Status, X_T, Y_T, Theta_T, Path2, WPs_Rem2, SubState2, Counter2, Adv_V_Out, Turn_V_Out},
    {Adv_V_Out, -Turn_V_Out, State_New}.   % turn negated to match robot's convention


%%% ═══════════════════════════════════════════════════════════════════════════
%%% Logging — uses process dict for transient state (T0, tick count) so the
%%% main state tuple stays unchanged. Output format:
%%%   ===LOG_START_TRAJ===                       (sentinel, one line)
%%%   TLOG_HEADER,T_ms,phase,x,y,theta_deg,...   (CSV header, one line)
%%%   TLOG,123,cruise,12.34,5.67,...             (CSV rows, ~30Hz)
%%%   ===LOG_END_TRAJ===                         (sentinel, one line)
%%% ═══════════════════════════════════════════════════════════════════════════

log_step(Status, Final_Status, SubState, X, Y, Theta, Speed,
         Adv_V, Turn_V, WPs, Counter) ->
    Just_Started  = (Status =:= idle) andalso (Final_Status =:= running),
    Just_Finished = (Status =/= finished) andalso (Final_Status =:= finished),

    %% On idle→running: emit start sentinel + CSV header, initialise T0/tick.
    if Just_Started ->
            put(traj_log_t0, erlang:system_time(millisecond)),
            put(traj_log_tick, 0),
            io:format("===LOG_START_TRAJ===~n", []),
            io:format("TLOG_HEADER,T_ms,phase,x,y,theta_deg,speed,adv_v,turn_v,dist_wp,counter,wps_left~n", []);
       true -> ok
    end,

    %% Periodic data row (only while logging is active, i.e. T0 is set).
    case get(traj_log_t0) of
        undefined -> ok;
        T0 ->
            Tk = get(traj_log_tick),
            if Tk rem ?LOG_RATE_DIV =:= 0 ->
                    Phase = case Final_Status of
                        running -> SubState;
                        _       -> Final_Status
                    end,
                    Dist_WP = case WPs of
                        []             -> 0.0;
                        [{Wx, Wy} | _] ->
                            Dxw = Wx - X, Dyw = Wy - Y,
                            math:sqrt(Dxw*Dxw + Dyw*Dyw)
                    end,
                    T_ms = erlang:system_time(millisecond) - T0,
                    io:format("TLOG,~p,~p,~.2f,~.2f,~.2f,~.2f,~.2f,~.2f,~.2f,~p,~p~n",
                              [T_ms, Phase, X, Y, Theta, Speed,
                               Adv_V, Turn_V, Dist_WP, Counter, length(WPs)]);
               true -> ok
            end,
            put(traj_log_tick, Tk + 1)
    end,

    %% On any→finished: emit end sentinel, clear logging state.
    if Just_Finished ->
            io:format("===LOG_END_TRAJ===~n", []),
            erase(traj_log_t0),
            erase(traj_log_tick);
       true -> ok
    end,
    ok.


%%% ═══════════════════════════════════════════════════════════════════════════
%%% Substate logic — only invoked when status is `running`.
%%% ═══════════════════════════════════════════════════════════════════════════

%% After the last waypoint, pivot in place toward FINAL_HEADING_TARGET (0°).
%% Bang-bang turn signal; the section-5 ramp smooths the start/stop. When the
%% heading lands within FINAL_HEADING_TOL of target → declare finished.
substep(_X, _Y, Theta_Rad, _Speed, Path, [], final_align, Counter) ->
    Theta_Deg = Theta_Rad * 180.0 / math:pi(),
    Err = norm_angle(?FINAL_HEADING_TARGET - Theta_Deg),
    if abs(Err) =< ?FINAL_HEADING_TOL ->
            {0.0, 0.0, Path, [], final_align, Counter, finished};
       Err > 0.0 ->
            {0.0,  ?MAX_TURN_V, Path, [], final_align, Counter, running};
       true ->
            {0.0, -?MAX_TURN_V, Path, [], final_align, Counter, running}
    end;

%% Empty waypoint list with any other substate → no more work, finished.
substep(_X, _Y, _Theta_Rad, _Speed, Path, [], SubState, Counter) ->
    {0.0, 0.0, Path, [], SubState, Counter, finished};

substep(X, Y, Theta_Rad, Speed, Path, [{WPx, WPy} | Rest_WPs] = WPs_Rem, SubState, Counter) ->
    Dx_WP = WPx - X,
    Dy_WP = WPy - Y,
    Dist_To_WP = math:sqrt(Dx_WP*Dx_WP + Dy_WP*Dy_WP),

    case SubState of
        cruise ->
            if Dist_To_WP < ?BRAKE_DIST ->
                    %% Within brake distance → transition to braking, output zero.
                    {0.0, 0.0, Path, WPs_Rem, braking, 0, running};
               true ->
                    %% Pure Pursuit on the spline.
                    Path_New = advance_path(Path, X, Y, Theta_Rad),
                    {Lx, Ly} = find_lookahead(Path_New, X, Y),
                    Dx = Lx - X,
                    Dy = Ly - Y,
                    L2 = Dx*Dx + Dy*Dy,
                    Kappa = if L2 > 0.001 ->
                                   Local_Y = -Dx * math:sin(Theta_Rad)
                                           +  Dy * math:cos(Theta_Rad),
                                   2.0 * Local_Y / L2;
                               true -> 0.0
                            end,
                    Turn_Tgt = clamp(Kappa * ?MAX_SPEED * ?WHEEL_BASE / 2.0,
                                     -?MAX_TURN_V, ?MAX_TURN_V),
                    %% Scale forward speed down when curvature is high (sharp turn).
                    %% κ=0 → full MAX_SPEED; κ=1/WHEEL_BASE → MIN_CORNER_SPEED.
                    Speed_Factor = clamp(1.0 - abs(Kappa) * ?WHEEL_BASE,
                                         ?MIN_CORNER_SPEED / ?MAX_SPEED, 1.0),
                    {?MAX_SPEED * Speed_Factor, Turn_Tgt,
                     Path_New, WPs_Rem, cruise, 0, running}
            end;

        braking ->
            %% Output zero, wait for robot to actually stop near the waypoint.
            if (Dist_To_WP < ?WAYPOINT_REACHED) andalso (abs(Speed) < ?SETTLE_VEL) ->
                    {0.0, 0.0, Path, WPs_Rem, dwelling, ?DWELL_TICKS, running};
               true ->
                    {0.0, 0.0, Path, WPs_Rem, braking, 0, running}
            end;

        dwelling ->
            %% Stay parked, count down. When done, advance.
            %% If this was the LAST waypoint, go to final_align (orient to 0°)
            %% instead of trying to cruise to nowhere.
            if Counter =< 0 ->
                    case Rest_WPs of
                        [] -> {0.0, 0.0, Path, [], final_align, 0, running};
                        _  -> {0.0, 0.0, Path, Rest_WPs, cruise, 0, running}
                    end;
               true ->
                    {0.0, 0.0, Path, WPs_Rem, dwelling, Counter - 1, running}
            end
    end.


%%% ═══════════════════════════════════════════════════════════════════════════
%%% Catmull-Rom Spline
%%% ═══════════════════════════════════════════════════════════════════════════

build_path(WPs) ->
    [P1, P2 | _] = WPs,
    P_Last = lists:last(WPs),
    P_Prev = lists:nth(length(WPs) - 1, WPs),
    Extended = [reflect(P1, P2) | WPs] ++ [reflect(P_Last, P_Prev)],
    sample_all_segments(Extended).

reflect({X1, Y1}, {X2, Y2}) ->
    {2.0 * X1 - X2, 2.0 * Y1 - Y2}.

sample_all_segments([P0, P1, P2, P3 | Rest]) ->
    sample_segment(P0, P1, P2, P3) ++ sample_all_segments([P1, P2, P3 | Rest]);
sample_all_segments(_) ->
    [].

sample_segment(P0, P1, P2, P3) ->
    N = ?SPLINE_RES,
    [catmull_rom(P0, P1, P2, P3, float(I) / float(N)) || I <- lists:seq(0, N - 1)].

catmull_rom({X0,Y0}, {X1,Y1}, {X2,Y2}, {X3,Y3}, T) ->
    T2 = T*T, T3 = T2*T,
    X = 0.5 * ((2.0*X1)
              + (-X0 + X2)                       * T
              + ( 2.0*X0 - 5.0*X1 + 4.0*X2 - X3) * T2
              + (-X0 + 3.0*X1 - 3.0*X2 + X3)     * T3),
    Y = 0.5 * ((2.0*Y1)
              + (-Y0 + Y2)                       * T
              + ( 2.0*Y0 - 5.0*Y1 + 4.0*Y2 - Y3) * T2
              + (-Y0 + 3.0*Y1 - 3.0*Y2 + Y3)     * T3),
    {X, Y}.


%%% ═══════════════════════════════════════════════════════════════════════════
%%% Pure Pursuit helpers
%%% ═══════════════════════════════════════════════════════════════════════════

advance_path([], _, _, _) -> [];
advance_path([_] = Path, _, _, _) -> Path;
advance_path([{Px, Py} | Rest] = Path, X, Y, Theta_Rad) ->
    Dx = Px - X, Dy = Py - Y,
    Local_X = Dx * math:cos(Theta_Rad) + Dy * math:sin(Theta_Rad),
    if Local_X < 0.0 -> advance_path(Rest, X, Y, Theta_Rad);
       true          -> Path
    end.

find_lookahead([Last], _, _) -> Last;
find_lookahead([{Px, Py} = P | Rest], X, Y) ->
    Dx = Px - X, Dy = Py - Y,
    if Dx*Dx + Dy*Dy >= ?LOOKAHEAD * ?LOOKAHEAD -> P;
       true -> find_lookahead(Rest, X, Y)
    end.


%%% ═══════════════════════════════════════════════════════════════════════════
%%% Utility
%%% ═══════════════════════════════════════════════════════════════════════════

norm_angle(Angle) ->
    A = math:fmod(Angle, 360.0),
    if A >  180.0 -> A - 360.0;
       A < -180.0 -> A + 360.0;
       true       -> A
    end.

clamp(Val, Min, Max) ->
    if Val < Min -> Min;
       Val > Max -> Max;
       true      -> Val
    end.
