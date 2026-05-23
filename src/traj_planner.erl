-module(traj_planner).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Trajectory path-following controller — no lifecycle state.
%%%
%%% This module handles the math of following a Catmull-Rom spline through
%%% the WAYPOINTS list: pure-pursuit steering, speed scaling on curvature,
%%% asymmetric acceleration/braking ramps, waypoint dwell, and final heading
%%% alignment. It knows nothing about idle/paused/finished — those states
%%% live in main_loop, which calls step/3 only when lifecycle == running.
%%%
%%% Sub-state machine (only active during running lifecycle):
%%%
%%%   cruise      ──(dist < BRAKE_DIST)──▶ braking
%%%   braking     ──(near WP & stopped)──▶ dwelling
%%%   dwelling    ──(counter = 0, more WPs)──▶ cruise
%%%   dwelling    ──(counter = 0, last WP)──▶ final_align
%%%   final_align ──(heading within tol)──▶  [finished = true returned]
%%%
%%% When finished/1 returns true, main_loop transitions lifecycle to finished.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([init/0, step/3, finished/1, config_string/0]).
-include("robot_types.hrl").

%% ─── Physical ────────────────────────────────────────────────────────────────
-define(WHEEL_BASE,       18.5).   %% cm

%% ─── Speed ───────────────────────────────────────────────────────────────────
-define(MAX_SPEED,         10.0).   %% cm/s — straight-line cruise speed
-define(MIN_CORNER_SPEED,  1.0).   %% cm/s — floor when curvature is high
-define(MAX_TURN_V,       18.0).   %% cm/s — cap on differential turn velocity

%% ─── Ramps (asymmetric: slow accel, crisp brake) ─────────────────────────────
-define(ACCEL_RATE,        8.0).   %% cm/s²
-define(BRAKE_RATE,       15.0).   %% cm/s²
-define(TURN_ACCEL_RATE,  15.0).   %% cm/s²
-define(TURN_BRAKE_RATE,  25.0).   %% cm/s²

%% ─── Pure Pursuit ─────────────────────────────────────────────────────────────
-define(LOOKAHEAD,         8.0).   %% cm

%% ─── Waypoint stop logic ─────────────────────────────────────────────────────
-define(BRAKE_DIST,        6.0).   %% cm — begin braking this far from waypoint
-define(WAYPOINT_REACHED,  4.0).   %% cm — close-enough radius to declare arrival
-define(SETTLE_VEL,        1.0).   %% cm/s — speed below this = considered stopped
-define(DWELL_TICKS,     600).     %% ~2 s at 300 Hz

%% ─── Final alignment ─────────────────────────────────────────────────────────
-define(FINAL_HEADING_TARGET,  0.0).   %% deg
-define(FINAL_HEADING_TOL,     5.0).   %% deg

%% ─── Waypoints ───────────────────────────────────────────────────────────────
%% First point must equal the robot's zeroed launch pose (0,0).
-define(WAYPOINTS, [
    {  0.0,   0.0},
    {  35.0,  0.0},
    {  5.0,   0.0},
    {  100.0,  -25.0},
    {  150.0,  25.0},
    {  200.0,  -25.0},
    {  250.0,   25.0},
    {  300.0,  0.0}
]).

-record(traj_state, {
    path         = []      :: [{float(), float()}],
    wps_rem      = []      :: [{float(), float()}],
    substate     = cruise  :: cruise | braking | dwelling | final_align,
    counter      = 0       :: non_neg_integer(),   %% dwell countdown
    prev_adv_v   = 0.0     :: float(),
    prev_turn_v  = 0.0     :: float(),
    done         = false   :: boolean()
}).

%% Build the spline path and seed the waypoint list. Called by main_loop on
%% idle→running. traj_planner starts clean — no memory of a previous run.
-spec init() -> #traj_state{}.
init() ->
    Path = spline:build_path(?WAYPOINTS, 30),
    [_Start | Rest_WPs] = ?WAYPOINTS,
    #traj_state{path = Path, wps_rem = Rest_WPs}.

%% Called by main_loop every tick while lifecycle == running.
%% Sensors = #{speed => float(), dt => float()}
%% Pose    = #pose{} (owned by main_loop / odometry)
%% Returns {Output, NewState} where Output = #{adv_v, turn_v, finished, substate, debug}
-spec step(map(), #pose{}, #traj_state{}) -> {map(), #traj_state{}}.
step(Sensors, Pose, State) ->
    #{speed := Speed, dt := Dt} = Sensors,
    #pose{x = X, y = Y, theta = Theta_Deg} = Pose,
    #traj_state{path = Path, wps_rem = WPs_Rem, substate = SubState,
                counter = Counter, prev_adv_v = Prev_Adv, prev_turn_v = Prev_Turn} = State,

    Theta_Rad = Theta_Deg * math:pi() / 180.0,

    %% Controller substep
    {Adv_Tgt, Turn_Tgt, Path2, WPs2, SS2, Cnt2, Done} =
        substep(X, Y, Theta_Rad, Speed, Path, WPs_Rem, SubState, Counter),

    %% Ramps: slow accel / crisp brake
    DV   = Adv_Tgt  - Prev_Adv,
    Adv_Out = if DV > 0.0 -> Prev_Adv  + min(DV,   ?ACCEL_RATE      * Dt);
                 DV < 0.0 -> Prev_Adv  + max(DV,  -?BRAKE_RATE      * Dt);
                 true     -> Prev_Adv
              end,

    DTurn = Turn_Tgt - Prev_Turn,
    Turn_Out = if DTurn > 0.0 -> Prev_Turn + min(DTurn,  ?TURN_ACCEL_RATE * Dt);
                  DTurn < 0.0 -> Prev_Turn + max(DTurn, -?TURN_BRAKE_RATE * Dt);
                  true        -> Prev_Turn
               end,

    NewState = State#traj_state{
        path        = Path2,
        wps_rem     = WPs2,
        substate    = SS2,
        counter     = Cnt2,
        prev_adv_v  = Adv_Out,
        prev_turn_v = Turn_Out,
        done        = Done
    },

    Output = #{adv_v    => Adv_Out,
               turn_v   => -Turn_Out,   %% negated to match robot's differential convention
               finished => Done,
               substate => SS2,
               wps_left => length(WPs2),
               debug    => #{dist_wp => dist_to_head(X, Y, WPs_Rem),
                             counter => Cnt2}},
    {Output, NewState}.

%% True after final_align completes. main_loop uses this to transition lifecycle.
-spec finished(#traj_state{}) -> boolean().
finished(#traj_state{done = D}) -> D.

-spec config_string() -> string().
config_string() ->
    io_lib:format(
        "traj_planner: MAX_SPEED=~p MIN_CORNER=~p MAX_TURN_V=~p "
        "LOOKAHEAD=~p BRAKE_DIST=~p DWELL_TICKS=~p",
        [?MAX_SPEED, ?MIN_CORNER_SPEED, ?MAX_TURN_V,
         ?LOOKAHEAD, ?BRAKE_DIST, ?DWELL_TICKS]).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Sub-state controller
%%% ═══════════════════════════════════════════════════════════════════════════

%% Final alignment — pivot in place toward 0°; declare done when within tol.
substep(_X, _Y, Theta_Rad, _Speed, Path, [], final_align, Counter) ->
    Theta_Deg = Theta_Rad * 180.0 / math:pi(),
    Err = norm_angle(?FINAL_HEADING_TARGET - Theta_Deg),
    if abs(Err) =< ?FINAL_HEADING_TOL ->
           {0.0, 0.0, Path, [], final_align, Counter, true};
       Err > 0.0 ->
           {0.0,  ?MAX_TURN_V, Path, [], final_align, Counter, false};
       true ->
           {0.0, -?MAX_TURN_V, Path, [], final_align, Counter, false}
    end;

%% Empty waypoint list (any other substate) — done.
substep(_X, _Y, _Theta_Rad, _Speed, Path, [], SubState, Counter) ->
    {0.0, 0.0, Path, [], SubState, Counter, true};

substep(X, Y, Theta_Rad, Speed, Path, [{WPx, WPy} | Rest_WPs] = WPs_Rem, SubState, Counter) ->
    Dx_WP      = WPx - X,
    Dy_WP      = WPy - Y,
    Dist_To_WP = math:sqrt(Dx_WP*Dx_WP + Dy_WP*Dy_WP),

    case SubState of
        cruise ->
            if Dist_To_WP < ?BRAKE_DIST ->
                   {0.0, 0.0, Path, WPs_Rem, braking, 0, false};
               true ->
                   Path_New   = pure_pursuit:advance(Path, X, Y, Theta_Rad),
                   {Lx, Ly}   = pure_pursuit:find_lookahead(Path_New, X, Y, ?LOOKAHEAD),
                   Kappa      = pure_pursuit:curvature({Lx, Ly}, X, Y, Theta_Rad),
                   Turn_Tgt   = clamp(Kappa * ?MAX_SPEED * ?WHEEL_BASE / 2.0,
                                      -?MAX_TURN_V, ?MAX_TURN_V),
                   Spd_Factor = clamp(1.0 - abs(Kappa) * ?WHEEL_BASE,
                                      ?MIN_CORNER_SPEED / ?MAX_SPEED, 1.0),
                   {?MAX_SPEED * Spd_Factor, Turn_Tgt, Path_New, WPs_Rem, cruise, 0, false}
            end;

        braking ->
            if (Dist_To_WP < ?WAYPOINT_REACHED) andalso (abs(Speed) < ?SETTLE_VEL) ->
                   {0.0, 0.0, Path, WPs_Rem, dwelling, ?DWELL_TICKS, false};
               true ->
                   {0.0, 0.0, Path, WPs_Rem, braking, 0, false}
            end;

        dwelling ->
            if Counter =< 0 ->
                   case Rest_WPs of
                       [] -> {0.0, 0.0, Path, [], final_align, 0, false};
                       _  -> {0.0, 0.0, Path, Rest_WPs, cruise, 0, false}
                   end;
               true ->
                   {0.0, 0.0, Path, WPs_Rem, dwelling, Counter - 1, false}
            end
    end.

%%% ─── Utilities ──────────────────────────────────────────────────────────────

dist_to_head(_X, _Y, []) -> 0.0;
dist_to_head(X, Y, [{Wx, Wy} | _]) ->
    Dx = Wx - X, Dy = Wy - Y,
    math:sqrt(Dx*Dx + Dy*Dy).

norm_angle(A0) ->
    A = math:fmod(A0, 360.0),
    if A >  180.0 -> A - 360.0;
       A < -180.0 -> A + 360.0;
       true       -> A
    end.

clamp(V, Lo, Hi) ->
    if V < Lo -> Lo; V > Hi -> Hi; true -> V end.
