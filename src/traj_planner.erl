-module(traj_planner).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Trajectory path-following controller — no lifecycle state.
%%%
%%% This module handles the math of following a Catmull-Rom spline through
%%% the operator-injected waypoint list: pure-pursuit steering, speed scaling
%%% on curvature, asymmetric acceleration/braking ramps, and waypoint dwell.
%%% It knows nothing about idle/paused/finished — those states live in
%%% main_loop, which calls step/3 only when lifecycle == running.
%%%
%%% Sub-state machine (only active during running lifecycle):
%%%
%%%   cruise   ──(dist < BRAKE_DIST)──▶ braking
%%%   braking  ──(near WP & stopped)──▶ dwelling
%%%   dwelling ──(counter = 0, more WPs)──▶ cruise
%%%   dwelling ──(counter = 0, last WP)──▶  [finished = true returned]
%%%
%%% When finished/1 returns true, main_loop transitions lifecycle to finished.
%%% No return-to-origin and no final-heading alignment: when the last waypoint
%%% has been dwelled the controller simply declares done.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([init/0, step/3, finished/1, config_string/0,
         append_waypoint/3, cancel_last/1, cancel_last_n/2,
         clear_remaining/1, last_waypoint_abs/1, waypoint_count/1,
         print_waypoints/1]).
-export_type([traj_state/0]).

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

%% ─── Waypoints ───────────────────────────────────────────────────────────────
%% Default is empty: the operator injects waypoints over the wire protocol
%% (PROTO frames decoded in main_loop). When this list is empty the controller
%% starts in a benign "nothing to do" state — step/3 returns zero velocities
%% and done=true so main_loop drops straight from running to finished.
-define(WAYPOINTS, []).

-record(traj_state, {
    path         = []      :: [{float(), float()}],
    wps_rem      = []      :: [{float(), float()}],
    wps_full     = []      :: [{float(), float()}],  %% lockstep with wps_rem
    substate     = cruise  :: cruise | braking | dwelling,
    counter      = 0       :: non_neg_integer(),   %% dwell countdown
    prev_adv_v   = 0.0     :: float(),
    prev_turn_v  = 0.0     :: float(),
    done         = false   :: boolean()
}).

-type traj_state() :: #traj_state{}.

%% Build the spline path and seed the waypoint list. Called by main_loop on
%% idle→running. traj_planner starts clean — no memory of a previous run.
%% Empty WAYPOINTS produces an empty state; step/3 will immediately report done.
-spec init() -> traj_state().
init() ->
    init_from(?WAYPOINTS).

init_from([]) ->
    #traj_state{};
init_from([_Start | Rest_WPs] = All) ->
    Path = spline:build_path(All, 30),
    #traj_state{path = Path, wps_rem = Rest_WPs, wps_full = Rest_WPs}.

%% Called by main_loop every tick while lifecycle == running.
%% Sensors = #{speed => float(), dt => float()}
%% Pose    = #pose{} (owned by main_loop / odometry)
%% Returns {Output, NewState} where Output = #{adv_v, turn_v, finished, substate, debug}
-spec step(map(), #pose{}, traj_state()) -> {map(), traj_state()}.
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
        wps_full    = WPs2,   %% maintain lockstep invariant wps_full == wps_rem
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

%% True after the last waypoint completes. main_loop uses this to transition
%% lifecycle to finished. There is no return-to-origin and no final-heading
%% alignment — the trajectory just stops.
-spec finished(traj_state()) -> boolean().
finished(#traj_state{done = D}) -> D.

-spec config_string() -> string().
config_string() ->
    io_lib:format(
        "traj_planner: MAX_SPEED=~p MIN_CORNER=~p MAX_TURN_V=~p "
        "LOOKAHEAD=~p BRAKE_DIST=~p DWELL_TICKS=~p",
        [?MAX_SPEED, ?MIN_CORNER_SPEED, ?MAX_TURN_V,
         ?LOOKAHEAD, ?BRAKE_DIST, ?DWELL_TICKS]).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Public waypoint-array API (Phase 4)
%%%
%%% Pure functions over #traj_state{}. main_loop calls these from the wire-
%%% protocol decoder (Phase 5). All operations preserve the lockstep invariant
%%% wps_full == wps_rem; the field split is kept for forward-compatibility.
%%% ═══════════════════════════════════════════════════════════════════════════

-spec append_waypoint(float(), float(), traj_state()) -> traj_state().
append_waypoint(AbsX, AbsY, S = #traj_state{wps_full = WF, wps_rem = WR}) ->
    NewWP = {AbsX, AbsY},
    %% Coming out of an empty/finished state we also clear the done flag so the
    %% controller is willing to accept a fresh run once a path is built.
    S#traj_state{
        wps_full = WF ++ [NewWP],
        wps_rem  = WR ++ [NewWP],
        done     = false,
        substate = cruise,
        counter  = 0
    }.

%% No-op on empty so a stray cancel signal with no trajectory loaded is benign.
-spec cancel_last(traj_state()) -> traj_state().
cancel_last(S = #traj_state{wps_full = []}) -> S;
cancel_last(S = #traj_state{wps_full = WF, wps_rem = WR}) ->
    reset_if_empty(S#traj_state{
        wps_full = lists:droplast(WF),
        wps_rem  = lists:droplast(WR)
    }).

-spec cancel_last_n(non_neg_integer(), traj_state()) -> traj_state().
cancel_last_n(0, S) -> S;
cancel_last_n(_, S = #traj_state{wps_full = []}) -> S;
cancel_last_n(N, S = #traj_state{wps_full = WF, wps_rem = WR}) ->
    Keep = max(0, length(WF) - N),
    reset_if_empty(S#traj_state{
        wps_full = lists:sublist(WF, Keep),
        wps_rem  = lists:sublist(WR, Keep)
    }).

%% Clear-all is a hard stop+reset: empties the list and rewinds the substate
%% machine. step/3 will return done=true on the next tick (empty wps clause)
%% which lets main_loop's lifecycle FSM transition running → finished naturally.
-spec clear_remaining(traj_state()) -> traj_state().
clear_remaining(S) ->
    S#traj_state{wps_full = [], wps_rem = [], path = [],
                 substate = cruise, counter = 0, done = false}.

%% Stop-and-reset when a cancel drains the list. Same shape as clear_remaining
%% minus the explicit path wipe (path is already orphaned wrt the new list and
%% will be rebuilt in Phase 6).
reset_if_empty(S = #traj_state{wps_full = []}) ->
    S#traj_state{path = [], substate = cruise, counter = 0, done = false};
reset_if_empty(S) -> S.

-spec last_waypoint_abs(traj_state()) -> {float(), float()} | empty.
last_waypoint_abs(#traj_state{wps_full = []}) -> empty;
last_waypoint_abs(#traj_state{wps_full = WF}) -> lists:last(WF).

-spec waypoint_count(traj_state()) -> non_neg_integer().
waypoint_count(#traj_state{wps_full = WF}) -> length(WF).

-spec print_waypoints(traj_state()) -> ok.
print_waypoints(#traj_state{wps_full = []}) ->
    io:format("WPS: (empty)~n");
print_waypoints(#traj_state{wps_full = WF}) ->
    io:format("WPS: ~p waypoint(s)~n", [length(WF)]),
    lists:foldl(
        fun({X, Y}, I) ->
            io:format("  #~2..0B  x=~7.2f  y=~7.2f~n", [I, X, Y]),
            I + 1
        end, 1, WF),
    ok.

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Sub-state controller
%%% ═══════════════════════════════════════════════════════════════════════════

%% Empty waypoint list — declare done. Covers both "no trajectory loaded at
%% start" and "operator cleared mid-run" cases.
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
               Path =:= [] ->
                   %% Phase 5: spline rebuild is deferred. Waypoints exist but
                   %% no path has been computed for them yet — freeze in place
                   %% instead of feeding an empty list to pure_pursuit.
                   {0.0, 0.0, Path, WPs_Rem, cruise, 0, false};
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
                       [] -> {0.0, 0.0, Path, [], dwelling, 0, true};
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

clamp(V, Lo, Hi) ->
    if V < Lo -> Lo; V > Hi -> Hi; true -> V end.
