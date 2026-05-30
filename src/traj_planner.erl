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
%%%   cruise   ──(dist < BRAKE_DIST, last WP)──▶ braking
%%%   cruise   ──(dist < WAYPOINT_REACHED, intermediate WP)──▶ pop, stay cruise
%%%   braking  ──(near WP & stopped)──▶ dwelling
%%%   dwelling ──(counter = 0)──▶  [finished = true returned]
%%%
%%% Waypoint bookkeeping: two lists are kept at all times.
%%%   wps_remaining — still to visit (head = next target)
%%%   wps_traversed — already visited, in chronological order
%%% Their concatenation wps_traversed ++ wps_remaining is the full
%%% intended trajectory. When a WP is reached, it migrates from the head of
%%% wps_remaining to the tail of wps_traversed; nothing is ever silently
%%% dropped. Every pop emits a WP_REACHED 3-block print.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([init/0, step/3, finished/1, has_remaining/1, config_string/0,
         append_waypoint/3, cancel_last/1, cancel_last_n/2,
         clear_remaining/1, last_waypoint_abs/1, waypoint_count/1,
         print_waypoints/1, print_transition/3,
         wps_remaining/1, wps_traversed/1, wps_full/1]).
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
%% Braking trigger is speed-adaptive: Brake_Dist = v² / (2·BRAKE_RATE) + BRAKE_MARGIN.
%% Margin keeps the kinematic stop point inside the WAYPOINT_REACHED ring across
%% the full speed range (a fixed trigger would stop short at low approach speeds).
-define(BRAKE_MARGIN,      2.5).   %% cm — added to kinematic stop distance
-define(WAYPOINT_REACHED,  4.0).   %% cm — close-enough radius to declare arrival
-define(SETTLE_VEL,        1.0).   %% cm/s — speed below this = considered stopped
-define(DWELL_TICKS,     600).     %% ~2 s at 300 Hz — only applied at the final WP

%% ─── Waypoints ───────────────────────────────────────────────────────────────
%% Default is empty: the operator injects waypoints over the wire protocol
%% (PROTO frames decoded in main_loop). When this list is empty the controller
%% starts in a benign "nothing to do" state — step/3 returns zero velocities
%% and done=true so main_loop's lifecycle FSM drops to finished.
-define(WAYPOINTS, []).

-record(traj_state, {
    path                   = []      :: [{float(), float()}],
    wps_remaining          = []      :: [{float(), float()}],
    wps_traversed          = []      :: [{float(), float()}],
    substate               = cruise  :: cruise | braking | dwelling,
    counter                = 0       :: non_neg_integer(),     %% dwell countdown
    prev_adv_v             = 0.0     :: float(),
    prev_turn_v            = 0.0     :: float(),
    done                   = false   :: boolean(),
    %% Tangent-preservation seed for mid-cruise rebuilds. second_last_visited
    %% lags last_visited by one waypoint so rebuild_path/1 can splice the new
    %% spline through a point the robot has already passed through.
    last_visited_wp        = none    :: {float(), float()} | none,
    second_last_visited_wp = none    :: {float(), float()} | none,
    %% Latched on every step/3 tick. Used as the rebuild seed before any
    %% waypoint has been visited, so a first ADD from idle/finished anchors
    %% the new spline to the robot's current world position.
    last_pose              = #pose{} :: #pose{}
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
    #traj_state{path = Path, wps_remaining = Rest_WPs}.

%% Called by main_loop every tick while lifecycle == running.
%% Sensors = #{speed => float(), dt => float()}
%% Pose    = #pose{} (owned by main_loop / odometry)
%% Returns {Output, NewState} where Output = #{adv_v, turn_v, finished, substate, debug}
-spec step(map(), #pose{}, traj_state()) -> {map(), traj_state()}.
step(Sensors, Pose, State) ->
    #{speed := Speed, dt := Dt} = Sensors,
    #pose{x = X, y = Y, theta = Theta_Deg} = Pose,
    #traj_state{path = Path, wps_remaining = WPs_Rem, substate = SubState,
                counter = Counter, prev_adv_v = Prev_Adv, prev_turn_v = Prev_Turn,
                wps_traversed = WPs_Trav} = State,

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

    %% Detect WP pop(s) this tick. substep may pop more than one in the
    %% intermediate-WP pass-through recursion, so we compare lengths and slice
    %% the prefix of WPs_Rem that was consumed.
    Popped_Count = length(WPs_Rem) - length(WPs2),
    Popped       = lists:sublist(WPs_Rem, Popped_Count),
    WPs_Trav2    = WPs_Trav ++ Popped,
    {LastVis, SecondLastVis} =
        update_visited(Popped, State#traj_state.last_visited_wp,
                                State#traj_state.second_last_visited_wp),

    NewState = State#traj_state{
        path                   = Path2,
        wps_remaining          = WPs2,
        wps_traversed          = WPs_Trav2,
        substate               = SS2,
        counter                = Cnt2,
        prev_adv_v             = Adv_Out,
        prev_turn_v            = Turn_Out,
        done                   = Done,
        last_visited_wp        = LastVis,
        second_last_visited_wp = SecondLastVis,
        last_pose              = Pose
    },

    %% Emit WP_REACHED whenever any waypoint migrated to traversed this tick.
    if Popped_Count > 0 ->
           io:format("===WP_REACHED===~n"),
           print_waypoints(NewState);
       true -> ok
    end,

    Output = #{adv_v    => Adv_Out,
               turn_v   => -Turn_Out,   %% negated to match robot's differential convention
               finished => Done,
               substate => SS2,
               wps_left => length(WPs2),
               debug    => #{dist_wp => dist_to_head(X, Y, WPs_Rem),
                             counter => Cnt2}},
    {Output, NewState}.

%% Shift-register update: handles 0/1/N popped WPs in one tick.
update_visited([], LV, SLV)   -> {LV, SLV};
update_visited([P], LV, _SLV) -> {P, LV};
update_visited(Ps, _LV, _SLV) ->
    [Last, SecondLast | _] = lists:reverse(Ps),
    {Last, SecondLast}.

%% True after the last waypoint completes. main_loop uses this to transition
%% lifecycle to finished.
-spec finished(traj_state()) -> boolean().
finished(#traj_state{done = D}) -> D.

%% True when there are still waypoints to visit. main_loop reads this after
%% every apply_cmd_action to decide the wire-driven transitions
%% (finished+ADD → paused, running/paused + cancel-drains → paused/idle).
-spec has_remaining(traj_state()) -> boolean().
has_remaining(#traj_state{wps_remaining = WR}) -> WR =/= [].

-spec config_string() -> string().
config_string() ->
    io_lib:format(
        "traj_planner: MAX_SPEED=~p MIN_CORNER=~p MAX_TURN_V=~p "
        "LOOKAHEAD=~p BRAKE_MARGIN=~p BRAKE_RATE=~p DWELL_TICKS=~p",
        [?MAX_SPEED, ?MIN_CORNER_SPEED, ?MAX_TURN_V,
         ?LOOKAHEAD, ?BRAKE_MARGIN, ?BRAKE_RATE, ?DWELL_TICKS]).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Public waypoint-array API
%%%
%%% Pure functions over #traj_state{}. main_loop calls these from the wire-
%%% protocol decoder. wps_traversed is preserved by every mutator unless the
%%% operator explicitly Clear_All's (or main_loop wipes it on a → idle
%%% transition).
%%% ═══════════════════════════════════════════════════════════════════════════

%% New WP is positioned relative to the last appended WP (which may already be
%% traversed). Falls back to current pose only when no WPs have ever been
%% appended. Always appended to wps_remaining, never to wps_traversed.
-spec append_waypoint(float(), float(), traj_state()) -> traj_state().
append_waypoint(AbsX, AbsY, S = #traj_state{wps_remaining = WR}) ->
    NewWP = {AbsX, AbsY},
    %% Coming out of an empty/finished state we also clear the done flag so the
    %% controller is willing to accept a fresh run once a path is built.
    S1 = S#traj_state{
        wps_remaining = WR ++ [NewWP],
        done          = false,
        substate      = cruise,
        counter       = 0
    },
    rebuild_path(S1).

%% Cancel operates on wps_remaining only. Traversed WPs are real history and
%% are never erased by cancel. No-op when remaining is empty.
-spec cancel_last(traj_state()) -> traj_state().
cancel_last(S = #traj_state{wps_remaining = []}) -> S;
cancel_last(S = #traj_state{wps_remaining = WR}) ->
    rebuild_path(reset_if_no_remaining(S#traj_state{
        wps_remaining = lists:droplast(WR)
    })).

-spec cancel_last_n(non_neg_integer(), traj_state()) -> traj_state().
cancel_last_n(0, S) -> S;
cancel_last_n(_, S = #traj_state{wps_remaining = []}) -> S;
cancel_last_n(N, S = #traj_state{wps_remaining = WR}) ->
    Keep = max(0, length(WR) - N),
    rebuild_path(reset_if_no_remaining(S#traj_state{
        wps_remaining = lists:sublist(WR, Keep)
    })).

%% Clear_All is a hard wipe: both lists empty, visited-shift register cleared,
%% substate machine rewound. main_loop's post-apply_cmd_action logic uses
%% has_remaining/1 == false to decide the lifecycle consequence.
%% last_pose is also reset — every →idle path funnels through here, and a
%% stale last_pose would otherwise seed rebuild_path/1 from the previous
%% run's end position when the next ADD lands in idle.
-spec clear_remaining(traj_state()) -> traj_state().
clear_remaining(S) ->
    S#traj_state{wps_remaining = [], wps_traversed = [], path = [],
                 substate = cruise, counter = 0, done = false,
                 last_visited_wp = none, second_last_visited_wp = none,
                 last_pose = #pose{}}.

%% Stop the substate machine when a cancel drains wps_remaining. Traversed
%% history is preserved here (only Clear_All / main_loop wipes it).
reset_if_no_remaining(S = #traj_state{wps_remaining = []}) ->
    S#traj_state{path = [], substate = cruise, counter = 0, done = false};
reset_if_no_remaining(S) -> S.

%%% ─── Spline rebuild ────────────────────────────────────────────────────────
%% Seed precedence: second_last_visited → last_visited → last_pose. The first
%% two preserve tangent continuity with the path the robot already followed;
%% the pose fallback anchors a fresh-start spline to the robot's current world
%% position so pure_pursuit's lookahead begins where the robot actually is.
rebuild_path(S = #traj_state{wps_remaining = WR}) ->
    Seed = case S#traj_state.second_last_visited_wp of
               none ->
                   case S#traj_state.last_visited_wp of
                       none ->
                           P = S#traj_state.last_pose,
                           {P#pose.x, P#pose.y};
                       LW -> LW
                   end;
               SLW -> SLW
           end,
    NewPath =
        case WR of
            []     -> [];
            [Only] -> straight_line(Seed, Only, 30);
            _      -> spline:build_path_continuing(Seed, WR, 30)
        end,
    S#traj_state{path = NewPath}.

%% Linear interpolation fallback for the 1-waypoint case. Catmull-Rom needs
%% at least two real control points; with a single target we just stretch a
%% straight segment from Seed to it so pure_pursuit has something to chase.
straight_line({Ax, Ay}, {Bx, By}, N) ->
    [{Ax + (Bx - Ax) * float(I) / float(N),
      Ay + (By - Ay) * float(I) / float(N)} || I <- lists:seq(0, N - 1)].

%% Last appended WP — anchor for the next relative ADD. Looks at remaining
%% first (newest appends live there), falls back to traversed (everything
%% appended has been visited), then empty.
-spec last_waypoint_abs(traj_state()) -> {float(), float()} | empty.
last_waypoint_abs(#traj_state{wps_remaining = WR, wps_traversed = WT}) ->
    case WR of
        [] -> case WT of [] -> empty; _ -> lists:last(WT) end;
        _  -> lists:last(WR)
    end.

%% Total WPs ever appended (and not cancelled): traversed + remaining.
-spec waypoint_count(traj_state()) -> non_neg_integer().
waypoint_count(#traj_state{wps_remaining = WR, wps_traversed = WT}) ->
    length(WR) + length(WT).

-spec wps_remaining(traj_state()) -> [{float(), float()}].
wps_remaining(#traj_state{wps_remaining = WR}) -> WR.

-spec wps_traversed(traj_state()) -> [{float(), float()}].
wps_traversed(#traj_state{wps_traversed = WT}) -> WT.

-spec wps_full(traj_state()) -> [{float(), float()}].
wps_full(#traj_state{wps_remaining = WR, wps_traversed = WT}) -> WT ++ WR.

%% Three-block dump: full, traversed, remaining. Called on every state
%% transition by main_loop and on every WP pop by step/3.
-spec print_waypoints(traj_state()) -> ok.
print_waypoints(S = #traj_state{}) ->
    print_block("WPS_FULL",      wps_full(S)),
    print_block("WPS_TRAVERSED", wps_traversed(S)),
    print_block("WPS_REMAINING", wps_remaining(S)),
    ok.

print_block(Label, []) ->
    io:format("~s: (empty)~n", [Label]);
print_block(Label, WPs) ->
    io:format("~s: ~p waypoint(s)~n", [Label, length(WPs)]),
    lists:foldl(
        fun({X, Y}, I) ->
            io:format("  #~2..0B  x=~7.2f  y=~7.2f~n", [I, X, Y]),
            I + 1
        end, 1, WPs),
    ok.

%% Emit a one-line transition tag followed by the 3-block WP dump. Called by
%% main_loop on every lifecycle transition the operator should see.
-spec print_transition(atom(), atom(), traj_state()) -> ok.
print_transition(From, To, S) ->
    io:format("===STATE: ~p -> ~p===~n", [From, To]),
    print_waypoints(S).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Sub-state controller
%%% ═══════════════════════════════════════════════════════════════════════════

%% Empty waypoint list — declare done. Reached only on the first running tick
%% when no WPs were ever queued; mid-run cancel-drain is handled by main_loop's
%% override which transitions out of running before step/3 runs.
substep(_X, _Y, _Theta_Rad, _Speed, Path, [], SubState, Counter) ->
    {0.0, 0.0, Path, [], SubState, Counter, true};

substep(X, Y, Theta_Rad, Speed, Path, [{WPx, WPy} | Rest_WPs] = WPs_Rem, SubState, Counter) ->
    Dx_WP      = WPx - X,
    Dy_WP      = WPy - Y,
    Dist_To_WP = math:sqrt(Dx_WP*Dx_WP + Dy_WP*Dy_WP),

    Is_Last    = (Rest_WPs =:= []),
    %% v²/(2a) + margin — collapses to BRAKE_MARGIN at standstill, expands with
    %% approach speed so the stop point always lands inside WAYPOINT_REACHED.
    Brake_Dist = (Speed * Speed) / (2.0 * ?BRAKE_RATE) + ?BRAKE_MARGIN,

    case SubState of
        cruise ->
            if (not Is_Last) andalso (Dist_To_WP < ?WAYPOINT_REACHED) ->
                   %% Intermediate WP: pass through without braking or dwelling.
                   %% The spline is smooth at WPs, so stopping here is wasteful.
                   %% Recurse to evaluate the next WP this same tick.
                   substep(X, Y, Theta_Rad, Speed, Path, Rest_WPs, cruise, 0);
               Is_Last andalso (Dist_To_WP < Brake_Dist) ->
                   {0.0, 0.0, Path, WPs_Rem, braking, 0, false};
               Path =:= [] ->
                   %% Defensive: mutators rebuild the spline so a non-empty
                   %% wps_remaining normally implies a non-empty path. This
                   %% branch only fires across the boundary where Clear_All
                   %% has just wiped path before the running→finished lands.
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
                   %% With the new "no intermediate dwell" rule (only the last
                   %% WP enters braking→dwelling), Rest_WPs is always [] here.
                   %% Keep the case for defensive completeness in case future
                   %% changes restore dwell at intermediates.
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
