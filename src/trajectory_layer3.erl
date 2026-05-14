-module(trajectory_layer3).

%% Public API (used by main_loop)
-export([init/0, step/2]).

%% Modular building blocks (exported so they can be tested / swapped)
-export([
    update_pose/2,
    compute_command/2,
    distance/2,
    bearing_error/2,
    forward_speed/2,
    turn_speed/1,
    arrived/2,
    heading_factor/1,
    approach_speed/1,
    ramp/5,
    norm_angle/1,
    clamp/3
]).

%% ─── Physical ────────────────────────────────────────────────────────────────
-define(WHEEL_BASE, 19.3).        % cm, distance between wheels

%% ─── Speed ───────────────────────────────────────────────────────────────────
-define(CRUISE_SPEED,    8.0).    % cm/s, forward speed on a long straight leg
-define(APPROACH_SPEED,  2.5).    % cm/s, slowest forward speed near a waypoint
-define(MAX_TURN_V,      6.0).    % cm/s, max differential turn speed (clamped)

%% ─── Ramps ───────────────────────────────────────────────────────────────────
-define(ACCEL_RATE,       4.0).   % cm/s^2 – gentle ramp-up
-define(BRAKE_RATE,      15.0).   % cm/s^2 – fast but smooth ramp-down
-define(TURN_ACCEL_RATE,  8.0).   % cm/s^2
-define(TURN_BRAKE_RATE, 15.0).   % cm/s^2

%% ─── Geometry gates ──────────────────────────────────────────────────────────
-define(ARRIVAL_RADIUS,    4.0).  % cm – within this, waypoint counts as reached
-define(SLOWDOWN_RADIUS,  20.0).  % cm – start tapering speed inside this
-define(HEADING_FULL_DEG,  8.0).  % deg – within this, full forward speed allowed
-define(HEADING_STOP_DEG, 35.0).  % deg – beyond this, forward speed is zero

%% ─── Steering ────────────────────────────────────────────────────────────────
-define(TURN_GAIN, 0.25).         % cm/s of differential per degree of heading error

%% ─── Waypoints ───────────────────────────────────────────────────────────────
%% Coordinates in cm. First point is the start position (zeroed at launch).
-define(WAYPOINTS, [
    {  0.0,   0.0},
    { 30.0,  30.0},
    { 60.0, -30.0},
    { 90.0,  30.0},
    {120.0, -30.0},
    {150.0,  30.0},
    {150.0, -30.0},
    {120.0,  30.0},
    { 90.0, -30.0},
    { 60.0,  30.0},
    { 30.0, -30.0},
    {  0.0,   0.0}
]).

%% ─── State tuple ─────────────────────────────────────────────────────────────
%% {Status, Pose, Targets, Prev_Adv_V, Prev_Turn_V}
%%   Status      – idle | running | paused | finished
%%   Pose        – {X, Y, Theta}  (cm, cm, degrees), zeroed at launch
%%   Targets     – list of {X, Y} waypoints still to visit (head = current)
%%   Prev_Adv_V  – last forward velocity command (for ramping)
%%   Prev_Turn_V – last turn velocity command (for ramping)

init() ->
    [_Start | Rest] = ?WAYPOINTS,    % first waypoint is our launch position
    {idle, {0.0, 0.0, 0.0}, Rest, 0.0, 0.0}.

step(State, {Dt, Speed_L, Speed_R, Forward, Robot_Up}) ->
    {Status, Pose, Targets, Prev_Adv_V, Prev_Turn_V} = State,

    %% 1. ODOMETRY ────────────────────────────────────────────────────────────
    Pose_Odo = update_pose(Pose, {Dt, Speed_L, Speed_R}),

    %% 2. STATE MACHINE ───────────────────────────────────────────────────────
    New_Status = next_status(Status, Forward, Robot_Up),

    %% 3. COORDINATE RESET on idle→running transition ─────────────────────────
    Pose_T = case {Status, New_Status} of
        {idle, running} -> {0.0, 0.0, 0.0};
        _               -> Pose_Odo
    end,

    %% 4. RETARGET: pop waypoints we've already reached ───────────────────────
    Targets_T = retarget(Targets, Pose_T),

    %% 5. COMPUTE COMMANDS for current target ─────────────────────────────────
    {Adv_V_Req, Turn_V_Req, Next_Status} = case {New_Status, Targets_T} of
        {running, []}                  -> {0.0, 0.0, finished};
        {running, [Target | _]}        ->
            {A, T} = compute_command(Target, Pose_T),
            {A, T, running};
        {_, _}                         -> {0.0, 0.0, New_Status}
    end,

    %% 6. RAMPS ───────────────────────────────────────────────────────────────
    Adv_V_Out  = ramp(Prev_Adv_V,  Adv_V_Req,  ?ACCEL_RATE,      ?BRAKE_RATE,      Dt),
    Turn_V_Out = ramp(Prev_Turn_V, Turn_V_Req, ?TURN_ACCEL_RATE, ?TURN_BRAKE_RATE, Dt),

    State_New = {Next_Status, Pose_T, Targets_T, Adv_V_Out, Turn_V_Out},
    {Adv_V_Out, -Turn_V_Out, State_New}.    % Turn negated to match robot convention


%%% ═══════════════════════════════════════════════════════════════════════════
%%% Core control: given a target and a pose, produce motor-frame commands.
%%% ═══════════════════════════════════════════════════════════════════════════

%% compute_command/2 – the heart of the controller.
%% Inputs:  Target = {Tx, Ty}, Pose = {X, Y, Theta_Deg}
%% Output:  {Adv_V_Req, Turn_V_Req}
%%   Adv_V_Req  – desired forward speed (cm/s), pre-ramp
%%   Turn_V_Req – desired differential turn speed (cm/s), pre-ramp
%% Forward speed is gated by heading alignment: poorly aligned → zero forward,
%% just rotate. Speed also tapers as the robot nears the target.
compute_command(Target, Pose) ->
    Dist = distance(Pose, Target),
    Err  = bearing_error(Pose, Target),
    Adv  = forward_speed(Dist, Err),
    Turn = turn_speed(Err),
    {Adv, Turn}.

%% distance/2 – Euclidean distance from Pose to Target.
distance({X, Y, _}, {Tx, Ty}) ->
    Dx = Tx - X, Dy = Ty - Y,
    math:sqrt(Dx*Dx + Dy*Dy);
distance({X, Y}, {Tx, Ty}) ->
    Dx = Tx - X, Dy = Ty - Y,
    math:sqrt(Dx*Dx + Dy*Dy).

%% bearing_error/2 – signed heading error to Target in degrees, range [-180, 180].
%% Positive = target is to the robot's left.
bearing_error({X, Y, Theta_Deg}, {Tx, Ty}) ->
    Bearing_Deg = math:atan2(Ty - Y, Tx - X) * 180.0 / math:pi(),
    norm_angle(Bearing_Deg - Theta_Deg).

%% forward_speed/2 – combines distance taper with heading gate.
forward_speed(Dist, Err_Deg) ->
    approach_speed(Dist) * heading_factor(abs(Err_Deg)).

%% turn_speed/1 – proportional steering on heading error, clamped.
turn_speed(Err_Deg) ->
    clamp(?TURN_GAIN * Err_Deg, -?MAX_TURN_V, ?MAX_TURN_V).

%% arrived/2 – has the robot reached this waypoint?
arrived(Pose, Target) ->
    distance(Pose, Target) < ?ARRIVAL_RADIUS.

%% heading_factor/1 – piecewise linear gate on forward speed.
%%   |err| ≤ HEADING_FULL_DEG → 1.0 (full speed)
%%   |err| ≥ HEADING_STOP_DEG → 0.0 (rotate only, no translation)
%%   between                  → linear ramp
heading_factor(Abs_Err) ->
    if
        Abs_Err =< ?HEADING_FULL_DEG -> 1.0;
        Abs_Err >= ?HEADING_STOP_DEG -> 0.0;
        true ->
            (?HEADING_STOP_DEG - Abs_Err) /
            (?HEADING_STOP_DEG - ?HEADING_FULL_DEG)
    end.

%% approach_speed/1 – tapers cruise speed down to APPROACH_SPEED as the robot
%% gets within SLOWDOWN_RADIUS of the target.
approach_speed(Dist) ->
    if
        Dist >= ?SLOWDOWN_RADIUS -> ?CRUISE_SPEED;
        true ->
            T = Dist / ?SLOWDOWN_RADIUS,            % 0 at target, 1 at slowdown ring
            ?APPROACH_SPEED + (?CRUISE_SPEED - ?APPROACH_SPEED) * T
    end.


%%% ═══════════════════════════════════════════════════════════════════════════
%%% Odometry & retargeting
%%% ═══════════════════════════════════════════════════════════════════════════

%% update_pose/2 – integrate wheel speeds into a new pose.
%% Uses midpoint heading for 2nd-order accuracy.
update_pose({X, Y, Theta_Deg}, {Dt, Speed_L, Speed_R}) ->
    D_Dist        = (Speed_L + Speed_R) / 2.0 * Dt,
    D_Theta_Rad   = ((Speed_R - Speed_L) / ?WHEEL_BASE) * Dt,
    D_Theta_Deg   = D_Theta_Rad * 180.0 / math:pi(),
    Theta_New     = norm_angle(Theta_Deg + D_Theta_Deg),
    Theta_Mid_Rad = (Theta_Deg + D_Theta_Deg / 2.0) * math:pi() / 180.0,
    X_New = X + D_Dist * math:cos(Theta_Mid_Rad),
    Y_New = Y + D_Dist * math:sin(Theta_Mid_Rad),
    {X_New, Y_New, Theta_New}.

%% retarget/2 – drop any leading waypoints that are already within arrival
%% radius. Recursive so multiple bunched-up waypoints clear in one tick.
retarget([], _Pose) -> [];
retarget([Target | Rest] = Targets, Pose) ->
    case arrived(Pose, Target) of
        true  -> retarget(Rest, Pose);
        false -> Targets
    end.

%% next_status/3 – pure transition function for the state machine.
next_status(idle,     Forward, Robot_Up) when Forward, Robot_Up    -> running;
next_status(idle,     _,       _)                                  -> idle;
next_status(running,  _,       Robot_Up) when not Robot_Up         -> paused;
next_status(running,  _,       _)                                  -> running;
next_status(paused,   Forward, Robot_Up) when Forward, Robot_Up    -> running;
next_status(paused,   _,       _)                                  -> paused;
next_status(finished, _,       _)                                  -> finished.


%%% ═══════════════════════════════════════════════════════════════════════════
%%% Utility
%%% ═══════════════════════════════════════════════════════════════════════════

%% ramp/5 – move Prev toward Target by at most Accel*Dt (when magnitude is
%% growing) or Brake*Dt (when magnitude is shrinking toward zero or reversing).
ramp(Prev, Target, Accel, Brake, Dt) ->
    DV = Target - Prev,
    Slowing =
        (Prev > 0.0 andalso Target < Prev) orelse
        (Prev < 0.0 andalso Target > Prev),
    Limit = if Slowing -> Brake * Dt; true -> Accel * Dt end,
    if
        DV >  Limit -> Prev + Limit;
        DV < -Limit -> Prev - Limit;
        true        -> Target
    end.

%% norm_angle/1 – wrap an angle (degrees) into (-180, 180].
norm_angle(Angle) ->
    A = math:fmod(Angle, 360.0),
    if A >  180.0 -> A - 360.0;
       A < -180.0 -> A + 360.0;
       true       -> A
    end.

%% clamp/3 – constrain Val to [Min, Max].
clamp(Val, Min, Max) ->
    if Val < Min -> Min;
       Val > Max -> Max;
       true      -> Val
    end.
