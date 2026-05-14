-module(trajectory_layer).
-export([init/0, step/2]).

%% --- PHYSICAL CONSTANTS ---
-define(WHEEL_BASE, 19.3).      % The distance between the two wheels in cm

%% --- SPEED LIMITS ---
-define(MAX_SPEED, 8.0).        % Max forward speed (cm/s). Set to 3.0 for slow movement.
-define(MAX_TURN_SPEED, 8.0).   % Max rotation speed (cm/s differential). Set to 3.0 for slow turns.

%% --- ACCELERATION & BRAKING ---
-define(ACCEL_RATE, 5.0).       % Speeding up rate (cm/s^2). 1.0 is very gentle.
-define(BRAKE_RATE, 20.0).      % Braking rate (cm/s^2). 20.0 allows instant stopping.

-define(TURN_ACCEL_RATE, 5.0).  % Turn start rate (cm/s^2). 1.0 is very gentle.
-define(TURN_BRAKE_RATE, 20.0). % Turn stop rate (cm/s^2). 20.0 prevents overshoot loops.

%% --- DECELERATION WINDOWS ---
-define(DECEL_DIST, 30.0).      % Start slowing down 30cm before the waypoint.
-define(TURN_DECEL_ANG, 60.0).  % Start slowing rotation 60 degrees before the target angle.

%% --- THRESHOLDS ---
-define(DIST_THRESHOLD, 2.0).   % Reach waypoint if within 2cm.
-define(ANG_THRESHOLD, 2.0).    % Consider heading reached if within 2 degrees.

%% --- WAYPOINTS ---
-define(WAYPOINTS, [{0.0, 50.0}, {50.0, 50.0}, {50.0, 0.0}, {0.0, 0.0}]).

%% Initialize the trajectory state
init() ->
    {idle, 0.0, 0.0, 0.0, ?WAYPOINTS, none, 0.0, 0.0}.

%% Main step function called every loop iteration
step(State, {Dt, Speed_L, Speed_R, Forward, Robot_Up}) ->
    {Status, X, Y, Theta, WPs, Target, Prev_Adv_V, Prev_Turn_V} = State,

    %% 1. ODOMETRY: Calculate movement since last loop
    D_Dist = (Speed_L + Speed_R) / 2.0 * Dt,                             % Forward distance traveled
    D_Theta_Rad = ((Speed_R - Speed_L) / ?WHEEL_BASE) * Dt,              % Change in heading in radians
    D_Theta_Deg = D_Theta_Rad * 180.0 / math:pi(),                       % Change in heading in degrees
    
    Theta_New = norm_angle(Theta + D_Theta_Deg),                         % Update heading with looparound
    Theta_Avg_Rad = (Theta + D_Theta_Deg / 2.0) * math:pi() / 180.0,     % Average heading for trig
    X_New = X + D_Dist * math:cos(Theta_Avg_Rad),                        % Update X coordinate
    Y_New = Y + D_Dist * math:sin(Theta_Avg_Rad),                        % Update Y coordinate

    %% 2. STATE MACHINE: Handle Idle, Running, and Paused states
    New_Status = case Status of
        idle ->
            if Forward and Robot_Up -> running;                          % Start if standing and Forward pressed
               true -> idle
            end;
        running ->
            if not Robot_Up -> paused;                                   % Pause if the robot falls down
               true -> running
            end;
        paused ->
            if Robot_Up and Forward -> running;                          % Resume if standing and Forward pressed
               true -> paused
            end;
        finished -> finished
    end,

    %% 3. COORDINATE RESET: If just starting, set current spot as 0,0
    {X_Track, Y_Track, Theta_Track} = if
        (Status == idle) and (New_Status == running) -> {0.0, 0.0, 0.0};
        true -> {X_New, Y_New, Theta_New}
    end,

    %% 4. WAYPOINT LOGIC: Calculate where we want to go
    {Adv_V_Req, Turn_V_Req, WPs_New, Target_New} = if
        New_Status == running ->
            Active_Target = case Target of
                none -> 
                    case WPs of
                        [First | Rest] -> {First, Rest};                 % Load next waypoint from list
                        [] -> {none, []}
                    end;
                _ -> {Target, WPs}
            end,

            case Active_Target of
                {none, _} -> {0.0, 0.0, [], none};                        % No more waypoints
                {{Tx, Ty}, Rem_WPs} ->
                    Dx = Tx - X_Track,                                   % X distance to target
                    Dy = Ty - Y_Track,                                   % Y distance to target
                    Dist = math:sqrt(Dx*Dx + Dy*Dy),                     % Straight line distance
                    Angle_To_Target = math:atan2(Dy, Dx) * 180.0 / math:pi(), % Goal angle
                    Heading_Err = norm_angle(Angle_To_Target - Theta_Track),  % Angle error

                    if Dist < ?DIST_THRESHOLD ->
                        case Rem_WPs of
                            [] -> {0.0, 0.0, [], none};                  % Reached final destination
                            [Next | Tail] -> {0.0, 0.0, Tail, Next}      % Reached waypoint, move to next
                        end;
                    true ->
                        %% --- BLENDED CONTROL LAW ---
                        
                        % Calculate turn speed based on angle error (slows down as it gets closer)
                        Req_Turn_Raw = (Heading_Err / ?TURN_DECEL_ANG) * ?MAX_TURN_SPEED,
                        Req_Turn = clamp(Req_Turn_Raw, -?MAX_TURN_SPEED, ?MAX_TURN_SPEED),

                        % Calculate forward speed based on angle (stop moving if angle error is high)
                        Heading_Factor = max(0.0, 1.0 - (abs(Heading_Err) / 45.0)),
                        % Calculate forward speed based on distance (slow down as we approach waypoint)
                        Req_Adv = min(?MAX_SPEED, (Dist / ?DECEL_DIST) * ?MAX_SPEED) * Heading_Factor,

                        {Req_Adv, Req_Turn, Rem_WPs, {Tx, Ty}}
                    end
            end;
        true ->
            {0.0, 0.0, WPs, Target}                                      % Stop output if not running
    end,

    Final_Status = if
        (New_Status == running) and (Target_New == none) -> finished;
        true -> New_Status
    end,

    %% 5. RAMPS: Apply separate Acceleration (Gas) and Braking (Brakes) rates
    
    % Forward Ramp
    DV = Adv_V_Req - Prev_Adv_V,
    Adv_V = if
        DV > 0.0 -> Prev_Adv_V + min(DV, ?ACCEL_RATE * Dt);              % Speeding up
        DV < 0.0 -> Prev_Adv_V + max(DV, -?BRAKE_RATE * Dt);             % Braking (Powerful stop)
        true -> Prev_Adv_V
    end,

    % Turning Ramp
    DTurn = Turn_V_Req - Prev_Turn_V,
    Turn_V = if
        DTurn > 0.0 -> Prev_Turn_V + min(DTurn, ?TURN_ACCEL_RATE * Dt);  % Starting turn
        DTurn < 0.0 -> Prev_Turn_V + max(DTurn, -?TURN_BRAKE_RATE * Dt); % Braking turn (Prevents overshoot)
        true -> Prev_Turn_V
    end,

    State_New = {Final_Status, X_Track, Y_Track, Theta_Track, WPs_New, Target_New, Adv_V, Turn_V},
    {Adv_V, -Turn_V, State_New}.                                         % Return velocities (Turn negated for robot)

%% Helper: Normalize angle to [-180, 180]
norm_angle(Angle) ->
    A = math:fmod(Angle, 360.0),
    if A > 180.0 -> A - 360.0;
       A < -180.0 -> A + 360.0;
       true -> A
    end.

%% Helper: Clamp value between Min and Max
clamp(Val, Min, Max) ->
    if Val < Min -> Min;
       Val > Max -> Max;
       true -> Val
    end.
