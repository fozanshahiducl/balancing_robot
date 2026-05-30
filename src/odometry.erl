-module(odometry).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Differential-drive pose integration from wheel speeds.
%%%
%%% integrate/3 updates a #pose{} record every tick using the midpoint-angle
%%% approximation (Euler with heading averaged over the tick).
%%%
%%% Called every tick regardless of trajectory lifecycle so the pose
%%% continues to track reality during paused/idle/finished states.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([integrate/3, config_string/0]).
-include("robot_types.hrl").

-define(WHEEL_BASE, 18.5).   %% cm, left-to-right distance between contact patches

%% Speed_L, Speed_R in cm/s; Dt in s.
-spec integrate({float(), float()}, float(), #pose{}) -> #pose{}.
integrate({Speed_L, Speed_R}, Dt, Pose) ->
    #pose{x = X, y = Y, theta = Theta} = Pose,
    Speed = (Speed_L + Speed_R) / 2.0,
    D_Dist = Speed * Dt,

    D_Theta_Rad = ((Speed_R - Speed_L) / ?WHEEL_BASE) * Dt,
    D_Theta_Deg = D_Theta_Rad * 180.0 / math:pi(),
    Theta_New = norm_angle(Theta + D_Theta_Deg),

    %% Midpoint heading for position integration reduces error vs. using start heading.
    Theta_Mid_Rad = ((Theta + Theta_New) / 2.0) * math:pi() / 180.0,
    X_New = X + D_Dist * math:cos(Theta_Mid_Rad),
    Y_New = Y + D_Dist * math:sin(Theta_Mid_Rad),
    #pose{x = X_New, y = Y_New, theta = Theta_New}.

-spec config_string() -> string().
config_string() ->
    io_lib:format("odometry: WHEEL_BASE=~p cm", [?WHEEL_BASE]).

%%% ─── Internal ───────────────────────────────────────────────────────────────

norm_angle(Angle) ->
    A = math:fmod(Angle, 360.0),
    if A >  180.0 -> A - 360.0;
       A < -180.0 -> A + 360.0;
       true       -> A
    end.
