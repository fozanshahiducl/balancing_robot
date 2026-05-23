-module(gyro_yaw).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Gyro-integrated yaw estimate.
%%%
%%% Stateless: caller owns the previous-tick yaw value and passes it in.
%%% Mirrors odometry.erl in shape.
%%%
%%% Axis: out_x_g — yaw rate, since up = +X in the robot frame
%%% (forward = -Z, right = +Y, up = +X). Existing pitch convention uses
%%% out_y_g (right axis), which confirms IMU chip frame == robot frame.
%%%
%%% Pipeline per tick:
%%%   1. Subtract bias  Gx0 captured at startup calibration
%%%   2. Project onto world-vertical using current pitch estimate
%%%   3. Euler-integrate and normalize to [-180, 180] deg
%%%
%%% Log-only in this version — not consumed by control.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([init/0, reset/0, step/5, config_string/0]).

-define(DEG_TO_RAD, math:pi()/180.0).

-spec init() -> float().
init() -> 0.0.

-spec reset() -> float().
reset() -> 0.0.

%% Yaw_Prev_Deg, Gx_Dps, Gx0_Dps, Pitch_Deg in deg / deg-per-second; Dt in s.
%% Returns {New yaw [deg, normalized], bias-corrected yaw rate [dps]}.
-spec step(float(), float(), float(), float(), float()) -> {float(), float()}.
step(Yaw_Prev_Deg, Gx_Dps, Gx0_Dps, Pitch_Deg, Dt) ->
    Gx_Corr   = Gx_Dps - Gx0_Dps,
    Pitch_Rad = Pitch_Deg * ?DEG_TO_RAD,
    %% cos(pitch) projects body-X gyro onto world-vertical. <1.5% effect at
    %% the angles a balancing robot operates at; included for correctness.
    Rate_Dps  = Gx_Corr * math:cos(Pitch_Rad),
    Yaw_New   = norm_angle(Yaw_Prev_Deg + Rate_Dps * Dt),
    {Yaw_New, Gx_Corr}.

-spec config_string() -> string().
config_string() ->
    "gyro_yaw: axis=out_x_g, pitch_proj=on, bias_corr=on".

%%% ─── Internal ───────────────────────────────────────────────────────────────

norm_angle(Angle) ->
    A = math:fmod(Angle, 360.0),
    if A >  180.0 -> A - 360.0;
       A < -180.0 -> A + 360.0;
       true       -> A
    end.
