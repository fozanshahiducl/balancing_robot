-module(mag_filter).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Yaw fusion — complementary filter.
%%%
%%% Hardware reads and calibration live in magnetometer.erl. This module is
%%% purely an algorithm: each tick it integrates the wheel-yaw delta on top
%%% of the previous fused state, then on mag-read ticks pulls (1-α) of the
%%% way toward the absolute compass heading supplied by the caller (α=0.98,
%%% odometry-dominant). Produces yaw_fused (math convention, CCW+ from +X,
%%% degrees), returned to main_loop for use as Yaw_Override in odometry.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([init/0, step/2, set_offset/1, reset_yaw_accumulator/1,
         yaw_fused/1, yaw_mag_raw/1, mag_compass/1, cal_status/1,
         override/1, config_string/0]).

%% ─── Complementary filter ────────────────────────────────────────────────────
%% Applied only on mag-read ticks (do_mag_read=true from main_loop), at the
%% ?MAG_READ_EVERY_N main-loop cadence. Between fusion ticks yaw_fused tracks
%% wheel-odometry directly via the prediction step.
-define(ALPHA,             0.98).    %% wheel-odometry-yaw weight; 0.02 goes to mag

%% ─── Wheel geometry (for internal odometry yaw integration) ─────────────────
-define(WHEEL_BASE,        18.5).    %% cm

-record(mag_state, {
    yaw_mag_raw       = 0.0       :: float(),   %% absolute compass heading from last mag read (alias of mag_compass for back-compat)
    yaw_mag           = 0.0       :: float(),   %% launch-relative compass delta, compass-CW+
    yaw_mag_offset    = undefined :: float() | undefined,
    mag_compass       = 0.0       :: float(),   %% canonical absolute compass heading, deg CW+, [0, 360)
    yaw_odo           = 0.0       :: float(),   %% accumulated wheel-odometry yaw, math CCW+
    yaw_fused         = 0.0       :: float(),   %% output of complementary filter, math CCW+
    cal_status        = not_cal   :: atom(),    %% passthrough from caller (magnetometer:calibrate)
    override          = undefined :: undefined | {float(), float()}  %% passthrough
}).

%%% ─── Public API ─────────────────────────────────────────────────────────────

-spec init() -> #mag_state{}.
init() -> #mag_state{}.

%% Called every tick from main_loop.
%% Input map keys:
%%   mag_compass     — current absolute compass heading [0, 360), supplied by
%%                     magnetometer:read/1 on mag-read ticks (or cached value
%%                     on non-mag-read ticks via magnetometer:last_heading/1).
%%   cal_status      — current cal FSM atom, supplied by magnetometer:calibrate/3.
%%   override        — undefined | {Adv_V, Turn_V}, supplied by magnetometer:calibrate/3
%%                     (the cal-spin velocity command when cal_status =:= spinning).
%%   speed_l, speed_r — wheel speeds cm/s (for odometry yaw integration).
%%   dt              — tick duration in seconds.
%%   do_mag_read     — boolean: this tick is a fresh-mag tick (every Nth main loop).
%%
%% Returns {Output, NewState} where:
%%   Output = #{yaw_fused, yaw_mag_raw, yaw_odo, yaw_mag, mag_compass,
%%              cal_status, override, did_mag_read}
%%   yaw_odo / yaw_fused: degrees, math convention (CCW+, theta=0 along +X)
%%   yaw_mag / mag_compass / yaw_mag_raw: degrees, compass-CW+ (chip native)
-spec step(map(), #mag_state{}) -> {map(), #mag_state{}}.
step(Input, State) ->
    #{mag_compass := Mag_Compass,
      cal_status  := Cal_Status,
      override    := Override,
      speed_l     := SL, speed_r := SR,
      dt          := Dt,
      do_mag_read := Do_Mag_Read} = Input,

    %% ─── 1. Wheel-odometry yaw integration (CCW+, math convention) ──────
    %% (SR - SL) so that turning left (SR > SL) gives a positive delta,
    %% matching odometry.erl. Runs every tick — wheels are precise short-term.
    D_Theta_Rad = ((SR - SL) / ?WHEEL_BASE) * Dt,
    D_Theta_Deg = D_Theta_Rad * 180.0 / math:pi(),
    Yaw_Odo1    = norm_angle(State#mag_state.yaw_odo + D_Theta_Deg),

    %% Prediction step (every tick): keep fused tracking the wheels between
    %% mag corrections, so when the next correction lands it operates on a
    %% prediction that already includes today's wheel motion.
    Yaw_Fused_Pred = norm_angle(State#mag_state.yaw_fused + D_Theta_Deg),

    %% ─── 2. Launch-relative compass delta (every tick, for logging) ──────
    Yaw_Mag1 = case State#mag_state.yaw_mag_offset of
        undefined -> 0.0;
        Off       -> norm_angle(Mag_Compass - Off)
    end,

    %% ─── 3. Complementary filter (only on mag-read ticks) ────────────────
    %% Sign alignment: yaw_odo / yaw_fused are math CCW+ (CCW physical → +),
    %% but the chip's atan2-derived compass grows CW+ (CW physical → +).
    %% Negate Yaw_Mag1 before blending so both inputs share a convention.
    %%
    %% Wrap-safe blend: a naive `α·pred + (1-α)·target` is a linear average,
    %% which goes the LONG way around the circle when pred and target
    %% straddle ±180° (e.g. pred=+179°, target=-179° are 2° apart but the
    %% linear blend pulls toward 0°, not 180°). Instead, compute the shortest
    %% signed delta from pred to target and step (1-α) of that delta. Same
    %% time-constant; just expressed in a form that respects the topology.
    Yaw_Fused1 = case {State#mag_state.yaw_mag_offset, Do_Mag_Read} of
        {undefined, _} -> Yaw_Fused_Pred;                    %% no reference yet
        {_, false}     -> Yaw_Fused_Pred;                    %% not a fusion tick
        {_, true}      ->
            Aligned_Mag = -Yaw_Mag1,
            Diff        = norm_angle(Aligned_Mag - Yaw_Fused_Pred),
            norm_angle(Yaw_Fused_Pred + (1.0 - ?ALPHA) * Diff)
    end,

    State1 = State#mag_state{
        yaw_odo     = Yaw_Odo1,
        yaw_fused   = Yaw_Fused1,
        yaw_mag_raw = Mag_Compass,      %% back-compat alias
        yaw_mag     = Yaw_Mag1,
        mag_compass = Mag_Compass,
        cal_status  = Cal_Status,
        override    = Override
    },

    Output = #{yaw_fused    => Yaw_Fused1,
               yaw_mag_raw  => Mag_Compass,
               yaw_odo      => Yaw_Odo1,
               yaw_mag      => Yaw_Mag1,
               mag_compass  => Mag_Compass,
               cal_status   => Cal_Status,
               override     => Override,
               did_mag_read => Do_Mag_Read},
    {Output, State1}.

%% Capture the current mag compass reading as the launch-relative zero.
%% Called by main_loop on idle→running and finished→running restarts —
%% specifically on a mag-read tick, so mag_compass is guaranteed fresh.
-spec set_offset(#mag_state{}) -> #mag_state{}.
set_offset(State) ->
    State#mag_state{
        yaw_mag_offset = State#mag_state.mag_compass,
        yaw_odo        = 0.0,
        yaw_fused      = 0.0
    }.

%% Reset the yaw accumulator (e.g. after a fall) without touching the offset.
-spec reset_yaw_accumulator(#mag_state{}) -> #mag_state{}.
reset_yaw_accumulator(State) ->
    State#mag_state{yaw_odo = 0.0, yaw_fused = 0.0}.

%% Field accessors (let main_loop stay opaque to the record internals).
-spec yaw_fused(#mag_state{}) -> float().
yaw_fused(S)   -> S#mag_state.yaw_fused.

-spec yaw_mag_raw(#mag_state{}) -> float().
yaw_mag_raw(S) -> S#mag_state.yaw_mag_raw.

-spec mag_compass(#mag_state{}) -> float().
mag_compass(S) -> S#mag_state.mag_compass.

-spec cal_status(#mag_state{}) -> atom().
cal_status(S)  -> S#mag_state.cal_status.

-spec override(#mag_state{}) -> undefined | {float(), float()}.
override(S)    -> S#mag_state.override.

-spec config_string() -> string().
config_string() ->
    io_lib:format("mag_filter: ALPHA=~p WHEEL_BASE=~p", [?ALPHA, ?WHEEL_BASE]).

%%% ─── Utilities ──────────────────────────────────────────────────────────────

norm_angle(A0) ->
    A = math:fmod(A0, 360.0),
    if A >  180.0 -> A - 360.0;
       A < -180.0 -> A + 360.0;
       true       -> A
    end.
