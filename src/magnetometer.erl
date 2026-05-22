-module(magnetometer).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Magnetometer hardware module.
%%%
%%% Two responsibilities — no sensor fusion, no odometry:
%%%
%%%   1. read/1 — read the LSM9DS1 over SPI2, apply hard-iron correction and
%%%      a first-order soft-iron scale, return the current compass heading in
%%%      degrees [0, 360).
%%%
%%%   2. calibrate/3 — one tick of the calibration sub-state machine. Triggered
%%%      by L+R button combo. The user picks up the robot and tumbles it
%%%      through varied 3-D orientations while the module collects raw mag
%%%      samples and learns per-axis hard-iron offsets from the bounding-box
%%%      centre of the sphere they trace out. Tick-driven so the main loop
%%%      keeps running.
%%%
%%% Cal sub-state machine:
%%%
%%%   not_cal ──(L+R edge)─────────────────────────────▶ settling
%%%   settling ──(CAL_SETTLE_MS elapsed)───────────────▶ spinning
%%%   spinning ──(sample count + range + sphericity)───▶ settling_done
%%%   settling_done ──(10 s grace)─────────────────────▶ done
%%%   spinning ──(CAL_TIMEOUT_MS)──────────────────────▶ failed
%%%   spinning ──(L+R edge)────────────────────────────▶ not_cal
%%%   done | failed ──(L+R edge)───────────────────────▶ settling   (restart)
%%%
%%% Externally (cal_status/1 and the Output map from calibrate/3) the
%%% settling_done substate is reported as spinning, so callers see the same
%%% atom set as before and no main_loop changes are needed for the new grace
%%% period. Override stays {0.0, 0.0} for the whole spinning+settling_done
%%% window, locking the motors idle.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([init/0, read/2, calibrate/3,
         last_heading/1, cal_status/1, override/1]).

%% ─── Cal timing ──────────────────────────────────────────────────────────────
-define(CAL_SETTLE_MS,           500).    %% ms to wait (settling → spinning)
-define(CAL_TIMEOUT_MS,        90000).    %% ms: if spinning longer → failed
-define(CAL_MIN_ELAPSED_MS,     5000).    %% ms minimum before auto-complete
-define(CAL_SETTLING_DONE_MS,  10000).    %% ms grace after sphere fit so the
                                          %% user can set the robot back down
                                          %% before balancing re-engages.

%% ─── Cal quality gates ──────────────────────────────────────────────────────
%% Brussels-tuned. Earth's total field ≈ 0.49 gauss (lat 50.85°N), giving
%% peak-to-peak ≈ 0.98 gauss per axis on a full sphere tumble. 0.80 ≈ 82% of
%% that — strong quality bar. Revisit for deployment elsewhere.
-define(CAL_MIN_RANGE_GAUSS,    0.80).
-define(CAL_SPHERICITY_MAX,     1.20).    %% max_range / min_range ≤ this
-define(CAL_MIN_SAMPLES,         150).

%% ─── Cooldown ────────────────────────────────────────────────────────────────
-define(LR_COOLDOWN_MS,          600).    %% min ms between L+R cal toggles

-record(mag_cal, {
    cal_status         = not_cal   :: not_cal | settling | spinning
                                    | settling_done | done | failed,
    %% Hard/soft-iron corrections (learned by cal, applied by read/1).
    %% Defaults from six bench cals (2026-05-22): hard-iron stable across
    %% runs, soft-iron ~10% Z excess remains (sphericity ~1.10 floor).
    x_off              = -0.05     :: float(),
    y_off              =  0.07     :: float(),
    z_off              =  0.36     :: float(),   %% not used in atan2 — robot
                                                 %% is upright in operation —
                                                 %% but proves cal was 3-D.
    scale_ratio        = 1.0       :: float(),
    %% Range tracking during spinning (robot frame: Mx forward, My right,
    %% Mz up).
    x_min              = 1.0e30    :: float(),
    x_max              = -1.0e30   :: float(),
    y_min              = 1.0e30    :: float(),
    y_max              = -1.0e30   :: float(),
    z_min              = 1.0e30    :: float(),
    z_max              = -1.0e30   :: float(),
    n_samples          = 0         :: non_neg_integer(),
    start_ms           = 0         :: non_neg_integer(),
    settling_done_until_ms = 0     :: non_neg_integer(),
    last_cal_toggle_ms = 0         :: non_neg_integer(),
    %% Last raw read in robot frame (cached so calibrate/3 can run between
    %% hardware reads without re-reading the chip).
    mx_last            = 0.0       :: float(),
    my_last            = 0.0       :: float(),
    mz_last            = 0.0       :: float(),
    %% Last accelerometer triple in robot body frame (forward, right, up),
    %% g units. Cached by calibrate/3 so the final post-cal compass print can
    %% tilt-compensate even though the cal FSM itself doesn't call read/2.
    %% Default {0.0, 0.0, 1.0} = upright; reduces tilt math to body-frame.
    accel_last         = {0.0, 0.0, 1.0} :: {float(), float(), float()},
    %% Last computed heading [0, 360) — returned by last_heading/1 on ticks
    %% that do not perform a fresh hardware read.
    last_heading       = 0.0       :: float()
}).

%%% ─── Public API ─────────────────────────────────────────────────────────────

-spec init() -> #mag_cal{}.
init() -> #mag_cal{}.

%% Perform one hardware read of the LSM9DS1, apply stored hard-iron offsets
%% and the first-order soft-iron scale, then compute a tilt-compensated
%% compass heading using the supplied accelerometer triple. Returns degrees
%% [0, 360).
%%
%% Chip axes: chip-X is mounted vertically (up), chip-Y points right, chip-Z
%% points rear. Robot body frame: Mf forward, Mr right, Mu up.
%%   Mf (robot) = −chip-Z
%%   Mr (robot) = +chip-Y
%%   Mu (robot) = +chip-X
%%
%% The accel triple must already be in the same robot body frame (caller
%% does the chip→robot remap, parallel to how the balance code consumes
%% chip-frame Ax/Az).
-spec read(#mag_cal{}, {float(), float(), float()})
      -> {float(), #mag_cal{}}.
read(State, Accel) ->
    [Mx_chip, My_chip, Mz_chip] =
        pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]),
    Mf = -Mz_chip,   %% robot forward
    Mr =  My_chip,   %% robot right
    Mu =  Mx_chip,   %% robot up
    Mf_C = Mf - State#mag_cal.x_off,
    Mr_C = (Mr - State#mag_cal.y_off) * State#mag_cal.scale_ratio,
    Mu_C = Mu - State#mag_cal.z_off,
    Heading = tilt_compensated_heading(Mf_C, Mr_C, Mu_C, Accel),
    {Heading, State#mag_cal{mx_last      = Mf,
                            my_last      = Mr,
                            mz_last      = Mu,
                            accel_last   = Accel,
                            last_heading = Heading}}.

%% Tilt-compensated heading via vector projection.
%%
%% Inputs are post-hard-iron mag components in the robot body frame
%% (forward, right, up), and the accelerometer triple in the same frame
%% (g units, specific force — so {0,0,+1} when robot is perfectly upright).
%%
%% Approach: derive gravity unit vector g_hat from accel, project the mag
%% vector onto the plane perpendicular to g_hat (horizontal mag Mh), build
%% horizontal forward and right reference axes by similarly projecting the
%% body F axis and taking (g_hat × Fh), then heading = atan2(Mh·Rh, Mh·Fh).
%% The leading minus sign and to_compass_360 wrap match the existing
%% compass-bearing convention (0° = magnetic north, 90° = east).
%%
%% Free-fall guard: if |Accel| < 0.1 g the gravity direction is undefined,
%% so fall back to body-frame YZ heading for that tick.
-spec tilt_compensated_heading(float(), float(), float(),
                               {float(), float(), float()}) -> float().
tilt_compensated_heading(Mf, Mr, Mu, {Af, Ar, Au}) ->
    G = math:sqrt(Af*Af + Ar*Ar + Au*Au),
    if G < 0.1 ->
           %% Free-fall / dropped — gravity direction undefined. Body-frame
           %% fallback (same formula as pre-tilt-comp code).
           to_compass_360(-math:atan2(Mr, Mf) * 180.0 / math:pi());
       true ->
           Gf = -Af / G,  Gr = -Ar / G,  Gu = -Au / G,
           MdotG = Mf*Gf + Mr*Gr + Mu*Gu,
           MhF = Mf - MdotG*Gf,
           MhR = Mr - MdotG*Gr,
           MhU = Mu - MdotG*Gu,
           %% Body forward = (1,0,0); project onto horizontal plane.
           FhF = 1.0 - Gf*Gf,
           FhR =     - Gf*Gr,
           FhU =     - Gf*Gu,
           %% Horizontal right = Fh × g_hat. Order matters: g_hat × Fh would
           %% give horizontal LEFT, flipping the compass to CCW. With this
           %% order the upright reduction matches the body-frame formula's
           %% CW compass sign convention.
           RhF = FhR*Gu - FhU*Gr,
           RhR = FhU*Gf - FhF*Gu,
           RhU = FhF*Gr - FhR*Gf,
           Rad = math:atan2(MhF*RhF + MhR*RhR + MhU*RhU,
                            MhF*FhF + MhR*FhR + MhU*FhU),
           to_compass_360(-Rad * 180.0 / math:pi())
    end.

%% One tick of the calibration sub-state machine.
%%
%% Input map keys:
%%   lr_edge      — boolean: L+R button rising edge this tick
%%   robot_up     — boolean (ignored; hand-tumble cal does not require upright)
%%   do_mag_read  — boolean: a fresh hardware read happened this tick. Sample
%%                  collection and the per-sample CSV print are gated on this
%%                  to avoid duplicating the same cached point every tick.
%%   accel        — {Af, Ar, Au} robot-frame accelerometer triple. Optional;
%%                  cached into accel_last so the final post-cal compass
%%                  print can tilt-compensate. Falls back to previous cached
%%                  value if absent.
%%
%% Returns {NewState, Output} where Output is
%%   #{cal_status => atom(), override => undefined | {float(), float()}}.
%% Output cal_status maps settling_done → spinning for external observers.
-spec calibrate(map(), #mag_cal{}, non_neg_integer()) -> {#mag_cal{}, map()}.
calibrate(Input, State, Now_ms) ->
    #{lr_edge := LR_Edge, robot_up := _Robot_Up,
      do_mag_read := Do_Mag_Read} = Input,
    Accel = maps:get(accel, Input, State#mag_cal.accel_last),
    Cooldown_Ok = (Now_ms - State#mag_cal.last_cal_toggle_ms) >= ?LR_COOLDOWN_MS,
    State0 = State#mag_cal{accel_last = Accel},
    Mx = State0#mag_cal.mx_last,
    My = State0#mag_cal.my_last,
    Mz = State0#mag_cal.mz_last,
    {_Cal_New, State1} =
        cal_step(State0, Mx, My, Mz, LR_Edge, Do_Mag_Read, Cooldown_Ok, Now_ms),
    Output = #{cal_status => external_status(State1#mag_cal.cal_status),
               override   => override(State1)},
    {State1, Output}.

%% Last heading computed by read/1 (returned without performing a new read).
-spec last_heading(#mag_cal{}) -> float().
last_heading(S) -> S#mag_cal.last_heading.

-spec cal_status(#mag_cal{}) -> atom().
cal_status(S) -> external_status(S#mag_cal.cal_status).

-spec override(#mag_cal{}) -> undefined | {float(), float()}.
override(S) ->
    case S#mag_cal.cal_status of
        spinning       -> {0.0, 0.0};   %% motors idle — user hand-tumbles
        settling_done  -> {0.0, 0.0};   %% motors idle — user sets robot down
        _              -> undefined
    end.

%% External-visible cal_status: hides the settling_done substate so callers
%% (main_loop LED dispatch, Cal_Active gate) see the same atom set as before.
external_status(settling_done) -> spinning;
external_status(S)             -> S.

%%% ─── Cal sub-state machine ──────────────────────────────────────────────────

cal_step(S, Mx, My, Mz, LR_Edge, Do_Mag_Read, Cooldown_Ok, Now_ms) ->
    CS = S#mag_cal.cal_status,
    case CS of
        not_cal ->
            if LR_Edge andalso Cooldown_Ok ->
                   io:format("[Mag] not_cal -> settling~n"),
                   {settling, S#mag_cal{cal_status         = settling,
                                        start_ms           = Now_ms,
                                        last_cal_toggle_ms = Now_ms}};
               true ->
                   {not_cal, S}
            end;

        settling ->
            if Now_ms - S#mag_cal.start_ms >= ?CAL_SETTLE_MS ->
                   io:format("[Mag] settling -> spinning "
                             "(hand-tumble robot through all orientations)~n"),
                   io:format("MAGCAL_HDR,t_ms,Mx,My,Mz~n"),
                   {spinning, S#mag_cal{
                       cal_status      = spinning,
                       start_ms        = Now_ms,
                       n_samples       = 0,
                       x_min = 1.0e30,  x_max = -1.0e30,
                       y_min = 1.0e30,  y_max = -1.0e30,
                       z_min = 1.0e30,  z_max = -1.0e30}};
               true ->
                   {settling, S}
            end;

        spinning ->
            if LR_Edge andalso Cooldown_Ok ->
                   io:format("[Mag] spinning -> not_cal (user cancelled)~n"),
                   {not_cal, S#mag_cal{
                       cal_status         = not_cal,
                       last_cal_toggle_ms = Now_ms}};
               Now_ms - S#mag_cal.start_ms >= ?CAL_TIMEOUT_MS ->
                   io:format("[Mag] cal FAILED after ~p ms -- "
                             "Range_X=~.4f Range_Y=~.4f Range_Z=~.4f "
                             "n_samples=~p~n",
                             [Now_ms - S#mag_cal.start_ms,
                              S#mag_cal.x_max - S#mag_cal.x_min,
                              S#mag_cal.y_max - S#mag_cal.y_min,
                              S#mag_cal.z_max - S#mag_cal.z_min,
                              S#mag_cal.n_samples]),
                   {failed, S#mag_cal{cal_status = failed}};
               Do_Mag_Read ->
                   S2 = collect_sample(Mx, My, Mz, S, Now_ms),
                   case check_cal_done(S2, Now_ms) of
                       true ->
                           Until = Now_ms + ?CAL_SETTLING_DONE_MS,
                           io:format("[Mag] spinning -> settling_done "
                                     "(set robot down within ~p s)~n",
                                     [?CAL_SETTLING_DONE_MS div 1000]),
                           {settling_done,
                            S2#mag_cal{cal_status             = settling_done,
                                       settling_done_until_ms = Until}};
                       false ->
                           {spinning, S2}
                   end;
               true ->
                   %% No fresh sample this tick — just hold state.
                   {spinning, S}
            end;

        settling_done ->
            if Now_ms >= S#mag_cal.settling_done_until_ms ->
                   S3 = compute_corrections(S),
                   Mf_C = S#mag_cal.mx_last - S3#mag_cal.x_off,
                   Mr_C = (S#mag_cal.my_last - S3#mag_cal.y_off)
                              * S3#mag_cal.scale_ratio,
                   Mu_C = S#mag_cal.mz_last - S3#mag_cal.z_off,
                   Compass_Now = tilt_compensated_heading(
                                   Mf_C, Mr_C, Mu_C, S#mag_cal.accel_last),
                   Range_X = S#mag_cal.x_max - S#mag_cal.x_min,
                   Range_Y = S#mag_cal.y_max - S#mag_cal.y_min,
                   Range_Z = S#mag_cal.z_max - S#mag_cal.z_min,
                   Sphericity = sphericity(Range_X, Range_Y, Range_Z),
                   Duration   = Now_ms - S#mag_cal.start_ms
                                    - ?CAL_SETTLING_DONE_MS,
                   io:format("[MagCal] DONE n_samples=~p duration_ms=~p~n",
                             [S#mag_cal.n_samples, Duration]),
                   io:format("[MagCal]   X: min=~.4f max=~.4f range=~.4f "
                             "off=~.4f~n",
                             [S#mag_cal.x_min, S#mag_cal.x_max,
                              Range_X, S3#mag_cal.x_off]),
                   io:format("[MagCal]   Y: min=~.4f max=~.4f range=~.4f "
                             "off=~.4f~n",
                             [S#mag_cal.y_min, S#mag_cal.y_max,
                              Range_Y, S3#mag_cal.y_off]),
                   io:format("[MagCal]   Z: min=~.4f max=~.4f range=~.4f "
                             "off=~.4f~n",
                             [S#mag_cal.z_min, S#mag_cal.z_max,
                              Range_Z, S3#mag_cal.z_off]),
                   io:format("[MagCal]   scale_ratio=~.4f sphericity=~.3f~n",
                             [S3#mag_cal.scale_ratio, Sphericity]),
                   io:format("[MagCal]   compass_now=~.2f deg~n",
                             [Compass_Now]),
                   {done, S3#mag_cal{cal_status   = done,
                                     last_heading = Compass_Now}};
               true ->
                   {settling_done, S}
            end;

        done ->
            if LR_Edge andalso Cooldown_Ok ->
                   io:format("[Mag] done -> settling (restart)~n"),
                   {settling, S#mag_cal{
                       cal_status         = settling,
                       start_ms           = Now_ms,
                       last_cal_toggle_ms = Now_ms}};
               true ->
                   {done, S}
            end;

        failed ->
            if LR_Edge andalso Cooldown_Ok ->
                   io:format("[Mag] failed -> settling (restart)~n"),
                   {settling, S#mag_cal{
                       cal_status         = settling,
                       start_ms           = Now_ms,
                       last_cal_toggle_ms = Now_ms}};
               true ->
                   {failed, S}
            end
    end.

%% Update per-axis min/max, bump sample counter, emit one CSV row for the
%% offline sphere-fit script. Called only when a fresh hardware read landed
%% this tick.
collect_sample(Mx, My, Mz, S, Now_ms) ->
    X_Min = min(Mx, S#mag_cal.x_min), X_Max = max(Mx, S#mag_cal.x_max),
    Y_Min = min(My, S#mag_cal.y_min), Y_Max = max(My, S#mag_cal.y_max),
    Z_Min = min(Mz, S#mag_cal.z_min), Z_Max = max(Mz, S#mag_cal.z_max),
    N     = S#mag_cal.n_samples + 1,
    io:format("MAGCAL,~p,~.4f,~.4f,~.4f~n", [Now_ms, Mx, My, Mz]),
    S#mag_cal{x_min = X_Min, x_max = X_Max,
              y_min = Y_Min, y_max = Y_Max,
              z_min = Z_Min, z_max = Z_Max,
              n_samples = N}.

check_cal_done(S, Now_ms) ->
    Range_X = S#mag_cal.x_max - S#mag_cal.x_min,
    Range_Y = S#mag_cal.y_max - S#mag_cal.y_min,
    Range_Z = S#mag_cal.z_max - S#mag_cal.z_min,
    Elapsed = Now_ms - S#mag_cal.start_ms,
    (Elapsed >= ?CAL_MIN_ELAPSED_MS)
        andalso (S#mag_cal.n_samples >= ?CAL_MIN_SAMPLES)
        andalso (Range_X >= ?CAL_MIN_RANGE_GAUSS)
        andalso (Range_Y >= ?CAL_MIN_RANGE_GAUSS)
        andalso (Range_Z >= ?CAL_MIN_RANGE_GAUSS)
        andalso (sphericity(Range_X, Range_Y, Range_Z) =< ?CAL_SPHERICITY_MAX).

%% max(Rx, Ry, Rz) / min(Rx, Ry, Rz). 1.0 = perfect sphere, larger = pancake.
%% Guarded against zero — returns a huge value so the check fails closed.
sphericity(Rx, Ry, Rz) ->
    Lo = lists:min([Rx, Ry, Rz]),
    Hi = lists:max([Rx, Ry, Rz]),
    if Lo > 0.0 -> Hi / Lo;
       true     -> 1.0e30
    end.

compute_corrections(S) ->
    X_Off = (S#mag_cal.x_max + S#mag_cal.x_min) / 2.0,
    Y_Off = (S#mag_cal.y_max + S#mag_cal.y_min) / 2.0,
    Z_Off = (S#mag_cal.z_max + S#mag_cal.z_min) / 2.0,
    Range_X = S#mag_cal.x_max - S#mag_cal.x_min,
    Range_Y = S#mag_cal.y_max - S#mag_cal.y_min,
    %% Normalize the Y axis onto the X axis range so the cleaned data forms a
    %% unit-ish circle, not an ellipse. Applied as My_Corr = (My - y_off) * Scale.
    Scale = if Range_Y > 0.0 -> Range_X / Range_Y; true -> 1.0 end,
    S#mag_cal{x_off = X_Off, y_off = Y_Off, z_off = Z_Off,
              scale_ratio = Scale}.

%%% ─── Utilities ──────────────────────────────────────────────────────────────

%% Map a degree value (typically from math:atan2's (-180, 180] range) into
%% the compass convention [0, 360). Pure representation change; the underlying
%% rotation direction (CW+ on this chip) is preserved.
to_compass_360(A0) ->
    A = math:fmod(A0, 360.0),
    if A < 0.0 -> A + 360.0;
       true    -> A
    end.
