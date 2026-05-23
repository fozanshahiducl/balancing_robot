-module(csv_logger).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Serial trajectory logger.
%%%
%%% Rows are rate-limited (every LOG_RATE_DIV ticks) and emitted immediately
%%% to the serial port for real-time capture by the Python script.
%%% No SD card buffering.
%%%
%%% CSV column order:
%%%   T_ms, lifecycle, substate, x, y, speed,
%%%   adv_v, turn_v, dist_wp, wps_left, yaw_odo, yaw_gyro, gx_dps
%%% ═══════════════════════════════════════════════════════════════════════════

-export([reset/0, append/0, emit_serial/1]).

-define(LOG_RATE_DIV, 10).   %% emit every Nth tick; at 300 Hz → 30 Hz serial rate

-define(CSV_HEADER,
    "T_ms,lifecycle,substate,x,y,speed,"
    "adv_v,turn_v,dist_wp,wps_left,yaw_odo,yaw_gyro,gx_dps\n").

%%% ─── Public API ─────────────────────────────────────────────────────────────

%% Call on lifecycle idle→running to reset the tick counter and emit the header.
-spec reset() -> ok.
reset() ->
    put(csv_t0,   erlang:system_time(millisecond)),
    put(csv_tick, 0),
    io:format("===LOG_START_TRAJ===~n"),
    io:format("TLOG_HEADER," ?CSV_HEADER),
    ok.

%% Rate-limit check: returns 'logged' every LOG_RATE_DIV ticks, 'skipped' otherwise.
%% Caller should gate emit_serial/1 on the 'logged' return.
-spec append() -> logged | skipped.
append() ->
    Tick = case get(csv_tick) of undefined -> 0; T -> T end,
    put(csv_tick, Tick + 1),
    if Tick rem ?LOG_RATE_DIV =:= 0 -> logged;
       true                          -> skipped
    end.

%% Write one CSV row to serial immediately (for the Python capture script).
-spec emit_serial(map()) -> ok.
emit_serial(Row) ->
    T0   = case get(csv_t0) of undefined -> erlang:system_time(millisecond); V -> V end,
    T_ms = erlang:system_time(millisecond) - T0,
    #{lifecycle  := LC,
      substate   := SS,
      x          := X,
      y          := Y,
      speed      := Speed,
      adv_v      := Adv,
      turn_v     := Turn,
      dist_wp    := Dist,
      wps_left   := WpsL,
      yaw_odo    := YO,
      yaw_gyro   := YG,
      gx_dps     := GxC} = Row,
    io:format("TLOG,~p,~p,~p,~.2f,~.2f,~.2f,~.2f,~.2f,~.2f,~p,~.2f,~.2f,~.2f~n",
              [T_ms, LC, SS, X, Y, Speed, Adv, Turn, Dist, WpsL, YO, YG, GxC]).
