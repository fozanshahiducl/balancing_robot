-module(spline).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Catmull-Rom spline generation.
%%%
%%% build_path/2 takes an ordered list of {X,Y} waypoints and returns a dense
%%% list of {X,Y} subpoints suitable for pure_pursuit lookahead.
%%%
%%% The first and last waypoints are reflected outward to create phantom
%%% control points so every segment starts and ends with the correct tangent.
%%%
%%% build_path_continuing/3 is the mid-cruise rebuild variant. Instead of
%%% reflecting the second waypoint to seed the initial tangent, it takes an
%%% explicit Seed control point — typically the second-last visited waypoint,
%%% a point the robot has already passed through. That preserves the tangent
%%% at the splice with the previous path so pure_pursuit doesn't see a step.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([build_path/2, build_path_continuing/3, config_string/0]).

-define(SPLINE_RES, 30).   %% subpoints per segment

%% Number of subpoints depends on waypoint count and SPLINE_RES.
-spec build_path([{float(), float()}], pos_integer()) -> [{float(), float()}].
build_path(WPs, _Res) ->
    [P1, P2 | _] = WPs,
    P_Last = lists:last(WPs),
    P_Prev = lists:nth(length(WPs) - 1, WPs),
    Extended = [reflect(P1, P2) | WPs] ++ [reflect(P_Last, P_Prev)],
    sample_all_segments(Extended).

%% Mid-cruise rebuild. Seed replaces the leading reflected phantom; the
%% trailing phantom is still computed by reflection. Degenerates safely on
%% short waypoint lists (Catmull-Rom needs two real control points minimum).
-spec build_path_continuing({float(), float()},
                            [{float(), float()}],
                            pos_integer()) ->
    [{float(), float()}].
build_path_continuing(_Seed, [], _Res) -> [];
build_path_continuing(_Seed, [_Single], _Res) -> [];
build_path_continuing(Seed, WPs, _Res) ->
    P_Last = lists:last(WPs),
    P_Prev = lists:nth(length(WPs) - 1, WPs),
    Extended = [Seed | WPs] ++ [reflect(P_Last, P_Prev)],
    sample_all_segments(Extended).

-spec config_string() -> string().
config_string() ->
    io_lib:format("spline: SPLINE_RES=~p", [?SPLINE_RES]).

%%% ─── Internal ───────────────────────────────────────────────────────────────

reflect({X1, Y1}, {X2, Y2}) ->
    {2.0 * X1 - X2, 2.0 * Y1 - Y2}.

sample_all_segments([P0, P1, P2, P3 | Rest]) ->
    sample_segment(P0, P1, P2, P3) ++ sample_all_segments([P1, P2, P3 | Rest]);
sample_all_segments(_) ->
    [].

sample_segment(P0, P1, P2, P3) ->
    N = ?SPLINE_RES,
    [catmull_rom(P0, P1, P2, P3, float(I) / float(N)) || I <- lists:seq(0, N - 1)].

catmull_rom({X0,Y0}, {X1,Y1}, {X2,Y2}, {X3,Y3}, T) ->
    T2 = T*T, T3 = T2*T,
    X = 0.5 * ((2.0*X1)
              + (-X0 + X2)                        * T
              + ( 2.0*X0 - 5.0*X1 + 4.0*X2 - X3) * T2
              + (-X0 + 3.0*X1 - 3.0*X2 + X3)      * T3),
    Y = 0.5 * ((2.0*Y1)
              + (-Y0 + Y2)                        * T
              + ( 2.0*Y0 - 5.0*Y1 + 4.0*Y2 - Y3) * T2
              + (-Y0 + 3.0*Y1 - 3.0*Y2 + Y3)      * T3),
    {X, Y}.
