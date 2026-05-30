-module(pure_pursuit).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Pure Pursuit path-following helpers.
%%%
%%% advance/4   — drop path points that are already behind the robot.
%%% find_lookahead/4 — pick the first path point at least LOOKAHEAD cm away.
%%% curvature/4 — signed curvature κ from robot pose to a lookahead point.
%%%
%%% All functions are stateless pure math; caller owns the path list.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([advance/4, find_lookahead/4, curvature/4, config_string/0]).

-define(LOOKAHEAD, 8.0).   %% cm

%% Drop path points already behind the robot (local_x < 0 in robot frame).
-spec advance([{float(), float()}], float(), float(), float()) -> [{float(), float()}].
advance([], _, _, _) -> [];
advance([_] = Path, _, _, _) -> Path;
advance([{Px, Py} | Rest] = Path, X, Y, Theta_Rad) ->
    Dx = Px - X, Dy = Py - Y,
    Local_X = Dx * math:cos(Theta_Rad) + Dy * math:sin(Theta_Rad),
    if Local_X < 0.0 -> advance(Rest, X, Y, Theta_Rad);
       true          -> Path
    end.

%% Return the first path point at least LOOKAHEAD cm from (X,Y).
%% Falls back to the last point if none is far enough.
-spec find_lookahead([{float(), float()}], float(), float(), float()) -> {float(), float()}.
find_lookahead([Last], _, _, _) -> Last;
find_lookahead([{Px, Py} = P | Rest], X, Y, Lookahead) ->
    Dx = Px - X, Dy = Py - Y,
    if Dx*Dx + Dy*Dy >= Lookahead * Lookahead -> P;
       true -> find_lookahead(Rest, X, Y, Lookahead)
    end.

%% Signed curvature κ from robot position+heading to lookahead point.
%% Positive = turn left, negative = turn right (robot-frame convention).
%% Returns 0 if the lookahead coincides with the robot (degenerate case).
-spec curvature({float(), float()}, float(), float(), float()) -> float().
curvature({Lx, Ly}, X, Y, Theta_Rad) ->
    Dx = Lx - X, Dy = Ly - Y,
    L2 = Dx*Dx + Dy*Dy,
    if L2 > 0.001 ->
           Local_Y = -Dx * math:sin(Theta_Rad) + Dy * math:cos(Theta_Rad),
           2.0 * Local_Y / L2;
       true -> 0.0
    end.

-spec config_string() -> string().
config_string() ->
    io_lib:format("pure_pursuit: LOOKAHEAD=~p cm", [?LOOKAHEAD]).
