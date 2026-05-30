-module(input_combo).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Button combo edge and hold detection — stateless helpers.
%%%
%%% The caller owns all timing state; these functions are pure transforms.
%%%
%%% rising_edge/2  — true on the single tick where a combo goes false→true.
%%% hold_check/3   — accumulates held-time in ms; resets to 0 on release.
%%%                  Caller compares the returned value against its threshold.
%%%
%%% Short-press vs long-hold pattern (caller logic):
%%%   - Track hold_check each tick.
%%%   - Long hold fires when held_ms exceeds threshold, even without release.
%%%   - Short press fires on the rising_edge tick that begins the press
%%%     (or on release if the press was brief — caller decides convention).
%%%   - A cooldown timer after each action prevents double-firing.
%%% ═══════════════════════════════════════════════════════════════════════════

-export([rising_edge/2, falling_edge/2, hold_check/3, config_string/0]).

%% True only on the single tick where Curr becomes true.
-spec rising_edge(boolean(), boolean()) -> boolean().
rising_edge(true,  false) -> true;
rising_edge(_Curr, _Prev) -> false.

%% True only on the single tick where Curr becomes false.
%% Pair with the caller's stored held_ms to distinguish a tap (short held)
%% from the trailing release of a long-hold action.
-spec falling_edge(boolean(), boolean()) -> boolean().
falling_edge(false, true) -> true;
falling_edge(_Curr, _Prev) -> false.

%% Returns updated hold duration in ms.
%%   Curr false          → 0 (released, reset)
%%   Curr true, was off  → Dt_ms (start counting)
%%   Curr true, was on   → Held_ms + Dt_ms (accumulate)
-spec hold_check(boolean(), boolean(), non_neg_integer()) -> non_neg_integer().
hold_check(false, _Prev, _Held_ms) -> 0;
hold_check(true, _Prev, Held_ms)   -> Held_ms.   %% caller adds Dt_ms before comparing

-spec config_string() -> string().
config_string() ->
    "input_combo: (stateless, no tuning parameters)".
