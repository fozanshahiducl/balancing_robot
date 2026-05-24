-module(led_control).

%%% ═══════════════════════════════════════════════════════════════════════════
%%% Named LED states — maps system phase to LED colors.
%%%
%%% LED 1 = left indicator (system status).
%%% LED 2 = right indicator (trajectory sub-phase).
%%%
%%% LED table:
%%%   Phase                      LED1          LED2
%%%   Booting / accel cal        red           red
%%%   Idle (post-boot)           green         green
%%%   Trajectory running         green         cyan
%%%   Trajectory paused          green         yellow
%%%   Traj reset cooldown (1 s)  green         blue
%%%   Trajectory finished        green         white
%%%
%%% cmd-rx overrides only on LED1 (LED2 stays w/ lifecycle so we still see it):
%%%   cmd-rx active        yellow   (sticky while fsm not idle)
%%%   cmd-rx frame ok      white    (brief pulse per frame)
%%%   cmd-rx committed     white    (~250ms)
%%%   cmd-rx proto err     red      (~500ms)
%%% ═══════════════════════════════════════════════════════════════════════════

-export([accel_calibrating/0, idle/0,
         traj_running/0, traj_paused/0, traj_reset_cooldown/0, traj_finished/0,
         cmd_active/0, cmd_frame_ok/0, cmd_committed/0, cmd_error/0]).

accel_calibrating()   -> set({1,0,0}, {1,0,0}).
idle()                -> set({0,1,0}, {0,1,0}).

traj_running()        -> set({0,1,0}, {0,1,1}).
traj_paused()         -> set({0,1,0}, {1,1,0}).
traj_reset_cooldown() -> set({0,1,0}, {0,0,1}).
traj_finished()       -> set({0,1,0}, {1,1,1}).

cmd_active()          -> set1({1,1,0}).
cmd_frame_ok()        -> set1({1,1,1}).
cmd_committed()       -> set1({1,1,1}).
cmd_error()           -> set1({1,0,0}).

%%% ─── Internal ───────────────────────────────────────────────────────────────

set(C1, C2) ->
    grisp_led:color(1, C1),
    grisp_led:color(2, C2).

set1(C1) ->
    grisp_led:color(1, C1).
