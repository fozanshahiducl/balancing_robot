-module(stability_engine).

-export([controller/4]).

-define(ADV_V_MAX, 15.0). %originally 30.0
-define(ADV_ACCEL, 40.0). %original 75.0

-define(TURN_V_MAX, 40.0). %original 80.0
-define(TURN_ACCEL, 200.0). %original 400.0


%V_ref_new must be looped to V_ref
controller({Dt, Angle, Speed}, {Pid_Speed, Pid_Stability}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}) ->

    %% Advance velocity ramp — track the GOAL value (not just its sign).
    %% Adv_V_Ref moves toward Adv_V_Goal at rate ADV_ACCEL, clamped to ±ADV_V_MAX.
    %% Downstream Speed PI / Stability PD are unchanged — they still consume
    %% Adv_V_Ref_New exactly as before; only what value gets fed there has changed.
    Adv_Diff = Adv_V_Goal - Adv_V_Ref,
    Adv_V_Ref_Pre =
        if Adv_Diff >  0.0 -> Adv_V_Ref + min(Adv_Diff,  ?ADV_ACCEL*Dt);
           Adv_Diff <  0.0 -> Adv_V_Ref + max(Adv_Diff, -?ADV_ACCEL*Dt);
           true            -> Adv_V_Ref
        end,
    %% Snap to exact 0 inside a small deadband when Goal is 0, to avoid float drift.
    Adv_V_Ref_New = pid_controller:saturation(
        if (Adv_V_Goal == 0.0) andalso (abs(Adv_V_Ref_Pre) < 0.5) -> 0.0;
           true -> Adv_V_Ref_Pre
        end, ?ADV_V_MAX),

    %% Same magnitude-tracking ramp for turning.
    Turn_Diff = Turn_V_Goal - Turn_V_Ref,
    Turn_V_Ref_Pre =
        if Turn_Diff >  0.0 -> Turn_V_Ref + min(Turn_Diff,  ?TURN_ACCEL*Dt);
           Turn_Diff <  0.0 -> Turn_V_Ref + max(Turn_Diff, -?TURN_ACCEL*Dt);
           true             -> Turn_V_Ref
        end,
    Turn_V_Ref_New = pid_controller:saturation(
        if (Turn_V_Goal == 0.0) andalso (abs(Turn_V_Ref_Pre) < 0.5) -> 0.0;
           true -> Turn_V_Ref_Pre
        end, ?TURN_V_MAX),

    %Speed PI
    Pid_Speed ! {self(), {set_point, Adv_V_Ref_New}},
    Pid_Speed ! {self(), {input, Speed}},
    receive {_, {control, Target_angle}} -> ok end,

    %TODO: send Target_angle to log

    % io:format("~p~n",[Target_angle]),

    %Stability PD
    Pid_Stability ! {self(), {set_point, Target_angle}},
    Pid_Stability ! {self(), {input, Angle}},
    receive {_, {control, Acc}} -> ok end,

    {Acc, Adv_V_Ref_New, Turn_V_Ref_New}.

