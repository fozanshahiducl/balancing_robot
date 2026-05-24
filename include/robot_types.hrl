%%% ═══════════════════════════════════════════════════════════════════════════
%%% Shared type definitions used by odometry, traj_planner, and main_loop.
%%% Keep only types that genuinely cross module boundaries here.
%%% Module-private records stay in their owning .erl file.
%%% ═══════════════════════════════════════════════════════════════════════════

-record(pose, {
    x     = 0.0 :: float(),   %% cm, rightward from launch origin
    y     = 0.0 :: float(),   %% cm, forward from launch origin
    theta = 0.0 :: float()    %% deg, normalized [-180, 180]; 0 = launch heading
}).

%% wp injection wire protocl - single byte frames, laptop->lora->esp->i2c->grisp
%% bits after hera_com:get_bits/1:
%%   b7 = Arm_Ready (esp sets this, not us)
%%   b6 = PROTO (1=proto frame, 0=normal drive byt)
%%   b5 = KIND  (only when PROTO=1: 1=data, 0=ctrl)
%%   b4..b0 = payload, 5 bits (ctrl code or 5 data bits)

%% ctrl codes (b4..b0, sent w/ PROTO=1 KIND=0):
-define(CMD_CTRL_ABORT,           2#00000).
-define(CMD_CTRL_CANCEL_LAST,     2#10000).
-define(CMD_CTRL_CLEAR_ALL,       2#10001).
-define(CMD_CTRL_CANCEL_N_HDR,    2#10010).
-define(CMD_CTRL_CANCEL_N_COMMIT, 2#10011).
-define(CMD_CTRL_ADD_HEADER,      2#11000).
-define(CMD_CTRL_X_TO_Y,          2#11100).
-define(CMD_CTRL_ADD_COMMIT,      2#11110).

%% masks for the CtrlByte bits
-define(CMD_PROTO_MASK,   2#01000000).
-define(CMD_KIND_MASK,    2#00100000).
-define(CMD_PAYLOAD_MASK, 2#00011111).

%% ADD seq (ui sends in order, ~60ms per frame):
%%   ADD_HEADER, data X_hi, data X_lo, X_TO_Y, data Y_hi, data Y_lo, ADD_COMMIT
%% signed encoding per axis - 10bits across two 5bit halves
%%   ui:    enc = val + CMD_SIGN_OFFSET
%%   bot:   val = ((hi bsl 5) bor lo) - CMD_SIGN_OFFSET
%% ui clamps to +/-CMD_MAX_OFFSET, decoder rejects if outside
-define(CMD_SIGN_OFFSET, 50).
-define(CMD_MAX_OFFSET,  50).

%% led pulse lengths (ticks at ~300hz)
-define(CMD_LED_PULSE_FRAME_OK,   1).    %% 1 tick flash per frame
-define(CMD_LED_PULSE_COMMITTED, 75).    %% ~250ms on commit ok
-define(CMD_LED_PULSE_ERROR,    150).    %% ~500ms on err
