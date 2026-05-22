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
