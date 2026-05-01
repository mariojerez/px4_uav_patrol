"""End-to-end launch entrypoint for the energy-cost calibration suite.

What this launch does, in order:

  1. (Optional) Start PX4 SITL (gz_x500 in an empty world) and the
     Micro XRCE-DDS Agent. Disabled by default because most users prefer to
     start PX4 SITL in a separate terminal so its output stays visible.
  2. Generate the suite of missions (1 hover + 5 straight-line + 9 turn
     mission JSONs and matching meta sidecars) into `output_dir/missions/`.
  3. Bring up the patrol_mission action server.
  4. Bring up the calibration_runner, which sequentially sends an
     ExecutePatrol goal per mission with auto_arm=true and writes per-trial
     CSVs under `output_dir/calibration_data/`.
  5. After the runner exits, fit the constants and write
     `energy_constants.yaml` + plots, and (optionally) replace the SRS
     placeholders.

Launch arguments:

  output_dir        Where missions, CSVs, and constants go (default
                    /app/calibration_output).
  start_px4         Bring up PX4 SITL via `make px4_sitl gz_x500` (default
                    false — user starts SITL externally to keep its
                    stdout visible).
  start_agent       Bring up MicroXRCEAgent (default false; same rationale).
  px4_dir           Path to PX4-Autopilot (default ~/PX4-Autopilot).
  n_trials          Trials per condition (default 7).
  run_hover         Include hover mission (default true).
  run_straight_line Include straight-line missions (default true).
  run_turn          Include turn missions (default true).
  update_srs        After fitting, rewrite srs.md placeholders (default
                    true).
  srs_path          Path to srs.md (default /app/srs.md).
  dry_run           Force n_trials=1 for a smoke test (default false).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _generate_missions(context, *args, **kwargs):
    """OpaqueFunction that calls gen_calibration_missions before the runner
    starts. Runs synchronously in the launch process so the manifest is on
    disk before the runner reads it."""
    output_dir = LaunchConfiguration("output_dir").perform(context)
    n_trials_str = LaunchConfiguration("n_trials").perform(context)
    dry_run_str = LaunchConfiguration("dry_run").perform(context)
    domain_config = LaunchConfiguration("domain_config").perform(context)
    starting_point_name = LaunchConfiguration(
        "starting_point_name"
    ).perform(context)
    n_trials = 1 if dry_run_str.lower() == "true" else int(n_trials_str)

    pkg_share = get_package_share_directory("px4_uav_patrol")
    gen_script = os.path.join(
        pkg_share, "calibration", "gen_calibration_missions.py",
    )
    cmd = [
        "python3", gen_script,
        "--output-dir", output_dir,
        "--n-trials", str(n_trials),
        "--domain-config", domain_config,
        "--starting-point-name", starting_point_name,
    ]
    return [
        LogInfo(msg=f"Generating calibration missions (n_trials={n_trials}) "
                f"from {domain_config} ({starting_point_name}) "
                f"under {output_dir}"),
        ExecuteProcess(cmd=cmd, output="screen", shell=False),
    ]


def _post_run_actions(context, *args, **kwargs):
    """Run fit_energy_constants and (optionally) update_srs after the
    calibration_runner exits."""
    output_dir = LaunchConfiguration("output_dir").perform(context)
    update_srs_flag = LaunchConfiguration("update_srs").perform(context)
    srs_path = LaunchConfiguration("srs_path").perform(context)
    pkg_share = get_package_share_directory("px4_uav_patrol")

    fit_script = os.path.join(
        pkg_share, "calibration", "fit_energy_constants.py",
    )
    update_script = os.path.join(
        pkg_share, "calibration", "update_srs.py",
    )

    actions = [
        ExecuteProcess(
            cmd=[
                "python3", fit_script,
                "--data-dir", os.path.join(output_dir, "calibration_data"),
                "--output-dir", output_dir,
            ],
            output="screen", shell=False,
        ),
    ]
    if update_srs_flag.lower() == "true":
        actions.append(ExecuteProcess(
            cmd=[
                "python3", update_script,
                "--srs", srs_path,
                "--constants", os.path.join(output_dir, "energy_constants.yaml"),
            ],
            output="screen", shell=False,
        ))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument("output_dir",
                              default_value="/app/calibration_output"),
        DeclareLaunchArgument("start_px4", default_value="false"),
        DeclareLaunchArgument("start_agent", default_value="false"),
        DeclareLaunchArgument("px4_dir",
                              default_value=os.path.expanduser("~/PX4-Autopilot")),
        DeclareLaunchArgument("px4_world", default_value="empty"),
        DeclareLaunchArgument("n_trials", default_value="7"),
        DeclareLaunchArgument("run_hover", default_value="true"),
        DeclareLaunchArgument("run_straight_line", default_value="true"),
        DeclareLaunchArgument("run_turn", default_value="true"),
        DeclareLaunchArgument("update_srs", default_value="true"),
        DeclareLaunchArgument("srs_path", default_value="/app/srs.md"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("wait_for_fmu_s", default_value="120.0"),
        DeclareLaunchArgument(
            "domain_config",
            default_value="/app/shepherding-rl/configs/lantern_farm.json",
            description=(
                "Shepherd-rl domain config whose starting_point becomes the "
                "local Cartesian origin for all missions."
            ),
        ),
        DeclareLaunchArgument(
            "starting_point_name",
            default_value="Starting Point 0",
            description=(
                "Which starting_point zone to use when the config has "
                "multiple (e.g. lantern_farm.json has 0 and 1)."
            ),
        ),
    ]

    output_dir = LaunchConfiguration("output_dir")

    def _px4_sitl_setup(context, *args, **kwargs):
        """Optional PX4 SITL bring-up. When ``start_px4=true``, spawn the
        drone at the same starting_point used by the mission generator so
        that mission waypoints land on the SITL world.
        """
        pkg_share = get_package_share_directory("px4_uav_patrol")
        gen_module = os.path.join(
            pkg_share, "calibration", "gen_calibration_missions.py",
        )
        # Import the generator's extract_starting_point helper at launch time
        # so we don't duplicate the JSON-parsing logic.
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "_gen_calibration_missions", gen_module,
        )
        gcm = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(gcm)
        domain_config = LaunchConfiguration("domain_config").perform(context)
        sp_name = LaunchConfiguration("starting_point_name").perform(context)
        try:
            home_lat, home_lon = gcm.extract_starting_point(
                domain_config, sp_name,
            )
        except (FileNotFoundError, ValueError) as e:
            home_lat, home_lon = (
                gcm._FALLBACK_ORIGIN_LAT, gcm._FALLBACK_ORIGIN_LON,
            )
            print(
                f"[energy_calibration.launch] starting_point lookup failed "
                f"({e}); falling back to ({home_lat}, {home_lon})"
            )
        return [ExecuteProcess(
            cmd=[
                "bash", "-c",
                "if [ \"$PX4_SITL_ENABLED\" = \"true\" ]; then "
                "cd \"$PX4_DIR\" && "
                "PX4_GZ_WORLD=$PX4_WORLD "
                "PX4_HOME_LAT=$PX4_HOME_LAT_VAL "
                "PX4_HOME_LON=$PX4_HOME_LON_VAL "
                "make px4_sitl gz_x500; "
                "else echo 'PX4 SITL bring-up skipped (start_px4=false). "
                "When running SITL externally, export "
                "PX4_HOME_LAT='\"$PX4_HOME_LAT_VAL\"' "
                "PX4_HOME_LON='\"$PX4_HOME_LON_VAL\"' '"
                "before make px4_sitl so the drone spawns at the same "
                "starting_point the missions use.'; fi",
            ],
            output="screen",
            additional_env={
                "PX4_SITL_ENABLED": LaunchConfiguration("start_px4"),
                "PX4_DIR": LaunchConfiguration("px4_dir"),
                "PX4_WORLD": LaunchConfiguration("px4_world"),
                "PX4_HOME_LAT_VAL": f"{home_lat:.10f}",
                "PX4_HOME_LON_VAL": f"{home_lon:.10f}",
            },
        )]

    px4_sitl = OpaqueFunction(function=_px4_sitl_setup)

    micro_xrce_agent = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "if [ \"$XRCE_ENABLED\" = \"true\" ]; then "
            "MicroXRCEAgent udp4 -p 8888; "
            "else echo 'MicroXRCEAgent bring-up skipped (start_agent=false)'; fi",
        ],
        output="screen",
        additional_env={"XRCE_ENABLED": LaunchConfiguration("start_agent")},
    )

    patrol_mission_node = Node(
        package="px4_uav_patrol",
        executable="patrol_mission",
        name="patrol_mission",
        output="screen",
    )

    runner_args = [
        "--manifest", PythonExpression([
            "'", output_dir, "' + '/missions/calibration_manifest.json'",
        ]),
        "--output-dir", output_dir,
        "--wait-for-fmu-s", LaunchConfiguration("wait_for_fmu_s"),
    ]
    # Suite scope toggles
    runner_args_extra = []
    # Compose --no-hover etc. via PythonExpression so the user's `false`
    # toggles propagate without wrapping each arg in IfCondition.
    runner_args_extra.append(PythonExpression([
        "'--no-hover' if '", LaunchConfiguration("run_hover"), "'.lower() == 'false' else ''",
    ]))
    runner_args_extra.append(PythonExpression([
        "'--no-straight-line' if '",
        LaunchConfiguration("run_straight_line"),
        "'.lower() == 'false' else ''",
    ]))
    runner_args_extra.append(PythonExpression([
        "'--no-turn' if '", LaunchConfiguration("run_turn"),
        "'.lower() == 'false' else ''",
    ]))

    pkg_share = get_package_share_directory("px4_uav_patrol")
    runner_script = os.path.join(
        pkg_share, "calibration", "calibration_runner.py",
    )
    calibration_runner = ExecuteProcess(
        cmd=["python3", runner_script] + runner_args + runner_args_extra,
        output="screen",
        shell=False,
    )

    # Run gen_calibration_missions inline before everything else.
    gen_step = OpaqueFunction(function=_generate_missions)

    # When the runner exits, kick off fitting + SRS update.
    post_run = RegisterEventHandler(OnProcessExit(
        target_action=calibration_runner,
        on_exit=[OpaqueFunction(function=_post_run_actions)],
    ))

    # Stagger the runner start so the action server has time to register.
    delayed_runner = TimerAction(
        period=5.0, actions=[calibration_runner],
    )

    return LaunchDescription(args + [
        gen_step,
        px4_sitl,
        micro_xrce_agent,
        patrol_mission_node,
        delayed_runner,
        post_run,
    ])
