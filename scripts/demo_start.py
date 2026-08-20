"""One-step GUI bootstrap for the NHC12 + Robotiq + D455 demo.

Open the demo USD, press Play, open this file in Isaac Sim's Script Editor,
and press Ctrl+Enter. The script loads the local IK and capture modules into
THIS file's own globals (the same globals the Script Editor console uses),
starts the IK follow controller, and exposes ``demo_capture()`` in the
Script Editor globals. Re-running this file safely replaces the previous IK
callback.
"""

# Do not synchronously pump the app-update loop after scheduling an async
# task (e.g. ik_follow_start()'s asyncio.ensure_future). Isaac Sim's
# omni.kit.async_engine drives its asyncio loop from the same Kit update
# event (on_event=lambda _: self._loop.run_once()), so a synchronous pump
# call while a task is pending re-enters _run_once(): the inner call drains
# _ready, and the outer call's popleft() then raises
# IndexError: pop from an empty deque, aborting that round of scheduling.
# Evidence: T002 (outputs/T002/step1_kit_log_excerpt.txt,
# outputs/T002/step3_kit_log_excerpt.txt).

import builtins
import hashlib
import os

# Must match capture_d455.py's REGISTRY_NAME -- that module can't be
# imported yet at the point this is used (it's exec'd, not imported, and
# only after this runs), so the string is duplicated here rather than
# shared.
_CAPTURE_REGISTRY_NAME = "_nhc12_d455_capture_registry"


def _record_capture_source_sha256_at_bootstrap(path):
    """T008 Step 0: write the sha256 of capture_d455.py as it exists right
    now, before exec'ing it, into the same builtins registry
    capture_d455.py's own _registry() uses. That module's staleness check
    reads this back as the "loaded" baseline -- see
    _loaded_source_sha256_at_bootstrap() there for why the file's own
    exec-time hash stopped being a valid baseline once every call started
    re-exec'ing it fresh."""
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    if not hasattr(builtins, _CAPTURE_REGISTRY_NAME):
        setattr(builtins, _CAPTURE_REGISTRY_NAME, {})
    getattr(builtins, _CAPTURE_REGISTRY_NAME)["capture_source_sha256_at_bootstrap"] = digest.hexdigest()


def _demo_project_root():
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")
    layer = stage.GetRootLayer()
    stage_path = layer.realPath or layer.identifier
    if not stage_path or not os.path.isfile(stage_path):
        raise RuntimeError("Open the saved local demo USD before running this script.")
    return os.path.dirname(os.path.dirname(os.path.abspath(stage_path)))


def _execute_local_script(path):
    with open(path, "r", encoding="utf-8") as source:
        code = compile(source.read(), path, "exec")
    exec(code, globals())


def demo_start():
    import omni.timeline
    import omni.usd

    root = _demo_project_root()
    stage = omni.usd.get_context().get_stage()
    required_prims = [
        "/World/ArmWithHandOnly",
        "/World/ArmWithHandOnly/Robotiq_2F_85/Robotiq_2F_85/base_link",
        "/World/IKTarget",
        "/World/object",
    ]
    missing = [path for path in required_prims if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"The active stage is not the demo scene; missing: {missing}")

    timeline = omni.timeline.get_timeline_interface()

    # An earlier generation stopped, pumped, and only then played, because
    # its capture setup step required the timeline to be STOPPED when it
    # ran (it snapshotted a pre-Play disk-authored cube state) -- skipping
    # that order made every demo_capture() call fail with "Create the D455
    # capture graph while the timeline is stopped". T005's capture setup
    # (scripts/capture_d455.py) has no such precondition -- it isn't even
    # called from here any more, see below -- so that stop/pump/play dance
    # no longer has a reason to exist and is simplified to just: play if not
    # already playing.
    if not timeline.is_playing():
        timeline.play()

    _execute_local_script(os.path.join(root, "scripts", "ik_controller.py"))

    capture_script_path = os.path.join(root, "scripts", "capture_d455.py")
    if os.path.isfile(capture_script_path):
        _record_capture_source_sha256_at_bootstrap(capture_script_path)
        _execute_local_script(capture_script_path)
        # capture_d455.py's setup function is a coroutine now (T005, B1).
        # Calling it synchronously here would only produce a never-awaited
        # coroutine object and a RuntimeWarning. demo_capture_async() awaits
        # it itself as its first step (it's cheap to call repeatedly --
        # cached by stage_id), so this script does not call or schedule it
        # at all.
        capture_setup_result = {
            "status": "deferred",
            "reason": "setup runs inside the first demo_capture()",
        }
    else:
        capture_setup_result = {
            "status": "skipped",
            "reason": "scripts/capture_d455.py not present",
        }

    # ik_follow_start() schedules an async task and returns immediately
    # ("status": "scheduled"). Do not pump the update loop here to force it
    # forward -- Kit drives its own update loop; let it run and check
    # ik_follow_status() from the console afterward.
    ik_start_result = ik_follow_start()

    result = {
        "status": "ready",
        "project_root": root,
        "capture_setup_result": capture_setup_result,
        "ik_start_result": ik_start_result,
        "next_action": (
            "Wait a second or two, then call ik_follow_status() from the "
            "console. Once running is True, drag /World/IKTarget, then call "
            "demo_capture() and read the result back with "
            "demo_capture_status()."
        ),
    }
    print(result)
    return result


def demo_stop():
    """Stop the IK callback and restore the validated static arm pose."""
    result = ik_follow_stop()
    print(result)
    return result


DEMO_START_RESULT = demo_start()
