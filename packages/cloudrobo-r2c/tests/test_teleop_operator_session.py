"""Operator session state machine tests (local + cloud modes)."""


from cloudrobo_r2c.teleop.session import (
    FINISHED,
    IDLE,
    PAUSED,
    RUNNING,
    OperatorSession,
)


def press(session, now, primary=False, secondary=True):
    return session.on_buttons(primary, secondary, now)


def release(session, now, primary=False, secondary=False):
    return session.on_buttons(primary, secondary, now)


def test_local_two_phase_start():
    s = OperatorSession()
    assert s.state == IDLE
    press(s, 100.0)
    events = release(s, 150.0)  # B/Y release edge -> start
    assert [e.kind for e in events] == ["start"]
    assert s.state == PAUSED
    assert s.session_id
    # next B/Y release -> RUNNING
    press(s, 200.0)
    events = release(s, 250.0)
    assert [e.kind for e in events] == ["resume"]
    assert s.state == RUNNING
    assert s.teleop_mode() == "CONTROLLING"


def test_local_pause_resume_toggle():
    s = OperatorSession()
    press(s, 100.0)  # start
    release(s, 150.0)
    assert s.state == PAUSED
    press(s, 200.0)  # resume
    release(s, 250.0)
    assert s.state == RUNNING
    press(s, 300.0)  # short press -> PAUSED
    release(s, 350.0)
    assert s.state == PAUSED
    press(s, 400.0)  # short press -> RUNNING
    release(s, 450.0)
    assert s.state == RUNNING


def test_local_long_press_finishes():
    s = OperatorSession()
    press(s, 100.0)
    release(s, 150.0)
    press(s, 200.0)
    release(s, 250.0)
    assert s.state == RUNNING
    press(s, 300.0)
    # long press >= 500 ms settles finish while held
    events = s.on_buttons(False, True, 850.0)
    assert [e.kind for e in events] == ["finish"]
    assert s.state == FINISHED
    assert s.session_id == ""
    # release does not re-start while require_all_released until all released
    release(s, 900.0)
    assert s.state == FINISHED


def test_finished_allows_new_start_after_full_release():
    s = OperatorSession()
    press(s, 100.0)
    release(s, 150.0)
    press(s, 200.0)
    release(s, 250.0)
    press(s, 300.0)
    s.on_buttons(False, True, 850.0)  # finish
    release(s, 900.0)                 # clear require_all_released
    events = release(s, 1000.0)       # fresh start requires another press+release
    assert events == []
    press(s, 1100.0)
    events = release(s, 1150.0)
    assert [e.kind for e in events] == ["start"]


def test_auto_start_enters_running_directly():
    s = OperatorSession(auto_start=True)
    event = s.auto_start_now(100.0)
    assert event is not None and event.kind == "start"
    assert s.state == RUNNING
    assert s.auto_start_now(200.0) is None  # already running


def test_cloud_start_requires_session_and_is_two_phase():
    s = OperatorSession(cloud_mode=True)
    assert s.on_cloud_command("start", "", 1, 100.0) is False
    assert s.state == IDLE
    assert s.on_cloud_command("start", "sess-1", 2, 200.0) is True
    assert s.state == PAUSED
    # local B/Y confirms into RUNNING
    press(s, 300.0)
    events = release(s, 350.0)
    assert [e.kind for e in events] == ["resume"]
    assert s.state == RUNNING


def test_cloud_stale_session_commands_ignored():
    s = OperatorSession(cloud_mode=True)
    assert s.on_cloud_command("start", "sess-1", 1, 100.0) is True
    assert s.state == PAUSED
    # confirm into RUNNING (operator B/Y release), then exercise session checks
    press(s, 200.0)
    release(s, 250.0)
    assert s.state == RUNNING
    assert s.on_cloud_command("pause", "sess-2", 2, 300.0) is False
    assert s.on_cloud_command("resume", "sess-2", 3, 350.0) is False
    assert s.on_cloud_command("pause", "sess-1", 4, 400.0) is True
    assert s.state == PAUSED
    # finish applies while paused
    assert s.on_cloud_command("finish", "sess-1", 5, 500.0) is True
    assert s.state == FINISHED
    # duplicate seq is ignored
    assert s.on_cloud_command("quit", "", 5, 600.0) is False


def test_cloud_start_matrix_application():
    s = OperatorSession(cloud_mode=True)
    mats = {
        "pos_left": [1, 0, 0, 0, 1, 0, 0, 0, 1],
        "rot_left": [0, -1, 0, 1, 0, 0, 0, 0, 1],
    }
    assert s.on_cloud_command("start", "s", 1, 100.0, start_matrices=mats)
    payload = s.status_payload()
    adj = payload["cloud_adjust"]
    assert adj is not None
    assert adj["seq"] == 1
    assert adj["left"]["posR"] == mats["pos_left"]
    assert adj["right"] is None


def test_local_long_press_not_finish_when_idle():
    s = OperatorSession()
    press(s, 100.0)
    events = s.on_buttons(False, True, 900.0)
    assert events == []
    assert s.state == IDLE
