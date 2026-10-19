"""Graceful shutdown for GUI-owned local ROS launch processes."""
import os
import signal


def stop_ros_process(process):
    """Let ros2 launch forward SIGINT and reap its children before escalation."""
    # QProcess.NotRunning is zero; no Qt import needed by this helper.
    if process.state() == 0:
        return
    try:
        os.kill(int(process.processId()), signal.SIGINT)
    except ProcessLookupError:
        return
    if not process.waitForFinished(8000):
        process.terminate()
        if not process.waitForFinished(3000):
            process.kill()
            process.waitForFinished(1000)
