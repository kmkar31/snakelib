import time


class Watchdog:
    """Simple watchdog that keeps track of time between triggers

    This is used to denote to the CommandManager when sensor data is not received so that
    CommandManager does not send joint commands when on sensor data is available.
    """

    def __init__(self, timeout):
        """Initialize the Watchdog class.

        Args:
            timeout: time in nanoseconds that defines the timeout of the watchdog
        """

        self._trigger_time = time.perf_counter_ns()

        if timeout < 0:
            raise ValueError("Specified timeout is less than zero.")
        else:
            self._timeout = timeout

        """Set trigger time to be past the timeout so that the Watchdog is by default 
        timed out until it is triggered."""
        self._trigger_time += 1.1 * self._timeout

    def trigger(self):
        """Trigger the watchdog to reset the timer."""
        self._trigger_time = time.perf_counter_ns()

    def timed_out(self):
        """Returns True if timeout is exceeded, False otherwise."""

        if (time.perf_counter_ns() - self._trigger_time) > self._timeout:
            return True
        else:
            return False
