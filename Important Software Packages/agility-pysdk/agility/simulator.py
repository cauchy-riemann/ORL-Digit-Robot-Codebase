import os
import random
import shutil
import signal
import socket
import subprocess
import tempfile
import time
from hashlib import md5
from .exceptions import *

# Only export the Simulator class
__all__ = ['Simulator']

def get_child_pids(pid):
    """
    Utility function for getting child pids from a pid. Used to avoid
    issues stemming from packaging psutil.
    """
    with open(f'/proc/{pid}/task/{pid}/children', 'rb') as f:
        children = [int(p) for p in f.read().decode('utf-8').split()]
        for child in children:
            # Recursively add children of children to the list
            children += get_child_pids(child)
    return children

class Simulator:
    """Launches a simulator instance.

    Example::

        import asyncio
        import agility
        import agility.messages as msgs

        async def main():
            # Connect to simulator on port 8080
            async with agility.JsonApi() as api:
                # Send messages, launch other tasks
                info = await api.query(msgs.GetRobotInfo())
                print(f"Robot name: {info.robot_name}")
            # When the 'async with' block exits, the connection shuts down

        # Launch simulator on port 8080
        with agility.Simulator("/path/to/ar-control") as sim:
            # asyncio.run(main()) # Python 3.7+ only
            asyncio.get_event_loop().run_until_complete(main()) # Python 3.6
        # When the 'with' block exits, the simulator shuts down

    Args:
        simulator_path: The relative or absolute path to the simulator binary.
        *args: All additional arguments should be strings that will be passed
               to the simulator as if they were command line arguments. These
               can be paths to TOML files, configuration mode names, or
               explicit TOML data.

    Keyword args:
        auto_port: Defaults to False, if True then the simulator is instructed
            to automatically select a port that doesn't conflict with other
            running simulators. The selected port is available in the port
            property of the returned simulator object. This option only works
            with versions of ar-control that support automatic port selection.
        extract_appimage: Defaults to True. If True, then will extract the
            appimage to a folder in /tmp/ and run the resulting executable.
            If False, will run the appimage directly. This option is provided
            as a workaround for a docker/FUSE incompatibility issue.
        timeout: The time in seconds to search for the simulator port number.
            Useful on slow or overloaded systems, where starting the simulator
            may take some time.
        capture_output: Defaults to True. If True, then stdout and stderr are
            captured by this object and accessed via the stdout and stderr
            properties. If False, then the output of ar-control will be
            forwarded to this script's output streams.

    The returned simulator object can be used as a context manager to
    automatically close the simulator when finished::

        with agility.Simulator('/path/to/ar-control') as sim:
            async with agility.JsonApi() as api:
                # Do things with API...

    Alternatively, :meth:`close` can be called to shut down the simulator when
    finished. When the simulator shuts down using either method, it checks to
    see if the simulator has already unexpectedly terminated and raises a
    :class:`.SimulatorTerminated` exception if so. This check can also be run
    at any other time while the simulator is running by calling :meth:`check`.

    """
    _tempdirs = {}
    def __init__(
            self,
            simulator_path: str,
            *args,
            auto_port: bool = False,
            extract_appimage: bool = True,
            timeout: int = 5,
            capture_output: bool = True,
    ):

        if extract_appimage:
            # Appimages have a compatibility issue with docker (related to FUSE)
            # so explicitly extract the appimage to a temp folder first. The
            # --appimage-extract-and-run option was previously used here, but it
            # causes bigger problems with waiting for a clean shutdown.

            # Cache the md5 sum of the file in case we've already extracted
            # the appimage
            with open(simulator_path, 'rb') as f:
                app_md5_sum = md5(f.read()).hexdigest()

            if app_md5_sum not in self._tempdirs:
                self._tempdirs[app_md5_sum] = tempfile.TemporaryDirectory()
                shutil.copy(simulator_path,
                            self._tempdirs[app_md5_sum].name + '/ar-control')
                subprocess.run(['./ar-control', '--appimage-extract'],
                               cwd=self._tempdirs[app_md5_sum].name,
                               stdout=subprocess.DEVNULL)
            app = self._tempdirs[app_md5_sum].name + '/squashfs-root/AppRun'
        else:
            app = simulator_path

        # Launch the control process. Capture both stdout and stderr using
        # pipes, use unbuffered mode, and capture as bytes instead of as
        # decoded strings. After starting the program, convert the
        # stdout/stderr pipes to nonblocking mode. All of these settings are
        # necessary to successfully read lines from the control program as they
        # are output instead of only when the program terminates.
        if auto_port:
            args = ('server.port=0', *args)

        stdout = None
        stderr = None
        if capture_output:
            stdout=subprocess.PIPE
            stderr=subprocess.PIPE
        self._proc = subprocess.Popen([app, *args], stdout=stdout,
                                      stderr=stderr, bufsize=0)
        if capture_output:
            os.set_blocking(self._proc.stdout.fileno(), False)
            os.set_blocking(self._proc.stderr.fileno(), False)
        self._stdout = ''
        self._stderr = ''
        self._closed_normally = False
        self._crashed = False
        self.returncode = None
        self._capture_output = capture_output

        # Check for immediate crashes
        self.check()

        # Get automatically assigned port if auto port assignment is enabled
        self._port = None
        if auto_port:
            # Read the port once the web thread starts
            start_time = time.time()
            while True:
                self.check()
                try:
                    pids = [self._proc.pid, *get_child_pids(self._proc.pid)]
                    for pid in pids:
                        path = '{}/ar-control/{}/port'.format(
                            os.getenv('XDG_RUNTIME_DIR', '/run'), pid)

                        if os.path.isfile(path):
                            break

                    with open(path, 'r') as f:
                        self._port = int(f.read())
                    break
                except:
                    time.sleep(0.1)
                # Give up after 'timeout' seconds
                if time.time() - start_time > timeout:
                    raise RuntimeError('Unable to get port number used by '
                                       'simulator, does this simulator version '
                                       'not support automatic port selection?')

    def check(self) -> None:
        """Raises an exception if the simulator has unexpectedly shut down.

        Does nothing if the simulator is running or was shut down by calling
        :meth:`close` or exiting a 'with' block. If the simulator is otherwise
        not running, raises a :class:`.SimulatorTerminated` exception with
        information about why the simulator terminated extracted from
        :attr:`stdout` and :attr:`stderr`.

        """
        if (not self._crashed and
            not self._closed_normally and
            self._proc.poll() is not None):
            # Only raise an exception for a crash once even if check is called
            # multiple times
            self._crashed = True
            # To give a good error message, check stderr and stdout
            if self.stderr:
                # Usually an exception message
                # Remove backtrace if present
                err = self.stderr.partition('\nBacktrace:\n')[0]
                # Strip the part before the reason
                reason = err.partition(' thrown, terminating\nReason: ')[2]
                if reason:
                    err = reason
                raise SimulatorTerminated(err.strip())
            if self.stdout:
                # Look for thread crash message in stdout
                # This happens if the web thread crashes, as the simulator
                # keeps running in fallback mode if another thread crashes
                err = self.stdout.partition(
                    ' crashed with exception message: ')[2]
                if err:
                    err = err.partition('Shutting down...')[0]
                    raise SimulatorTerminated(err.strip())
                # Otherwise, just dump all of stdout
                raise SimulatorTerminated(
                    'Simulator terminated unexpectedly, stdout:\n' +
                    self.stdout)
            # Simulator isn't running but also didn't print anything
            raise SimulatorTerminated('Simulator terminated unexpectedly')

    def close(self) -> None:
        """Shuts down the simulator.

        Does not need to be called directly if a 'with' block is used. Raises a
        :class:`.SimulatorTerminated` exception if the simulator already shut
        down unexpectedly.

        """
        # Do nothing if called before the _closed_normally is set, which
        # happens if the simulator fails to launch in the first place
        if not hasattr(self, '_closed_normally'):
            return
        if not self._closed_normally and not self._crashed:
            self.check()
            children = get_child_pids(self._proc.pid)
            for child in children:
                # Terminate the child process, move on if it's already dead
                try:
                    os.kill(child, signal.SIGTERM)
                except:
                    continue

                # Wait until it's terminated
                while True:
                    try:
                       # Will throw OSError if child is dead
                       os.kill(child, 0)
                    except OSError:
                       break
            # First try ending the process gracefully
            termination_timeout = 5
            self._proc.terminate()
            try:
                # Read any additional stdout and stderr, and give up after a
                # few seconds.
                outs, errs = self._proc.communicate(timeout=termination_timeout)
                if self._capture_output:
                    self._stdout += outs.decode('utf-8')
                    self._stderr += errs.decode('utf-8')
            except subprocess.TimeoutExpired:
                # If it's not dead after sigterm and a few seconds, kill it hard
                self._proc.kill()
                outs, errs = self._proc.communicate()
                if self._capture_output:
                    self._stdout += outs.decode('utf-8')
                    self._stderr += errs.decode('utf-8')
                raise SimulatorTerminated(
                    f'Simulator took more than {termination_timeout} seconds to '
                    'shut down.')

            self._closed_normally = True
        self.returncode = self._proc.returncode

    @property
    def port(self) -> int:
        """The port chosen by the simulator in automatic port selection mode.

        Returns None if automatic port selection was not enabled.
        """
        return self._port

    @property
    def stdout(self) -> str:
        """Returns all of the text printed to stdout since the start."""
        # Read in chunks because the OS only buffers about 64k at a time
        if not self._capture_output:
            return self._stdout
        try:
            chunk = self._proc.stdout.read()
            while chunk:
                self._stdout += chunk.decode('utf-8')
                chunk = self._proc.stdout.read()
        except:
            pass
        return self._stdout

    @property
    def stderr(self) -> str:
        """Returns all of the text printed to stderr since the start."""
        if not self._capture_output:
            return self._stderr
        try:
            chunk = self._proc.stderr.read()
            while chunk:
                self._stderr += chunk.decode('utf-8')
                chunk = self._proc.stderr.read()
        except:
            pass
        return self._stderr

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __del__(self):
        # Make sure to clean up when garbage collected or the interpreter exits
        self.close()
