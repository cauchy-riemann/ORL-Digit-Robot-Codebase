import asyncio
import inspect
import json
import time
import websockets
from typing import (
    Any,
    Awaitable,
    Callable,
    Iterable,
    List,
    Optional,
    Tuple,
    Union,
)

from .messages import (
    Message,
    QueryGroup,
    RequestPrivilege,
    from_json,
)
from .exceptions import *
from .monitors import *

# Only export the JsonApi class
__all__ = ['JsonApi']

# Methods taking messages as input accept either an agility.messages.Message, a
# tuple of a string (type) and dictionary (fields), or just a message type.
MessageInput = Union[Message, Tuple[str, dict], List[Union[str, dict]], str]


def _normalize(message: MessageInput):
    """Converts type-name-only MessageInput to a JSON-like message."""
    if isinstance(message, str):
        return [message, {}]
    return message


def _to_json(message: MessageInput, refnum: Union[int, None] = None) -> str:
    """Converts any acceptable MessageInput object to JSON."""
    if isinstance(message, str):
        message = _normalize(message)
    if isinstance(message, tuple) or isinstance(message, list):
        message = list(message)
        message = message[:2] # Strip existing refnum if present
        if refnum is not None:
            message.append(refnum)
        return json.dumps(message)
    if isinstance(message, Message):
        message._refnum = refnum
        return message.to_json()
    raise ValueError('Cannot convert input to JSON')


def _is_message_input(message: Any) -> bool:
    """Determines whether the given type represents a single message."""
    if isinstance(message, Message) or isinstance(message, str):
        return True
    try:
        if isinstance(message[0], str) and isinstance(message[1], dict):
            return True
    except:
        pass
    return False


def _extract_type(message: MessageInput) -> str:
    """Extracts the message type from any MessageInput object."""
    if isinstance(message, str):
        return message
    if isinstance(message, Message):
        return message.type
    try:
        if isinstance(message[0], str) and isinstance(message[1], dict):
            return message[0]
    except:
        pass
    return None


class JsonApi:
    """The main class used to manage sending and receiving JSON API messages.

    Example::

        import asyncio
        import agility
        import agility.messages as msgs

        async def main():
            # By default, connects to simulator on localhost port 8080
            async with agility.JsonApi() as api:
                # Send messages, launch other tasks
                info = await api.query(msgs.GetRobotInfo())
                print(f"Robot name: {info.robot_name}")
            # When the 'async with' block exits, the connection shuts down

        # asyncio.run(main()) # Python 3.7+ only
        asyncio.get_event_loop().run_until_complete(main()) # Python 3.6

    Args:
        address: Defaults to '127.0.0.1' for simulator use, set to '10.10.1.1'
            to connect to the physical robot.
        port: Defaults to 8080, change this if the simulator is running on a
            different port (e.g. when running multiple simulations in parallel).
        connect_timeout: Defaults to 1.0 seconds, this affects how long this
            class tries to connect to the robot before giving up. The default
            is generally appropriate for simulation, while on hardware setting
            this to a larger value or infinity is recommended.

    """
    def __init__(
            self,
            address: str = '127.0.0.1',
            port: int = 8080,
            connect_timeout: float = 1.0,
    ):
        self._address = address
        self._port = port
        self._connect_timeout = connect_timeout
        self._refnum_counter = 1
        self._handlers_by_num = {} # handler # -> (function, str/int key)
        self._handlers_by_key = {} # str/int key -> list of functions
        self._handler_counter = 1 # generate handler # the same as refnums
        self._monitors = set()
        self._futures = set()
        self._tasks = set()
        self._keepalive_task = None
        self._ws = None
        self._conn = None
        self._closing = False

    @property
    def address(self) -> str:
        """Returns the address that this object is connected to."""
        return self._address

    @property
    def port(self) -> int:
        """Returns the port number that this object is connected to."""
        return self._port


    async def send(
            self,
            message: Union[MessageInput, Iterable[MessageInput]],
    ) -> int:
        """Sends a message to the robot without waiting for any response.

        Examples::

            # Sends an action-stand message with no fields specified
            await api.send("action-stand")

            # Sends an action-stand message with an explicit base pose
            await api.send(["action-stand", {"base-pose": {"xyz": [0, 0, 1]}}])

            # Same as above, but using the message creation helper classes
            import agility.messages as msgs
            await api.send(msgs.ActionStand(base_pose=msgs.Pose(xyz=[0, 0, 1])))

        This should be used to send messages that either do not produce a
        response or for which the response will be intentionally ignored.
        Automatically chooses a reference number to associate with the message.
        Any existing reference number in the input message is ignored.

        Args:
            message: Can be a :class:`.Message`, a tuple or list that gets
                converted into a valid JSON message by json.dumps(), or just a
                message type name.

        Returns:
            The automatically selected reference number sent with the message.

        """
        if self._closing:
            raise ConnectionClosedError()
        refnum = self._refnum_counter
        self._refnum_counter += 1
        await self._ws.send(_to_json(message, refnum))
        return refnum


    def monitor(self, key: Union[int, str]) -> MessageMonitor:
        """Allows monitoring selected messages using an async for loop.

        Example::

            # Print all warning messages to the console
            async with api.monitor("warning") as m:
                async for msg in m:
                    print(f"warning: {msg.info}")

        This is a higher-level method than :meth:`handle` for reading arbitrary
        messages sent by the robot. It should be used whenever an async
        iterator in a task is a more convenient abstraction than a callback
        function.

        Args:
            key: The message key to match, generally 'all', 'default', or a
                message type. See :meth:`handle` for details about what keys
                are accepted.

        Returns:
            A :class:`.MessageMonitor` object that returns messages matching
            the key as an async iterator.

        """
        return MessageMonitor(self, key)


    async def query(
            self,
            queries: Union[MessageInput, Iterable[MessageInput]],
    ) -> Union[List[Message], Message]:
        """Sends one or more queries and waits for the response.

        Examples::

            # Get a response from a single query
            response = await api.query("get-robot-info")

            # Get responses for a group of simultaneous queries
            # The list can mix and match different ways of specifying messages
            responses = await api.query([
                "get-timestamp",
                ["get-object", {"object": {"object-id": 10}}],
                msgs.GetObjectKinematics(object={"object-id": 10}),
            ])

        This can be used both for world-state queries that are allowed in a
        query-group message as well as for other query-like messages that are
        not allowed in a query group (e.g. get-action). However, when using a
        message type that isn't allowed in a query group, it must not be
        grouped with any other messages and the period cannot be set.

        When a single query returns an error message, this method raises a
        :class:`.JsonApiError` instead of returning the error message as a
        response. When one query in a group returns an error message, the error
        message is included in the list of responses.

        Args:
            messages: A message or list of messages to send as queries.
                If a list is provided, they will be combined into a single
                query-group message.

        Returns:
            Returns either a single message as a response to a single query or
            a list of messages as a response to multiple queries.

        """
        # Construct query message to be sent
        single_query = _is_message_input(queries)
        single_query_type = _extract_type(queries)
        if not single_query:
            queries = QueryGroup(queries=[_normalize(q) for q in queries])

        # Send query and wait for the response
        response = await self.get_response(queries)
        if response.type == 'error':
            raise JsonApiError(response.info)
        if (response.type == 'query-group' and
            single_query_type != 'query-group'):
            if single_query:
                return response.queries[0]
            return response.queries
        return response


    def periodic_query(
            self,
            queries: Union[MessageInput, Iterable[MessageInput]],
            period: float,
    ) -> 'PeriodicQueryMonitor':
        """Sends one or more queries as a periodic query-group.

        Examples::

            # Get a response from a single query
            query = msgs.GetGpsCoordinates(
                object={"robot-frame": msgs.RobotFrame.BASE})
            async with api.periodic_query(query, 0.1) as q:
                async for result in q:
                    # get-gps-coordinates will return an error until the robot
                    # has moved around enough to localize
                    if result.type != "error":
                        print(f"lat: {result.latitude}, "
                              f"lon: {result.longitude}")

            # Get responses for a group of simultaneous queries
            # The list can mix and match different ways of specifying messages
            queries = ["get-timestamp",
                       ["get-object", {"object": {"object-id": 10}}],
                       msgs.GetObjectKinematics(object={"object-id": 10})]
            async with api.periodic_query(queries, 0.1) as q:
                async for results in q:
                    print(f"time: {results[0].run_time}, "
                          f"name: {results[1].attributes.name}, "
                          f"pose: {results[2].transform.rpyxyz}")

        This can only be used with world-state queries that are allowed in a
        query-group message. This excludes several "get-" prefixed messages,
        such as "get-action-command", as noted in the message documentation.

        This method will only raise a :class:`.JsonApiError` if the query or
        list of queries is malformed in some way. If an otherwise valid query
        returns an error message, e.g. because the selected object was not
        found, the error response is returned directly by the
        :class:`.PeriodicQueryMonitor`. If a query is expected to potentially
        return an error in this way, the type of the response message should be
        checked.

        Args:
            messages: A message or list of messages to send as a query-group.
            period: The number of seconds between query responses.

        Returns:
            Returns a :class:`.PeriodicQueryMonitor` object that can be used to
            read responses from and cancel periodic queries.

        """
        # Record whether input was a bare Message or a list. This allows a
        # message and a one-message list to be handled slightly differently.
        bare_query = _is_message_input(queries)
        if bare_query:
            queries = [queries]
        queries = [_normalize(q) for q in queries]
        message = QueryGroup(queries=queries, period=period)

        # Sending is handled by the PeriodicQueryMonitor object
        return PeriodicQueryMonitor(self, message, bare_query)


    async def wait_action(
            self,
            action: MessageInput,
            failure_timeout: float = 1.0,
            remove_after: bool = True,
    ) -> str:
        """Sends an action and waits for it to succeed.

        Examples::

            # Send a goto command and wait until successful, then remove the
            # command once the target is first reached
            await api.wait_action(msgs.ActionGoto(target={"xy": [10, 0]}))

            # Set a longer failure timeout than the default, so if the robot's
            # path is blocked for up to a minute it will continue to wait.
            # Also, keep the command in place when finished so the robot will
            # persistently try to stay at the target location if pushed away.
            await api.wait_action(msgs.ActionGoto(target={"xy": [10, 0]}),
                                  failure_timeout=60, remove_after=False)

        If the action stays in a failure state continuously for longer than the
        failure timeout, raises an :class:`.ActionFailed` exception. If the
        action becomes inactive while waiting for it to reach success, raises
        an :class:`.ActionCanceled` exception. The latter condition can be the
        result of another client stealing the change-action-command privilege.

        Raises a :class:`.JsonApiError` exception if the sent action returns an
        error instead of being applied. Warnings about the action are ignored
        by this method and can be passed to a more general message handler
        instead.

        Note that the change-action-command privilege must be obtained before
        an action command can be sent. This is not done automatically by this
        method.

        Args:
            action: An action message to send.

        Keyword args:
            failure_timeout: The time in seconds that the action needs to stay
                in a failure state before this method will throw an
                ActionFailed exception. Defaults to 1 second, set to zero to
                fail immediately or infinity to ignore failure and continue
                waiting indefinitely until the action completes.
            remove_after: If True (default), the action is removed using
                remove-action when the action succeeds or fails. If set to
                false, the action stays as the current command after
                wait_action completes.

        Returns:
            Returns the status info string of the success message, which is
            often an empty string but in some cases contains useful
            information.

        """
        try:
            async with self.monitor_action(action,
                                           remove_after=remove_after) as am:
                # Coroutine and variables used to implement failure timeout
                timeout_task = None
                failure_info = ''
                failed = False
                async def timeout_coro():
                    await asyncio.sleep(failure_timeout)
                    await am.cancel()

                async for status, info in am:
                    # Start the failure timeout task
                    if status == 'failure':
                        if failure_timeout == 0:
                            # Fail immediately if timeout is zero
                            raise ActionFailed(failure_info)
                        if timeout_task is None:
                            timeout_task = asyncio.ensure_future(timeout_coro())
                            failure_info = info
                            failed = True
                    elif timeout_task: # No more failure, cancel timeout
                        timeout_task.cancel()
                        timeout_task = None
                        failed = False

                    # On success, return, which also cancels the action
                    if status == 'success':
                        return info

            # If this is reached, the loop terminated without sending an
            # inactive action status (shouldn't happen)
            raise ActionCanceled()
        except:
            # If this is reached, the loop either ended because of the failure
            # timeout or because the action became inactive (possibly due to
            # loss of privilege)
            if failed:
                raise ActionFailed(failure_info)
            else:
                raise


    def monitor_action(
            self,
            action: MessageInput,
            remove_after: bool = True,
    ) -> 'ActionMonitor':
        """Sends an action and returns an ActionMonitor object.

        Example::

            async with api.monitor_action("action-stand") as m:
                async for status, info in m:
                    print(f"status: {status}, info: {info}")

        The returned :class:`.ActionMonitor` object can be used to monitor
        status messages sent regarding the action, and will automatically stop
        when the action becomes inactive. If the action message is replied to
        with an error message, the ActionMonitor throws a
        :class:`.JsonApiError`. Warnings about the action are ignored by the
        ActionMonitor and can be passed to a more general message handler
        instead.

        Note that the change-action-command privilege must be obtained before
        an action command can be sent. This is not done automatically by this
        method.

        Args:
            action: An action message to send.

        Keyword args:
            remove_after: If True (default), the action is removed using
                remove-action when the :class:`.ActionMonitor` object is
                canceled or leaves its `async with` context. If set to False,
                the action stays in place.

        Returns:
            An :class:`.ActionMonitor` object that can be used to track the
            status of the action (running, failure, success). The ActionMonitor
            will automatically stop iterating and clean up when the action
            becomes inactive, but does not do anything special if it reaches a
            failure status. Actions can go from failure back into running on
            their own if conditions allow.

        """
        return ActionMonitor(self, action, remove_after)


    async def request_privilege(
            self,
            privilege: str,
            priority: int = 0,
    ) -> None:
        """Helper for requesting a privilege.

        Example::

            await api.request_privilege("change-action-command")

        Requests a named privilege and raises a :class:`.PrivilegeNotObtained`
        exception if it was not granted.

        Args:
            privilege: The privilege to request.

        Keyword args:
            priority: The priority to request the privilege with (default 0).
                If another client already has the privilege at a higher
                priority, the request will fail and this method will raise a
                :class:`.PrivilegeNotObtained` exception.

        """
        response = await self.query(RequestPrivilege(privilege=privilege,
                                                     priority=priority))
        for p in response.privileges:
            if p.privilege == privilege:
                if not p.has:
                    raise PrivilegeNotObtained(
                        f'Failed to get {privilege} privilege')


    def handle(
            self,
            key: Union[int, str],
            handler: Union[Callable[[Message], bool],
                           Callable[[Awaitable[Message]], bool]],
    ) -> int:
        """Installs a handler function for messages from the robot.

        This is a low-level method, :meth:`monitor` should be used instead in
        most cases.

        The handler can either be a normal function or a coroutine (async def).
        The provided handler must take the :class:`.Message` to handle as the
        only argument. It should return False or None if the message should be
        passed to handlers further down in the hierarchy or True if the message
        was completely handled and should not be processed further.

        Each handler has a particular key that identifies which messages it
        gets called for and when it gets called in the hierarchy. The key types
        and hierarchy of handlers is as follows:

            1. All: api.handle('all', handler)
            2. Reference number: api.handle(123, handler)
            3. Message type: api.handle('message-type', handler)
            4. Default: api.handle('default', handler)

        Multiple handlers can be installed at once for a given key. In this
        case, all of the handlers for the same key are called for a given
        message, but the relative ordering is undefined. When any of multiple
        handlers for the same key returns True, the message is not passed to
        the next set of handlers in the hierarchy, but it is always passed to
        all of the handlers for the same key.

        Returns an integer that can be passed to :meth:`remove_handler` to
        remove the installed handler.

        Args:
            key: int for reference number handlers, the message type name for
                message type handlers, or the values 'all' or 'default'
            handler: Callable handler

        Returns:
            A handler number that can be used to remove the handler later. This
            will be unique for each call of handle even if the same handler
            function is installed multiple times.

        """
        handler_num = self._handler_counter
        self._handler_counter += 1
        self._handlers_by_num[handler_num] = (handler, key)
        if key not in self._handlers_by_key:
            self._handlers_by_key[key] = []
        self._handlers_by_key[key].append(handler)
        return handler_num


    def remove_handler(self, handler_num: int) -> None:
        """Removes an installed message handler.

        Args:
            handler_num: The integer returned by :meth:`handle` for the handler
                to be removed. Raises an exception if this does not match an
                installed handler.

        """
        handler, key = self._handlers_by_num[handler_num]
        del self._handlers_by_num[handler_num]
        key_handlers = self._handlers_by_key[key]
        key_handlers.remove(handler) # remove one copy of handler from list
        if not key_handlers: # remove empty lists from key dict
            del self._handlers_by_key[key]


    async def get_response(
            self,
            message: MessageInput,
            ignore_warnings=True,
    ) -> Message:
        """Sends a message and returns a single response.

        This is a low-level method, :meth:`query` should be used instead in
        most cases.

        This method will return error messages directly instead of raising a
        :class:`.JsonApiError` exception.

        Args:
            message: The message to send.

        Keyword args:
            ignore_warnings: If True (default), warning messages received in
                response to the sent message are ignored and can be handled by
                a more general message handler (if any). Set to False to output
                warnings directly, but this is generally not desired because
                some queries that normally return a single response or error
                message can produce a warning in addition to these messages,
                causing the "main" response to be lost.

        Returns:
            A :class:`.Message` object representing the response to the sent
            message.

        """
        future = asyncio.get_event_loop().create_future()
        def handler(response):
            if (future.done() or
                (ignore_warnings and response.type == 'warning')):
                return False # Warnings get passed to other handlers
            future.set_result(response)
            return True
        refnum, hnum = await self._handle_responses(message, handler)
        self._futures.add(future)
        try:
            await future
        except: # Happens when close() is called while waiting
            pass
        self._futures.remove(future)
        if future.cancelled():
            raise ConnectionClosedError()
        self.remove_handler(hnum)
        return future.result()


    def get_responses(
            self,
            message: MessageInput,
            ignore_warnings=True,
    ) -> ResponseMonitor:
        """Sends a message and allows waiting for multiple responses.

        This is a low-level method, :meth:`monitor`, :meth:`periodic_query`, or
        :meth:`monitor_action` should be used instead in most cases.

        Args:
            message: The message to send. Note that the message is not actually
                sent until the returned :class:`.ResponseMonitor` object is
                iterated upon with an `async for` loop.

        Keyword args:
            ignore_warnings: If True (default), warning messages received in
                response to the sent message are ignored by the returned
                :class:`.ResponseMonitor` and can be handled by a more general
                message handler (if any). Set to False to have the
                ResponseMonitor output warnings directly.

        Returns:
            A :class:`.ResponseMonitor` object that sends the message and
            returns responses as an async iterator.

        """
        # Sending is handled by the ResponseMonitor object
        return ResponseMonitor(self, message, ignore_warnings=ignore_warnings)


    async def connect(self) -> None:
        """Connects to the server. Not needed if using `async with`."""
        # Try connecting repeatedly until connect_timeout is reached
        t0 = time.monotonic()
        wait_time = 0.04
        while True:
            try:
                self._conn = websockets.connect(
                    f'ws://{self._address}:{self._port}',
                    subprotocols=['json-v1-agility'],
                    ping_timeout=3,
                    ping_interval=1,
                    close_timeout=1,
                )
                self._ws = await self._conn.__aenter__()
                self._tasks.add(asyncio.ensure_future(self._recv()))
                self._keepalive_task = asyncio.ensure_future(self._keepalive())
                return
            except ConnectionRefusedError as e:
                t_rem = self._connect_timeout - (time.monotonic() - t0)
                if t_rem < 0:
                    raise e
                await asyncio.sleep(min(wait_time, t_rem))
                wait_time = min(1, wait_time * 1.5)


    async def close(self, exc=None) -> None:
        """Closes connection to server. Not needed if using `async with`."""
        if self._closing:
            return
        self._closing = True
        if self._keepalive_task:
            self._keepalive_task.cancel() # faster close if explicitly canceled
        for f in self._futures:
            f.cancel()
        for m in self._monitors.copy(): # cancel() removes from _monitors
            await m.cancel()            # so need to iterate over copy of set
        if self._conn is not None:
            if exc is not None:
                await self._conn.__aexit__(type(exc), exc, exc.__traceback__)
            else:
                await self._conn.__aexit__(None, None, None)
            self._conn = None
            self._ws = None


    async def __aenter__(self):
        await self.connect()
        return self


    async def __aexit__(self, exc_type, exc, tb):
        await self.close(exc)
        await asyncio.gather(*self._tasks) # wait on tasks, raise any exceptions


    async def _recv(self):
        try:
            async for msg in self._ws:
                # Convert to Message
                msg = from_json(msg)

                # Launch a separate task for each received message so handlers
                # can be coroutines without blocking new messages
                self._tasks.add(asyncio.ensure_future(self._run_handlers(msg)))

                # Clean up finished tasks
                for t in self._tasks.copy():
                    if t.done() and not t.cancelled() and t.exception() is None:
                        self._tasks.remove(t)

            # Websocket connection was terminated
            await self.close()
        except:
            # Some other exception occurred
            await self.close()
            raise


    async def _run_handlers(self, msg: Message):
        # Try to run a handler, which can be a normal function or a coroutine
        async def try_handler(key, msg) -> bool:
            if key in self._handlers_by_key:
                any_true = False
                for fun in self._handlers_by_key[key]:
                    try:
                        out = fun(msg)
                    except Exception as e:
                        await self.close()
                        raise e # exception will be raised in aexit
                    if inspect.isawaitable(out):
                        out = await out
                    any_true = any_true or out
                return any_true

        # Try handlers in their priority order
        if await try_handler('all', msg):
            return
        if msg._refnum is not None and await try_handler(msg._refnum, msg):
            return
        if await try_handler(msg.type, msg):
            return
        if await try_handler('default', msg):
            return


    async def _handle_responses(
            self,
            message: MessageInput,
            handler: Union[Callable[[Message], bool],
                           Callable[[Awaitable[Message]], bool]],
    ) -> int:
        """Sends a message to the robot and installs a handler for responses.

        This function acts like a combination of send() and handle(), but
        installs the handler before the message is actually sent to avoid a
        race condition. This is a low-level method used internally by other
        methods. Client code should use get_responses() to accomplish a similar
        task.

        Args:
            message: As in send()
            handler: As in handle()

        Returns:
            A tuple of (reference_number, handler_number). Both are plain
            integers, so take care not to mix them up.

        """
        refnum = self._refnum_counter
        self._refnum_counter += 1
        hnum = self.handle(refnum, handler)
        await self._ws.send(_to_json(message, refnum))
        return refnum, hnum


    async def _keepalive(self) -> None:
        """Sends unsolicited pongs to the server to keep it from sending pings.

        The websockets library has a bug that causes it to raise a spurious
        exception if it receives a ping just before the connection is closed
        normally. This commonly occurs when code using JsonApi sends a final
        message, waits a second or two, and then closes the connection, because
        the server sends pings every 0.5 seconds after the last received
        message. This works around the websockets bug by sending unsolicited
        pongs so the server never sends pings.

        """
        while not self._closing:
            # Send pongs faster than the the default ping interval (0.5)
            await self._ws.pong(b'\x00') # Need to send at least one byte
            await asyncio.sleep(0.25)
