import asyncio
from typing import (
    AsyncIterator,
    Tuple,
    Union,
)

from .messages import (
    Message,
    RemoveAction,
    CancelQuery
)
from .exceptions import *

# Only export monitor classes
__all__ = [
    'MessageMonitor',
    'ResponseMonitor',
    'PeriodicQueryMonitor',
    'ActionMonitor',
]

class MessageMonitor:
    """Allows monitoring selected messages using an async for loop.

    Returned by :meth:`.JsonApi.monitor`. Alternative to :meth:`.JsonApi.handle`
    used when a coroutine with an async for loop is more convenient than a
    handler function that gets called on each message matching the key.
    Internally uses JsonApi.handle and automatically deals with removing the
    handler when finished.

    Example::

        # Monitor all 'error' messages sent by the robot
        async with api.monitor('error') as m:
            async for msg in m:
                print(msg.info)
                # Break to stop monitoring

    It is also possible to manually call the :meth:`cancel` method instead of
    using an `async with` block, but in this case it is important to make sure
    that cancel does eventually get called when responses are no longer needed.
    Failure to do so will leave an unused coroutine and message handler
    installed, and if many of these accumulate over the program lifetime it
    could cause performance issues.

    """
    def __init__(self, api: 'JsonApi', key: Union[int, str]):
        self._api = api
        self._key = key
        self._hnum = None
        self._canceled = False
        self._queue = asyncio.Queue()
        if key is not None:
            self._hnum = self._api.handle(self._key, self._handler)
        self._api._monitors.add(self)

    async def cancel(self) -> bool:
        """Stops monitoring for messages.

        This terminates the `async for` iteration, and can be called from
        outside the task with the loop. Returns False if already canceled. Not
        necessary if using `async with`.

        """
        if self._canceled:
            return False
        self._canceled = True
        self._api._monitors.remove(self)
        if self._hnum is not None:
            self._api.remove_handler(self._hnum)
        await self._queue.put(None) # Wakes up anext
        return True

    def __aiter__(self) -> AsyncIterator[Message]:
        return self

    async def __anext__(self) -> Message:
        # Don't await the queue if already canceled
        if self._canceled:
            raise StopAsyncIteration

        # Wait for new message from handler
        response = await self._queue.get()
        if self._canceled:
            raise StopAsyncIteration
        return response

    async def __aenter__(self) -> 'MessageMonitor':
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        await self.cancel()

    async def _handler(self, response) -> bool:
        await self._queue.put(response)
        return True


class ResponseMonitor(MessageMonitor):
    """Allows reading multiple responses to a sent message.

    This object is returned by :meth:`.JsonApi.get_responses` and serves as an
    async iterator returning each message sent by the server in response to a
    sent message. Usage is similar to :class:`MessageMonitor`.

    """
    def __init__(self, api: 'JsonApi', message: Message, ignore_warnings=True):
        super().__init__(api, None)
        self._message = message
        self._ignore_warnings = ignore_warnings
        self._refnum = None

    async def __anext__(self) -> Message:
        # Send message if not already sent
        if not self._canceled and self._message is not None:
            msg = self._message
            self._message = None
            self._refnum, self._hnum = await self._api._handle_responses(
                msg, self._handler)
        return await super().__anext__()

    async def _handler(self, response) -> bool:
        if self._ignore_warnings and response.type == 'warning':
            return False # Warnings get passed to other handlers
        await self._queue.put(response)
        return True


class PeriodicQueryMonitor(ResponseMonitor):
    """Periodic query management object.

    This object is returned by :meth:`.JsonApi.periodic_query` and serves as an
    async iterator returning the responses to a periodic query. Usage is
    similar to :class:`MessageMonitor`.

    """
    def __init__(self, api: 'JsonApi', message: Message, bare_query: bool):
        super().__init__(api, message)
        self._bare_query = bare_query

    async def cancel(self) -> bool:
        if await super().cancel():
            try:
                await self._api.send(CancelQuery(reference_number=self._refnum))
            except ConnectionClosedError:
                pass
            return True
        return False

    async def __anext__(self) -> Message:
        while True:
            msg = await super().__anext__()
            if msg.type == 'error':
                raise JsonApiError(msg.info)
            if msg.type == 'query-group':
                if self._bare_query:
                    return msg.queries[0]
                return msg.queries


class ActionMonitor(ResponseMonitor):
    """Action status monitoring object.

    This object is returned by :meth:`.JsonApi.monitor_action` and serves as an
    async iterator returning the action status and info each time it changes.
    Can also be used to cancel the action that it is monitoring. Usage is
    similar to :class:`MessageMonitor`.

    """
    def __init__(
            self,
            api: 'JsonApi',
            message: Message,
            remove_after: bool
    ):
        super().__init__(api, message)
        self._remove_after = remove_after
        self._already_inactive = False

    async def cancel(self) -> bool:
        if await super().cancel():
            if self._remove_after and not self._already_inactive:
                try:
                    await self._api.send(
                        RemoveAction(reference_number=self._refnum))
                except ConnectionClosedError:
                    pass
            return True
        return False

    async def __anext__(self) -> Tuple[str, str]:
        while True:
            msg = await super().__anext__()
            if msg.type == 'error':
                raise JsonApiError(msg.info)
            if msg.type == 'action-status-changed':
                if msg.status == 'inactive':
                    self._already_inactive = True
                    await self.cancel()
                return msg.status, msg.info
