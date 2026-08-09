"""Turn ProjectAirSim's asyncio API into AirSim's synchronous `.join()` style.

Classic AirSim code calls `client.takeoffAsync().join()` from ordinary
synchronous scripts. ProjectAirSim is asyncio-native and its commands are
two-stage: awaiting the coroutine hands back an asyncio.Task, and awaiting that
Task waits for the vehicle to finish the manoeuvre.

One event loop runs on a private thread and *every* call is routed through it,
including the synchronous ones. That is deliberate: the ProjectAirSim client
correlates requests with responses over a shared NNG socket, so letting a user
thread and the loop thread both issue requests would be a race for a very
confusing bug. Serialising on one thread costs nothing here and removes it.
"""

import asyncio
import concurrent.futures
import threading
from typing import Any, Callable


class CommandFuture:
    """What the shim's `*Async` methods hand back — AirSim's `.join()` contract."""

    def __init__(self, future: "concurrent.futures.Future[Any]"):
        self._future = future

    def join(self, timeout: float = None) -> Any:
        """Block until the manoeuvre finishes, re-raising anything it raised."""
        return self._future.result(timeout=timeout)

    def cancel(self) -> bool:
        return self._future.cancel()

    def done(self) -> bool:
        return self._future.done()


class AsyncBridge:
    def __init__(self, name: str = "airsim-shim-loop"):
        self._loop = asyncio.new_event_loop()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()
        self._ready.wait(timeout=10)
        if not self._ready.is_set():
            raise RuntimeError("shim event loop failed to start")

    def _run(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.call_soon(self._ready.set)
        self._loop.run_forever()

    def call(self, fn: Callable, *args, **kwargs) -> Any:
        """Run a plain synchronous client call on the loop thread and wait."""
        future: "concurrent.futures.Future[Any]" = concurrent.futures.Future()

        def invoke() -> None:
            if not future.set_running_or_notify_cancel():
                return
            try:
                future.set_result(fn(*args, **kwargs))
            except BaseException as exc:  # noqa: BLE001 - propagate to caller
                future.set_exception(exc)

        self._loop.call_soon_threadsafe(invoke)
        return future.result()

    def submit(self, coro_fn: Callable, *args, **kwargs) -> CommandFuture:
        """Start a two-stage ProjectAirSim command; return an AirSim-style handle.

        Starts immediately and returns without blocking, so a caller can fire a
        manoeuvre and join it later — the behaviour AirSim scripts rely on.
        """

        async def runner() -> Any:
            task = await coro_fn(*args, **kwargs)  # -> asyncio.Task
            return await task

        return CommandFuture(asyncio.run_coroutine_threadsafe(runner(), self._loop))

    def close(self) -> None:
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5)
        self._loop.close()
