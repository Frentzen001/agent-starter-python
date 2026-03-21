from __future__ import annotations

from livekit.agents import RunContext, function_tool

from agent.base_agent import RuntimeAwareAgent
from runtime.session_runtime import SessionRuntime


class ConciergeAgent(RuntimeAwareAgent):
    def __init__(self, runtime: SessionRuntime | None = None) -> None:
        runtime = runtime or SessionRuntime.build_test_runtime(always_awake=True)
        self.runtime = runtime
        super().__init__(runtime, instructions=runtime.instructions_for('concierge'), id='concierge_agent')

    @function_tool()
    async def start_tour(self, context: RunContext) -> tuple[object, str]:
        result = await self.runtime.start_tour()
        return self.runtime.tour_agent, result

    @function_tool()
    async def go_to_location(self, context: RunContext, location_name: str) -> tuple[object, str] | str:
        result = await self.runtime.go_to_location(location_name)
        if self.runtime.tour_domain.status().state.value in {'navigating', 'paused', 'completed'}:
            return self.runtime.tour_agent, result
        return result

    @function_tool()
    async def lookup_garage_faq(self, context: RunContext, question: str) -> str:
        return await self.runtime.lookup_garage_faq(question)

    @function_tool()
    async def get_tour_status(self, context: RunContext) -> str:
        return self.runtime.get_tour_status()

    @function_tool()
    async def get_navigation_status(self, context: RunContext) -> str:
        return self.runtime.navigation_status()
