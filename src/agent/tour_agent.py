from __future__ import annotations

from livekit.agents import RunContext, function_tool

from agent.base_agent import RuntimeAwareAgent


class TourAgent(RuntimeAwareAgent):
    def __init__(self, runtime) -> None:
        self.runtime = runtime
        super().__init__(runtime, instructions=runtime.instructions_for('tour'), id='tour_agent')

    @function_tool()
    async def resume_tour(self, context: RunContext) -> str:
        return await self.runtime.resume_tour()

    @function_tool()
    async def pause_tour(self, context: RunContext) -> str:
        return await self.runtime.pause_tour()

    @function_tool()
    async def stop_robot(self, context: RunContext) -> tuple[object, str]:
        result = await self.runtime.stop_robot()
        return self.runtime.concierge_agent, result

    @function_tool()
    async def go_to_location(self, context: RunContext, location_name: str) -> str:
        return await self.runtime.go_to_location(location_name)

    @function_tool()
    async def lookup_garage_faq(self, context: RunContext, question: str) -> str:
        return await self.runtime.lookup_garage_faq(question)

    @function_tool()
    async def get_tour_status(self, context: RunContext) -> str:
        return self.runtime.get_tour_status()

    @function_tool()
    async def get_navigation_status(self, context: RunContext) -> str:
        return self.runtime.navigation_status()

    @function_tool()
    async def return_to_concierge(self, context: RunContext) -> tuple[object, str]:
        return self.runtime.concierge_agent, 'Switching back to concierge mode. What would you like help with next?'
