from __future__ import annotations

from typing import TYPE_CHECKING

from livekit.agents import Agent, llm

if TYPE_CHECKING:
    from runtime.session_runtime import SessionRuntime


class RuntimeAwareAgent(Agent):
    def __init__(self, runtime: 'SessionRuntime', *, instructions: str, id: str | None = None) -> None:
        self.runtime = runtime
        super().__init__(instructions=instructions, id=id)

    async def on_user_turn_completed(self, turn_ctx: llm.ChatContext, new_message: llm.ChatMessage) -> None:
        text = self.runtime.extract_user_text(new_message)
        decision = await self.runtime.handle_user_turn(text)
        if decision.instructions_changed:
            await self.runtime.refresh_agent_instructions()
        if not decision.allow_response:
            if decision.immediate_response:
                await self.session.say(decision.immediate_response, allow_interruptions=True)
            elif decision.sleep_prompt:
                await self.session.say(decision.sleep_prompt, allow_interruptions=True)
            raise llm.StopResponse()
