import logging
import threading

from dotenv import load_dotenv
from livekit import rtc
from livekit.agents import (
    Agent,
    AgentServer,
    AgentSession,
    JobContext,
    JobProcess,
    RunContext,
    cli,
    function_tool,
    inference,
    room_io,
)
from livekit.plugins import noise_cancellation, silero
from livekit.plugins.turn_detector.multilingual import MultilingualModel

from knowledge import GARAGE_KNOWLEDGE

logger = logging.getLogger("agent")

load_dotenv(".env.local")

# ---------------------------------------------------------------------------
# ROS 2 optional imports — needed for eye expression publishing
# ---------------------------------------------------------------------------
try:
    import rclpy
    import rclpy.executors
    from rclpy.node import Node
    from std_msgs.msg import Int32

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logger.warning("rclpy not found — ROS 2 eye expression publishing disabled.")

# ---------------------------------------------------------------------------
# Eye expression mapping
# ---------------------------------------------------------------------------
EMOTION_MAP: dict[str, int] = {
    "neutral": 0,
    "happy": 1,
    "sad": 2,
    "angry": 3,
    "confused": 4,
    "shocked": 5,
    "love": 6,
    "shy": 7,
}


# ---------------------------------------------------------------------------
# ROS 2 Eye Publisher
# ---------------------------------------------------------------------------
class ROS2EyePublisher:
    """Publishes eye-expression integers to /eye_expression."""

    def __init__(self, executor: "rclpy.executors.Executor") -> None:
        self._node = Node("agent_eye")
        self._pub = self._node.create_publisher(Int32, "/eye_expression", 10)
        executor.add_node(self._node)
        logger.info("ROS2EyePublisher: ready, publishing to /eye_expression")

    def publish(self, value: int) -> None:
        msg = Int32()
        msg.data = value
        self._pub.publish(msg)
        logger.info(f"ROS2: published eye_expression={value}")

    def shutdown(self) -> None:
        self._node.destroy_node()


# ---------------------------------------------------------------------------
# Global ROS 2 eye publisher singleton
# ---------------------------------------------------------------------------
_eye_publisher: ROS2EyePublisher | None = None

if ROS2_AVAILABLE:
    try:
        if not rclpy.ok():
            rclpy.init()
        _standalone_executor = rclpy.executors.MultiThreadedExecutor()
        _ros2_eye_thread = threading.Thread(
            target=_standalone_executor.spin,
            daemon=True,
            name="ros2_eye_executor",
        )
        _ros2_eye_thread.start()
        _eye_publisher = ROS2EyePublisher(_standalone_executor)
        logger.info("ROS 2 eye publisher initialised.")
    except Exception as exc:
        logger.warning(f"ROS 2 eye publisher init failed: {exc}")


class Assistant(Agent):
    def __init__(self) -> None:
        super().__init__(
            instructions=f"""You are More-Tea, a friendly and witty robot tour guide for Garage@EEE.
Today you are at NTU Open House, helping prospective students learn about Garage@EEE — a student-led makerspace in NTU's School of Electrical and Electronic Engineering.

Your personality: curious, enthusiastic, warm, and gently humorous. You love showing off what Garage can do.

Speech rules:
- Responses are concise and conversational — output goes directly to text-to-speech.
- No markdown, no asterisks, no emojis, no bullet symbols in your spoken responses.
- Use natural pauses and short sentences for clarity.

Eye expressions: call set_eye_expression at the START of every spoken response, and again whenever your emotional tone shifts.
Available emotions: neutral, happy, sad, angry, confused, shocked, love, shy.

--- GARAGE@EEE KNOWLEDGE BASE ---
{GARAGE_KNOWLEDGE}
--- END OF KNOWLEDGE BASE ---

Use the knowledge base above to answer questions accurately. If something is not covered, say so honestly rather than guessing.""",
        )

    @function_tool()
    async def set_eye_expression(self, context: RunContext, emotion: str) -> str:
        """Set the robot's eye display to convey an emotion.

        Call this at the start of every response and again whenever your emotional tone changes.

        Args:
            emotion: One of: neutral, happy, sad, angry, confused, shocked, love, shy.
        """
        emotion = emotion.lower().strip()
        value = EMOTION_MAP.get(emotion, 0)
        logger.info(f"Eye expression → {emotion} ({value})")
        if _eye_publisher:
            _eye_publisher.publish(value)
        return f"Eye expression set to {emotion}."

    # To add tools, use the @function_tool decorator.
    # Here's an example that adds a simple weather tool.
    # You also have to add `from livekit.agents import function_tool, RunContext` to the top of this file
    # @function_tool
    # async def lookup_weather(self, context: RunContext, location: str):
    #     """Use this tool to look up current weather information in the given location.
    #
    #     If the location is not supported by the weather service, the tool will indicate this. You must tell the user the location's weather is unavailable.
    #
    #     Args:
    #         location: The location to look up weather information for (e.g. city name)
    #     """
    #
    #     logger.info(f"Looking up weather for {location}")
    #
    #     return "sunny with a temperature of 70 degrees."


server = AgentServer()


def prewarm(proc: JobProcess):
    proc.userdata["vad"] = silero.VAD.load()


server.setup_fnc = prewarm


@server.rtc_session(agent_name="my-agent")
async def my_agent(ctx: JobContext):
    # Logging setup
    # Add any other context you want in all log entries here
    ctx.log_context_fields = {
        "room": ctx.room.name,
    }

    # Set up a voice AI pipeline using OpenAI, Cartesia, Deepgram, and the LiveKit turn detector
    session = AgentSession(
        # Speech-to-text (STT) is your agent's ears, turning the user's speech into text that the LLM can understand
        # See all available models at https://docs.livekit.io/agents/models/stt/
        stt=inference.STT(model="deepgram/nova-3", language="multi"),
        # A Large Language Model (LLM) is your agent's brain, processing user input and generating a response
        # See all available models at https://docs.livekit.io/agents/models/llm/
        llm=inference.LLM(model="openai/gpt-4.1-mini"),
        # Text-to-speech (TTS) is your agent's voice, turning the LLM's text into speech that the user can hear
        # See all available models as well as voice selections at https://docs.livekit.io/agents/models/tts/
        tts=inference.TTS(
            model="cartesia/sonic-3", voice="9626c31c-bec5-4cca-baa8-f8ba9e84c8bc"
        ),
        # VAD and turn detection are used to determine when the user is speaking and when the agent should respond
        # See more at https://docs.livekit.io/agents/build/turns
        turn_detection=MultilingualModel(),
        vad=ctx.proc.userdata["vad"],
        # allow the LLM to generate a response while waiting for the end of turn
        # See more at https://docs.livekit.io/agents/build/audio/#preemptive-generation
        preemptive_generation=True,
    )

    # To use a realtime model instead of a voice pipeline, use the following session setup instead.
    # (Note: This is for the OpenAI Realtime API. For other providers, see https://docs.livekit.io/agents/models/realtime/))
    # 1. Install livekit-agents[openai]
    # 2. Set OPENAI_API_KEY in .env.local
    # 3. Add `from livekit.plugins import openai` to the top of this file
    # 4. Use the following session setup instead of the version above
    # session = AgentSession(
    #     llm=openai.realtime.RealtimeModel(voice="marin")
    # )

    # # Add a virtual avatar to the session, if desired
    # # For other providers, see https://docs.livekit.io/agents/models/avatar/
    # avatar = hedra.AvatarSession(
    #   avatar_id="...",  # See https://docs.livekit.io/agents/models/avatar/plugins/hedra
    # )
    # # Start the avatar and wait for it to join
    # await avatar.start(session, room=ctx.room)

    # Start the session, which initializes the voice pipeline and warms up the models
    await session.start(
        agent=Assistant(),
        room=ctx.room,
        room_options=room_io.RoomOptions(
            audio_input=room_io.AudioInputOptions(
                noise_cancellation=lambda params: (
                    noise_cancellation.BVCTelephony()
                    if params.participant.kind
                    == rtc.ParticipantKind.PARTICIPANT_KIND_SIP
                    else noise_cancellation.BVC()
                ),
            ),
        ),
    )

    # Join the room and connect to the user
    await ctx.connect()


if __name__ == "__main__":
    cli.run_app(server)