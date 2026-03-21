from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping
from urllib.parse import urlparse


def _env_flag(env: Mapping[str, str], name: str, *, default: bool = False) -> bool:
    value = env.get(name)
    if value is None:
        return default
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _env_value(env: Mapping[str, str], name: str, *, default: str) -> str:
    value = env.get(name)
    if value is None:
        return default
    stripped = value.strip()
    return stripped or default


@dataclass(frozen=True)
class HostedModelConfig:
    stt_model: str = 'deepgram/nova-3'
    stt_language: str = 'multi'
    llm_model: str = 'openai/gpt-4o'
    tts_model: str = 'cartesia/sonic-3'
    tts_voice: str = '9626c31c-bec5-4cca-baa8-f8ba9e84c8bc'


@dataclass(frozen=True)
class LocalServiceConfig:
    base_url: str
    model: str
    api_key: str
    health_url: str
    language: str | None = None
    voice: str | None = None
    instructions: str | None = None
    response_format: str | None = None

    @property
    def host(self) -> str:
        return urlparse(self.base_url).netloc or self.base_url


@dataclass(frozen=True)
class LocalModelConfig:
    llm: LocalServiceConfig
    stt: LocalServiceConfig
    tts: LocalServiceConfig


@dataclass(frozen=True)
class ModelStackConfig:
    use_local_models: bool
    fallback_to_hosted: bool
    local: LocalModelConfig
    hosted: HostedModelConfig

    @classmethod
    def from_env(cls, env: Mapping[str, str]) -> 'ModelStackConfig':
        local_llm_base_url = _env_value(env, 'LOCAL_LLM_BASE_URL', default='http://127.0.0.1:11434/v1')
        local_stt_base_url = _env_value(env, 'LOCAL_STT_BASE_URL', default='http://127.0.0.1:8000/v1')
        local_tts_base_url = _env_value(env, 'LOCAL_TTS_BASE_URL', default='http://127.0.0.1:8880/v1')

        return cls(
            use_local_models=_env_flag(env, 'USE_LOCAL_MODELS', default=False),
            fallback_to_hosted=_env_flag(env, 'LOCAL_MODEL_FALLBACK_TO_LIVEKIT', default=True),
            local=LocalModelConfig(
                llm=LocalServiceConfig(
                    base_url=local_llm_base_url,
                    model=_env_value(env, 'LOCAL_LLM_MODEL', default='qwen2.5:7b-instruct'),
                    api_key=_env_value(env, 'LOCAL_LLM_API_KEY', default='ollama'),
                    health_url=_env_value(
                        env,
                        'LOCAL_LLM_HEALTH_URL',
                        default=_default_health_url('llm', local_llm_base_url),
                    ),
                ),
                stt=LocalServiceConfig(
                    base_url=local_stt_base_url,
                    model=_env_value(env, 'LOCAL_STT_MODEL', default='whisper-1'),
                    api_key=_env_value(env, 'LOCAL_STT_API_KEY', default='local-stt'),
                    health_url=_env_value(
                        env,
                        'LOCAL_STT_HEALTH_URL',
                        default=_default_health_url('stt', local_stt_base_url),
                    ),
                    language=_env_value(env, 'LOCAL_STT_LANGUAGE', default='en'),
                ),
                tts=LocalServiceConfig(
                    base_url=local_tts_base_url,
                    model=_env_value(env, 'LOCAL_TTS_MODEL', default='kokoro'),
                    api_key=_env_value(env, 'LOCAL_TTS_API_KEY', default='local-tts'),
                    health_url=_env_value(
                        env,
                        'LOCAL_TTS_HEALTH_URL',
                        default=_default_health_url('tts', local_tts_base_url),
                    ),
                    voice=_env_value(env, 'LOCAL_TTS_VOICE', default='af_heart'),
                    instructions=_env_value(
                        env,
                        'LOCAL_TTS_INSTRUCTIONS',
                        default='Speak in a friendly and concise tone.',
                    ),
                    response_format=_env_value(env, 'LOCAL_TTS_RESPONSE_FORMAT', default='pcm'),
                ),
            ),
            hosted=HostedModelConfig(
                stt_model=_env_value(env, 'HOSTED_STT_MODEL', default='deepgram/nova-3'),
                stt_language=_env_value(env, 'HOSTED_STT_LANGUAGE', default='multi'),
                llm_model=_env_value(env, 'HOSTED_LLM_MODEL', default='openai/gpt-4o'),
                tts_model=_env_value(env, 'HOSTED_TTS_MODEL', default='cartesia/sonic-3'),
                tts_voice=_env_value(
                    env,
                    'HOSTED_TTS_VOICE',
                    default='9626c31c-bec5-4cca-baa8-f8ba9e84c8bc',
                ),
            ),
        )

    def mode_label(self) -> str:
        return 'local' if self.use_local_models else 'hosted'



def _default_health_url(service: str, base_url: str) -> str:
    parsed = urlparse(base_url)
    if parsed.scheme not in {'http', 'https'}:
        return base_url

    if service == 'llm' and parsed.port == 11434:
        return f'{parsed.scheme}://{parsed.netloc}/api/tags'

    path = parsed.path.rstrip('/')
    if path.endswith('/v1'):
        path = path[:-3]
    return f'{parsed.scheme}://{parsed.netloc}{path}/models'
