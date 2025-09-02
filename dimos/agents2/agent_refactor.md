# DimOS Agents2: LangChain-Based Agent Refactor

## Overview

The `agents2` module represents a complete refactor of the DimOS agent system, migrating from a custom implementation to a LangChain-based architecture. This refactor provides better integration with modern LLM frameworks, standardized tool calling, and improved message handling.

## Architecture

### Core Components

#### 1. **AgentSpec** (`spec.py`)
- Abstract base class defining the agent interface
- Inherits from `Service[AgentConfig]` and `Module`
- Provides transport layer for publishing agent messages via LCM
- Defines abstract methods that all agents must implement:
  - `start()`, `stop()`, `clear_history()`
  - `append_history()`, `history()`
  - `query()` - main interaction method
- Rich console output for debugging agent conversations

#### 2. **Agent** (`agent.py`)
- Concrete implementation of `AgentSpec`
- Integrates with `SkillCoordinator` for tool/skill management
- Uses LangChain's `init_chat_model` for LLM interaction
- Key features:
  - Dynamic tool binding per conversation turn
  - Asynchronous agent loop with skill state management
  - Support for implicit skill execution
  - Message snapshot system for long-running skills

#### 3. **Message Types**
- Leverages LangChain's message types:
  - `SystemMessage` - system prompts
  - `HumanMessage` - user inputs
  - `AIMessage` - agent responses
  - `ToolMessage` - tool execution results
  - `ToolCall` - tool invocation requests

#### 4. **Configuration**
- `AgentConfig` dataclass with:
  - Model selection (extensive enum of supported models)
  - Provider selection (dynamically generated from LangChain)
  - System prompt configuration
  - Transport configuration (LCM by default)
  - Skills/tools configuration

### Key Differences from Old Agent System

| Aspect | Old System (`dimos/agents`) | New System (`dimos/agents2`) |
|--------|------------------------------|-------------------------------|
| **Framework** | Custom implementation | LangChain-based |
| **Message Handling** | Custom `AgentMessage` class | LangChain message types |
| **Tool Integration** | Custom `AbstractSkill` | LangChain tools + SkillCoordinator |
| **Model Support** | Manual provider implementations | LangChain's unified interface |
| **Streaming** | Custom stream handling | Integrated with SkillCoordinator |
| **Memory** | Custom `AbstractAgentSemanticMemory` | Not yet implemented (TODO) |
| **Configuration** | Multiple parameters | Unified `AgentConfig` dataclass |

## Migration Guide

### For Agent Users

**Old way:**
```python
from dimos.agents.modules.base_agent import BaseAgentModule

agent = BaseAgentModule(
    model="openai::gpt-4o-mini",
    system_prompt="You are a helpful assistant",
    skills=skill_library,
    temperature=0.0
)
```

**New way:**
```python
from dimos.agents2 import Agent, AgentSpec
from dimos.agents2.spec import Model, Provider

agent = Agent(
    system_prompt="You are a helpful assistant",
    model=Model.GPT_4O_MINI,
    provider=Provider.OPENAI
)
agent.register_skills(skill_container)
```

### For Skill Developers

**Old way:**
```python
from dimos.skills.skills import AbstractSkill

class MySkill(AbstractSkill):
    def execute(self, *args, **kwargs):
        return result
```

**New way:**
```python
from dimos.protocol.skill.skill import SkillContainer, skill

class MySkillContainer(SkillContainer):
    @skill()
    def my_skill(self, arg1: int, arg2: str) -> str:
        """Skill description for LLM."""
        return result
```

## Current Issues & TODOs

### Immediate Issues

1. **Python Version Compatibility**
   - ✅ Fixed: `type` alias syntax incompatible with Python 3.10
   - Solution: Use simple assignment `AnyMessage = Union[...]` instead of `type AnyMessage = ...`

### TODO Items

1. **Memory/RAG Integration**
   - Old system had `AbstractAgentSemanticMemory` for semantic search
   - New system needs LangChain memory integration
   - Consider using LangChain's memory abstractions

2. **Streaming Improvements**
   - Better handling of streaming responses
   - Integration with LangChain's streaming capabilities

3. **Testing**
   - Expand test coverage beyond basic `test_agent.py`
   - Add integration tests with real LLM providers
   - Test skill coordination edge cases

4. **Documentation**
   - Add docstrings to all public methods
   - Create usage examples
   - Document skill development patterns

5. **Performance**
   - Profile agent loop performance
   - Optimize message history management
   - Consider caching strategies for tools

6. **Error Handling**
   - Improve error recovery in agent loop
   - Better error messages for skill failures
   - Timeout handling for long-running skills

## Testing Strategy

### Unit Tests
- Test message handling and transformation
- Test skill registration and tool generation
- Test configuration parsing

### Integration Tests
- Test with mock LLM providers
- Test skill execution flow
- Test error scenarios

### System Tests
- End-to-end conversation flow
- Multi-turn interactions with tools
- Long-running skill management

## Code Quality Notes

### Strengths
- Clean separation of concerns (spec vs implementation)
- Good use of type hints and dataclasses
- Leverages established LangChain patterns
- Modular skill system

### Areas for Improvement
- Add comprehensive error handling
- Implement proper logging throughout
- Add metrics/observability
- Consider adding middleware support

## Performance Considerations

1. **Message History**: Currently keeps full history in memory
   - Consider implementing sliding window
   - Add history persistence option

2. **Tool Binding**: Re-binds tools on each turn
   - Could cache if tool set is stable
   - Profile impact on latency

3. **Async Handling**: Good use of async/await
   - Consider adding connection pooling for LLM calls
   - Implement proper backpressure handling

## Security Considerations

1. **Input Validation**: Need to validate tool arguments
2. **Prompt Injection**: Consider adding guards
3. **Rate Limiting**: Add support for rate limiting LLM calls
4. **Secrets Management**: Ensure API keys are handled securely

## Compatibility Matrix

| Python Version | Status | Notes |
|----------------|--------|-------|
| 3.10 | ✅ Supported | Use `AnyMessage = Union[...]` syntax |
| 3.11 | ✅ Supported | Same as 3.10 |
| 3.12+ | ✅ Supported | Could use `type` keyword but not required |

## Dependencies

### Required
- `langchain-core`: Core LangChain functionality
- `langchain`: Chat model initialization
- `rich`: Console output formatting
- `dimos.protocol.skill`: Skill coordination system
- `dimos.core`: DimOS module system

### Optional (Provider-Specific)
- `langchain-openai`: For OpenAI models
- `langchain-anthropic`: For Claude models
- `langchain-google-genai`: For Gemini models
- etc.

## Next Steps

1. **Immediate**
   - ✅ Fix Python 3.10 compatibility
   - Add proper error handling to agent loop
   - Implement basic memory support

2. **Short-term**
   - Expand test coverage
   - Add more comprehensive examples
   - Document migration path for existing agents

3. **Long-term**
   - Full feature parity with old agent system
   - Performance optimizations
   - Advanced features (multi-agent coordination, etc.)

## Implementation Progress

### Completed Tasks

#### 1. UnitreeSkillContainer Creation (✅ Complete)
- **File**: `dimos/robot/unitree_webrtc/unitree_skill_container.py`
- **Status**: Successfully converted all Unitree skills to new framework
- **Changes Made**:
  - Converted from `AbstractSkill`/`AbstractRobotSkill` to `SkillContainer` with `@skill` decorators
  - Migrated all movement skills (move, wait)
  - Migrated navigation skills (navigate_with_text, get_pose, navigate_to_goal, explore)
  - Migrated speech skill (speak with OpenAI TTS)
  - Migrated all Unitree control skills (damp, stand_up, sit, dance, flip, etc.)
  - Added proper type hints and docstrings for LangChain compatibility
  - Implemented helper methods for WebRTC communication

#### Key Skill Migration Patterns Applied:
1. **Simple Skills**: Direct conversion with `@skill()` decorator
   ```python
   # Old: class Wait(AbstractSkill)
   # New:
   @skill()
   def wait(self, seconds: float) -> str:
   ```

2. **Robot Skills**: Maintain robot reference in container init
   ```python
   def __init__(self, robot: Optional['UnitreeGo2'] = None):
       self._robot = robot
   ```

3. **Streaming Skills**: Use Stream and Reducer parameters
   ```python
   @skill(stream=Stream.passive, reducer=Reducer.latest)
   def explore(...) -> Generator[dict, None, None]:
   ```

4. **Image Output Skills**: Use Output parameter
   ```python
   @skill(output=Output.image)
   def take_photo(self) -> Image:
   ```

### Testing Complete (✅)
- Test file created: `dimos/agents2/temp/test_unitree_skills.py`
- Run file created: `dimos/agents2/temp/run_unitree_agents2.py`  
- **43 skills successfully registered** (41 dynamic + 2 explicit)
- Skills have proper LangChain-compatible schemas

### Dynamic Skill Generation Implementation (✅)
- **File**: `dimos/robot/unitree_webrtc/unitree_skill_container.py`
- **Method**: Dynamically generates skills from `UNITREE_WEBRTC_CONTROLS` list
- **Pattern**: 
  ```python
  def _create_dynamic_skill(self, skill_name, api_id, description, original_name):
      def dynamic_skill_func(self) -> str:
          return self._execute_sport_command(api_id, original_name)
      decorated_skill = skill()(dynamic_skill_func)
      setattr(self, skill_name, decorated_skill.__get__(self, self.__class__))
  ```

### Skills Successfully Migrated:
**Explicit Skills (2)**:
- `move` - Direct velocity control with duration
- `wait` - Time delay

**Dynamic Skills (41)** - Generated from UNITREE_WEBRTC_CONTROLS:
- **Basic Movement**: damp, balance_stand, stand_up, stand_down, recovery_stand, sit, rise_sit
- **Gaits**: switch_gait, continuous_gait, economic_gait  
- **Actions**: hello, stretch, wallow, scrape, pose
- **Dance**: dance1, dance2, wiggle_hips, moon_walk
- **Advanced**: front_flip, back_flip, left_flip, right_flip, front_jump, front_pounce, handstand, bound
- **Settings**: body_height, foot_raise_height, speed_level, trigger
- **And more...**

### Ready for Integration Testing
The system is now ready to test with:
- Real robot hardware (UnitreeGo2)
- Live LLM API calls (OpenAI GPT-4 or similar)
- Web interface integration

## Event Loop Fix (✅ Resolved)

### Final Solution:
The Tornado `AsyncIOMainLoop` used by Dask wraps an asyncio loop. We access the underlying loop via `asyncio_loop` attribute:

```python
# In query_async() and query()
if loop_type == "AsyncIOMainLoop":
    actual_loop = self._loop.asyncio_loop  # Get the wrapped asyncio loop
    return asyncio.ensure_future(self.agent_loop(query), loop=actual_loop)
```

### Fixed Issues:
1. **AsyncIOMainLoop.create_task Error**: Fixed by using the wrapped asyncio loop
2. **AsyncIOMainLoop.is_running Error**: Fixed by checking loop type before calling
3. **Event Loop Management**: 
   - Tornado AsyncIOMainLoop: Use wrapped `asyncio_loop` attribute
   - Standard asyncio loop: Use directly, start in thread if needed

### Known Limitation:
**Dynamic Skills & Dask**: Dynamically generated skills have pickle issues when sent over network. 
**Workaround**: Create the container locally on the same worker as the agent.

## Event Loop Implementation Details

### The Issue:
- Module class creates `self._loop` but doesn't run it
- `agent.query()` uses `asyncio.run_coroutine_threadsafe()` which requires a running loop
- This caused queries to hang or fail

### The Solution:
- Added event loop startup in `Agent.start()` method
- Automatically starts loop in background thread if not running
- Now `agent.query()` works immediately after `agent.start()`

### Clean Usage:
```python
agent = Agent(...)
agent.register_skills(container)
agent.start()  # This ensures event loop is running
result = agent.query("Hello!")  # Works without any thread management
```

### Two Clean Approaches:
1. **Sync API** (run_unitree_agents2.py): Use `agent.start()` then `agent.query()`
2. **Async API** (run_unitree_async.py): Use `async`/`await` throughout

## Current Status (August 2025)

### Working Features:
- ✅ LangChain-based agent with tool binding
- ✅ SkillCoordinator integration 
- ✅ UnitreeSkillContainer with 43 skills (41 dynamic + 2 explicit)
- ✅ Event loop compatibility (Tornado AsyncIOMainLoop & standard asyncio)
- ✅ Both sync and async query methods
- ✅ Skill streaming and implicit skills
- ✅ Message snapshot system

### Test Files (in agents2/temp/):
- `test_unitree_skills.py` - Tests skill registration
- `run_unitree_agents2.py` - Sync approach for running agent
- `run_unitree_async.py` - Async approach 
- `test_simple_query.py` - Basic query testing
- `test_event_loop.py` - Event loop testing
- `test_agent_query.py` - Agent query testing
- `test_tornado_fix.py` - Tornado compatibility testing

## Conclusion

The agents2 refactor successfully modernizes the DimOS agent system by adopting LangChain, providing better standardization and ecosystem compatibility. The Unitree robot skills have been fully migrated with dynamic generation, and event loop issues have been resolved. The foundation is solid and ready for integration testing with actual hardware.