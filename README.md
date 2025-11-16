# BonicBot Bridge

Python SDK for educational robotics programming with BonicBot A2

## Installation

```bash
pip install bonicbot-bridge
```

## Quick Start

```python
from bonicbot_bridge import BonicBot

bot = BonicBot()  # Connect to robot
bot.move_forward(0.3, duration=2)
bot.turn_left()
bot.stop()
```
