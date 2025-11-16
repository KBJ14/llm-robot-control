from fastapi import FastAPI
import requests
import json
from transformers import pipeline

app = FastAPI()

# LLM model (Qwen3-8B from Hugging Face)
generator = pipeline('text-generation', model='Qwen/Qwen3-8B', device_map='auto')  # GPU 사용 시 자동 할당

TOOL_SCHEMA = {
    "tools": [
        {"name": "navigate_to", "description": "Navigate to a location", "parameters": {"location": "string"}},
        {"name": "move_forward", "description": "Move forward by distance", "parameters": {"distance_m": "float"}},
        {"name": "rotate", "description": "Rotate by angle", "parameters": {"angle_deg": "float"}},
        {"name": "stop", "description": "Stop the robot"},
        {"name": "reset_world", "description": "Reset the simulation world"}
    ]
}

@app.post("/generate_action")
async def generate_action(data: dict):
    user_message = data.get('message')
    # LLM prompt with tool schema
    prompt = f"""You are a robot control assistant. Use tools to control the robot.

Available tools: {json.dumps(TOOL_SCHEMA)}

User: {user_message}

Respond with a tool call in JSON format if needed, or natural language response."""
    response = generator(prompt, max_length=200, do_sample=True, temperature=0.7)[0]['generated_text']
    # Parse tool call
    tool_call = None
    try:
        start = response.find('{')
        end = response.rfind('}') + 1
        if start != -1 and end > start:
            potential_json = response[start:end]
            parsed = json.loads(potential_json)
            if 'tool' in parsed:
                tool_call = parsed
    except:
        pass
    return {"response": response, "tool_call": tool_call}