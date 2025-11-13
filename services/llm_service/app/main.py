import json
import os
from typing import Any, Dict

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

app = FastAPI(title="LLM Service", version="0.1.0")

MODEL_ID = os.getenv("MODEL_ID", "Qwen/Qwen3-8B")
USE_MODEL = os.getenv("USE_MODEL", "false").lower() in ("1", "true", "yes")

pipe = None
if USE_MODEL:
    try:
        from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline
        tokenizer = AutoTokenizer.from_pretrained(MODEL_ID)
        model = AutoModelForCausalLM.from_pretrained(
            MODEL_ID,
            device_map="auto",
            torch_dtype="auto",
        )
        pipe = pipeline(
            "text-generation",
            model=model,
            tokenizer=tokenizer,
            max_new_tokens=256,
            do_sample=False,
            temperature=0.0,
        )
    except Exception as e:
        print(f"Failed to load model {MODEL_ID}: {e}. Falling back to rule-based parser.")
        pipe = None


class NLRequest(BaseModel):
    prompt: str


def rule_based_parse(prompt: str) -> Dict[str, Any]:
    p = prompt.lower()
    # Very simple heuristics for demo. Extend as needed.
    if any(k in p for k in ["go to", "navigate to", "move to"]):
        # extract rough coordinates like x=1.0 y=2.0 yaw=0.0 if present
        import re
        x = y = 0.0
        yaw = 0.0
        mx = re.search(r"x\s*[:=]\s*(-?\d+(?:\.\d+)?)", p)
        my = re.search(r"y\s*[:=]\s*(-?\d+(?:\.\d+)?)", p)
        myaw = re.search(r"yaw\s*[:=]\s*(-?\d+(?:\.\d+)?)", p)
        if mx: x = float(mx.group(1))
        if my: y = float(my.group(1))
        if myaw: yaw = float(myaw.group(1))
        return {
            "action": "navigate_to",
            "parameters": {"x": x, "y": y, "yaw": yaw, "frame_id": "map"},
        }
    elif any(k in p for k in ["rotate", "turn"]):
        import re
        ang = 0.5
        m = re.search(r"(\d+)(?:\s*deg|\s*degree)", p)
        if m:
            deg = float(m.group(1))
            from math import pi
            ang = deg * pi / 180.0
        return {"action": "rotate", "parameters": {"angular_z": ang, "duration_sec": 2.0}}
    elif any(k in p for k in ["forward", "backward", "move"]):
        speed = 0.2 if "back" not in p else -0.2
        dur = 2.0
        return {"action": "move", "parameters": {"linear_x": speed, "duration_sec": dur}}
    else:
        return {"action": "noop", "parameters": {}}


@app.post("/nl2json")
def nl2json(req: NLRequest):
    if pipe is not None:
        prompt = (
            "You are a robotics planner. Convert the instruction to a single JSON command "
            "with the structure {\"action\": <string>, \"parameters\": {..}}. "
            "Allowed actions: navigate_to(x,y,yaw,frame_id), move(linear_x, duration_sec, optional angular_z), rotate(angular_z, duration_sec). "
            "Return ONLY the JSON. Instruction: " + req.prompt
        )
        out = pipe(prompt)[0]["generated_text"][len(prompt):].strip()
        # Try to extract the first JSON object
        try:
            start = out.find('{')
            end = out.rfind('}')
            if start != -1 and end != -1:
                parsed = json.loads(out[start:end+1])
            else:
                parsed = rule_based_parse(req.prompt)
        except Exception:
            parsed = rule_based_parse(req.prompt)
        return parsed
    else:
        # Rule-based fallback
        return rule_based_parse(req.prompt)
