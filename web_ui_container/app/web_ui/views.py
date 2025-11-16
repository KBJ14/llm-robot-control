from django.shortcuts import render
from django.http import JsonResponse
import requests
import json

def index(request):
    # Get available actions
    try:
        with open('/app/tool_schema.json', 'r') as f:
            tool_schema = json.load(f)
        actions = tool_schema.get('tools', [])
    except:
        actions = []
    return render(request, 'index.html', {'actions': actions})

def send_prompt(request):
    if request.method == 'POST':
        prompt = request.POST.get('prompt')
        # Call llm_container for prompt only
        response = requests.post('http://llm:8001/generate_action', json={'message': prompt})
        data = response.json()
        return JsonResponse({'llm_response': data.get('response'), 'tool_call': data.get('tool_call')})
    return JsonResponse({'error': 'Invalid request'})

def send_action(request):
    if request.method == 'POST':
        action_data = request.POST.get('action')
        try:
            action = json.loads(action_data)
            # Call ros2_gazebo
            res = requests.post("http://ros2_gazebo:8080/execute_action", json=action, timeout=10)
            observation = res.json()
        except:
            observation = {"error": "Failed to execute action"}
        return JsonResponse({'observation': observation})
    return JsonResponse({'error': 'Invalid request'})

def get_robot_status(request):
    # Placeholder for robot status
    try:
        # Assume ros2_gazebo has a status endpoint
        res = requests.get("http://ros2_gazebo:8080/status")
        status = res.json()
    except:
        status = {"status": "Unknown"}
    return JsonResponse({'robot_status': status})