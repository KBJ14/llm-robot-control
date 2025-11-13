import json
import os

import requests
from django.conf import settings
from django.http import JsonResponse
from django.shortcuts import render
from django.views.decorators.csrf import csrf_exempt

ROS_BRIDGE_URL = getattr(settings, 'ROS_BRIDGE_URL', 'http://localhost:8000')
LLM_SERVICE_URL = getattr(settings, 'LLM_SERVICE_URL', 'http://localhost:8100')


def index(request):
    return render(request, 'index.html', {
        'ros_bridge_url': ROS_BRIDGE_URL,
        'llm_service_url': LLM_SERVICE_URL,
    })


@csrf_exempt
def nl2json_proxy(request):
    if request.method != 'POST':
        return JsonResponse({'error': 'POST only'}, status=405)
    try:
        data = json.loads(request.body.decode('utf-8'))
    except Exception:
        data = {'prompt': request.POST.get('prompt', '')}
    resp = requests.post(f"{LLM_SERVICE_URL}/nl2json", json={'prompt': data.get('prompt', '')})
    return JsonResponse(resp.json(), status=resp.status_code, safe=False)


@csrf_exempt
def execute_command(request):
    if request.method != 'POST':
        return JsonResponse({'error': 'POST only'}, status=405)
    payload = json.loads(request.body.decode('utf-8'))
    resp = requests.post(f"{ROS_BRIDGE_URL}/command", json=payload)
    return JsonResponse(resp.json(), status=resp.status_code, safe=False)
