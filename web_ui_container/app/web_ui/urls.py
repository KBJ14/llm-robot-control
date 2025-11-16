from django.urls import path
from . import views

urlpatterns = [
    path('', views.index, name='index'),
    path('send_prompt/', views.send_prompt, name='send_prompt'),
    path('send_action/', views.send_action, name='send_action'),
    path('get_robot_status/', views.get_robot_status, name='get_robot_status'),
]