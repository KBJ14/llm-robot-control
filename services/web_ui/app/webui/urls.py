from django.contrib import admin
from django.urls import path

from webuiapp import views

urlpatterns = [
    path('admin/', admin.site.urls),
    path('', views.index, name='index'),
    path('api/nl2json/', views.nl2json_proxy, name='nl2json'),
    path('api/execute/', views.execute_command, name='execute'),
]
