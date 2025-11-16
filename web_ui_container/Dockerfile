FROM python:3.10

RUN pip install django djangorestframework requests

WORKDIR /app

COPY app /app

EXPOSE 8000

CMD ["python", "manage.py", "runserver", "0.0.0.0:8000"]