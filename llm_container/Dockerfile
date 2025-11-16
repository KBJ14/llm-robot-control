FROM python:3.10

RUN pip install fastapi uvicorn requests transformers torch accelerate

WORKDIR /app

COPY app /app

EXPOSE 8001

CMD ["uvicorn", "llm_server:app", "--host", "0.0.0.0", "--port", "8001"]