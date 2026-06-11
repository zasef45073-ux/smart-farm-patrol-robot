FROM ros:humble-ros-base
WORKDIR /app

RUN apt update && apt install -y --no-install-recommends \
    python3-pip \
    ros-humble-cv-bridge && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt
COPY . .

EXPOSE 5000

CMD ["uvicorn", "server:app", "--host", "0.0.0.0", "--port", "5000"]
