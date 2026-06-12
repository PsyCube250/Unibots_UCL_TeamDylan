FROM nvcr.io/nvidia/l4t-ml:r36.2.0-py3

WORKDIR /workspace

RUN pip3 install --no-cache-dir "numpy<2" ultralytics

COPY . /workspace/

CMD ["python3", "Ball_detection.py"]