# Versioned software environment for benchmark runs (the image tag is recorded with every run).
FROM python:3.12-slim
ARG UNINBENCH_IMAGE=uninbench:0.1.0
ENV UNINBENCH_IMAGE=${UNINBENCH_IMAGE} PYTHONUNBUFFERED=1 MPLBACKEND=Agg OMP_NUM_THREADS=1
WORKDIR /opt/uninbench
COPY pyproject.toml README.md ./
COPY src ./src
RUN pip install --no-cache-dir -e ".[test]"
COPY tests ./tests
COPY examples ./examples
WORKDIR /work
ENTRYPOINT ["uninbench"]
