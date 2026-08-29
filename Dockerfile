FROM python:3.12-slim AS builder
WORKDIR /build
COPY pyproject.toml README.md ./
COPY src ./src
RUN pip wheel --no-cache-dir --wheel-dir /wheels .

FROM python:3.12-slim AS test
WORKDIR /work
COPY . .
RUN pip install --no-cache-dir ".[dev]"
USER 65532:65532
ENTRYPOINT []
CMD ["python", "-m", "pytest", "-p", "no:cacheprovider"]

FROM python:3.12-slim AS runtime
RUN addgroup --system --gid 10001 adapter && adduser --system --uid 10001 --ingroup adapter adapter
COPY --from=builder /wheels /wheels
RUN pip install --no-cache-dir /wheels/*.whl && rm -rf /wheels
USER 10001:10001
WORKDIR /app
EXPOSE 8080
ENTRYPOINT ["uvicorn", "embedded_robot_ros2.app:app"]
CMD ["--host", "0.0.0.0", "--port", "8080", "--no-access-log"]

