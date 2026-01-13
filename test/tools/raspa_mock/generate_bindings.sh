set -e

BASE_DIR=../../..

# generate gpio C bindings
clang2py \
    ${BASE_DIR}/elk-gpio-protocol/include/gpio_protocol/gpio_protocol.h \
    >gpio_protocol.py

# generate sensei gRPC bindings
python -m grpc_tools.protoc \
    -I${BASE_DIR}/sensei-grpc-api \
    --python_out=. \
    --pyi_out=. \
    --grpc_python_out=. \
    ${BASE_DIR}/sensei-grpc-api/sensei_rpc.proto
