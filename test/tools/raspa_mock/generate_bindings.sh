set -e

BASE_DIR=../../..

# generate gpio C bindings
clang2py \
    ${BASE_DIR}/elk-gpio-protocol/include/gpio_protocol/gpio_protocol.h \
    >gpio_protocol.py

# generate pin-proxy gRPC bindings
PIN_PROXY_DIR=../../../pin-proxy
python -m grpc_tools.protoc \
    -I${BASE_DIR}/pin-proxy \
    --python_out=. \
    --pyi_out=. \
    --grpc_python_out=. \
    ${BASE_DIR}/pin-proxy/pin_events.proto
