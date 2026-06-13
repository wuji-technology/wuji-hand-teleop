#!/bin/bash
echo "start generate proto..."
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo $DIR
export PATH=$PATH:"/home/pico/vcpkg/packages/protobuf_x64-linux/tools/protobuf" 
echo $PATH
protoc PXREAService.proto -I $DIR --plugin protoc-gen-grpc="/home/pico/vcpkg/packages/grpc_x64-linux/tools/grpc/grpc_cpp_plugin" --grpc_out $DIR --cpp_out $DIR
echo "done."

