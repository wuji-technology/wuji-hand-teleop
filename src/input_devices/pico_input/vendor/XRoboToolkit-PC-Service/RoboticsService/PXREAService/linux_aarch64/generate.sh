#!/bin/bash
echo "start generate proto..."
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo $DIR
export PATH=$PATH:"/media/bytedance/newSpace/grpcsrc/grpcbin/bin" 
echo $PATH
protoc PXREAService.proto -I $DIR --plugin protoc-gen-grpc="/media/bytedance/newSpace/grpcsrc/grpcbin/bin/grpc_cpp_plugin" --grpc_out $DIR --cpp_out $DIR
echo "done."

