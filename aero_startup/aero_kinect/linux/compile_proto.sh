protoc -I $1/protos --grpc_out=$1 --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` $1/protos/$2
protoc -I $1/protos --cpp_out=$1 $1/protos/$2
