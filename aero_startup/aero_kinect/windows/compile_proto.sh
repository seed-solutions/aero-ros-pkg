protoc_exe=$(find $HOME/ -name protoc.exe | grep -m 1 "third_party/protobuf/cmake/Release/")
grpc_exe=$(echo "$protoc_exe" | awk -F/ '{sub("/"$NF,"");print}')/../../../../examples/csharp/helloworld/packages/Grpc.Tools.0.7.1/tools/grpc_csharp_plugin.exe
$protoc_exe -I $1/protos --csharp_out $1/$1 --grpc_out $1/$1 --plugin=protoc-gen-grpc=$grpc_exe $1/protos/$2
