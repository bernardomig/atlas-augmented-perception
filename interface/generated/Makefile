
.PHONY: all get-deps gen-csharp gen-python

all:

gen-csharp:
	mkdir -p csharp
	packages/Grpc.Tools.1.12.0/tools/linux_x64/protoc \
		-I=../protos \
		--csharp_out=csharp \
		--grpc_out=csharp \
		--plugin=protoc-gen-grpc=packages/Grpc.Tools.1.12.0/tools/linux_x64/grpc_csharp_plugin \
		../protos/*.proto

gen-python:
	mkdir -p python
	touch python/__init__.py
	python -m grpc_tools.protoc \
		-I=../protos \
		--python_out=python \
		--grpc_python_out=python \
		../protos/*.proto

get-deps: get-deps-csharp get-deps-python

get-deps-csharp:
	mkdir -p packages
	(cd packages; nuget install Grpc.Tools)

get-deps-python:
	python -m pip install grpcio grpcio_tools
