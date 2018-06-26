
.PHONY: all get-deps gen-csharp gen-python

all:

gen-csharp:
	mkdir -p csharp
	protoc \
		-I=../protos \
		--csharp_out=csharp \
		../protos/*.proto

gen-python:
	mkdir -p python
	touch python/__init__.py
	protoc \
		-I=../protos \
		--python_out=python \
		../protos/*.proto
