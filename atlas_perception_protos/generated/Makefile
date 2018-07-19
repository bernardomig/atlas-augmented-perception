
.PHONY: all gen-csharp gen-python gen-js

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

gen-js:
	mkdir -p js
	protoc \
		-I=../protos \
		--js_out=js \
		../protos/*.proto
