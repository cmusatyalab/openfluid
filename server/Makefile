#
# Makefile to set up consistent build environment for generated files
#

PYTHON_VER := python3.8

PROTOC_CPP := /usr/bin/protoc
PROTOC_PY := $(PYTHON_VER) -m grpc_tools.protoc
PROTOC_IPATH := Flex/demo/proto
PROTOC_OPATH_PY := ./
PROTOC_OPATH_CPP := Flex/demo/proto
PROTOC_SRC := openfluid
FLEX_BUILD_PATH := Flex/demo/compiler/makelinux64

GENERATED_FILES = \
	$(PROTOC_OPATH_CPP)/*.pb.cc \
	$(PROTOC_OPATH_CPP)/*.pb.h \

.PHONY: all clean debug release protoc
all: debug release

release: protoc_c
	cd $(FLEX_BUILD_PATH); $(MAKE) release

debug: protoc_c
	cd $(FLEX_BUILD_PATH); $(MAKE) debug

clean:
	$(RM) $(GENERATED_FILES)
	cd $(FLEX_BUILD_PATH); $(MAKE) clean

protoc: protoc_c protoc_py

protoc_c:
	$(PROTOC_CPP) -I=$(PROTOC_IPATH) --cpp_out=$(PROTOC_OPATH_CPP) $(PROTOC_SRC).proto
	$(PROTOC_CPP) -I=$(PROTOC_IPATH) --cpp_out=$(PROTOC_OPATH_CPP) gabriel.proto

protoc_py:
	$(PROTOC_PY)  -I $(PROTOC_IPATH) --python_out=$(PROTOC_OPATH_PY) $(PROTOC_SRC).proto

clean_proc:
	$(RM) $(PROTOC_SRC)_pb2.py

run:
	./entrypoint.sh

.PRECIOUS: $(GENERATED_FILES)
