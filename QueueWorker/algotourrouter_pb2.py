# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: algotourrouter.proto
# Protobuf Python Version: 5.28.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import runtime_version as _runtime_version
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
_runtime_version.ValidateProtobufRuntimeVersion(
    _runtime_version.Domain.PUBLIC,
    5,
    28,
    1,
    '',
    'algotourrouter.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x14\x61lgotourrouter.proto\x12\x0e\x61lgotourrouter\"U\n\x0fRouteRequestRPC\x12\x10\n\x08startLon\x18\x01 \x01(\x01\x12\x10\n\x08startLat\x18\x02 \x01(\x01\x12\x0e\n\x06\x65ndLon\x18\x03 \x01(\x01\x12\x0e\n\x06\x65ndLat\x18\x04 \x01(\x01\"!\n\rRouteResponse\x12\x10\n\x08pathFile\x18\x01 \x01(\t2Y\n\nAlgorouter\x12K\n\tDoRouting\x12\x1f.algotourrouter.RouteRequestRPC\x1a\x1d.algotourrouter.RouteResponseb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'algotourrouter_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_ROUTEREQUESTRPC']._serialized_start=40
  _globals['_ROUTEREQUESTRPC']._serialized_end=125
  _globals['_ROUTERESPONSE']._serialized_start=127
  _globals['_ROUTERESPONSE']._serialized_end=160
  _globals['_ALGOROUTER']._serialized_start=162
  _globals['_ALGOROUTER']._serialized_end=251
# @@protoc_insertion_point(module_scope)
