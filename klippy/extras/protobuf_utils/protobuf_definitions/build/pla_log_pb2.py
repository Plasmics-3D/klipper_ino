# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: pla_log.proto
# Protobuf Python Version: 4.25.3
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import nanopb_pb2 as nanopb__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\rpla_log.proto\x1a\x0cnanopb.proto\">\n\x03log\x12\x1f\n\x07log_lvl\x18\x01 \x01(\x0e\x32\x0e.pla_log_lvl_e\x12\x16\n\x07message\x18\x02 \x01(\tB\x05\x92?\x02\x08\x64*8\n\rpla_log_lvl_e\x12\x07\n\x03off\x10\x00\x12\x08\n\x04info\x10\x01\x12\t\n\x05\x64\x65\x62ug\x10\x02\x12\t\n\x05\x65rror\x10\x03')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'pla_log_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_LOG'].fields_by_name['message']._options = None
  _globals['_LOG'].fields_by_name['message']._serialized_options = b'\222?\002\010d'
  _globals['_PLA_LOG_LVL_E']._serialized_start=95
  _globals['_PLA_LOG_LVL_E']._serialized_end=151
  _globals['_LOG']._serialized_start=31
  _globals['_LOG']._serialized_end=93
# @@protoc_insertion_point(module_scope)
