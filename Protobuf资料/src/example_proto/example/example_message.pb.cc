// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: example_message.proto

#include "example_message.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_example_5fmessage_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_MsgResult_example_5fmessage_2eproto;
class MsgResultDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<MsgResult> _instance;
} _MsgResult_default_instance_;
class TopMessageDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<TopMessage> _instance;
} _TopMessage_default_instance_;
static void InitDefaultsscc_info_MsgResult_example_5fmessage_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::_MsgResult_default_instance_;
    new (ptr) ::MsgResult();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::MsgResult::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_MsgResult_example_5fmessage_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_MsgResult_example_5fmessage_2eproto}, {}};

static void InitDefaultsscc_info_TopMessage_example_5fmessage_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::_TopMessage_default_instance_;
    new (ptr) ::TopMessage();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::TopMessage::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_TopMessage_example_5fmessage_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_TopMessage_example_5fmessage_2eproto}, {
      &scc_info_MsgResult_example_5fmessage_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_example_5fmessage_2eproto[2];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_example_5fmessage_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_example_5fmessage_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_example_5fmessage_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::MsgResult, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::MsgResult, result_),
  PROTOBUF_FIELD_OFFSET(::MsgResult, error_code_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::TopMessage, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::TopMessage, message_type_),
  PROTOBUF_FIELD_OFFSET(::TopMessage, msg_result_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::MsgResult)},
  { 7, -1, sizeof(::TopMessage)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::_MsgResult_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::_TopMessage_default_instance_),
};

const char descriptor_table_protodef_example_5fmessage_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\025example_message.proto\"/\n\tMsgResult\022\016\n\006"
  "result\030\001 \001(\010\022\022\n\nerror_code\030\002 \001(\014\"P\n\nTopM"
  "essage\022\"\n\014message_type\030\001 \001(\0162\014.Messagety"
  "pe\022\036\n\nmsg_result\030\002 \001(\0132\n.MsgResult*e\n\013Me"
  "ssagetype\022\031\n\025REQUEST_RESPONSE_NONE\020\000\022\034\n\030"
  "REQUEST_HEARTBEAT_SIGNAL\020\001\022\035\n\031RESPONSE_H"
  "EARTBEAT_RESULT\020\002b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_example_5fmessage_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_example_5fmessage_2eproto_sccs[2] = {
  &scc_info_MsgResult_example_5fmessage_2eproto.base,
  &scc_info_TopMessage_example_5fmessage_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_example_5fmessage_2eproto_once;
static bool descriptor_table_example_5fmessage_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_example_5fmessage_2eproto = {
  &descriptor_table_example_5fmessage_2eproto_initialized, descriptor_table_protodef_example_5fmessage_2eproto, "example_message.proto", 265,
  &descriptor_table_example_5fmessage_2eproto_once, descriptor_table_example_5fmessage_2eproto_sccs, descriptor_table_example_5fmessage_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_example_5fmessage_2eproto::offsets,
  file_level_metadata_example_5fmessage_2eproto, 2, file_level_enum_descriptors_example_5fmessage_2eproto, file_level_service_descriptors_example_5fmessage_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_example_5fmessage_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_example_5fmessage_2eproto), true);
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Messagetype_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_example_5fmessage_2eproto);
  return file_level_enum_descriptors_example_5fmessage_2eproto[0];
}
bool Messagetype_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}


// ===================================================================

void MsgResult::InitAsDefaultInstance() {
}
class MsgResult::_Internal {
 public:
};

MsgResult::MsgResult()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:MsgResult)
}
MsgResult::MsgResult(const MsgResult& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  error_code_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (!from._internal_error_code().empty()) {
    error_code_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.error_code_);
  }
  result_ = from.result_;
  // @@protoc_insertion_point(copy_constructor:MsgResult)
}

void MsgResult::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_MsgResult_example_5fmessage_2eproto.base);
  error_code_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  result_ = false;
}

MsgResult::~MsgResult() {
  // @@protoc_insertion_point(destructor:MsgResult)
  SharedDtor();
}

void MsgResult::SharedDtor() {
  error_code_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void MsgResult::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const MsgResult& MsgResult::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_MsgResult_example_5fmessage_2eproto.base);
  return *internal_default_instance();
}


void MsgResult::Clear() {
// @@protoc_insertion_point(message_clear_start:MsgResult)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  error_code_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  result_ = false;
  _internal_metadata_.Clear();
}

const char* MsgResult::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // bool result = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          result_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // bytes error_code = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_error_code();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* MsgResult::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:MsgResult)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // bool result = 1;
  if (this->result() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1, this->_internal_result(), target);
  }

  // bytes error_code = 2;
  if (this->error_code().size() > 0) {
    target = stream->WriteBytesMaybeAliased(
        2, this->_internal_error_code(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:MsgResult)
  return target;
}

size_t MsgResult::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:MsgResult)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // bytes error_code = 2;
  if (this->error_code().size() > 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::BytesSize(
        this->_internal_error_code());
  }

  // bool result = 1;
  if (this->result() != 0) {
    total_size += 1 + 1;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void MsgResult::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:MsgResult)
  GOOGLE_DCHECK_NE(&from, this);
  const MsgResult* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<MsgResult>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:MsgResult)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:MsgResult)
    MergeFrom(*source);
  }
}

void MsgResult::MergeFrom(const MsgResult& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:MsgResult)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.error_code().size() > 0) {

    error_code_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.error_code_);
  }
  if (from.result() != 0) {
    _internal_set_result(from._internal_result());
  }
}

void MsgResult::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:MsgResult)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MsgResult::CopyFrom(const MsgResult& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:MsgResult)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MsgResult::IsInitialized() const {
  return true;
}

void MsgResult::InternalSwap(MsgResult* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  error_code_.Swap(&other->error_code_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(result_, other->result_);
}

::PROTOBUF_NAMESPACE_ID::Metadata MsgResult::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void TopMessage::InitAsDefaultInstance() {
  ::_TopMessage_default_instance_._instance.get_mutable()->msg_result_ = const_cast< ::MsgResult*>(
      ::MsgResult::internal_default_instance());
}
class TopMessage::_Internal {
 public:
  static const ::MsgResult& msg_result(const TopMessage* msg);
};

const ::MsgResult&
TopMessage::_Internal::msg_result(const TopMessage* msg) {
  return *msg->msg_result_;
}
TopMessage::TopMessage()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:TopMessage)
}
TopMessage::TopMessage(const TopMessage& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_msg_result()) {
    msg_result_ = new ::MsgResult(*from.msg_result_);
  } else {
    msg_result_ = nullptr;
  }
  message_type_ = from.message_type_;
  // @@protoc_insertion_point(copy_constructor:TopMessage)
}

void TopMessage::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_TopMessage_example_5fmessage_2eproto.base);
  ::memset(&msg_result_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&message_type_) -
      reinterpret_cast<char*>(&msg_result_)) + sizeof(message_type_));
}

TopMessage::~TopMessage() {
  // @@protoc_insertion_point(destructor:TopMessage)
  SharedDtor();
}

void TopMessage::SharedDtor() {
  if (this != internal_default_instance()) delete msg_result_;
}

void TopMessage::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const TopMessage& TopMessage::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_TopMessage_example_5fmessage_2eproto.base);
  return *internal_default_instance();
}


void TopMessage::Clear() {
// @@protoc_insertion_point(message_clear_start:TopMessage)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaNoVirtual() == nullptr && msg_result_ != nullptr) {
    delete msg_result_;
  }
  msg_result_ = nullptr;
  message_type_ = 0;
  _internal_metadata_.Clear();
}

const char* TopMessage::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // .Messagetype message_type = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          _internal_set_message_type(static_cast<::Messagetype>(val));
        } else goto handle_unusual;
        continue;
      // .MsgResult msg_result = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_msg_result(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* TopMessage::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:TopMessage)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .Messagetype message_type = 1;
  if (this->message_type() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1, this->_internal_message_type(), target);
  }

  // .MsgResult msg_result = 2;
  if (this->has_msg_result()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::msg_result(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:TopMessage)
  return target;
}

size_t TopMessage::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:TopMessage)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .MsgResult msg_result = 2;
  if (this->has_msg_result()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *msg_result_);
  }

  // .Messagetype message_type = 1;
  if (this->message_type() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_message_type());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void TopMessage::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:TopMessage)
  GOOGLE_DCHECK_NE(&from, this);
  const TopMessage* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<TopMessage>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:TopMessage)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:TopMessage)
    MergeFrom(*source);
  }
}

void TopMessage::MergeFrom(const TopMessage& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:TopMessage)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_msg_result()) {
    _internal_mutable_msg_result()->::MsgResult::MergeFrom(from._internal_msg_result());
  }
  if (from.message_type() != 0) {
    _internal_set_message_type(from._internal_message_type());
  }
}

void TopMessage::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:TopMessage)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void TopMessage::CopyFrom(const TopMessage& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:TopMessage)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TopMessage::IsInitialized() const {
  return true;
}

void TopMessage::InternalSwap(TopMessage* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(msg_result_, other->msg_result_);
  swap(message_type_, other->message_type_);
}

::PROTOBUF_NAMESPACE_ID::Metadata TopMessage::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::MsgResult* Arena::CreateMaybeMessage< ::MsgResult >(Arena* arena) {
  return Arena::CreateInternal< ::MsgResult >(arena);
}
template<> PROTOBUF_NOINLINE ::TopMessage* Arena::CreateMaybeMessage< ::TopMessage >(Arena* arena) {
  return Arena::CreateInternal< ::TopMessage >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
