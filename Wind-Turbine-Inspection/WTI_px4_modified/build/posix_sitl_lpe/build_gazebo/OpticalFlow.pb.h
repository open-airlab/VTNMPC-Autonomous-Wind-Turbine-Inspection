// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: OpticalFlow.proto

#ifndef PROTOBUF_OpticalFlow_2eproto__INCLUDED
#define PROTOBUF_OpticalFlow_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace sensor_msgs {
namespace msgs {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_OpticalFlow_2eproto();
void protobuf_AssignDesc_OpticalFlow_2eproto();
void protobuf_ShutdownFile_OpticalFlow_2eproto();

class OpticalFlow;

// ===================================================================

class OpticalFlow : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:sensor_msgs.msgs.OpticalFlow) */ {
 public:
  OpticalFlow();
  virtual ~OpticalFlow();

  OpticalFlow(const OpticalFlow& from);

  inline OpticalFlow& operator=(const OpticalFlow& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const OpticalFlow& default_instance();

  void Swap(OpticalFlow* other);

  // implements Message ----------------------------------------------

  inline OpticalFlow* New() const { return New(NULL); }

  OpticalFlow* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const OpticalFlow& from);
  void MergeFrom(const OpticalFlow& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(OpticalFlow* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required int32 time_usec = 1;
  bool has_time_usec() const;
  void clear_time_usec();
  static const int kTimeUsecFieldNumber = 1;
  ::google::protobuf::int32 time_usec() const;
  void set_time_usec(::google::protobuf::int32 value);

  // required int32 sensor_id = 2;
  bool has_sensor_id() const;
  void clear_sensor_id();
  static const int kSensorIdFieldNumber = 2;
  ::google::protobuf::int32 sensor_id() const;
  void set_sensor_id(::google::protobuf::int32 value);

  // required int32 integration_time_us = 3;
  bool has_integration_time_us() const;
  void clear_integration_time_us();
  static const int kIntegrationTimeUsFieldNumber = 3;
  ::google::protobuf::int32 integration_time_us() const;
  void set_integration_time_us(::google::protobuf::int32 value);

  // required float integrated_x = 4;
  bool has_integrated_x() const;
  void clear_integrated_x();
  static const int kIntegratedXFieldNumber = 4;
  float integrated_x() const;
  void set_integrated_x(float value);

  // required float integrated_y = 5;
  bool has_integrated_y() const;
  void clear_integrated_y();
  static const int kIntegratedYFieldNumber = 5;
  float integrated_y() const;
  void set_integrated_y(float value);

  // required float integrated_xgyro = 6;
  bool has_integrated_xgyro() const;
  void clear_integrated_xgyro();
  static const int kIntegratedXgyroFieldNumber = 6;
  float integrated_xgyro() const;
  void set_integrated_xgyro(float value);

  // required float integrated_ygyro = 7;
  bool has_integrated_ygyro() const;
  void clear_integrated_ygyro();
  static const int kIntegratedYgyroFieldNumber = 7;
  float integrated_ygyro() const;
  void set_integrated_ygyro(float value);

  // required float integrated_zgyro = 8;
  bool has_integrated_zgyro() const;
  void clear_integrated_zgyro();
  static const int kIntegratedZgyroFieldNumber = 8;
  float integrated_zgyro() const;
  void set_integrated_zgyro(float value);

  // required float temperature = 9;
  bool has_temperature() const;
  void clear_temperature();
  static const int kTemperatureFieldNumber = 9;
  float temperature() const;
  void set_temperature(float value);

  // required int32 quality = 10;
  bool has_quality() const;
  void clear_quality();
  static const int kQualityFieldNumber = 10;
  ::google::protobuf::int32 quality() const;
  void set_quality(::google::protobuf::int32 value);

  // required int32 time_delta_distance_us = 11;
  bool has_time_delta_distance_us() const;
  void clear_time_delta_distance_us();
  static const int kTimeDeltaDistanceUsFieldNumber = 11;
  ::google::protobuf::int32 time_delta_distance_us() const;
  void set_time_delta_distance_us(::google::protobuf::int32 value);

  // required float distance = 12;
  bool has_distance() const;
  void clear_distance();
  static const int kDistanceFieldNumber = 12;
  float distance() const;
  void set_distance(float value);

  // @@protoc_insertion_point(class_scope:sensor_msgs.msgs.OpticalFlow)
 private:
  inline void set_has_time_usec();
  inline void clear_has_time_usec();
  inline void set_has_sensor_id();
  inline void clear_has_sensor_id();
  inline void set_has_integration_time_us();
  inline void clear_has_integration_time_us();
  inline void set_has_integrated_x();
  inline void clear_has_integrated_x();
  inline void set_has_integrated_y();
  inline void clear_has_integrated_y();
  inline void set_has_integrated_xgyro();
  inline void clear_has_integrated_xgyro();
  inline void set_has_integrated_ygyro();
  inline void clear_has_integrated_ygyro();
  inline void set_has_integrated_zgyro();
  inline void clear_has_integrated_zgyro();
  inline void set_has_temperature();
  inline void clear_has_temperature();
  inline void set_has_quality();
  inline void clear_has_quality();
  inline void set_has_time_delta_distance_us();
  inline void clear_has_time_delta_distance_us();
  inline void set_has_distance();
  inline void clear_has_distance();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::int32 time_usec_;
  ::google::protobuf::int32 sensor_id_;
  ::google::protobuf::int32 integration_time_us_;
  float integrated_x_;
  float integrated_y_;
  float integrated_xgyro_;
  float integrated_ygyro_;
  float integrated_zgyro_;
  float temperature_;
  ::google::protobuf::int32 quality_;
  ::google::protobuf::int32 time_delta_distance_us_;
  float distance_;
  friend void  protobuf_AddDesc_OpticalFlow_2eproto();
  friend void protobuf_AssignDesc_OpticalFlow_2eproto();
  friend void protobuf_ShutdownFile_OpticalFlow_2eproto();

  void InitAsDefaultInstance();
  static OpticalFlow* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// OpticalFlow

// required int32 time_usec = 1;
inline bool OpticalFlow::has_time_usec() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void OpticalFlow::set_has_time_usec() {
  _has_bits_[0] |= 0x00000001u;
}
inline void OpticalFlow::clear_has_time_usec() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void OpticalFlow::clear_time_usec() {
  time_usec_ = 0;
  clear_has_time_usec();
}
inline ::google::protobuf::int32 OpticalFlow::time_usec() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.time_usec)
  return time_usec_;
}
inline void OpticalFlow::set_time_usec(::google::protobuf::int32 value) {
  set_has_time_usec();
  time_usec_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.time_usec)
}

// required int32 sensor_id = 2;
inline bool OpticalFlow::has_sensor_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void OpticalFlow::set_has_sensor_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void OpticalFlow::clear_has_sensor_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void OpticalFlow::clear_sensor_id() {
  sensor_id_ = 0;
  clear_has_sensor_id();
}
inline ::google::protobuf::int32 OpticalFlow::sensor_id() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.sensor_id)
  return sensor_id_;
}
inline void OpticalFlow::set_sensor_id(::google::protobuf::int32 value) {
  set_has_sensor_id();
  sensor_id_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.sensor_id)
}

// required int32 integration_time_us = 3;
inline bool OpticalFlow::has_integration_time_us() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void OpticalFlow::set_has_integration_time_us() {
  _has_bits_[0] |= 0x00000004u;
}
inline void OpticalFlow::clear_has_integration_time_us() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void OpticalFlow::clear_integration_time_us() {
  integration_time_us_ = 0;
  clear_has_integration_time_us();
}
inline ::google::protobuf::int32 OpticalFlow::integration_time_us() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.integration_time_us)
  return integration_time_us_;
}
inline void OpticalFlow::set_integration_time_us(::google::protobuf::int32 value) {
  set_has_integration_time_us();
  integration_time_us_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.integration_time_us)
}

// required float integrated_x = 4;
inline bool OpticalFlow::has_integrated_x() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void OpticalFlow::set_has_integrated_x() {
  _has_bits_[0] |= 0x00000008u;
}
inline void OpticalFlow::clear_has_integrated_x() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void OpticalFlow::clear_integrated_x() {
  integrated_x_ = 0;
  clear_has_integrated_x();
}
inline float OpticalFlow::integrated_x() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.integrated_x)
  return integrated_x_;
}
inline void OpticalFlow::set_integrated_x(float value) {
  set_has_integrated_x();
  integrated_x_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.integrated_x)
}

// required float integrated_y = 5;
inline bool OpticalFlow::has_integrated_y() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void OpticalFlow::set_has_integrated_y() {
  _has_bits_[0] |= 0x00000010u;
}
inline void OpticalFlow::clear_has_integrated_y() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void OpticalFlow::clear_integrated_y() {
  integrated_y_ = 0;
  clear_has_integrated_y();
}
inline float OpticalFlow::integrated_y() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.integrated_y)
  return integrated_y_;
}
inline void OpticalFlow::set_integrated_y(float value) {
  set_has_integrated_y();
  integrated_y_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.integrated_y)
}

// required float integrated_xgyro = 6;
inline bool OpticalFlow::has_integrated_xgyro() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void OpticalFlow::set_has_integrated_xgyro() {
  _has_bits_[0] |= 0x00000020u;
}
inline void OpticalFlow::clear_has_integrated_xgyro() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void OpticalFlow::clear_integrated_xgyro() {
  integrated_xgyro_ = 0;
  clear_has_integrated_xgyro();
}
inline float OpticalFlow::integrated_xgyro() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.integrated_xgyro)
  return integrated_xgyro_;
}
inline void OpticalFlow::set_integrated_xgyro(float value) {
  set_has_integrated_xgyro();
  integrated_xgyro_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.integrated_xgyro)
}

// required float integrated_ygyro = 7;
inline bool OpticalFlow::has_integrated_ygyro() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void OpticalFlow::set_has_integrated_ygyro() {
  _has_bits_[0] |= 0x00000040u;
}
inline void OpticalFlow::clear_has_integrated_ygyro() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void OpticalFlow::clear_integrated_ygyro() {
  integrated_ygyro_ = 0;
  clear_has_integrated_ygyro();
}
inline float OpticalFlow::integrated_ygyro() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.integrated_ygyro)
  return integrated_ygyro_;
}
inline void OpticalFlow::set_integrated_ygyro(float value) {
  set_has_integrated_ygyro();
  integrated_ygyro_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.integrated_ygyro)
}

// required float integrated_zgyro = 8;
inline bool OpticalFlow::has_integrated_zgyro() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void OpticalFlow::set_has_integrated_zgyro() {
  _has_bits_[0] |= 0x00000080u;
}
inline void OpticalFlow::clear_has_integrated_zgyro() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void OpticalFlow::clear_integrated_zgyro() {
  integrated_zgyro_ = 0;
  clear_has_integrated_zgyro();
}
inline float OpticalFlow::integrated_zgyro() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.integrated_zgyro)
  return integrated_zgyro_;
}
inline void OpticalFlow::set_integrated_zgyro(float value) {
  set_has_integrated_zgyro();
  integrated_zgyro_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.integrated_zgyro)
}

// required float temperature = 9;
inline bool OpticalFlow::has_temperature() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void OpticalFlow::set_has_temperature() {
  _has_bits_[0] |= 0x00000100u;
}
inline void OpticalFlow::clear_has_temperature() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void OpticalFlow::clear_temperature() {
  temperature_ = 0;
  clear_has_temperature();
}
inline float OpticalFlow::temperature() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.temperature)
  return temperature_;
}
inline void OpticalFlow::set_temperature(float value) {
  set_has_temperature();
  temperature_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.temperature)
}

// required int32 quality = 10;
inline bool OpticalFlow::has_quality() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void OpticalFlow::set_has_quality() {
  _has_bits_[0] |= 0x00000200u;
}
inline void OpticalFlow::clear_has_quality() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void OpticalFlow::clear_quality() {
  quality_ = 0;
  clear_has_quality();
}
inline ::google::protobuf::int32 OpticalFlow::quality() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.quality)
  return quality_;
}
inline void OpticalFlow::set_quality(::google::protobuf::int32 value) {
  set_has_quality();
  quality_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.quality)
}

// required int32 time_delta_distance_us = 11;
inline bool OpticalFlow::has_time_delta_distance_us() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void OpticalFlow::set_has_time_delta_distance_us() {
  _has_bits_[0] |= 0x00000400u;
}
inline void OpticalFlow::clear_has_time_delta_distance_us() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void OpticalFlow::clear_time_delta_distance_us() {
  time_delta_distance_us_ = 0;
  clear_has_time_delta_distance_us();
}
inline ::google::protobuf::int32 OpticalFlow::time_delta_distance_us() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.time_delta_distance_us)
  return time_delta_distance_us_;
}
inline void OpticalFlow::set_time_delta_distance_us(::google::protobuf::int32 value) {
  set_has_time_delta_distance_us();
  time_delta_distance_us_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.time_delta_distance_us)
}

// required float distance = 12;
inline bool OpticalFlow::has_distance() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void OpticalFlow::set_has_distance() {
  _has_bits_[0] |= 0x00000800u;
}
inline void OpticalFlow::clear_has_distance() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void OpticalFlow::clear_distance() {
  distance_ = 0;
  clear_has_distance();
}
inline float OpticalFlow::distance() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.OpticalFlow.distance)
  return distance_;
}
inline void OpticalFlow::set_distance(float value) {
  set_has_distance();
  distance_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.OpticalFlow.distance)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace sensor_msgs

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_OpticalFlow_2eproto__INCLUDED
