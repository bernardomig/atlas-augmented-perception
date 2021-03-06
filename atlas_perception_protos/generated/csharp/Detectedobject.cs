// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: detectedobject.proto
#pragma warning disable 1591, 0612, 3021
#region Designer generated code

using pb = global::Google.Protobuf;
using pbc = global::Google.Protobuf.Collections;
using pbr = global::Google.Protobuf.Reflection;
using scg = global::System.Collections.Generic;
namespace Atlas.Augmented {

  /// <summary>Holder for reflection information generated from detectedobject.proto</summary>
  public static partial class DetectedobjectReflection {

    #region Descriptor
    /// <summary>File descriptor for detectedobject.proto</summary>
    public static pbr::FileDescriptor Descriptor {
      get { return descriptor; }
    }
    private static pbr::FileDescriptor descriptor;

    static DetectedobjectReflection() {
      byte[] descriptorData = global::System.Convert.FromBase64String(
          string.Concat(
            "ChRkZXRlY3RlZG9iamVjdC5wcm90bxIPYXRsYXMuYXVnbWVudGVkGhFib3Vu",
            "ZGluZ2JveC5wcm90byJeCg5EZXRlY3RlZE9iamVjdBIKCgJpZBgBIAEoBBIN",
            "CgVsYWJlbBgCIAEoCRIxCgtib3VuZGluZ0JveBgDIAEoCzIcLmF0bGFzLmF1",
            "Z21lbnRlZC5Cb3VuZGluZ0JveGIGcHJvdG8z"));
      descriptor = pbr::FileDescriptor.FromGeneratedCode(descriptorData,
          new pbr::FileDescriptor[] { global::Atlas.Augmented.BoundingboxReflection.Descriptor, },
          new pbr::GeneratedClrTypeInfo(null, new pbr::GeneratedClrTypeInfo[] {
            new pbr::GeneratedClrTypeInfo(typeof(global::Atlas.Augmented.DetectedObject), global::Atlas.Augmented.DetectedObject.Parser, new[]{ "Id", "Label", "BoundingBox" }, null, null, null)
          }));
    }
    #endregion

  }
  #region Messages
  public sealed partial class DetectedObject : pb::IMessage<DetectedObject> {
    private static readonly pb::MessageParser<DetectedObject> _parser = new pb::MessageParser<DetectedObject>(() => new DetectedObject());
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public static pb::MessageParser<DetectedObject> Parser { get { return _parser; } }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public static pbr::MessageDescriptor Descriptor {
      get { return global::Atlas.Augmented.DetectedobjectReflection.Descriptor.MessageTypes[0]; }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    pbr::MessageDescriptor pb::IMessage.Descriptor {
      get { return Descriptor; }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public DetectedObject() {
      OnConstruction();
    }

    partial void OnConstruction();

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public DetectedObject(DetectedObject other) : this() {
      id_ = other.id_;
      label_ = other.label_;
      BoundingBox = other.boundingBox_ != null ? other.BoundingBox.Clone() : null;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public DetectedObject Clone() {
      return new DetectedObject(this);
    }

    /// <summary>Field number for the "id" field.</summary>
    public const int IdFieldNumber = 1;
    private ulong id_;
    /// <summary>
    /// The unique id of the detected object
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public ulong Id {
      get { return id_; }
      set {
        id_ = value;
      }
    }

    /// <summary>Field number for the "label" field.</summary>
    public const int LabelFieldNumber = 2;
    private string label_ = "";
    /// <summary>
    /// The classification label
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public string Label {
      get { return label_; }
      set {
        label_ = pb::ProtoPreconditions.CheckNotNull(value, "value");
      }
    }

    /// <summary>Field number for the "boundingBox" field.</summary>
    public const int BoundingBoxFieldNumber = 3;
    private global::Atlas.Augmented.BoundingBox boundingBox_;
    /// <summary>
    /// The tridimensional bounding box related to the object
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public global::Atlas.Augmented.BoundingBox BoundingBox {
      get { return boundingBox_; }
      set {
        boundingBox_ = value;
      }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public override bool Equals(object other) {
      return Equals(other as DetectedObject);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public bool Equals(DetectedObject other) {
      if (ReferenceEquals(other, null)) {
        return false;
      }
      if (ReferenceEquals(other, this)) {
        return true;
      }
      if (Id != other.Id) return false;
      if (Label != other.Label) return false;
      if (!object.Equals(BoundingBox, other.BoundingBox)) return false;
      return true;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public override int GetHashCode() {
      int hash = 1;
      if (Id != 0UL) hash ^= Id.GetHashCode();
      if (Label.Length != 0) hash ^= Label.GetHashCode();
      if (boundingBox_ != null) hash ^= BoundingBox.GetHashCode();
      return hash;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public override string ToString() {
      return pb::JsonFormatter.ToDiagnosticString(this);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public void WriteTo(pb::CodedOutputStream output) {
      if (Id != 0UL) {
        output.WriteRawTag(8);
        output.WriteUInt64(Id);
      }
      if (Label.Length != 0) {
        output.WriteRawTag(18);
        output.WriteString(Label);
      }
      if (boundingBox_ != null) {
        output.WriteRawTag(26);
        output.WriteMessage(BoundingBox);
      }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public int CalculateSize() {
      int size = 0;
      if (Id != 0UL) {
        size += 1 + pb::CodedOutputStream.ComputeUInt64Size(Id);
      }
      if (Label.Length != 0) {
        size += 1 + pb::CodedOutputStream.ComputeStringSize(Label);
      }
      if (boundingBox_ != null) {
        size += 1 + pb::CodedOutputStream.ComputeMessageSize(BoundingBox);
      }
      return size;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public void MergeFrom(DetectedObject other) {
      if (other == null) {
        return;
      }
      if (other.Id != 0UL) {
        Id = other.Id;
      }
      if (other.Label.Length != 0) {
        Label = other.Label;
      }
      if (other.boundingBox_ != null) {
        if (boundingBox_ == null) {
          boundingBox_ = new global::Atlas.Augmented.BoundingBox();
        }
        BoundingBox.MergeFrom(other.BoundingBox);
      }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    public void MergeFrom(pb::CodedInputStream input) {
      uint tag;
      while ((tag = input.ReadTag()) != 0) {
        switch(tag) {
          default:
            input.SkipLastField();
            break;
          case 8: {
            Id = input.ReadUInt64();
            break;
          }
          case 18: {
            Label = input.ReadString();
            break;
          }
          case 26: {
            if (boundingBox_ == null) {
              boundingBox_ = new global::Atlas.Augmented.BoundingBox();
            }
            input.ReadMessage(boundingBox_);
            break;
          }
        }
      }
    }

  }

  #endregion

}

#endregion Designer generated code
