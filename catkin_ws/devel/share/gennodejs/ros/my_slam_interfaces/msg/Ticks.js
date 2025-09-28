// Auto-generated. Do not edit!

// (in-package my_slam_interfaces.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Ticks {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left_ticks = null;
      this.right_ticks = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('left_ticks')) {
        this.left_ticks = initObj.left_ticks
      }
      else {
        this.left_ticks = 0;
      }
      if (initObj.hasOwnProperty('right_ticks')) {
        this.right_ticks = initObj.right_ticks
      }
      else {
        this.right_ticks = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Ticks
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [left_ticks]
    bufferOffset = _serializer.int64(obj.left_ticks, buffer, bufferOffset);
    // Serialize message field [right_ticks]
    bufferOffset = _serializer.int64(obj.right_ticks, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Ticks
    let len;
    let data = new Ticks(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_ticks]
    data.left_ticks = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [right_ticks]
    data.right_ticks = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_slam_interfaces/Ticks';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b2e3ba80aef7601eec29b4f6edd523f2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    int64 left_ticks
    int64 right_ticks
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Ticks(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.left_ticks !== undefined) {
      resolved.left_ticks = msg.left_ticks;
    }
    else {
      resolved.left_ticks = 0
    }

    if (msg.right_ticks !== undefined) {
      resolved.right_ticks = msg.right_ticks;
    }
    else {
      resolved.right_ticks = 0
    }

    return resolved;
    }
};

module.exports = Ticks;
