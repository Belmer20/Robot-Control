// Auto-generated. Do not edit!

// (in-package jetbot_mini.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Time_step_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time_step = null;
    }
    else {
      if (initObj.hasOwnProperty('time_step')) {
        this.time_step = initObj.time_step
      }
      else {
        this.time_step = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Time_step_msg
    // Serialize message field [time_step]
    bufferOffset = _serializer.int64(obj.time_step, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Time_step_msg
    let len;
    let data = new Time_step_msg(null);
    // Deserialize message field [time_step]
    data.time_step = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jetbot_mini/Time_step_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46804f01808531bde26c16e8419d8a77';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 time_step
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Time_step_msg(null);
    if (msg.time_step !== undefined) {
      resolved.time_step = msg.time_step;
    }
    else {
      resolved.time_step = 0
    }

    return resolved;
    }
};

module.exports = Time_step_msg;
