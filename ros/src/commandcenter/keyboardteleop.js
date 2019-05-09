/**
 * @author Russell Toris - rctoris@wpi.edu
 */

var KEYBOARDTELEOP = KEYBOARDTELEOP || {
  REVISION : '0.3.0'
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * Manages connection to the server and all interactions with ROS.
 *
 * Emits the following events:
 *   * 'change' - emitted with a change in speed occurs
 *
 * @constructor
 * @param options - possible keys include:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the Twist topic to publish to, like '/cmd_vel'
 *   * throttle (optional) - a constant throttle for the speed
 */
KEYBOARDTELEOP.Teleop = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/cmd_vel';
  var throttle = options.throttle || 1.0;

  // used to externally throttle the speed (e.g., from a slider)
  this.drive_scale = 22;
  this.dig_scale = 20;
  this.dump_scale = 30;
  this.enabled = false;
  this.digging = false;

  // linear x and y movement and angular z movement
  var x = 0;
  var y = 0;
  var z = 0;
  var d_x = 0;
  var d_y = 0;

  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });
  var digVel = new ROSLIB.Topic({
    ros : ros,
    name : '/digging/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });

  this.startDigging = function() {
	handleKey('i',true);
  }

  // sets up a key listener on the page used for keyboard teleoperation
  var handleKey = function(keyCode, keyDown) {
    // used to check for changes in speed
    var oldX = x;
    var oldY = y;
    var oldZ = z;
    var olddX = d_x;
    var olddY = d_y;
    
    var l_pub = false;
    var d_pub = false;

    var speed = 0;
    var dig_speed = 0;
    var dump_speed = 0;
    // throttle the speed by the slider and throttle constant
    
    if (keyDown === true && that.enabled === true) {
      speed = throttle * that.drive_scale;
      dig_speed = throttle * that.dig_scale;
      dump_speed = throttle * that.dump_scale;
    }
        // check which key was pressed
        switch (keyCode) {
          case 65:
            // turn left
            z = -1 * speed;
            l_pub = true;
            break;
          case 87:
            // up
            x = -1 * speed;
            l_pub = true;
            break;
          case 68:
            // turn right
            z = 1 * speed;
            l_pub = true;
            break;
          case 83:
            // down
            x = 1 * speed;
            l_pub = true;
            break;
          case 73:
            // I dig
            d_x = -1 * dig_speed;
            d_pub = true;
            break;
          case 75:
            // k dig backwards "kick"
            d_x = 1 * dig_speed;
            d_pub = true;
            break;
          case 80:
            // p dump
            d_y = -1 * dump_speed;
            d_pub = true;
            break;
          case 59:
            // dump backwards
            d_y = 1 * dump_speed;
            d_pub = true;
            break;
          default:
            l_pub = false;
            d_pub = false;
        }
    
    if (that.digging === true) {
        console.log("digging");
        l_pub = false;
        d_pub = true;
        dig_speed = throttle * that.dig_scale;
        d_x = -1 * dig_speed;
    }

    // publish the command
    if (l_pub === true) {
      var l_twist = new ROSLIB.Message({
        angular : {
          x : 0,
          y : 0,
          z : z
        },
        linear : {
          x : x,
          y : y,
          z : 0
        }
      });
      cmdVel.publish(l_twist);
      // check for changes
      if (oldX !== x || oldY !== y || oldZ !== z) {
        that.emit('change', l_twist);
      }
    }
    if (d_pub === true) {
      var d_twist = new ROSLIB.Message({
        angular : {
          x : 0,
          y : 0,
          z : 0
        },
        linear : {
          x : d_x,
          y : d_y,
          z : 0
        }
      });
      digVel.publish(d_twist);
      // check for changes
      if (olddX !== x || olddY !== y) {
        that.emit('change', d_twist);
      }
    }
  };

  // handle the key
  var body = document.getElementsByTagName('body')[0];
  body.addEventListener('keydown', function(e) {
    handleKey(e.keyCode, true);
  }, false);
  
  body.addEventListener('keyup', function(e) {
    handleKey(e.keyCode, false);
  }, false);
};
KEYBOARDTELEOP.Teleop.prototype.__proto__ = EventEmitter2.prototype;



