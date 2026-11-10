#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

// =====================================================================
//  Robot controller – glue between the WebSocket protocol and the
//  motion engine, IK solver, emergency stop, camera and settings.
//  Runs entirely in the Arduino loop task.
//
//  Client -> ESP32 messages ("type" field):
//    set_joint {id, angle}          set_all {angles[6]}       home
//    ik_target {x, y, z, orient, pitch, move}
//    emergency_stop                 set_speed {value}
//    camera {framesize, quality, brightness, contrast, saturation, hmirror, vflip, led}
//    save_calibration {joints[6], model{L0..L3}}
//    set_hostname {hostname}        restart     factory_reset     get_config
//
//  ESP32 -> client messages:
//    status, config, ik_result, info, error
// =====================================================================

void controllerBegin();
void controllerLoop();

#endif // ROBOT_CONTROLLER_H
