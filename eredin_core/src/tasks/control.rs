use crate::*;
use crate::app::task_omni_control;
use eredin_types::{Odometry};

// For docs, types are
/*
pub mod eredin_types{
  pub struct Odometry {
    pub pose:     [f32; 7], // x, y, z, qw, qx, qy, qz
    pub velocity: [f32; 6], // vx, vy, vz, wx, wy, wz
  }
  pub struct Actuators {
    pub actuators: [f32;4] // t1, t2, t3, t4
  }
}
*/

pub async fn task_omni_control(con: task_omni_control::Context<'static>) {
  let odom = con.shared.odometry;
  let acts = con.shared.actuators;
  let mut p_lock = (odom, acts);

  let mut local_odom = Odometry {
    pose: [0.0; 7],
    velocity: [0.0; 6],
  }; 

  //let mut local_time: f32 = 0.0;

  let goal_x: f32 = 4.0; // target x position
  let goal_y: f32 = -4.0; // target y position

  let _l: f32 = 0.75; // half wheelbase
  let _w: f32 = 0.625; // track width
  let r: f32 = 0.3; // wheel radius

  let mut e_x: f32 = 0.0;
  let mut e_y: f32 = 0.0;

  let kp: f32 = 1.5;

  loop {
    p_lock.lock(|odom, acts| {
      for i in 0..4 {
        local_odom.pose[i]     = odom.pose[i];
        local_odom.velocity[i] = odom.velocity[i];
        //acts.actuators[i] = odom.pose[i] * 0.1; // dummy control law, just for testing

        e_x = goal_x - local_odom.pose[0];
        e_y = goal_y - local_odom.pose[1];

        acts.actuators[0] = (kp*e_x - kp*e_y)/r;
        acts.actuators[1] = (kp*e_x - kp*e_y)/r;
        acts.actuators[2] = (kp*e_x + kp*e_y)/r;
        acts.actuators[3] = (kp*e_x + kp*e_y)/r;

      }

    });

    //local_time += 0.02; // assuming control loop runs at 50Hz

    Mono::delay(20.millis()).await;
  }
}
