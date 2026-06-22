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

pub async fn task_mc_control(con: task_omni_control::Context<'static>) {
  let odom = con.shared.odometry;
  let acts = con.shared.actuators;
  let mut p_lock = (odom, acts);

  loop {
    p_lock.lock(|odom, acts| {
      // For now, just set actuators to zero
      for i in 0..4 {
        acts.actuators[i] = 0.0;
      }
    });
    Mono::delay(200.millis()).await;
  }
}

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

pub async fn task_orbital_control(con: task_omni_control::Context<'static>) {
  let odom = con.shared.odometry;
  let acts = con.shared.actuators;
  let mut p_lock = (odom, acts);
  let mut serial = con.shared.serial1;

  let mut local_odom = Odometry {
    pose: [0.0; 7],
    velocity: [0.0; 6],
  }; 

  let target_elements  = [5000000.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // target orbital elements a e i raan argp nu
  let mut current_elements = [6371000.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // current orbital elements a e
  
  // B as DMatrix, for svd
  // Allow for B and K non snake case for readability
  #[allow(non_snake_case)]
  let mut B       = na::SMatrix::<f32, 6, 3>::zeros();
  #[allow(non_snake_case)]
  let mut K       = na::SMatrix::<f32, 6, 6>::zeros();
  let mut delta_x = na::SVector::<f32,    6>::zeros();
  let mut _u       = na::SVector::<f32,    6>::zeros();
  let mut a_rtn   : na::SVector::<f32,    3>;

  loop {
    p_lock.lock(|odom, _| {
      for i in 0..6 {
        local_odom.pose[i]     = odom.pose[i];
        local_odom.velocity[i] = odom.velocity[i];

      }
    });
    current_elements[0] = local_odom.pose[0]; // a
    current_elements[1] = local_odom.pose[1]; // e
    current_elements[2] = local_odom.pose[2]; // i
    current_elements[3] = local_odom.pose[3]; // raan
    current_elements[4] = local_odom.pose[4]; // argp
    current_elements[5] = local_odom.pose[5]; // nu
    
    // Start with errors
    delta_x[0] = target_elements[0] - current_elements[0]; // a
    delta_x[1] = target_elements[1] - current_elements[1]; // e

    // Atan2 for angles to get correct wrapping
    for j in 2..6 {
      let error = target_elements[j] - current_elements[j];
      //delta_x[j] = atan2f(error.sin(), error.cos()); // wrap to [-pi, pi] 
      delta_x[j] = libm::atan2f(libm::sinf(error), libm::cosf(error)); // wrap to [-pi, pi] 
    }

    // Declare variables to work cleaner
    let a     = current_elements[0];
    let e     = current_elements[1];
    let i     = current_elements[2];
    let _raan = current_elements[3];
    let argp  = current_elements[4];
    let nu    = current_elements[5];

    let mu = 3.986004418e14;

    // Derived variables
    let p = a * (1.0 - e*e); 
    let h = libm::sqrtf(mu * p);
    let r_mag = p / (1.0 + e * libm::cosf(nu));
    let theta = argp + nu;

    let e_safe = if e.abs() < 1e-8 { 1e-8 * e.signum() } else { e };
    let sin_i = libm::sinf(i);
    let sin_i_safe = if sin_i.abs() < 1e-8 {  1e-8 * sin_i.signum() } else { sin_i };

    B[(0, 0)] = (2.0 * a * a / h) * e * libm::sinf(nu);
    B[(0, 1)] = (2.0 * a * a / h) * (p / r_mag);

    B[(1, 0)] = (p / h) * libm::sinf(nu);
    B[(1, 1)] = (1.0 / h) * ((p + r_mag) * libm::cosf(nu) + r_mag * e);

    B[(2, 2)] = (r_mag * libm::cosf(theta)) / h;
    B[(3, 2)] = (r_mag * libm::sinf(theta)) / (h * sin_i_safe);

    B[(4, 0)] = -(p / (h * e_safe)) * libm::cosf(nu);
    B[(4, 1)] = ((p + r_mag) / (h * e_safe)) * libm::sinf(nu);
    B[(4, 2)] = -(r_mag * libm::sinf(theta) * libm::cosf(i) ) / (h * sin_i_safe);

    B[(5, 0)] = (p / (h * e_safe)) * libm::cosf(nu);
    B[(5, 1)] = -((p + r_mag) / (h * e_safe)) * libm::sinf(nu);

    // Set gains on K matrix
    K[(0, 0)] = 5e-4; // a
    K[(1, 1)] = 0.1;  // e
    K[(2, 2)] = 0.1;  // i

    // WE USE TARGET ELEMENTS FOR GAIN SCHEDULING
    K[(3, 3)] = if target_elements[2].abs() < 1e-3 { 0.0 } else { 0.1 };
    K[(4, 4)] = if target_elements[1].abs() < 1e-3 { 0.0 } else { 0.5 };

    K[(5, 5)] = 0.0; // We just coast to phase

    // Compute control input
    _u = K * delta_x; 

    //let svd_option = na::SVD::try_new(B, false, false, 1e-2, 100);
    let svd_option = na::SVD::try_new(B, true, true, 1e-4, 200);
    if svd_option.is_none() {
      Mono::delay(50.millis()).await;
      continue;
    }
    let svd = svd_option.unwrap();

    // Option for a_rtn too
    let a_rtn_option = svd.solve(&_u, 1e-6);
    a_rtn = a_rtn_option.unwrap();
    

    a_rtn[0] = a_rtn[0].clamp(-10.0, 10.0);
    a_rtn[1] = a_rtn[1].clamp(-10.0, 10.0);
    a_rtn[2] = a_rtn[2].clamp(-10.0, 10.0);

    // Well publish in RTN, so that it is more akin to actual thruster commands, sim will convert to ECI
    p_lock.lock(|_, acts| {
      acts.actuators[0] = a_rtn[0]; // a_r
      acts.actuators[1] = a_rtn[1]; // a_t
      acts.actuators[2] = a_rtn[2]; // a_n
      acts.actuators[3] = 0.0;      // dummy, we only have 3 DOF control authority in this example
    });

    // Compute and print latitude and longitude from current elements, print via serial
    let lat_arg = argp + nu;
    
    // Precomputed trigonometry
    let sin_i_coords   = libm::sinf(i);
    let cos_i_coords   = libm::cosf(i);
    let sin_lat_coords = libm::sinf(lat_arg);
    let cos_lat_coords = libm::cosf(lat_arg);

    // Compute latitude
    let mut latitude = libm::asinf(sin_i_coords * sin_lat_coords);

    // Compute longitude with atan2
    let y_coords = cos_i_coords * sin_lat_coords;
    let x_coords = cos_lat_coords;
    let mut longitude = _raan + libm::atan2f(y_coords, x_coords);

    // Wrap longitude to [-pi, pi]
    longitude = longitude - 3.14159265 * 2.0 * libm::floorf((longitude + 3.14159265) / (2.0 * 3.14159265));

    // Convert to degrees for easier interpretation
    latitude = latitude * 180.0 / 3.14159265;
    longitude = longitude * 180.0 / 3.14159265;

    // Lock serial for print
    serial.lock(|serial| {
      writeln!(serial, "LAT: {:.6} | LON: {:.6}", latitude, longitude).unwrap();
    });

    Mono::delay(50.millis()).await;
  }
}
