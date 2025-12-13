#![allow(dead_code)]

use core::f32::consts::PI;

use libm::{asinf, atan2f, sqrtf};

#[derive(Clone, Copy, Debug, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn norm(self) -> f32 {
        sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n > 0.0 {
            Self::new(self.x / n, self.y / n, self.z / n)
        } else {
            self
        }
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    pub fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }

    pub fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    pub fn scale(self, k: f32) -> Self {
        Self::new(self.x * k, self.y * k, self.z * k)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

impl Quaternion {
    pub fn normalized(self) -> Self {
        let n = sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if n > 0.0 {
            Self {
                w: self.w / n,
                x: self.x / n,
                y: self.y / n,
                z: self.z / n,
            }
        } else {
            self
        }
    }

    pub fn to_euler_deg(self) -> (f32, f32, f32) {
        let q = self.normalized();

        // aerospace sequence: roll (x), pitch (y), yaw (z)
        let sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        let cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        let roll = atan2f(sinr_cosp, cosr_cosp);

        let sinp = 2.0 * (q.w * q.y - q.z * q.x);
        let pitch = if sinp.abs() >= 1.0 {
            sinp.signum() * (PI / 2.0)
        } else {
            asinf(sinp)
        };

        let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        let yaw = atan2f(siny_cosp, cosy_cosp);

        (roll * 180.0 / PI, pitch * 180.0 / PI, yaw * 180.0 / PI)
    }
}

#[derive(Clone, Copy, Debug)]
pub enum AhrsAlgorithm {
    Madgwick,
    Mahony,
}

impl Default for AhrsAlgorithm {
    fn default() -> Self {
        Self::Madgwick
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Calibration {
    pub gyro_bias_dps: Vec3,
    pub acc_bias_g: Vec3,
    pub acc_scale: Vec3,
    pub mag_bias_u_t: Vec3,
    pub mag_scale: Vec3,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            gyro_bias_dps: Vec3::new(0.0, 0.0, 0.0),
            acc_bias_g: Vec3::new(0.0, 0.0, 0.0),
            acc_scale: Vec3::new(1.0, 1.0, 1.0),
            mag_bias_u_t: Vec3::new(0.0, 0.0, 0.0),
            mag_scale: Vec3::new(1.0, 1.0, 1.0),
        }
    }
}

impl Calibration {
    pub fn apply_gyro_dps(&self, gyro_dps: Vec3) -> Vec3 {
        gyro_dps.sub(self.gyro_bias_dps)
    }

    pub fn apply_acc_g(&self, acc_g: Vec3) -> Vec3 {
        Vec3::new(
            (acc_g.x - self.acc_bias_g.x) * self.acc_scale.x,
            (acc_g.y - self.acc_bias_g.y) * self.acc_scale.y,
            (acc_g.z - self.acc_bias_g.z) * self.acc_scale.z,
        )
    }

    pub fn apply_mag_u_t(&self, mag_u_t: Vec3) -> Vec3 {
        Vec3::new(
            (mag_u_t.x - self.mag_bias_u_t.x) * self.mag_scale.x,
            (mag_u_t.y - self.mag_bias_u_t.y) * self.mag_scale.y,
            (mag_u_t.z - self.mag_bias_u_t.z) * self.mag_scale.z,
        )
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Madgwick {
    pub q: Quaternion,
    pub beta: f32,
}

impl Default for Madgwick {
    fn default() -> Self {
        Self {
            q: Quaternion::default(),
            beta: 0.12, // 常用初值，可通过 shell 调整
        }
    }
}

impl Madgwick {
    pub fn update_imu(&mut self, gyro_rad_s: Vec3, acc_g: Vec3, dt_s: f32) {
        let mut q = self.q;

        // Normalize accelerometer
        let a = acc_g.normalized();
        if a.norm() == 0.0 {
            return;
        }

        // Auxiliary variables
        let _2q0 = 2.0 * q.w;
        let _2q1 = 2.0 * q.x;
        let _2q2 = 2.0 * q.y;
        let _2q3 = 2.0 * q.z;
        let _4q0 = 4.0 * q.w;
        let _4q1 = 4.0 * q.x;
        let _4q2 = 4.0 * q.y;
        let _8q1 = 8.0 * q.x;
        let _8q2 = 8.0 * q.y;
        let q0q0 = q.w * q.w;
        let q1q1 = q.x * q.x;
        let q2q2 = q.y * q.y;
        let q3q3 = q.z * q.z;

        // Gradient descent algorithm corrective step (6-axis)
        let s0 = _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y;
        let s1 = _4q1 * q3q3 - _2q3 * a.x + 4.0 * q0q0 * q.x - _2q0 * a.y - _4q1
            + _8q1 * q1q1
            + _8q1 * q2q2
            + _4q1 * a.z;
        let s2 = 4.0 * q0q0 * q.y + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y - _4q2
            + _8q2 * q1q1
            + _8q2 * q2q2
            + _4q2 * a.z;
        let s3 = 4.0 * q1q1 * q.z - _2q1 * a.x + 4.0 * q2q2 * q.z - _2q2 * a.y;

        let recip_norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        let (s0, s1, s2, s3) = if recip_norm > 0.0 {
            (
                s0 / recip_norm,
                s1 / recip_norm,
                s2 / recip_norm,
                s3 / recip_norm,
            )
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };

        // Compute rate of change of quaternion
        let q_dot0 =
            0.5 * (-q.x * gyro_rad_s.x - q.y * gyro_rad_s.y - q.z * gyro_rad_s.z) - self.beta * s0;
        let q_dot1 =
            0.5 * (q.w * gyro_rad_s.x + q.y * gyro_rad_s.z - q.z * gyro_rad_s.y) - self.beta * s1;
        let q_dot2 =
            0.5 * (q.w * gyro_rad_s.y - q.x * gyro_rad_s.z + q.z * gyro_rad_s.x) - self.beta * s2;
        let q_dot3 =
            0.5 * (q.w * gyro_rad_s.z + q.x * gyro_rad_s.y - q.y * gyro_rad_s.x) - self.beta * s3;

        // Integrate to yield quaternion
        q.w += q_dot0 * dt_s;
        q.x += q_dot1 * dt_s;
        q.y += q_dot2 * dt_s;
        q.z += q_dot3 * dt_s;

        self.q = q.normalized();
    }

    pub fn update_9dof(&mut self, gyro_rad_s: Vec3, acc_g: Vec3, mag_u_t: Vec3, dt_s: f32) {
        // Implementation based on Madgwick's original algorithm (with magnetometer)
        let mut q = self.q;

        let a = acc_g.normalized();
        if a.norm() == 0.0 {
            return;
        }
        let m = mag_u_t.normalized();
        if m.norm() == 0.0 {
            self.update_imu(gyro_rad_s, acc_g, dt_s);
            return;
        }

        let q0 = q.w;
        let q1 = q.x;
        let q2 = q.y;
        let q3 = q.z;

        let _2q0 = 2.0 * q0;
        let _2q1 = 2.0 * q1;
        let _2q2 = 2.0 * q2;
        let _2q3 = 2.0 * q3;
        let _2q0q2 = 2.0 * q0 * q2;
        let _2q2q3 = 2.0 * q2 * q3;
        let q0q0 = q0 * q0;
        let q0q1 = q0 * q1;
        let q0q2 = q0 * q2;
        let q0q3 = q0 * q3;
        let q1q1 = q1 * q1;
        let q1q2 = q1 * q2;
        let q1q3 = q1 * q3;
        let q2q2 = q2 * q2;
        let q2q3 = q2 * q3;
        let q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        let hx = m.x * (q0q0 + q1q1 - q2q2 - q3q3)
            + m.y * (2.0 * (q1q2 - q0q3))
            + m.z * (2.0 * (q1q3 + q0q2));
        let hy = m.x * (2.0 * (q1q2 + q0q3))
            + m.y * (q0q0 - q1q1 + q2q2 - q3q3)
            + m.z * (2.0 * (q2q3 - q0q1));
        let _2bx = sqrtf(hx * hx + hy * hy);
        let _2bz = m.x * (2.0 * (q1q3 - q0q2))
            + m.y * (2.0 * (q2q3 + q0q1))
            + m.z * (q0q0 - q1q1 - q2q2 + q3q3);
        let _4bx = 2.0 * _2bx;
        let _4bz = 2.0 * _2bz;

        // Gradient descent algorithm corrective step (9-axis)
        let s0 = -_2q2 * (2.0 * (q1q3 - q0q2) - a.x) + _2q1 * (2.0 * (q0q1 + q2q3) - a.y)
            - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x)
            + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y)
            + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);

        let s1 = _2q3 * (2.0 * (q1q3 - q0q2) - a.x) + _2q0 * (2.0 * (q0q1 + q2q3) - a.y)
            - 4.0 * q1 * (1.0 - 2.0 * (q1q1 + q2q2) - a.z)
            + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x)
            + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y)
            + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);

        let s2 = -_2q0 * (2.0 * (q1q3 - q0q2) - a.x) + _2q3 * (2.0 * (q0q1 + q2q3) - a.y)
            - 4.0 * q2 * (1.0 - 2.0 * (q1q1 + q2q2) - a.z)
            + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x)
            + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y)
            + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);

        let s3 = _2q1 * (2.0 * (q1q3 - q0q2) - a.x)
            + _2q2 * (2.0 * (q0q1 + q2q3) - a.y)
            + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x)
            + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y)
            + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);

        let recip_norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        let (s0, s1, s2, s3) = if recip_norm > 0.0 {
            (
                s0 / recip_norm,
                s1 / recip_norm,
                s2 / recip_norm,
                s3 / recip_norm,
            )
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };

        // Compute rate of change of quaternion
        let q_dot0 =
            0.5 * (-q1 * gyro_rad_s.x - q2 * gyro_rad_s.y - q3 * gyro_rad_s.z) - self.beta * s0;
        let q_dot1 =
            0.5 * (q0 * gyro_rad_s.x + q2 * gyro_rad_s.z - q3 * gyro_rad_s.y) - self.beta * s1;
        let q_dot2 =
            0.5 * (q0 * gyro_rad_s.y - q1 * gyro_rad_s.z + q3 * gyro_rad_s.x) - self.beta * s2;
        let q_dot3 =
            0.5 * (q0 * gyro_rad_s.z + q1 * gyro_rad_s.y - q2 * gyro_rad_s.x) - self.beta * s3;

        // Integrate to yield quaternion
        q.w += q_dot0 * dt_s;
        q.x += q_dot1 * dt_s;
        q.y += q_dot2 * dt_s;
        q.z += q_dot3 * dt_s;

        self.q = q.normalized();
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Mahony {
    pub q: Quaternion,
    pub kp: f32,
    pub ki: f32,
    pub integral: Vec3,
}

impl Default for Mahony {
    fn default() -> Self {
        Self {
            q: Quaternion::default(),
            kp: 1.0,
            ki: 0.0,
            integral: Vec3::new(0.0, 0.0, 0.0),
        }
    }
}

impl Mahony {
    pub fn update(&mut self, gyro_rad_s: Vec3, acc_g: Vec3, mag_u_t: Option<Vec3>, dt_s: f32) {
        // Mahony filter (basic) - uses accelerometer for gravity direction, optional magnetometer for yaw.
        // If mag not available, yaw will drift.
        let mut q = self.q.normalized();

        let a = acc_g.normalized();
        if a.norm() == 0.0 {
            return;
        }

        // Estimated gravity direction from quaternion
        let vx = 2.0 * (q.x * q.z - q.w * q.y);
        let vy = 2.0 * (q.w * q.x + q.y * q.z);
        let vz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

        let mut ex = a.y * vz - a.z * vy;
        let mut ey = a.z * vx - a.x * vz;
        let mut ez = a.x * vy - a.y * vx;

        if let Some(mv) = mag_u_t {
            let m = mv.normalized();
            if m.norm() > 0.0 {
                // Estimated magnetic field direction
                let hx = 2.0 * m.x * (0.5 - q.y * q.y - q.z * q.z)
                    + 2.0 * m.y * (q.x * q.y - q.w * q.z)
                    + 2.0 * m.z * (q.x * q.z + q.w * q.y);
                let hy = 2.0 * m.x * (q.x * q.y + q.w * q.z)
                    + 2.0 * m.y * (0.5 - q.x * q.x - q.z * q.z)
                    + 2.0 * m.z * (q.y * q.z - q.w * q.x);
                let bx = sqrtf(hx * hx + hy * hy);
                let bz = 2.0 * m.x * (q.x * q.z - q.w * q.y)
                    + 2.0 * m.y * (q.y * q.z + q.w * q.x)
                    + 2.0 * m.z * (0.5 - q.x * q.x - q.y * q.y);

                let wx =
                    2.0 * bx * (0.5 - q.y * q.y - q.z * q.z) + 2.0 * bz * (q.x * q.z - q.w * q.y);
                let wy = 2.0 * bx * (q.x * q.y - q.w * q.z) + 2.0 * bz * (q.w * q.x + q.y * q.z);
                let wz =
                    2.0 * bx * (q.w * q.y + q.x * q.z) + 2.0 * bz * (0.5 - q.x * q.x - q.y * q.y);

                ex += m.y * wz - m.z * wy;
                ey += m.z * wx - m.x * wz;
                ez += m.x * wy - m.y * wx;
            }
        }

        // Integral feedback
        if self.ki > 0.0 {
            self.integral.x += ex * self.ki * dt_s;
            self.integral.y += ey * self.ki * dt_s;
            self.integral.z += ez * self.ki * dt_s;
        } else {
            self.integral = Vec3::new(0.0, 0.0, 0.0);
        }

        // Apply feedback terms
        let gx = gyro_rad_s.x + self.kp * ex + self.integral.x;
        let gy = gyro_rad_s.y + self.kp * ey + self.integral.y;
        let gz = gyro_rad_s.z + self.kp * ez + self.integral.z;

        // Integrate quaternion rate
        let q_dot0 = 0.5 * (-q.x * gx - q.y * gy - q.z * gz);
        let q_dot1 = 0.5 * (q.w * gx + q.y * gz - q.z * gy);
        let q_dot2 = 0.5 * (q.w * gy - q.x * gz + q.z * gx);
        let q_dot3 = 0.5 * (q.w * gz + q.x * gy - q.y * gx);

        q.w += q_dot0 * dt_s;
        q.x += q_dot1 * dt_s;
        q.y += q_dot2 * dt_s;
        q.z += q_dot3 * dt_s;

        self.q = q.normalized();
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AhrsState {
    pub algo: AhrsAlgorithm,
    pub madgwick: Madgwick,
    pub mahony: Mahony,
}

impl Default for AhrsState {
    fn default() -> Self {
        Self {
            algo: AhrsAlgorithm::Madgwick,
            madgwick: Madgwick::default(),
            mahony: Mahony::default(),
        }
    }
}

impl AhrsState {
    pub fn quaternion(&self) -> Quaternion {
        match self.algo {
            AhrsAlgorithm::Madgwick => self.madgwick.q,
            AhrsAlgorithm::Mahony => self.mahony.q,
        }
    }

    pub fn set_algo(&mut self, algo: AhrsAlgorithm) {
        let q = self.quaternion();
        self.algo = algo;
        self.madgwick.q = q;
        self.mahony.q = q;
    }

    pub fn update(&mut self, gyro_dps: Vec3, acc_g: Vec3, mag_u_t: Option<Vec3>, dt_s: f32) {
        let gyro_rad_s = gyro_dps.scale(PI / 180.0);
        match self.algo {
            AhrsAlgorithm::Madgwick => {
                if let Some(m) = mag_u_t {
                    self.madgwick.update_9dof(gyro_rad_s, acc_g, m, dt_s);
                } else {
                    self.madgwick.update_imu(gyro_rad_s, acc_g, dt_s);
                }
            }
            AhrsAlgorithm::Mahony => {
                self.mahony.update(gyro_rad_s, acc_g, mag_u_t, dt_s);
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum CalMode {
    None,
    GyroBias {
        remaining: u32,
        sum: Vec3,
        count: u32,
    },
    AccMinMax {
        remaining: u32,
        min: Vec3,
        max: Vec3,
        init: bool,
    },
    MagMinMax {
        remaining: u32,
        min: Vec3,
        max: Vec3,
        init: bool,
    },
}

impl Default for CalMode {
    fn default() -> Self {
        Self::None
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CalState {
    pub mode: CalMode,
}

pub fn update_minmax(min: &mut Vec3, max: &mut Vec3, v: Vec3, init: &mut bool) {
    if !*init {
        *min = v;
        *max = v;
        *init = true;
        return;
    }
    min.x = min.x.min(v.x);
    min.y = min.y.min(v.y);
    min.z = min.z.min(v.z);
    max.x = max.x.max(v.x);
    max.y = max.y.max(v.y);
    max.z = max.z.max(v.z);
}

pub fn solve_minmax_bias_scale(min: Vec3, max: Vec3) -> (Vec3, Vec3) {
    let bias = Vec3::new(
        (max.x + min.x) * 0.5,
        (max.y + min.y) * 0.5,
        (max.z + min.z) * 0.5,
    );

    let hx = (max.x - min.x) * 0.5;
    let hy = (max.y - min.y) * 0.5;
    let hz = (max.z - min.z) * 0.5;

    let avg = (hx + hy + hz) / 3.0;

    let sx = if hx.abs() > 1e-6 { avg / hx } else { 1.0 };
    let sy = if hy.abs() > 1e-6 { avg / hy } else { 1.0 };
    let sz = if hz.abs() > 1e-6 { avg / hz } else { 1.0 };

    (bias, Vec3::new(sx, sy, sz))
}
