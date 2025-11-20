use core::f32::consts::PI;

/// Evenly spread out particles along a strip
pub struct Particles {
    /// The spacing between each particle
    spacing: f32,
    /// The first particle at the start of the strip
    pos: f32,
}

impl Particles {
    pub fn new(spacing: f32) -> Self {
        Self { spacing, pos: 0.0 }
    }

    pub fn displace_by(&mut self, displacement: f32) {
        self.pos = fpmath::rem_euclid(self.pos + displacement, self.spacing)
    }

    /// The particles on the strip. Includes one paricle out of bounds on both ends of the strip.
    pub fn particles(&self, strip_length: f32) -> impl Iterator<Item = f32> {
        let mut curr = self.pos - self.spacing;
        core::iter::from_fn(move || {
            if curr > strip_length + self.spacing {
                return None;
            }
            let r = Some(curr);
            curr += self.spacing;
            r
        })
    }

    pub fn draw(&self, strip_length: usize) -> impl Iterator<Item = f32> {
        (0..strip_length)
            .map(|i| libm::powf(libm::sinf((i as f32 - self.pos) / self.spacing * PI), 2.0))
    }
}

/// Stolen from std
mod fpmath {
    pub fn rem_euclid(x: f32, rhs: f32) -> f32 {
        let r = x % rhs;
        if r < 0.0 { r + rhs.abs() } else { r }
    }
}
