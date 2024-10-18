use crate::basic::Point;

pub struct Transform {
    pub position: Point,
    // We could add scale and rotation here in the future if needed
}

impl Transform {
    pub fn new(position: Point) -> Self {
        Self { position }
    }
}
