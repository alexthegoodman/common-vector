// Basic 2D point structure
#[derive(Clone, Copy)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

// Bounding box for quick intersection tests
pub struct BoundingBox {
    pub min: Point,
    pub max: Point,
}

// Basic shape traits
pub trait Shape {
    fn bounding_box(&self) -> BoundingBox;
    fn contains_point(&self, point: &Point) -> bool;
}

// First, let's create a wrapper struct for our Point that we can use as a key in our HashMap
#[derive(PartialEq, Eq, Hash)]
pub struct PointKey {
    pub x: i32,
    pub y: i32,
}

impl From<Point> for PointKey {
    fn from(point: Point) -> Self {
        // Convert float coordinates to fixed-point representation
        // This approach uses a precision of 3 decimal places
        PointKey {
            x: (point.x * 1000.0) as i32,
            y: (point.y * 1000.0) as i32,
        }
    }
}
