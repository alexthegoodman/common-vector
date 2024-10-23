use cgmath::Vector2;
use wgpu::util::DeviceExt;

use crate::{
    basic::{Point, WindowSize},
    camera::{self, Camera},
    editor::size_to_ndc,
    vertex::Vertex,
};

#[derive(Clone, Copy, Debug)]
pub struct EdgePoint {
    pub point: Point,
    pub edge_index: usize,
}

pub fn closest_point_on_line_segment(start: Point, end: Point, point: Point) -> Point {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length_squared = dx * dx + dy * dy;

    if length_squared == 0.0 {
        return start;
    }

    let t = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared;
    let t = t.max(0.0).min(1.0);

    Point {
        x: start.x + t * dx,
        y: start.y + t * dy,
    }
}

// Helper function to get squared distance between two points
fn distance_squared(a: Point, b: Point) -> f32 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    dx * dx + dy * dy
}

// Helper function to get distance between two points
fn distance_sq(a: Point, b: Point) -> f32 {
    distance_squared(a, b).sqrt()
}

// pub fn closest_point_on_line_segment(start: Point, end: Point, point: Point) -> Point {
//     // Get vector from start to end
//     let line_vec = Vector2::new(end.x - start.x, end.y - start.y);

//     // Get vector from start to point
//     let point_vec = Vector2::new(point.x - start.x, point.y - start.y);

//     // Get length squared of line segment
//     let line_len_squared = line_vec.dot(line_vec);

//     // If line segment has zero length, return start point
//     if line_len_squared == 0.0 {
//         return start;
//     }

//     // Calculate projection of point onto line segment
//     let t = (point_vec.dot(line_vec) / line_len_squared).clamp(0.0, 1.0);

//     // Calculate closest point
//     Point {
//         x: start.x + t * line_vec.x,
//         y: start.y + t * line_vec.y,
//     }
// }

use cgmath::InnerSpace;

// Optional: Debug version that returns more information
#[derive(Debug)]
pub struct ClosestPointInfo {
    pub point: Point,
    pub distance: f32,
    pub normalized_t: f32, // Position along line segment (0 to 1)
    pub is_endpoint: bool, // Whether the closest point is at an endpoint
}

pub fn closest_point_on_line_segment_with_info(
    start: Point,
    end: Point,
    point: Point,
) -> ClosestPointInfo {
    let line_vec = Vector2::new(end.x - start.x, end.y - start.y);
    let point_vec = Vector2::new(point.x - start.x, point.y - start.y);

    let line_len_squared = line_vec.dot(line_vec);

    if line_len_squared == 0.0 {
        return ClosestPointInfo {
            point: start,
            distance: distance_sq(point, start),
            normalized_t: 0.0,
            is_endpoint: true,
        };
    }

    let t = (point_vec.dot(line_vec) / line_len_squared).clamp(0.0, 1.0);
    let is_endpoint = t == 0.0 || t == 1.0;

    let closest = Point {
        x: start.x + t * line_vec.x,
        y: start.y + t * line_vec.y,
    };

    ClosestPointInfo {
        point: closest,
        distance: distance_sq(point, closest),
        normalized_t: t,
        is_endpoint,
    }
}

pub fn distance(a: Point, b: Point) -> f32 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    (dx * dx + dy * dy).sqrt()
}

// draws a ring currently
pub fn draw_dot(
    device: &wgpu::Device,
    window_size: &WindowSize,
    point: Point,
    color: [f32; 4],
    camera: &Camera,
) -> (Vec<Vertex>, Vec<u32>, wgpu::Buffer, wgpu::Buffer) {
    // let (x, y) = size_to_ndc(window_size, point.x, point.y);
    // let world_point = camera.screen_to_world_ndc(Point { x, y });
    // let x = world_point.x;
    // let y = world_point.y;
    let x = point.x;
    let y = point.y;
    // println!("Draw ring... {:?} {:?}", x, y);
    let outer_radius = 9.0 / window_size.width.min(window_size.height) as f32; // 5 pixel outer radius
    let inner_radius = outer_radius * 0.7; // 70% of outer radius for inner circle
    let segments = 32 as u32; // Number of segments to approximate the circle

    let dot_layer = 1;

    let mut vertices = Vec::with_capacity((segments * 2) as usize);
    let mut indices: Vec<u32> = Vec::with_capacity((segments * 6) as usize);

    // use indices to fill space between inner and outer vertices
    for i in 0..segments {
        let i = i as u32;
        let angle = 2.0 * std::f32::consts::PI * i as f32 / segments as f32;
        let (sin, cos) = angle.sin_cos();

        // Outer vertex
        vertices.push(Vertex::new(
            x + outer_radius * cos,
            y + outer_radius * sin,
            dot_layer,
            color,
        ));

        // Inner vertex
        vertices.push(Vertex::new(
            x + inner_radius * cos,
            y + inner_radius * sin,
            dot_layer,
            color,
        ));

        let base = i * 2;
        let next_base = ((i + 1) % segments) * 2;

        // Two triangles to form a quad
        indices.extend_from_slice(&[
            base,
            base + 1,
            next_base + 1,
            base,
            next_base + 1,
            next_base,
        ]);
    }

    // println!("dot vertices {:?}", vertices);

    // Create a buffer for the vertices
    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Ring Dot Vertex Buffer"),
        contents: bytemuck::cast_slice(&vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });

    // Create a buffer for the indices
    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Ring Dot Index Buffer"),
        contents: bytemuck::cast_slice(&indices),
        usage: wgpu::BufferUsages::INDEX,
    });

    (vertices, indices, vertex_buffer, index_buffer)
}
