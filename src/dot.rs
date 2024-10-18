use wgpu::util::DeviceExt;

use crate::{basic::Point, editor::size_to_ndc, vertex::Vertex, WindowSize};

#[derive(Clone, Copy)]
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
) -> (Vec<Vertex>, Vec<u32>, wgpu::Buffer, wgpu::Buffer) {
    let (x, y) = size_to_ndc(window_size, point.x, point.y);
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
