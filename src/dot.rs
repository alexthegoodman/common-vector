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

pub fn draw_dot(
    device: &wgpu::Device,
    window_size: &WindowSize,
    point: Point,
    color: [f32; 4],
) -> (Vec<Vertex>, Vec<u32>, wgpu::Buffer, wgpu::Buffer) {
    let (x, y) = size_to_ndc(window_size, point.x, point.y);
    let dot_size = 5.0 / window_size.width.min(window_size.height) as f32; // 5 pixel dot

    let vertices = vec![
        Vertex::new(x - dot_size, y - dot_size, color),
        Vertex::new(x + dot_size, y - dot_size, color),
        Vertex::new(x + dot_size, y + dot_size, color),
        Vertex::new(x - dot_size, y + dot_size, color),
    ];

    let indices = vec![0, 1, 2, 0, 2, 3];

    // Create a buffer for the vertices
    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Dot Vertex Buffer"),
        contents: bytemuck::cast_slice(&vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });

    // Create a buffer for the indices
    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Dot Index Buffer"),
        contents: bytemuck::cast_slice(&indices),
        usage: wgpu::BufferUsages::INDEX,
    });

    // renderer.draw_indexed(surface, &vertices, &indices);

    (vertices, indices, vertex_buffer, index_buffer)
}
