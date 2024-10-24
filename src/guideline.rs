use crate::{
    basic::{Point, WindowSize},
    vertex::{get_z_layer, Vertex},
};
use wgpu::util::DeviceExt;

const GUIDE_LINE_THICKNESS: f32 = 2.0; // Thickness in pixels

pub fn create_guide_line_buffers(
    device: &wgpu::Device,
    window_size: &WindowSize,
    start: Point,
    end: Point,
    color: [f32; 4],
) -> (Vec<Vertex>, Vec<u32>, wgpu::Buffer, wgpu::Buffer) {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length = (dx * dx + dy * dy).sqrt();
    let unit_x = dx / length;
    let unit_y = dy / length;

    // Calculate the perpendicular unit vector
    let perp_x = -unit_y;
    let perp_y = unit_x;

    // Calculate the half-thickness in NDC space
    let half_thickness = GUIDE_LINE_THICKNESS / 2.0;
    let half_thickness_ndc_x = half_thickness / window_size.width as f32;
    let half_thickness_ndc_y = half_thickness / window_size.height as f32;

    // Calculate the four corners of the rectangle
    let p1 = point_to_ndc(
        Point {
            x: start.x + perp_x * half_thickness,
            y: start.y + perp_y * half_thickness,
        },
        window_size,
    );
    let p2 = point_to_ndc(
        Point {
            x: start.x - perp_x * half_thickness,
            y: start.y - perp_y * half_thickness,
        },
        window_size,
    );
    let p3 = point_to_ndc(
        Point {
            x: end.x - perp_x * half_thickness,
            y: end.y - perp_y * half_thickness,
        },
        window_size,
    );
    let p4 = point_to_ndc(
        Point {
            x: end.x + perp_x * half_thickness,
            y: end.y + perp_y * half_thickness,
        },
        window_size,
    );

    let layer = get_z_layer(3.0);

    let vertices = vec![
        Vertex::new(p1.x, p1.y, layer, color),
        Vertex::new(p2.x, p2.y, layer, color),
        Vertex::new(p3.x, p3.y, layer, color),
        Vertex::new(p4.x, p4.y, layer, color),
    ];

    let indices = vec![0, 1, 2, 2, 3, 0];

    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Guide Line Vertex Buffer"),
        contents: bytemuck::cast_slice(&vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });

    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Guide Line Index Buffer"),
        contents: bytemuck::cast_slice(&indices),
        usage: wgpu::BufferUsages::INDEX,
    });

    (vertices, indices, vertex_buffer, index_buffer)
}

pub fn point_to_ndc(point: Point, window_size: &WindowSize) -> Point {
    let aspect_ratio = window_size.width as f32 / window_size.height as f32;

    Point {
        x: ((point.x / window_size.width as f32) * 2.0 - 1.0),
        y: 1.0 - (point.y / window_size.height as f32) * 2.0,
    }
}
