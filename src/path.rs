use std::collections::HashMap;
use wgpu::util::DeviceExt;

use crate::{
    basic::{BoundingBox, Point, PointKey, Shape},
    editor::size_to_ndc,
    vertex::Vertex,
    WindowSize,
};

impl Shape for Path {
    fn bounding_box(&self) -> BoundingBox {
        // Implement bounding box calculation for Path
        // This is a simplified version; you may need to account for curves
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for command in &self.commands {
            match command {
                PathCommand::MoveTo(point) | PathCommand::LineTo(point) => {
                    min_x = min_x.min(point.x);
                    min_y = min_y.min(point.y);
                    max_x = max_x.max(point.x);
                    max_y = max_y.max(point.y);
                }
                PathCommand::QuadraticCurveTo(control, end)
                | PathCommand::CubicCurveTo(_, control, end) => {
                    min_x = min_x.min(control.x).min(end.x);
                    min_y = min_y.min(control.y).min(end.y);
                    max_x = max_x.max(control.x).max(end.x);
                    max_y = max_y.max(control.y).max(end.y);
                }
                PathCommand::ClosePath => {}
            }
        }

        BoundingBox {
            min: Point { x: min_x, y: min_y },
            max: Point { x: max_x, y: max_y },
        }
    }

    fn contains_point(&self, point: &Point) -> bool {
        // Implement point-in-path test
        // This is a complex operation and may require a separate function
        // For now, we'll return a placeholder value
        false
    }
}

pub fn get_path_data(
    window_size: &WindowSize,
    device: &wgpu::Device,
    commands: Vec<PathCommand>,
) -> (
    Vec<Vertex>,
    Vec<u32>,
    wgpu::Buffer,
    wgpu::Buffer,
    Vec<PathCommand>,
) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut vertex_map = HashMap::new();
    let mut current_point = Point { x: 0.0, y: 0.0 };
    let mut index = 0;

    let path_layer = 2;

    for command in &commands {
        match command {
            PathCommand::MoveTo(point) => {
                current_point = *point;
            }
            PathCommand::LineTo(point) => {
                let start_index = *vertex_map
                    .entry(PointKey::from(current_point))
                    .or_insert_with(|| {
                        let (x, y) = size_to_ndc(window_size, current_point.x, current_point.y);
                        let idx = index;
                        vertices.push(Vertex::new(x, y, path_layer, [1.0, 1.0, 1.0, 1.0])); // White color
                        index += 1;
                        idx
                    });

                let end_index = *vertex_map.entry(PointKey::from(*point)).or_insert_with(|| {
                    let (x, y) = size_to_ndc(window_size, point.x, point.y);
                    let idx = index;
                    vertices.push(Vertex::new(x, y, path_layer, [1.0, 1.0, 1.0, 1.0])); // White color
                    index += 1;
                    idx
                });

                indices.push(start_index as u32);
                indices.push(end_index as u32);
                current_point = *point;
            }
            // TODO: do to_ndc convertion on the rest of these... also probably use Vertex::new
            PathCommand::QuadraticCurveTo(control, end) => {
                // Approximate quadratic curve with line segments
                let steps = 10; // Adjust for desired smoothness
                for i in 1..=steps {
                    let t = i as f32 / steps as f32;
                    let x = (1.0 - t).powi(2) * current_point.x
                        + 2.0 * (1.0 - t) * t * control.x
                        + t.powi(2) * end.x;
                    let y = (1.0 - t).powi(2) * current_point.y
                        + 2.0 * (1.0 - t) * t * control.y
                        + t.powi(2) * end.y;
                    let point = Point { x, y };

                    let point_index =
                        *vertex_map.entry(PointKey::from(point)).or_insert_with(|| {
                            let idx = index;
                            vertices.push(Vertex {
                                position: [x, y, 0.0],
                                tex_coords: [0.0, 0.0],
                                color: [1.0, 1.0, 1.0, 1.0],
                            });
                            index += 1;
                            idx
                        });

                    indices.push(point_index as u32);
                }
                current_point = *end;
            }
            PathCommand::CubicCurveTo(control1, control2, end) => {
                // Approximate cubic curve with line segments
                let steps = 20; // Adjust for desired smoothness
                for i in 1..=steps {
                    let t = i as f32 / steps as f32;
                    let x = (1.0 - t).powi(3) * current_point.x
                        + 3.0 * (1.0 - t).powi(2) * t * control1.x
                        + 3.0 * (1.0 - t) * t.powi(2) * control2.x
                        + t.powi(3) * end.x;
                    let y = (1.0 - t).powi(3) * current_point.y
                        + 3.0 * (1.0 - t).powi(2) * t * control1.y
                        + 3.0 * (1.0 - t) * t.powi(2) * control2.y
                        + t.powi(3) * end.y;
                    let point = Point { x, y };

                    let point_index =
                        *vertex_map.entry(PointKey::from(point)).or_insert_with(|| {
                            let idx = index;
                            vertices.push(Vertex {
                                position: [x, y, 0.0],
                                tex_coords: [0.0, 0.0],
                                color: [1.0, 1.0, 1.0, 1.0], //white
                            });
                            index += 1;
                            idx
                        });

                    indices.push(point_index as u32);
                }
                current_point = *end;
            }
            PathCommand::ClosePath => {
                // Add a line to the first point of the path
                if let Some(&first_index) = indices.first() {
                    indices.push(first_index);
                }
            }
        }
    }

    // let indices: [u32; 6] = [0, 1, 2, 2, 3, 0]; // square

    // TODO: may want to create per shape
    // let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
    //     layout: &renderer_state.bind_group_layout,
    //     entries: &[
    //         wgpu::BindGroupEntry {
    //             binding: 0,
    //             resource: wgpu::BindingResource::TextureView(&renderer_state.texture_view),
    //         },
    //         wgpu::BindGroupEntry {
    //             binding: 1,
    //             resource: wgpu::BindingResource::Sampler(&renderer_state.sampler),
    //         },
    //         wgpu::BindGroupEntry {
    //             binding: 2,
    //             resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
    //                 buffer: &renderer_state.render_mode_buffer,
    //                 offset: 0,
    //                 size: None,
    //             }),
    //         },
    //     ],
    //     label: Some("Primary Atlas Texture Bind Group {config.button_id}"),
    // });

    // Create a buffer for the vertices
    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Vertex Buffer"),
        contents: bytemuck::cast_slice(&vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });

    // Create a buffer for the indices
    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Index Buffer"),
        contents: bytemuck::cast_slice(&indices),
        usage: wgpu::BufferUsages::INDEX,
    });

    (vertices, indices, vertex_buffer, index_buffer, commands)
}

impl Path {
    fn new(window_size: &WindowSize, device: &wgpu::Device, commands: Vec<PathCommand>) -> Self {
        let (vertices, indices, index_buffer, vertex_buffer, commands) =
            get_path_data(window_size, device, commands);

        Path {
            commands,
            vertices,
            indices,
            index_buffer,
            vertex_buffer,
        }
    }
}

struct Path {
    commands: Vec<PathCommand>,
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
}

pub enum PathCommand {
    MoveTo(Point),
    LineTo(Point),
    QuadraticCurveTo(Point, Point),
    CubicCurveTo(Point, Point, Point),
    ClosePath,
}
