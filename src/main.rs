use bytemuck::{Pod, Zeroable};
use std::{collections::HashMap, sync::Arc};
use uuid::Uuid;
use wgpu::util::DeviceExt;
use wgpu::{self, core::pipeline};
use winit::dpi::{PhysicalSize, Size};
use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::{Window, WindowBuilder},
};

// Basic 2D point structure
#[derive(Clone, Copy)]
struct Point {
    x: f32,
    y: f32,
}

// Basic shape traits
trait Shape {
    fn bounding_box(&self) -> BoundingBox;
    fn contains_point(&self, point: &Point) -> bool;
    fn draw(&self, renderer: &mut CmnRenderer, surface: &wgpu::Surface<'_>, viewport: &Viewport);
}

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

    // fn draw(&self, renderer: &mut CmnRenderer, surface: &wgpu::Surface<'_>) {
    //     // Implement path drawing logic
    //     // This will involve converting the path to triangles and rendering them
    //     let vertices = self.to_vertices();
    //     renderer.draw_vertices(surface, &vertices);
    // }

    fn draw(&self, renderer: &mut CmnRenderer, surface: &wgpu::Surface<'_>, viewport: &Viewport) {
        let (vertices, indices) = self.to_vertices_and_indices(viewport);
        renderer.draw_indexed(surface, &vertices, &indices);
    }
}

impl Shape for Polygon {
    fn bounding_box(&self) -> BoundingBox {
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for point in &self.points {
            min_x = min_x.min(point.x);
            min_y = min_y.min(point.y);
            max_x = max_x.max(point.x);
            max_y = max_y.max(point.y);
        }

        BoundingBox {
            min: Point { x: min_x, y: min_y },
            max: Point { x: max_x, y: max_y },
        }
    }

    fn contains_point(&self, point: &Point) -> bool {
        // Implement point-in-polygon test using the ray casting algorithm
        let mut inside = false;
        let mut j = self.points.len() - 1;
        for i in 0..self.points.len() {
            if ((self.points[i].y > point.y) != (self.points[j].y > point.y))
                && (point.x
                    < (self.points[j].x - self.points[i].x) * (point.y - self.points[i].y)
                        / (self.points[j].y - self.points[i].y)
                        + self.points[i].x)
            {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    // fn draw(&self, renderer: &mut CmnRenderer, surface: &wgpu::Surface<'_>) {
    //     // Implement polygon drawing logic
    //     // This will involve triangulating the polygon and rendering the resulting triangles
    //     let vertices = self.to_vertices();
    //     renderer.draw_vertices(surface, &vertices);
    // }

    fn draw(&self, renderer: &mut CmnRenderer, surface: &wgpu::Surface<'_>, viewport: &Viewport) {
        let (vertices, indices) = self.to_vertices_and_indices(viewport);
        renderer.draw_indexed(surface, &vertices, &indices);
    }
}

// First, let's create a wrapper struct for our Point that we can use as a key in our HashMap
#[derive(PartialEq, Eq, Hash)]
struct PointKey {
    x: i32,
    y: i32,
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

// impl Path {
//     fn to_vertices_and_indices(&self) -> (Vec<Vertex>, Vec<u32>) {
//         let mut vertices = Vec::new();
//         let mut indices = Vec::new();
//         let mut vertex_map = HashMap::new();
//         let mut current_point = Point { x: 0.0, y: 0.0 };
//         let mut index = 0;

//         for command in &self.commands {
//             match command {
//                 PathCommand::MoveTo(point) => {
//                     current_point = *point;
//                 }
//                 PathCommand::LineTo(point) => {
//                     let start_index = *vertex_map
//                         .entry(PointKey::from(current_point))
//                         .or_insert_with(|| {
//                             let idx = index;
//                             vertices.push(Vertex {
//                                 position: [current_point.x, current_point.y, 0.0],
//                                 tex_coords: [0.0, 0.0],
//                                 color: wgpu::Color::WHITE,
//                             });
//                             index += 1;
//                             idx
//                         });

//                     let end_index =
//                         *vertex_map.entry(PointKey::from(*point)).or_insert_with(|| {
//                             let idx = index;
//                             vertices.push(Vertex {
//                                 position: [point.x, point.y, 0.0],
//                                 tex_coords: [0.0, 0.0],
//                                 color: wgpu::Color::WHITE,
//                             });
//                             index += 1;
//                             idx
//                         });

//                     indices.push(start_index as u32);
//                     indices.push(end_index as u32);
//                     current_point = *point;
//                 }
//                 PathCommand::QuadraticCurveTo(control, end) => {
//                     // Approximate quadratic curve with line segments
//                     let steps = 10; // Adjust for desired smoothness
//                     for i in 1..=steps {
//                         let t = i as f32 / steps as f32;
//                         let x = (1.0 - t).powi(2) * current_point.x
//                             + 2.0 * (1.0 - t) * t * control.x
//                             + t.powi(2) * end.x;
//                         let y = (1.0 - t).powi(2) * current_point.y
//                             + 2.0 * (1.0 - t) * t * control.y
//                             + t.powi(2) * end.y;
//                         let point = Point { x, y };

//                         let point_index =
//                             *vertex_map.entry(PointKey::from(point)).or_insert_with(|| {
//                                 let idx = index;
//                                 vertices.push(Vertex {
//                                     position: [x, y, 0.0],
//                                     tex_coords: [0.0, 0.0],
//                                     color: wgpu::Color::WHITE,
//                                 });
//                                 index += 1;
//                                 idx
//                             });

//                         indices.push(point_index as u32);
//                     }
//                     current_point = *end;
//                 }
//                 PathCommand::CubicCurveTo(control1, control2, end) => {
//                     // Approximate cubic curve with line segments
//                     let steps = 20; // Adjust for desired smoothness
//                     for i in 1..=steps {
//                         let t = i as f32 / steps as f32;
//                         let x = (1.0 - t).powi(3) * current_point.x
//                             + 3.0 * (1.0 - t).powi(2) * t * control1.x
//                             + 3.0 * (1.0 - t) * t.powi(2) * control2.x
//                             + t.powi(3) * end.x;
//                         let y = (1.0 - t).powi(3) * current_point.y
//                             + 3.0 * (1.0 - t).powi(2) * t * control1.y
//                             + 3.0 * (1.0 - t) * t.powi(2) * control2.y
//                             + t.powi(3) * end.y;
//                         let point = Point { x, y };

//                         let point_index =
//                             *vertex_map.entry(PointKey::from(point)).or_insert_with(|| {
//                                 let idx = index;
//                                 vertices.push(Vertex {
//                                     position: [x, y, 0.0],
//                                     tex_coords: [0.0, 0.0],
//                                     color: wgpu::Color::WHITE,
//                                 });
//                                 index += 1;
//                                 idx
//                             });

//                         indices.push(point_index as u32);
//                     }
//                     current_point = *end;
//                 }
//                 PathCommand::ClosePath => {
//                     // Add a line to the first point of the path
//                     if let Some(&first_index) = indices.first() {
//                         indices.push(first_index);
//                     }
//                 }
//             }
//         }

//         (vertices, indices)
//     }
// }

// impl Polygon {
//     fn to_vertices_and_indices(&self) -> (Vec<Vertex>, Vec<u32>) {
//         let mut vertices = Vec::new();
//         let mut indices = Vec::new();

//         // Create vertices
//         for point in &self.points {
//             vertices.push(Vertex {
//                 position: [point.x, point.y, 0.0],
//                 tex_coords: [0.0, 0.0],
//                 color: wgpu::Color::WHITE,
//             });
//         }

//         // Triangulate the polygon using a simple fan triangulation
//         // Note: This method works correctly only for convex polygons
//         // For complex polygons, we'd need a more robust triangulation algorithm
//         if self.points.len() >= 3 {
//             for i in 1..self.points.len() - 1 {
//                 indices.push(0);
//                 indices.push(i as u32);
//                 indices.push((i + 1) as u32);
//             }
//         }

//         (vertices, indices)
//     }
// }

impl Vertex {
    fn new(x: f32, y: f32, color: [f32; 4]) -> Self {
        Vertex {
            position: [x, y, 0.0],
            tex_coords: [0.0, 0.0], // Default UV coordinates
            color,
        }
    }
}

struct Viewport {
    width: f32,
    height: f32,
}

impl Viewport {
    fn new(width: f32, height: f32) -> Self {
        Viewport { width, height }
    }

    fn to_ndc(&self, x: f32, y: f32) -> (f32, f32) {
        let ndc_x = (x / self.width) * 2.0 - 1.0;
        let ndc_y = -((y / self.height) * 2.0 - 1.0); // Flip Y-axis
        (ndc_x, ndc_y)
    }
}

impl Path {
    fn to_vertices_and_indices(&self, viewport: &Viewport) -> (Vec<Vertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut vertex_map = HashMap::new();
        let mut current_point = Point { x: 0.0, y: 0.0 };
        let mut index = 0;

        for command in &self.commands {
            match command {
                PathCommand::MoveTo(point) => {
                    current_point = *point;
                }
                PathCommand::LineTo(point) => {
                    let start_index = *vertex_map
                        .entry(PointKey::from(current_point))
                        .or_insert_with(|| {
                            let (x, y) = viewport.to_ndc(current_point.x, current_point.y);
                            let idx = index;
                            vertices.push(Vertex::new(x, y, [1.0, 1.0, 1.0, 1.0])); // White color
                            index += 1;
                            idx
                        });

                    let end_index =
                        *vertex_map.entry(PointKey::from(*point)).or_insert_with(|| {
                            let (x, y) = viewport.to_ndc(point.x, point.y);
                            let idx = index;
                            vertices.push(Vertex::new(x, y, [1.0, 1.0, 1.0, 1.0])); // White color
                            index += 1;
                            idx
                        });

                    indices.push(start_index as u32);
                    indices.push(end_index as u32);
                    current_point = *point;
                }
                // TODO: do to_ndc convertion on the rest of these...
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

        (vertices, indices)
    }
}

impl Polygon {
    fn to_vertices_and_indices(&self, viewport: &Viewport) -> (Vec<Vertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        // Create vertices
        for point in &self.points {
            let (x, y) = viewport.to_ndc(point.x, point.y);
            vertices.push(Vertex::new(x, y, [1.0, 1.0, 1.0, 1.0])); // white color
        }

        // Triangulate the polygon (assuming it's convex)
        if self.points.len() >= 3 {
            for i in 1..self.points.len() - 1 {
                indices.push(0);
                indices.push(i as u32);
                indices.push((i + 1) as u32);
            }
        }

        (vertices, indices)
    }
}
// Specific shape implementations
struct Rectangle {
    origin: Point,
    width: f32,
    height: f32,
}

struct Circle {
    center: Point,
    radius: f32,
}

struct Polygon {
    points: Vec<Point>,
}

struct Path {
    commands: Vec<PathCommand>,
}

enum PathCommand {
    MoveTo(Point),
    LineTo(Point),
    QuadraticCurveTo(Point, Point),
    CubicCurveTo(Point, Point, Point),
    ClosePath,
}

// Styling information
struct Style {
    fill_color: Option<Color>,
    stroke_color: Option<Color>,
    stroke_width: f32,
}

// Color representation
struct Color {
    r: f32,
    g: f32,
    b: f32,
    a: f32,
}

// Bounding box for quick intersection tests
struct BoundingBox {
    min: Point,
    max: Point,
}

// Scene graph node
struct Node {
    shape: Box<dyn Shape>,
    transform: Transform,
    style: Style,
    children: Vec<Node>,
}

// 2D transformation matrix
struct Transform {
    matrix: [[f32; 3]; 3],
}

// Layer for grouping and ordering shapes
struct Layer {
    nodes: Vec<Node>,
}

// Stage to hold all layers
struct Stage {
    layers: Vec<Layer>,
    width: f32,
    height: f32,
}

struct WindowSize {
    width: u32,
    height: u32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Vertex {
    pub position: [f32; 3],   // x, y, z coordinates
    pub tex_coords: [f32; 2], // u, v coordinates
    // color: [f32; 3],      // RGB color
    // pub color: wgpu::Color, // RGBA color
    pub color: [f32; 4],
}

// Ensure Vertex is Pod and Zeroable
unsafe impl Pod for Vertex {}
unsafe impl Zeroable for Vertex {}

impl Vertex {
    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0, // Corresponds to layout(location = 0) in shader
                    format: wgpu::VertexFormat::Float32x3, // x3 for position
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1, // Corresponds to layout(location = 1) in shader
                    format: wgpu::VertexFormat::Float32x2, // x2 for uv
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 5]>() as wgpu::BufferAddress,
                    shader_location: 2, // Corresponds to layout(location = 2) in shader
                    format: wgpu::VertexFormat::Float32x4, // x4 for color
                },
            ],
        }
    }
}

struct CmnRenderer {
    id: String,
    state: Option<RendererState>,
}

struct RendererState {
    // instance: wgpu::Instance,
    // surface: wgpu::Surface<'static>,
    adapter: wgpu::Adapter,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    pipeline: wgpu::RenderPipeline,
    depth_texture: wgpu::Texture,
    depth_view: wgpu::TextureView,
    // bind_group_layout: wgpu::BindGroupLayout,
}

impl CmnRenderer {
    fn new() -> Self {
        CmnRenderer {
            id: Uuid::new_v4().to_string(),
            state: None,
        }
    }

    async fn initialize(
        &mut self,
        window_size: WindowSize,
        instance: &wgpu::Instance,
        surface: &wgpu::Surface<'static>,
    ) -> () {
        // create surface outside so its easy to hook into
        let (adapter, device, queue) = self.create_adapter_and_device(&instance, &surface).await;

        let mut config = surface
            .get_default_config(&adapter, window_size.width, window_size.height)
            .unwrap();
        surface.configure(&device, &config);

        let (depth_texture, depth_view, depth_stencil_state) =
            self.create_depth_texture(&device, window_size);
        let pipeline = self.create_pipeline(surface, &device, &adapter, depth_stencil_state);

        self.state = Some(RendererState {
            // instance,
            // surface,
            adapter,
            device,
            queue,
            config,
            pipeline,
            depth_texture,
            depth_view,
            // bind_group_layout,
        });

        ()
    }

    // fn create_instance_and_surface(&self, window: &Window) -> (wgpu::Instance, wgpu::Surface) {

    //     (instance, surface)
    // }

    async fn create_adapter_and_device(
        &self,
        instance: &wgpu::Instance,
        surface: &wgpu::Surface<'_>,
    ) -> (wgpu::Adapter, wgpu::Device, wgpu::Queue) {
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .expect("Couldn't fetch GPU adapter");

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    memory_hints: wgpu::MemoryHints::default(),
                },
                None, // Trace path can be specified here for debugging purposes
            )
            .await
            .expect("Failed to create device");

        (adapter, device, queue)
    }

    pub fn create_sampler(&self, device: wgpu::Device) -> wgpu::Sampler {
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        // let sampler = Arc::new(sampler);

        sampler
    }

    pub fn create_depth_texture(
        &self,
        device: &wgpu::Device,
        window_size: WindowSize,
    ) -> (wgpu::Texture, wgpu::TextureView, wgpu::DepthStencilState) {
        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            size: wgpu::Extent3d {
                width: window_size.width,
                height: window_size.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            label: Some("Depth Texture"),
            view_formats: &[],
        });

        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        let depth_stencil_state = wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth24Plus,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::Less,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        };

        (depth_texture, depth_view, depth_stencil_state)
    }

    pub fn create_pipeline(
        &self,
        surface: &wgpu::Surface<'static>,
        device: &wgpu::Device,
        adapter: &wgpu::Adapter,
        depth_stencil_state: wgpu::DepthStencilState,
    ) -> wgpu::RenderPipeline {
        // let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        //     entries: &[
        //         wgpu::BindGroupLayoutEntry {
        //             binding: 0,
        //             visibility: wgpu::ShaderStages::FRAGMENT,
        //             ty: wgpu::BindingType::Texture {
        //                 multisampled: false,
        //                 view_dimension: wgpu::TextureViewDimension::D2,
        //                 sample_type: wgpu::TextureSampleType::Float { filterable: true },
        //             },
        //             count: None,
        //         },
        //         wgpu::BindGroupLayoutEntry {
        //             binding: 1,
        //             visibility: wgpu::ShaderStages::FRAGMENT,
        //             ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
        //             count: None,
        //         },
        //         wgpu::BindGroupLayoutEntry {
        //             binding: 2,
        //             visibility: wgpu::ShaderStages::FRAGMENT,
        //             ty: wgpu::BindingType::Buffer {
        //                 ty: wgpu::BufferBindingType::Uniform,
        //                 has_dynamic_offset: false,
        //                 min_binding_size: None,
        //             },
        //             count: None,
        //         },
        //     ],
        //     label: Some("Primary Atlas Texture Bind Group Layout"),
        // });

        // let bind_group_layout = Arc::new(bind_group_layout);

        // Define the layouts
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Pipeline Layout"),
            // bind_group_layouts: &[&bind_group_layout],
            bind_group_layouts: &[], // No bind group layouts
            push_constant_ranges: &[],
        });

        // Load the shaders
        let shader_module_vert_primary =
            device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("Primary Vert Shader"),
                source: wgpu::ShaderSource::Wgsl(include_str!("shaders/vert_primary.wgsl").into()),
            });

        let shader_module_frag_primary =
            device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("Primary Frag Shader"),
                source: wgpu::ShaderSource::Wgsl(include_str!("shaders/frag_primary.wgsl").into()),
            });

        let swapchain_capabilities = surface.get_capabilities(&adapter);
        let swapchain_format = swapchain_capabilities.formats[0]; // Choosing the first available format

        // Configure the render pipeline
        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Common Vector Primary Render Pipeline"),
            layout: Some(&pipeline_layout),
            multiview: None,
            cache: None,
            vertex: wgpu::VertexState {
                module: &shader_module_vert_primary,
                entry_point: "vs_main", // name of the entry point in your vertex shader
                buffers: &[Vertex::desc()], // Make sure your Vertex::desc() matches your vertex structure
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader_module_frag_primary,
                entry_point: "fs_main", // name of the entry point in your fragment shader
                targets: &[Some(wgpu::ColorTargetState {
                    format: swapchain_format,
                    // blend: Some(wgpu::BlendState::REPLACE),
                    blend: Some(wgpu::BlendState {
                        color: wgpu::BlendComponent {
                            src_factor: wgpu::BlendFactor::SrcAlpha,
                            dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                            operation: wgpu::BlendOperation::Add,
                        },
                        alpha: wgpu::BlendComponent {
                            src_factor: wgpu::BlendFactor::One,
                            dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                            operation: wgpu::BlendOperation::Add,
                        },
                    }),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            // primitive: wgpu::PrimitiveState::default(),
            // depth_stencil: None,
            // multisample: wgpu::MultisampleState::default(),
            primitive: wgpu::PrimitiveState {
                conservative: false,
                topology: wgpu::PrimitiveTopology::TriangleList, // how vertices are assembled into geometric primitives
                // strip_index_format: Some(wgpu::IndexFormat::Uint32),
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw, // Counter-clockwise is considered the front face
                // none cull_mode
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                // Other properties such as conservative rasterization can be set here
                unclipped_depth: false,
            },
            depth_stencil: Some(depth_stencil_state), // Optional, only if you are using depth testing
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
        });

        render_pipeline
    }

    fn draw_indexed(&mut self, surface: &wgpu::Surface<'_>, vertices: &[Vertex], indices: &[u32]) {
        let renderer_state = self.state.as_mut().expect("Couldn't get RendererState");

        println!("rendering indices {:?}", indices);

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
        let vertex_buffer =
            renderer_state
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Vertex Buffer"),
                    contents: bytemuck::cast_slice(vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });

        // Create a buffer for the indices
        let index_buffer =
            renderer_state
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Index Buffer"),
                    contents: bytemuck::cast_slice(&indices),
                    usage: wgpu::BufferUsages::INDEX,
                });

        let frame = surface
            .get_current_texture()
            .expect("Failed to acquire next swap chain texture");
        let view = frame
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        // Update the render pass to use the new vertex and index buffers
        let mut encoder = renderer_state
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: None,
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            // grey background
                            r: 0.15,
                            g: 0.15,
                            b: 0.15,
                            // white background
                            // r: 1.0,
                            // g: 1.0,
                            // b: 1.0,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                // depth_stencil_attachment: None,
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &renderer_state.depth_view, // This is the depth texture view
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0), // Clear to max depth
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None, // Set this if using stencil
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            println!("Render frame...");

            render_pass.set_pipeline(&renderer_state.pipeline);
            render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            render_pass.set_index_buffer(index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            render_pass.draw_indexed(0..indices.len() as u32, 0, 0..1);
        }

        renderer_state.queue.submit(Some(encoder.finish()));
        frame.present();
    }
}

fn main() {
    println!("Waiting for Floem power, for now, winit");

    // establish winit window and render loop (until ready to embed in Floem app)
    let window_size = WindowSize {
        width: 800,
        height: 500,
    };
    let window_size_winit = PhysicalSize::new(window_size.width, window_size.height);

    let event_loop = EventLoop::new().expect("Failed to create an event loop");
    let window = WindowBuilder::new()
        .with_title("Common Vector Demo Application")
        .with_resizable(true)
        .with_transparent(false)
        .with_inner_size(window_size_winit)
        .build(&event_loop)
        .unwrap();

    let mut renderer = CmnRenderer::new();

    // Create logical components (instance, adapter, device, queue, surface, etc.)
    let dx12_compiler = wgpu::Dx12Compiler::Dxc {
        dxil_path: None, // Specify a path to custom location
        dxc_path: None,  // Specify a path to custom location
    };

    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::PRIMARY,
        dx12_shader_compiler: dx12_compiler,
        flags: wgpu::InstanceFlags::empty(),
        gles_minor_version: wgpu::Gles3MinorVersion::Version2,
    });

    let surface = unsafe {
        instance
            .create_surface(window)
            .expect("Couldn't create GPU surface")
    };

    println!("Ready...");

    let viewport = Viewport::new(window_size.width as f32, window_size.height as f32); // Or whatever your window size is

    futures::executor::block_on(renderer.initialize(window_size, &instance, &surface));

    println!("Initialized...");

    let mut mouse_position = (0.0, 0.0);

    // test items
    let path = Path {
        commands: vec![
            PathCommand::MoveTo(Point { x: 10.0, y: 10.0 }),
            PathCommand::LineTo(Point { x: 100.0, y: 10.0 }),
            PathCommand::QuadraticCurveTo(
                Point { x: 150.0, y: 50.0 },
                Point { x: 100.0, y: 100.0 },
            ),
            PathCommand::ClosePath,
        ],
    };

    let polygon = Polygon {
        points: vec![
            Point { x: 200.0, y: 200.0 },
            Point { x: 300.0, y: 200.0 },
            Point { x: 250.0, y: 300.0 },
        ],
    };

    // let polygon = Polygon {
    //     points: vec![
    //         Point { x: 0.0, y: 0.0 },
    //         Point { x: 100.0, y: 0.0 },
    //         Point { x: 100.0, y: 100.0 },
    //         Point { x: 0.0, y: 100.0 },
    //     ],
    // };

    // execute winit render loop
    // let window = &window;
    event_loop
        .run(move |event, target| {
            if let Event::WindowEvent {
                window_id: _,
                event,
            } = event
            {
                match event {
                    WindowEvent::CursorMoved { position, .. } => {
                        // Update the mouse position
                        // println!("Mouse Position: {:?}", position);
                        mouse_position = (position.x as f64, position.y as f64);
                    }
                    WindowEvent::MouseInput {
                        state: ElementState::Pressed,
                        button: MouseButton::Left,
                        ..
                    } => {
                        // let window_size = (size.width as f64, size.height as f64);
                        // handle_click(window_size, mouse_position, &buttons, &labels);
                    }
                    WindowEvent::Resized(new_size) => {
                        // Reconfigure the surface with the new size
                        // let renderer_state =
                        //     renderer.state.as_mut().expect("Couldn't get RendererState");
                        // renderer_state.config.width = new_size.width.max(1);
                        // renderer_state.config.height = new_size.height.max(1);
                        // surface.configure(&renderer_state.device, &renderer_state.config);
                    }
                    WindowEvent::RedrawRequested => {
                        // renderer.redraw(&surface);
                        // path.draw(&mut renderer, &surface);
                        polygon.draw(&mut renderer, &surface, &viewport);
                    }
                    WindowEvent::CloseRequested => target.exit(),
                    _ => {}
                };
            }
        })
        .unwrap();
}
