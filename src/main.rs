use bytemuck::{Pod, Zeroable};
use std::{collections::HashMap, sync::Arc};
use wgpu;
use wgpu::util::DeviceExt;

// Basic 2D point structure
struct Point {
    x: f32,
    y: f32,
}

// Basic shape traits
trait Shape {
    fn bounding_box(&self) -> BoundingBox;
    fn contains_point(&self, point: &Point) -> bool;
    // fn draw(&self, renderer: &mut CmnRenderer);
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

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Vertex {
    pub position: [f32; 3],   // x, y, z coordinates
    pub tex_coords: [f32; 2], // u, v coordinates
    // color: [f32; 3],      // RGB color
    pub color: wgpu::Color, // RGBA color
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

fn main() {
    println!("Waiting for Floem power");
}
