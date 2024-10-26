use lyon_tessellation::{
    math::Point as LyonPoint, path::Path as LyonPath, BuffersBuilder, FillOptions, LineCap,
    LineJoin, StrokeOptions, StrokeTessellator, StrokeVertex, VertexBuffers,
};
use uuid::Uuid;
use wgpu::util::DeviceExt;

use crate::{
    basic::{Point, WindowSize},
    vertex::{get_z_layer, Vertex},
};

// Brush traits and types
#[derive(Debug, Clone)]
pub enum BrushStyle {
    Solid,
    Airbrush,
    Calligraphy { angle: f32 },
    Pattern { texture_id: String },
}

#[derive(Debug, Clone)]
pub struct BrushProperties {
    pub size: f32,
    pub opacity: f32,
    pub hardness: f32, // For soft/hard edges
    pub spacing: f32,  // Distance between brush stamps
    pub style: BrushStyle,
    pub color: [f32; 4],
}

impl Default for BrushProperties {
    fn default() -> Self {
        Self {
            size: 10.0,
            opacity: 1.0,
            hardness: 0.8,
            spacing: 0.25,
            style: BrushStyle::Solid,
            color: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

pub struct BrushStroke {
    pub id: Uuid,
    pub points: Vec<Point>,
    pub properties: BrushProperties,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub vertex_buffer: Option<wgpu::Buffer>,
    pub index_buffer: Option<wgpu::Buffer>,
}

impl BrushStroke {
    pub fn new(properties: BrushProperties) -> Self {
        Self {
            id: Uuid::new_v4(),
            points: Vec::new(),
            properties,
            vertices: Vec::new(),
            indices: Vec::new(),
            vertex_buffer: None,
            index_buffer: None,
        }
    }

    pub fn add_point(&mut self, point: Point, window_size: &WindowSize, device: &wgpu::Device) {
        self.points.push(point);
        self.update_geometry(window_size, device);
    }

    fn update_geometry(&mut self, window_size: &WindowSize, device: &wgpu::Device) {
        if self.points.len() < 2 {
            return;
        }

        let mut geometry = VertexBuffers::new();
        let mut path = LyonPath::builder();

        match self.properties.style {
            BrushStyle::Solid => self.build_solid_brush_geometry(&mut path),
            BrushStyle::Airbrush => self.build_airbrush_geometry(&mut path),
            BrushStyle::Calligraphy { angle } => self.build_calligraphy_geometry(&mut path, angle),
            BrushStyle::Pattern { .. } => self.build_pattern_geometry(&mut path),
        }

        path.end(false);
        let path = path.build();

        // Create stroke options based on brush properties
        let mut stroke_options = StrokeOptions::default();
        // stroke_options.line_width = self.properties.size;
        // stroke_options.line_join = LineJoin::Round;
        // stroke_options.line_cap = LineCap::Round;
        stroke_options.with_line_width(self.properties.size);
        stroke_options.with_line_join(LineJoin::Round);
        stroke_options.with_line_cap(LineCap::Round);

        let mut stroke_tessellator = StrokeTessellator::new();

        // Tessellate the path
        stroke_tessellator
            .tessellate_path(
                &path,
                &stroke_options,
                &mut BuffersBuilder::new(&mut geometry, |vertex: StrokeVertex| {
                    let x = (vertex.position().x / window_size.width as f32) * 2.0 - 1.0;
                    let y = 1.0 - (vertex.position().y / window_size.height as f32) * 2.0;
                    let color = [
                        self.properties.color[0],
                        self.properties.color[1],
                        self.properties.color[2],
                        self.properties.color[3] * self.properties.opacity,
                    ];
                    Vertex::new(x, y, get_z_layer(4.0), color)
                }),
            )
            .unwrap();

        // Update buffers
        self.vertices = geometry.vertices;
        self.indices = geometry.indices;

        self.vertex_buffer = Some(
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Brush Stroke Vertex Buffer"),
                contents: bytemuck::cast_slice(&self.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }),
        );

        self.index_buffer = Some(
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Brush Stroke Index Buffer"),
                contents: bytemuck::cast_slice(&self.indices),
                usage: wgpu::BufferUsages::INDEX,
            }),
        );
    }

    fn build_solid_brush_geometry(&self, path: &mut lyon_tessellation::path::Builder) {
        let points = &self.points;
        if points.len() < 2 {
            return;
        }

        path.begin(point_to_lyon(&points[0]));
        for window in points.windows(2) {
            path.line_to(point_to_lyon(&window[1]));
        }
    }

    fn build_airbrush_geometry(&self, path: &mut lyon_tessellation::path::Builder) {
        for point in &self.points {
            // Create small circles at each point with varying opacity
            let center = point_to_lyon(point);
            let radius = self.properties.size * 0.5;
            // path.circle(center, radius);
            path.add_circle(center, radius, lyon_tessellation::path::Winding::Negative);
        }
    }

    fn build_calligraphy_geometry(&self, path: &mut lyon_tessellation::path::Builder, angle: f32) {
        let points = &self.points;
        if points.len() < 2 {
            return;
        }

        // Calculate perpendicular vectors for calligraphy effect
        let angle_rad = angle.to_radians();
        let (sin_a, cos_a) = angle_rad.sin_cos();
        let width = self.properties.size * 0.5;

        for window in points.windows(2) {
            let p1 = &window[0];
            let p2 = &window[1];

            // Calculate offset points for thickness
            let offset_x = width * cos_a;
            let offset_y = width * sin_a;

            // Create four points for the stroke segment
            let top1 = Point {
                x: p1.x + offset_x,
                y: p1.y + offset_y,
            };
            let bottom1 = Point {
                x: p1.x - offset_x,
                y: p1.y - offset_y,
            };
            let top2 = Point {
                x: p2.x + offset_x,
                y: p2.y + offset_y,
            };
            let bottom2 = Point {
                x: p2.x - offset_x,
                y: p2.y - offset_y,
            };

            // Draw the segment
            path.begin(point_to_lyon(&top1));
            path.line_to(point_to_lyon(&top2));
            path.line_to(point_to_lyon(&bottom2));
            path.line_to(point_to_lyon(&bottom1));
            path.close();
        }
    }

    fn build_pattern_geometry(&self, path: &mut lyon_tessellation::path::Builder) {
        // Similar to solid brush but with repeated pattern stamps
        let points = &self.points;
        if points.len() < 2 {
            return;
        }

        let spacing = self.properties.spacing * self.properties.size;
        let mut distance = 0.0;

        path.begin(point_to_lyon(&points[0]));
        for window in points.windows(2) {
            let start = &window[0];
            let end = &window[1];
            let segment_length = distance_between_points(start, end);

            while distance < segment_length {
                let t = distance / segment_length;
                let x = start.x + (end.x - start.x) * t;
                let y = start.y + (end.y - start.y) * t;

                // Add pattern stamp at (x, y)
                let stamp_point = Point { x, y };
                path.add_circle(
                    point_to_lyon(&stamp_point),
                    self.properties.size * 0.5,
                    lyon_tessellation::path::Winding::Negative,
                );

                distance += spacing;
            }

            distance -= segment_length;
        }
    }
}

// Helper function to convert our Point to Lyon's Point
fn point_to_lyon(point: &Point) -> LyonPoint {
    LyonPoint::new(point.x, point.y)
}

fn distance_between_points(p1: &Point, p2: &Point) -> f32 {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    (dx * dx + dy * dy).sqrt()
}
