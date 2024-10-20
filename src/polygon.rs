use uuid::Uuid;
use wgpu::util::DeviceExt;

use crate::{
    basic::WindowSize,
    basic::{BoundingBox, Point, Shape},
    dot::{closest_point_on_line_segment, distance, EdgePoint},
    editor::size_to_ndc,
    transform::Transform,
    vertex::Vertex,
};

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
        // Convert the point to the polygon's local coordinate system
        let local_point = Point {
            x: (point.x - self.transform.position.x) / self.dimensions.0,
            y: (point.y - self.transform.position.y) / self.dimensions.1,
        };

        // Implement point-in-polygon test using the ray casting algorithm
        let mut inside = false;
        let mut j = self.points.len() - 1;
        for i in 0..self.points.len() {
            let pi = &self.points[i];
            let pj = &self.points[j];

            if ((pi.y > local_point.y) != (pj.y > local_point.y))
                && (local_point.x < (pj.x - pi.x) * (local_point.y - pi.y) / (pj.y - pi.y) + pi.x)
            {
                inside = !inside;
            }
            j = i;
        }
        inside
    }
}

use lyon_tessellation::{
    geom::CubicBezierSegment, math::Point as LyonPoint, path::Path as LyonPath, BuffersBuilder,
    FillOptions, FillTessellator, FillVertex, StrokeOptions, StrokeTessellator, StrokeVertex,
    VertexBuffers,
};

pub fn get_polygon_data(
    window_size: &WindowSize,
    device: &wgpu::Device,
    points: Vec<Point>,
    dimensions: (f32, f32),
    transform: &Transform,
    border_radius: f32,
    fill: [f32; 4],
) -> (
    Vec<Vertex>,
    Vec<u32>,
    wgpu::Buffer,
    wgpu::Buffer,
    // Vec<Point>,
) {
    let mut geometry: VertexBuffers<Vertex, u32> = VertexBuffers::new();
    let mut fill_tessellator = FillTessellator::new();
    let mut stroke_tessellator = StrokeTessellator::new();

    let path = create_rounded_polygon_path(points, dimensions, border_radius);

    // Fill the polygon
    fill_tessellator
        .tessellate_path(
            &path,
            &FillOptions::default(),
            &mut BuffersBuilder::new(&mut geometry, |vertex: FillVertex| {
                let x = ((vertex.position().x + transform.position.x) / window_size.width as f32)
                    * 2.0
                    - 1.0;
                let y = 1.0
                    - ((vertex.position().y + transform.position.y) / window_size.height as f32)
                        * 2.0;
                Vertex::new(x, y, 3, fill)
            }),
        )
        .unwrap();

    // Stroke the polygon (optional, for a border effect)
    stroke_tessellator
        .tessellate_path(
            &path,
            &StrokeOptions::default().with_line_width(2.0),
            &mut BuffersBuilder::new(&mut geometry, |vertex: StrokeVertex| {
                let x = ((vertex.position().x + transform.position.x) / window_size.width as f32)
                    * 2.0
                    - 1.0;
                let y = 1.0
                    - ((vertex.position().y + transform.position.y) / window_size.height as f32)
                        * 2.0;
                Vertex::new(x, y, 2, [0.0, 0.0, 0.0, 1.0]) // Black border
            }),
        )
        .unwrap();

    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Vertex Buffer"),
        contents: bytemuck::cast_slice(&geometry.vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });

    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Index Buffer"),
        contents: bytemuck::cast_slice(&geometry.indices),
        usage: wgpu::BufferUsages::INDEX,
    });

    (
        geometry.vertices,
        geometry.indices,
        vertex_buffer,
        index_buffer,
        // points,
    )
}

use lyon_tessellation::math::point;
use lyon_tessellation::math::Vector;

fn create_rounded_polygon_path(
    normalized_points: Vec<Point>,
    dimensions: (f32, f32),
    border_radius: f32,
) -> LyonPath {
    let mut builder = LyonPath::builder();
    let n = normalized_points.len();

    // Scale border radius to match dimensions
    let scaled_radius = border_radius / dimensions.0.min(dimensions.1);

    for i in 0..n {
        let p0 = normalized_points[(i + n - 1) % n];
        let p1 = normalized_points[i];
        let p2 = normalized_points[(i + 1) % n];

        let v1 = Vector::new(p1.x - p0.x, p1.y - p0.y);
        let v2 = Vector::new(p2.x - p1.x, p2.y - p1.y);

        let len1 = (v1.x * v1.x + v1.y * v1.y).sqrt();
        let len2 = (v2.x * v2.x + v2.y * v2.y).sqrt();

        let radius = scaled_radius.min(len1 / 2.0).min(len2 / 2.0);

        let offset1 = Vector::new(v1.x / len1 * radius, v1.y / len1 * radius);
        let offset2 = Vector::new(v2.x / len2 * radius, v2.y / len2 * radius);

        let p1_scaled = LyonPoint::new(p1.x * dimensions.0, p1.y * dimensions.1);

        let corner_start = point(
            p1_scaled.x - offset1.x * dimensions.0,
            p1_scaled.y - offset1.y * dimensions.1,
        );
        let corner_end = point(
            p1_scaled.x + offset2.x * dimensions.0,
            p1_scaled.y + offset2.y * dimensions.1,
        );

        if i == 0 {
            builder.begin(corner_start);
        }

        let control1 = p1_scaled;
        let control2 = p1_scaled;

        let bezier = CubicBezierSegment {
            from: corner_start,
            ctrl1: control1,
            ctrl2: control2,
            to: corner_end,
        };

        builder.cubic_bezier_to(bezier.ctrl1, bezier.ctrl2, bezier.to);
    }

    builder.close();
    builder.build()
}

impl Polygon {
    pub fn new(
        window_size: &WindowSize,
        device: &wgpu::Device,
        points: Vec<Point>,
        dimensions: (f32, f32),
        position: Point,
        border_radius: f32,
        fill: [f32; 4],
    ) -> Self {
        let transform = Transform::new(position);
        let (vertices, indices, vertex_buffer, index_buffer) = get_polygon_data(
            window_size,
            device,
            points.clone(),
            dimensions,
            &transform,
            border_radius,
            fill,
        );
        let id = Uuid::new_v4();

        Polygon {
            id,
            points,
            dimensions,
            transform,
            border_radius,
            fill,
            vertices,
            indices,
            vertex_buffer,
            index_buffer,
        }
    }

    pub fn update_data_from_window_size(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        let (vertices, indices, vertex_buffer, index_buffer) = get_polygon_data(
            window_size,
            device,
            self.points.clone(),
            self.dimensions,
            &self.transform,
            self.border_radius,
            self.fill,
        );

        self.vertices = vertices;
        self.indices = indices;
        self.vertex_buffer = vertex_buffer;
        self.index_buffer = index_buffer;
    }

    pub fn update_data_from_points(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
        points: Vec<Point>,
    ) {
        let (vertices, indices, vertex_buffer, index_buffer) = get_polygon_data(
            window_size,
            device,
            points.clone(),
            self.dimensions,
            &self.transform,
            self.border_radius,
            self.fill,
        );

        self.points = points;
        self.vertices = vertices;
        self.indices = indices;
        self.vertex_buffer = vertex_buffer;
        self.index_buffer = index_buffer;
    }

    pub fn update_data_from_dimensions(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
        dimensions: (f32, f32),
    ) {
        let (vertices, indices, vertex_buffer, index_buffer) = get_polygon_data(
            window_size,
            device,
            self.points.clone(),
            dimensions,
            &self.transform,
            self.border_radius,
            self.fill,
        );

        self.dimensions = dimensions;
        self.vertices = vertices;
        self.indices = indices;
        self.vertex_buffer = vertex_buffer;
        self.index_buffer = index_buffer;
    }

    pub fn update_data_from_position(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
        position: Point,
    ) {
        self.transform.position = position;

        let (vertices, indices, vertex_buffer, index_buffer) = get_polygon_data(
            window_size,
            device,
            self.points.clone(),
            self.dimensions,
            &self.transform,
            self.border_radius,
            self.fill,
        );

        self.vertices = vertices;
        self.indices = indices;
        self.vertex_buffer = vertex_buffer;
        self.index_buffer = index_buffer;
    }

    pub fn update_data_from_border_radius(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
        border_radius: f32,
    ) {
        let (vertices, indices, vertex_buffer, index_buffer) = get_polygon_data(
            window_size,
            device,
            self.points.clone(),
            self.dimensions,
            &self.transform,
            border_radius,
            self.fill,
        );

        self.border_radius = border_radius;
        self.vertices = vertices;
        self.indices = indices;
        self.vertex_buffer = vertex_buffer;
        self.index_buffer = index_buffer;
    }

    pub fn update_data_from_fill(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
        fill: [f32; 4],
    ) {
        let (vertices, indices, vertex_buffer, index_buffer) = get_polygon_data(
            window_size,
            device,
            self.points.clone(),
            self.dimensions,
            &self.transform,
            self.border_radius,
            fill,
        );

        self.fill = fill;
        self.vertices = vertices;
        self.indices = indices;
        self.vertex_buffer = vertex_buffer;
        self.index_buffer = index_buffer;
    }

    pub fn world_bounding_box(&self) -> BoundingBox {
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for point in &self.points {
            let world_x = point.x * self.dimensions.0 + self.transform.position.x;
            let world_y = point.y * self.dimensions.1 + self.transform.position.y;
            min_x = min_x.min(world_x);
            min_y = min_y.min(world_y);
            max_x = max_x.max(world_x);
            max_y = max_y.max(world_y);
        }

        BoundingBox {
            min: Point { x: min_x, y: min_y },
            max: Point { x: max_x, y: max_y },
        }
    }

    pub fn closest_point_on_edge(&self, mouse_pos: Point) -> Option<EdgePoint> {
        let mut closest_point = None;
        let mut min_distance = f32::MAX;

        // Convert mouse_pos to normalized coordinates
        let normalized_mouse_pos = Point {
            x: (mouse_pos.x - self.transform.position.x) / self.dimensions.0,
            y: (mouse_pos.y - self.transform.position.y) / self.dimensions.1,
        };

        for i in 0..self.points.len() {
            let start = self.points[i];
            let end = self.points[(i + 1) % self.points.len()];

            let point = closest_point_on_line_segment(start, end, normalized_mouse_pos);
            let distance = distance(point, normalized_mouse_pos);

            if distance < min_distance {
                min_distance = distance;
                closest_point = Some(EdgePoint {
                    point: Point {
                        x: point.x * self.dimensions.0 + self.transform.position.x,
                        y: point.y * self.dimensions.1 + self.transform.position.y,
                    },
                    edge_index: i,
                });
            }
        }

        // Convert the distance threshold to normalized space
        let normalized_threshold = 5.0 / self.dimensions.0.min(self.dimensions.1);

        if min_distance < normalized_threshold {
            closest_point
        } else {
            None
        }
    }

    pub fn add_point(
        &mut self,
        new_point: Point,
        edge_index: usize,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        println!("Add point");
        self.points.insert(edge_index + 1, new_point);
        self.update_data_from_points(window_size, device, self.points.clone());
    }

    pub fn move_point(&mut self, point_index: usize, new_position: Point) {
        if point_index < self.points.len() {
            self.points[point_index] = new_position;
        }
    }
}

pub struct Polygon {
    pub id: Uuid,
    pub points: Vec<Point>,
    pub dimensions: (f32, f32), // (width, height) in pixels
    pub fill: [f32; 4],
    pub transform: Transform,
    pub border_radius: f32,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
}

pub struct PolygonConfig {
    pub points: Vec<Point>,
    pub fill: [f32; 4],
    pub dimensions: (f32, f32), // (width, height) in pixels
    pub position: Point,
    pub border_radius: f32,
}

// Specific shape implementations
// pub struct Rectangle {
//     origin: Point,
//     width: f32,
//     height: f32,
// }

// pub struct Circle {
//     center: Point,
//     radius: f32,
// }
