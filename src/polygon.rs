use wgpu::util::DeviceExt;

use crate::{
    basic::{BoundingBox, Point, Shape},
    dot::{closest_point_on_line_segment, distance, EdgePoint},
    editor::size_to_ndc,
    transform::Transform,
    vertex::Vertex,
    WindowSize,
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
}

use lyon_tessellation::{
    math::Point as LyonPoint, path::Path as LyonPath, BuffersBuilder, FillOptions, FillTessellator,
    FillVertex, VertexBuffers,
};

pub fn get_polygon_data(
    window_size: &WindowSize,
    device: &wgpu::Device,
    points: Vec<Point>,
    dimensions: (f32, f32),
    transform: &Transform,
) -> (
    Vec<Vertex>,
    Vec<u32>,
    wgpu::Buffer,
    wgpu::Buffer,
    Vec<Point>,
) {
    let mut geometry: VertexBuffers<Vertex, u32> = VertexBuffers::new();
    let mut tessellator = FillTessellator::new();

    // Convert normalized points to lyon Path
    let mut builder = LyonPath::builder();
    let first_point = points[0];
    builder.begin(LyonPoint::new(
        first_point.x * dimensions.0,
        first_point.y * dimensions.1,
    ));
    for point in points.iter().skip(1) {
        builder.line_to(LyonPoint::new(
            point.x * dimensions.0,
            point.y * dimensions.1,
        ));
    }
    builder.close();
    let path = builder.build();

    // Perform tessellation
    tessellator
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
                Vertex::new(x, y, 2, [1.0, 1.0, 1.0, 1.0])
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
        points,
    )
}

impl Polygon {
    pub fn new(
        window_size: &WindowSize,
        device: &wgpu::Device,
        points: Vec<Point>,
        dimensions: (f32, f32),
        position: Point,
    ) -> Self {
        let transform = Transform::new(position);
        let (vertices, indices, vertex_buffer, index_buffer, points) =
            get_polygon_data(window_size, device, points, dimensions, &transform);

        Polygon {
            points,
            dimensions,
            transform,
            vertices,
            indices,
            vertex_buffer,
            index_buffer,
        }
    }

    pub fn update_data_from_points(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
        points: Vec<Point>,
    ) {
        let (vertices, indices, vertex_buffer, index_buffer, points) = get_polygon_data(
            window_size,
            device,
            points,
            self.dimensions,
            &self.transform,
        );

        self.points = points;
        // self.dimensions = dimensions;
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
        let (vertices, indices, vertex_buffer, index_buffer, points) = get_polygon_data(
            window_size,
            device,
            self.points.clone(),
            dimensions,
            &self.transform,
        );

        // self.points = points;
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

        let (vertices, indices, vertex_buffer, index_buffer, points) = get_polygon_data(
            window_size,
            device,
            self.points.clone(),
            self.dimensions,
            &self.transform,
        );

        self.vertices = vertices;
        self.indices = indices;
        self.vertex_buffer = vertex_buffer;
        self.index_buffer = index_buffer;
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
    pub points: Vec<Point>,
    pub dimensions: (f32, f32), // (width, height) in pixels
    pub transform: Transform,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
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
