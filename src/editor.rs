use crate::{
    basic::Point,
    dot::{distance, EdgePoint},
    polygon::Polygon,
    WindowSize,
};

pub struct Viewport {
    pub width: f32,
    pub height: f32,
}

impl Viewport {
    pub fn new(width: f32, height: f32) -> Self {
        Viewport { width, height }
    }

    pub fn to_ndc(&self, x: f32, y: f32) -> (f32, f32) {
        let ndc_x = (x / self.width) * 2.0 - 1.0;
        let ndc_y = -((y / self.height) * 2.0 - 1.0); // Flip Y-axis
        (ndc_x, ndc_y)
    }
}

pub fn size_to_ndc(window_size: &WindowSize, x: f32, y: f32) -> (f32, f32) {
    let ndc_x = (x / window_size.width as f32) * 2.0 - 1.0;
    let ndc_y = -((y / window_size.height as f32) * 2.0 - 1.0); // Flip Y-axis
    (ndc_x, ndc_y)
}

pub struct Editor {
    pub polygons: Vec<Polygon>,
    pub hover_point: Option<EdgePoint>,
    pub dragging_point: Option<(usize, usize)>, // (polygon_index, point_index)
    pub viewport: Viewport,
}

impl Editor {
    pub fn new(viewport: Viewport) -> Self {
        Editor {
            polygons: Vec::new(),
            hover_point: None,
            dragging_point: None,
            viewport,
        }
    }

    pub fn handle_mouse_move(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
        x: f32,
        y: f32,
    ) {
        let mouse_pos = Point { x, y };
        self.hover_point = None;

        if let Some((poly_index, point_index)) = self.dragging_point {
            let polygon = &mut self.polygons[poly_index];
            let normalized_pos = Point {
                x: (mouse_pos.x - polygon.transform.position.x) / polygon.dimensions.0,
                y: (mouse_pos.y - polygon.transform.position.y) / polygon.dimensions.1,
            };
            polygon.move_point(point_index, normalized_pos);
            polygon.update_data_from_points(window_size, device, polygon.points.clone());
        } else {
            for polygon in &self.polygons {
                if let Some(edge_point) = polygon.closest_point_on_edge(mouse_pos) {
                    self.hover_point = Some(edge_point);
                    break;
                }
            }
        }
    }

    pub fn handle_mouse_down(
        &mut self,
        x: f32,
        y: f32,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        let mouse_pos = Point { x, y };

        if let Some(hover_point) = self.hover_point {
            for (poly_index, polygon) in self.polygons.iter_mut().enumerate() {
                if let Some(edge_point) = polygon.closest_point_on_edge(mouse_pos) {
                    if (edge_point.point.x - hover_point.point.x).abs() < 1.0
                        && (edge_point.point.y - hover_point.point.y).abs() < 1.0
                    {
                        let normalized_point = Point {
                            x: (edge_point.point.x - polygon.transform.position.x)
                                / polygon.dimensions.0,
                            y: (edge_point.point.y - polygon.transform.position.y)
                                / polygon.dimensions.1,
                        };
                        polygon.add_point(
                            normalized_point,
                            edge_point.edge_index,
                            window_size,
                            device,
                        );
                        polygon.update_data_from_points(
                            window_size,
                            device,
                            polygon.points.clone(),
                        );
                        self.dragging_point = Some((poly_index, edge_point.edge_index + 1));
                        break;
                    }
                }
            }
        } else {
            for (poly_index, polygon) in self.polygons.iter_mut().enumerate() {
                for (point_index, &normalized_point) in polygon.points.iter().enumerate() {
                    let world_point = Point {
                        x: normalized_point.x * polygon.dimensions.0 + polygon.transform.position.x,
                        y: normalized_point.y * polygon.dimensions.1 + polygon.transform.position.y,
                    };
                    if distance(world_point, mouse_pos) < 5.0 {
                        self.dragging_point = Some((poly_index, point_index));
                        return;
                    }
                }
            }
        }
    }

    pub fn handle_mouse_up(&mut self) {
        self.dragging_point = None;
    }

    // fn draw(&self, renderer: &mut CmnRenderer, surface: &wgpu::Surface, device: &wgpu::Device) {
    //     for polygon in &self.polygons {
    //         polygon.draw(renderer, surface, &self.viewport, device);
    //     }

    //     if let Some(edge_point) = self.hover_point {
    //         draw_dot(
    //             surface,
    //             renderer,
    //             &self.viewport,
    //             edge_point.point,
    //             [0.0, 1.0, 0.0, 1.0],
    //         ); // Green dot
    //     }
    // }
}
