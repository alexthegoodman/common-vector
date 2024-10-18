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
            self.polygons[poly_index].move_point(point_index, mouse_pos);
            let points = self.polygons[poly_index].points.clone();
            self.polygons[poly_index].update_data(window_size, device, points);
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
                        polygon.add_point(
                            edge_point.point,
                            edge_point.edge_index,
                            window_size,
                            device,
                        );
                        self.dragging_point = Some((poly_index, edge_point.edge_index + 1));
                        break;
                    }
                }
            }
        } else {
            for (poly_index, polygon) in self.polygons.iter_mut().enumerate() {
                for (point_index, point) in polygon.points.iter().enumerate() {
                    if distance(*point, mouse_pos) < 5.0 {
                        self.dragging_point = Some((poly_index, point_index));
                        // update data while dragging
                        // TODO: update polygon.points[i] with latest position
                        polygon.update_data(window_size, device, polygon.points.clone());
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
