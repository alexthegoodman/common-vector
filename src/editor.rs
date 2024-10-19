use crate::basic::Shape;
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

pub struct GuideLine {
    pub start: Point,
    pub end: Point,
}

pub struct Editor {
    pub polygons: Vec<Polygon>,
    pub hover_point: Option<EdgePoint>,
    pub dragging_point: Option<(usize, usize)>, // (polygon_index, point_index)
    pub dragging_polygon: Option<usize>,
    pub guide_lines: Vec<GuideLine>,
    pub viewport: Viewport,
    pub drag_start: Option<Point>,
}

impl Editor {
    pub fn new(viewport: Viewport) -> Self {
        Editor {
            polygons: Vec::new(),
            hover_point: None,
            dragging_point: None,
            dragging_polygon: None,
            guide_lines: Vec::new(),
            viewport,
            drag_start: None,
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
                        return;
                    }
                }
            }
        }

        // Check if we're clicking on a polygon to drag
        for (poly_index, polygon) in self.polygons.iter().enumerate() {
            if polygon.contains_point(&mouse_pos) {
                self.dragging_polygon = Some(poly_index);
                self.drag_start = Some(mouse_pos);
                return;
            }
        }

        // If we haven't started dragging a polygon, check for point dragging
        for (poly_index, polygon) in self.polygons.iter().enumerate() {
            for (point_index, normalized_point) in polygon.points.iter().enumerate() {
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

    pub fn handle_mouse_move(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
        x: f32,
        y: f32,
    ) {
        let mouse_pos = Point { x, y };
        self.hover_point = None;
        self.guide_lines.clear();

        if let Some((poly_index, point_index)) = self.dragging_point {
            let polygon = &mut self.polygons[poly_index];
            let normalized_pos = Point {
                x: (mouse_pos.x - polygon.transform.position.x) / polygon.dimensions.0,
                y: (mouse_pos.y - polygon.transform.position.y) / polygon.dimensions.1,
            };
            polygon.move_point(point_index, normalized_pos);
            polygon.update_data_from_points(window_size, device, polygon.points.clone());
            self.update_guide_lines(poly_index, window_size);
        } else if let Some(poly_index) = self.dragging_polygon {
            if let Some(start) = self.drag_start {
                let dx = mouse_pos.x - start.x;
                let dy = mouse_pos.y - start.y;
                let polygon = &mut self.polygons[poly_index];
                let new_position = Point {
                    x: polygon.transform.position.x + dx,
                    y: polygon.transform.position.y + dy,
                };
                polygon.update_data_from_position(window_size, device, new_position);
                self.drag_start = Some(mouse_pos);
                self.update_guide_lines(poly_index, window_size);
            }
        } else {
            for polygon in &self.polygons {
                if let Some(edge_point) = polygon.closest_point_on_edge(mouse_pos) {
                    self.hover_point = Some(edge_point);
                    break;
                }
            }
        }
    }

    pub fn handle_mouse_up(&mut self) {
        self.dragging_point = None;
        self.dragging_polygon = None;
        self.drag_start = None;
        self.guide_lines.clear();
    }

    fn update_guide_lines(&mut self, dragged_poly_index: usize, window_size: &WindowSize) {
        let dragged_poly = &self.polygons[dragged_poly_index];
        let dragged_bbox = dragged_poly.world_bounding_box();

        for (index, other_poly) in self.polygons.iter().enumerate() {
            if index == dragged_poly_index {
                continue;
            }

            let other_bbox = other_poly.world_bounding_box();

            // Check for vertical alignment
            if self.is_close(dragged_bbox.min.x, other_bbox.min.x, 5.0) {
                self.guide_lines.push(GuideLine {
                    start: Point {
                        x: other_bbox.min.x,
                        y: 0.0,
                    },
                    end: Point {
                        x: other_bbox.min.x,
                        y: window_size.height as f32,
                    },
                });
                println!("Vertical guide at x={}", other_bbox.min.x);
            }
            if self.is_close(dragged_bbox.max.x, other_bbox.max.x, 5.0) {
                self.guide_lines.push(GuideLine {
                    start: Point {
                        x: other_bbox.max.x,
                        y: 0.0,
                    },
                    end: Point {
                        x: other_bbox.max.x,
                        y: window_size.height as f32,
                    },
                });
                println!("Vertical guide at x={}", other_bbox.max.x);
            }

            // Check for horizontal alignment
            if self.is_close(dragged_bbox.min.y, other_bbox.min.y, 5.0) {
                self.guide_lines.push(GuideLine {
                    start: Point {
                        x: 0.0,
                        y: other_bbox.min.y,
                    },
                    end: Point {
                        x: window_size.width as f32,
                        y: other_bbox.min.y,
                    },
                });
                println!("Horizontal guide at y={}", other_bbox.min.y);
            }
            if self.is_close(dragged_bbox.max.y, other_bbox.max.y, 5.0) {
                self.guide_lines.push(GuideLine {
                    start: Point {
                        x: 0.0,
                        y: other_bbox.max.y,
                    },
                    end: Point {
                        x: window_size.width as f32,
                        y: other_bbox.max.y,
                    },
                });
                println!("Horizontal guide at y={}", other_bbox.max.y);
            }
        }
    }

    fn is_close(&self, a: f32, b: f32, threshold: f32) -> bool {
        (a - b).abs() < threshold
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
