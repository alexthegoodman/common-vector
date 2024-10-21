use std::cell::RefCell;
use std::fmt::Display;
use std::sync::{Arc, Mutex, MutexGuard};

use floem_renderer::gpu_resources::GpuResources;
use uuid::Uuid;

use crate::basic::Shape;
use crate::polygon::PolygonConfig;
use crate::{
    basic::Point,
    basic::WindowSize,
    dot::{distance, EdgePoint},
    polygon::Polygon,
};

use strum::IntoEnumIterator;
use strum_macros::EnumIter;

#[derive(Eq, PartialEq, Clone, Copy, EnumIter, Debug)]
pub enum ControlMode {
    Point,
    Edge,
}

impl Display for ControlMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ControlMode::Point => f.write_str("Point"),
            ControlMode::Edge => f.write_str("Edge"),
        }
    }
}

#[derive(Clone, Copy)]
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

type PolygonClickHandler = dyn Fn() -> Option<Box<dyn FnMut(Uuid, PolygonConfig)>>;
pub type LayersUpdateHandler = dyn Fn() -> Option<Box<dyn FnMut(Vec<PolygonConfig>)>>;

pub struct Editor {
    pub polygons: Vec<Polygon>,
    pub hover_point: Option<EdgePoint>,
    pub hover_edge: Option<(usize, usize)>, // (polygon_index, edge_index)
    pub dragging_point: Option<(usize, usize)>, // (polygon_index, point_index)
    pub dragging_edge: Option<(usize, usize)>, // (polygon_index, edge_index)
    pub dragging_polygon: Option<usize>,
    pub guide_lines: Vec<GuideLine>,
    pub viewport: Arc<Mutex<Viewport>>,
    pub drag_start: Option<Point>,
    pub last_x: f32,
    pub last_y: f32,
    // pub button_handler: Option<Box<dyn Fn(&mut Editor)>>,
    // button_handler: RefCell<Option<Box<dyn Fn(MutexGuard<'_, Editor>) + Send + 'static>>>,
    pub handle_polygon_click: Option<Arc<PolygonClickHandler>>,
    pub gpu_resources: Option<Arc<GpuResources>>,
    pub handle_layers_update: Option<Arc<LayersUpdateHandler>>,
    pub control_mode: ControlMode,
}

use std::borrow::BorrowMut;

impl Editor {
    pub fn new(viewport: Arc<Mutex<Viewport>>) -> Self {
        Editor {
            polygons: Vec::new(),
            hover_point: None,
            hover_edge: None,
            dragging_point: None,
            dragging_edge: None,
            dragging_polygon: None,
            guide_lines: Vec::new(),
            viewport,
            drag_start: None,
            last_x: 0.0,
            last_y: 0.0,
            handle_polygon_click: None,
            gpu_resources: None,
            handle_layers_update: None,
            control_mode: ControlMode::Point,
        }
    }

    pub fn add_polygon(&mut self, polygon: Polygon) {
        self.polygons.push(polygon);
        self.run_layers_update();
    }

    pub fn run_layers_update(&self) {
        if (self.handle_layers_update.is_some()) {
            let handler_creator = self
                .handle_layers_update
                .as_ref()
                .expect("Couldn't get handler");
            let mut handle_update = handler_creator().expect("Couldn't get handler");

            let polygon_configs: Vec<PolygonConfig> = self
                .polygons
                .iter()
                .map(|polygon| polygon.to_config())
                .collect();

            // println!("Update layers... {:?}", polygon_configs.len());
            handle_update(polygon_configs);
        }
    }

    pub fn update_date_from_window_resize(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        for (poly_index, polygon) in self.polygons.iter_mut().enumerate() {
            polygon.update_data_from_window_size(window_size, device);
        }
    }

    pub fn handle_mouse_down(&mut self, window_size: &WindowSize, device: &wgpu::Device) {
        let x = self.last_x;
        let y = self.last_y;
        let mouse_pos = Point { x, y };

        match self.control_mode {
            ControlMode::Point => self.handle_mouse_down_point_mode(mouse_pos, window_size, device),
            ControlMode::Edge => self.handle_mouse_down_edge_mode(mouse_pos, window_size, device),
        }
    }

    pub fn handle_mouse_down_point_mode(
        &mut self,
        // x: f32,
        // y: f32,
        mouse_pos: Point,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        // let x = self.last_x;
        // let y = self.last_y;
        // let mouse_pos = Point { x, y };

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

                // TODO: make DRY with below
                if (self.handle_polygon_click.is_some()) {
                    let handler_creator = self
                        .handle_polygon_click
                        .as_ref()
                        .expect("Couldn't get handler");
                    let mut handle_click = handler_creator().expect("Couldn't get handler");
                    handle_click(
                        polygon.id,
                        PolygonConfig {
                            id: polygon.id,
                            name: polygon.name.clone(),
                            points: polygon.points.clone(),
                            dimensions: polygon.dimensions,
                            position: polygon.transform.position,
                            border_radius: polygon.border_radius,
                            fill: polygon.fill,
                        },
                    );
                }
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

    fn handle_mouse_down_edge_mode(
        &mut self,
        mouse_pos: Point,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        if let Some((poly_index, edge_index)) = self.hover_edge {
            self.dragging_edge = Some((poly_index, edge_index));
            return;
        }

        // If not hovering over an edge, check for polygon dragging (same as point mode)
        for (poly_index, polygon) in self.polygons.iter().enumerate() {
            if polygon.contains_point(&mouse_pos) {
                self.dragging_polygon = Some(poly_index);
                self.drag_start = Some(mouse_pos);

                // hard to make DRY
                if (self.handle_polygon_click.is_some()) {
                    let handler_creator = self
                        .handle_polygon_click
                        .as_ref()
                        .expect("Couldn't get handler");
                    let mut handle_click = handler_creator().expect("Couldn't get handler");
                    handle_click(
                        polygon.id,
                        PolygonConfig {
                            id: polygon.id,
                            name: polygon.name.clone(),
                            points: polygon.points.clone(),
                            dimensions: polygon.dimensions,
                            position: polygon.transform.position,
                            border_radius: polygon.border_radius,
                            fill: polygon.fill,
                        },
                    );
                }

                return;
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
        self.last_x = x;
        self.last_y = y;

        match self.control_mode {
            ControlMode::Point => self.handle_mouse_move_point_mode(mouse_pos, window_size, device),
            ControlMode::Edge => self.handle_mouse_move_edge_mode(mouse_pos, window_size, device),
        }
    }

    pub fn handle_mouse_move_point_mode(
        &mut self,
        mouse_pos: Point,
        window_size: &WindowSize,
        device: &wgpu::Device,
        // x: f32,
        // y: f32,
    ) {
        // let mouse_pos = Point { x, y };
        self.hover_point = None;
        self.guide_lines.clear();

        // self.last_x = x;
        // self.last_y = y;

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
                self.move_polygon(mouse_pos, start, poly_index, window_size, device);
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

    pub fn move_polygon(
        &mut self,
        mouse_pos: Point,
        start: Point,
        poly_index: usize,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
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

    fn handle_mouse_move_edge_mode(
        &mut self,
        mouse_pos: Point,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        self.hover_edge = None;
        self.guide_lines.clear();

        if let Some((poly_index, edge_index)) = self.dragging_edge {
            let polygon = &mut self.polygons[poly_index];
            polygon.move_edge(edge_index, mouse_pos, window_size, device);
            self.update_guide_lines(poly_index, window_size);
        } else if let Some(poly_index) = self.dragging_polygon {
            if let Some(start) = self.drag_start {
                self.move_polygon(mouse_pos, start, poly_index, window_size, device);
            }
        } else {
            for (poly_index, polygon) in self.polygons.iter().enumerate() {
                if let Some(edge_index) = polygon.closest_edge(mouse_pos) {
                    self.hover_edge = Some((poly_index, edge_index));
                    break;
                }
            }
        }
    }

    pub fn handle_mouse_up(&mut self) {
        self.dragging_point = None;
        self.dragging_polygon = None;
        self.drag_start = None;
        self.dragging_edge = None;
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
                // println!("Vertical guide at x={}", other_bbox.min.x);
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
                // println!("Vertical guide at x={}", other_bbox.max.x);
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
                // println!("Horizontal guide at y={}", other_bbox.min.y);
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
                // println!("Horizontal guide at y={}", other_bbox.max.y);
            }
        }
    }

    fn is_close(&self, a: f32, b: f32, threshold: f32) -> bool {
        (a - b).abs() < threshold
    }
}
