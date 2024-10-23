use std::cell::RefCell;
use std::fmt::Display;
use std::sync::{Arc, Mutex, MutexGuard};

use cgmath::{Matrix4, Point3, Vector2, Vector3, Vector4};
use floem_renderer::gpu_resources::{self, GpuResources};
use floem_winit::window::Window;
use std::f32::consts::PI;
use uuid::Uuid;
use winit::window::CursorIcon;

use crate::basic::Shape;
use crate::camera::{self, Camera, CameraBinding};
use crate::guideline::point_to_ndc;
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

// pub fn size_to_ndc(window_size: &WindowSize, x: f32, y: f32) -> (f32, f32) {
//     // let ndc_x = (x / window_size.width as f32) * 2.0 - 1.0;
//     // let ndc_y = -((y / window_size.height as f32) * 2.0 - 1.0); // Flip Y-axis
//     // Current (top-left origin)
//     // let ndc_y = 1.0 - ((2.0 * screen_y) / self.window_size.height as f32);

//     // For center origin, we might want:
//     let ndc_x = -((2.0 * x) / window_size.width as f32) + 1.0;
//     let ndc_y = -((2.0 * y) / window_size.height as f32) + 1.0;
//     // just flips the above 180 degrees
//     // let ndc_x = ((2.0 * (x - window_size.width as f32 / 2.0)) / window_size.width as f32);
//     // let ndc_y = ((2.0 * (y - window_size.height as f32 / 2.0)) / window_size.height as f32);

//     (ndc_x, ndc_y)
// }

// pub fn size_to_ndc(window_size: &WindowSize, x: f32, y: f32) -> (f32, f32) {
//     // Convert screen space (top-left origin) to NDC (center origin)
//     let ndc_x = (2.0 * x / window_size.width as f32) - 1.0; // X from -1 to 1
//     let ndc_y = 1.0 - (2.0 * y / window_size.height as f32); // Y from 1 to -1 (flip Y)

//     (ndc_x, ndc_y)
// }

pub fn size_to_ndc(window_size: &WindowSize, x: f32, y: f32) -> (f32, f32) {
    let ndc_x = x / window_size.width as f32;
    let ndc_y = y / window_size.height as f32;

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
    // pub last_x: f32,
    // pub last_y: f32,
    pub last_screen: Point, // last mouse position from input event top-left origin
    pub last_world: Point,
    pub last_top_left: Point,
    // pub button_handler: Option<Box<dyn Fn(&mut Editor)>>,
    // button_handler: RefCell<Option<Box<dyn Fn(MutexGuard<'_, Editor>) + Send + 'static>>>,
    pub handle_polygon_click: Option<Arc<PolygonClickHandler>>,
    pub gpu_resources: Option<Arc<GpuResources>>,
    pub handle_layers_update: Option<Arc<LayersUpdateHandler>>,
    pub control_mode: ControlMode,
    pub window: Option<Arc<Window>>,
    pub camera: Option<Camera>,
    pub is_panning: bool,
    pub last_mouse_pos: Option<Point>,
    pub camera_binding: Option<CameraBinding>,
    pub ds_ndc_pos: Point, // double-width sized ndc-style positioning (screen-oriented)
    pub ndc: Point,
}

use std::borrow::BorrowMut;

impl Editor {
    pub fn new(viewport: Arc<Mutex<Viewport>>) -> Self {
        let viewport_unwrapped = viewport.lock().unwrap();
        let window_size = WindowSize {
            width: viewport_unwrapped.width as u32,
            height: viewport_unwrapped.height as u32,
        };
        Editor {
            polygons: Vec::new(),
            hover_point: None,
            hover_edge: None,
            dragging_point: None,
            dragging_edge: None,
            dragging_polygon: None,
            guide_lines: Vec::new(),
            viewport: viewport.clone(),
            drag_start: None,
            // last_x: 0.0,
            // last_y: 0.0,
            handle_polygon_click: None,
            gpu_resources: None,
            handle_layers_update: None,
            control_mode: ControlMode::Point,
            window: None,
            camera: None,
            camera_binding: None,
            last_mouse_pos: None,
            is_panning: false,
            last_screen: Point { x: 0.0, y: 0.0 },
            last_world: Point { x: 0.0, y: 0.0 },
            ds_ndc_pos: Point { x: 0.0, y: 0.0 },
            last_top_left: Point { x: 0.0, y: 0.0 },
            ndc: Point { x: 0.0, y: 0.0 },
        }
    }

    pub fn update_camera_binding(&mut self, queue: &wgpu::Queue) {
        if (self.camera_binding.is_some()) {
            self.camera_binding
                .as_mut()
                .expect("Couldn't get camera binding")
                .update(queue, &self.camera.as_ref().expect("Couldn't get camera"));
        }
    }

    pub fn handle_wheel(&mut self, delta: f32, mouse_pos: Point, queue: &wgpu::Queue) {
        // let zoom_factor = if delta > 0.0 { 1.1 } else { 0.9 };
        let zoom_factor = delta / 10.0;
        let camera = self.camera.as_mut().expect("Couldnt't get camera");
        camera.zoom(zoom_factor, mouse_pos);
        self.update_camera_binding(queue);
    }

    // pub fn add_polygon(&mut self, polygon: Polygon) {
    //     let world_position = self.camera.screen_to_world(polygon.transform.position);
    //     let poly_len = self.polygons.len();
    //     self.polygons.push(polygon);
    //     self.polygons[poly_len].transform.position = world_position;
    //     self.run_layers_update();
    // }

    // pub fn add_polygon(&mut self, mut polygon: Polygon) {
    //     // Convert from screen coordinates (0 to window_size) to clip space (-1 to 1)
    //     let viewport = self.viewport.lock().unwrap();
    //     let clip_position = Point {
    //         x: (polygon.transform.position.x / viewport.width) * 2.0 - 1.0,
    //         y: -((polygon.transform.position.y / viewport.height) * 2.0 - 1.0), // Flip Y for clip space
    //     };

    //     let poly_len = self.polygons.len();
    //     polygon.transform.position = clip_position;
    //     self.polygons.push(polygon);
    //     self.run_layers_update();
    // }

    // pub fn add_polygon(&mut self, mut polygon: Polygon) {
    //     // Convert from screen coordinates (e.g., 0-800) to clip space (-1 to 1)
    //     // but don't apply any additional scaling
    //     let clip_position = Point {
    //         x: polygon.transform.position.x, // Keep original position for now
    //         y: polygon.transform.position.y, // We'll let the camera handle transformation
    //     };

    //     let poly_len = self.polygons.len();
    //     polygon.transform.position = clip_position;
    //     self.polygons.push(polygon);
    //     self.run_layers_update();
    // }

    pub fn add_polygon(&mut self, mut polygon: Polygon) {
        let camera = self.camera.as_ref().expect("Couldn't get camera");
        // let world_position = camera.screen_to_world(polygon.transform.position);
        let world_position = polygon.transform.position;
        println!(
            "add polygon position {:?} {:?}",
            world_position, polygon.transform.position
        );
        polygon.transform.position = world_position;
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

    pub fn update_cursor(&self) {
        let cursor = match self.control_mode {
            ControlMode::Point => {
                // if self.dragging_point.is_some() {
                //     CursorIcon::Grabbing
                // } else if self.hover_point.is_some() {
                //     CursorIcon::Grab
                // } else {
                //     CursorIcon::Default
                // }
                // I feel that the ring / dot is better, and Grab covers it up
                CursorIcon::Default
            }
            ControlMode::Edge => {
                if let Some((poly_index, edge_index)) = self.dragging_edge.or(self.hover_edge) {
                    let polygon = &self.polygons[poly_index];

                    // Get start point in world coordinates
                    let start_point = polygon.points[edge_index];
                    let start = Point {
                        x: start_point.x * polygon.dimensions.0 + polygon.transform.position.x,
                        y: start_point.y * polygon.dimensions.1 + polygon.transform.position.y,
                    };

                    // Get end point in world coordinates
                    let end_index = (edge_index + 1) % polygon.points.len();
                    let end_point = polygon.points[end_index];
                    let end = Point {
                        x: end_point.x * polygon.dimensions.0 + polygon.transform.position.x,
                        y: end_point.y * polygon.dimensions.1 + polygon.transform.position.y,
                    };

                    // Calculate angle in world coordinates
                    let dx = end.x - start.x;
                    // Flip dy for screen coordinates
                    let dy = start.y - end.y; // Note the flip here: start.y - end.y instead of end.y - start.y

                    // Use atan2 to get angle in radians
                    let mut angle = dy.atan2(dx);

                    // Normalize angle to 0-2PI range
                    if angle < 0.0 {
                        angle += 2.0 * PI;
                    }

                    // // Optional: Debug print
                    // if self.debug_cursor_angles {
                    //     println!("Edge angle: {:.2}° ({}rad)", angle * 180.0 / PI, angle);
                    // }

                    // Convert angle to degrees for easier understanding
                    let degrees = angle * 180.0 / PI;

                    // Determine cursor based on 8 primary directions
                    let normalized_degrees = ((degrees % 180.0) + 180.0) % 180.0;

                    // if (normalized_degrees >= 0.0 && normalized_degrees <= 22.5)
                    //     || (normalized_degrees >= 157.5 && normalized_degrees <= 180.0)
                    // {
                    //     // CursorIcon::EwResize
                    //     CursorIcon::NsResize
                    // } else if normalized_degrees >= 67.5 && normalized_degrees <= 112.5 {
                    //     // CursorIcon::NsResize
                    //     CursorIcon::EwResize
                    // } else if (normalized_degrees > 22.5 && normalized_degrees < 67.5) {
                    //     if degrees <= 90.0 || degrees >= 270.0 {
                    //         CursorIcon::NwseResize // Swapped from NeswResize
                    //     } else {
                    //         CursorIcon::NeswResize // Swapped from NwseResize
                    //     }
                    // } else {
                    //     if degrees <= 90.0 || degrees >= 270.0 {
                    //         CursorIcon::NeswResize // Swapped from NwseResize
                    //     } else {
                    //         CursorIcon::NwseResize // Swapped from NeswResize
                    //     }
                    // }

                    // Define diagonal variables based on the degrees
                    let is_north_west = degrees > 270.0 && degrees < 360.0;
                    let is_north_east = degrees > 0.0 && degrees < 90.0;
                    let is_south_west = degrees > 180.0 && degrees < 270.0; // Opposite diagonal
                    let is_south_east = degrees > 90.0 && degrees < 180.0; // Opposite diagonal

                    // Cursor determination based on the angle ranges
                    let cursor = if (normalized_degrees >= 0.0 && normalized_degrees <= 22.5)
                        || (normalized_degrees >= 157.5 && normalized_degrees <= 180.0)
                    {
                        // Horizontal edge (0° or 180°)
                        // CursorIcon::EwResize
                        CursorIcon::NsResize
                    } else if normalized_degrees >= 67.5 && normalized_degrees <= 112.5 {
                        // Vertical edge (90°)
                        // CursorIcon::NsResize
                        CursorIcon::EwResize
                    } else {
                        // Diagonal edge: choose based on diagonal direction
                        if is_north_east || is_south_west {
                            CursorIcon::NwseResize // North-West to South-East
                                                   // CursorIcon::NeswResize
                        } else if is_north_west || is_south_east {
                            CursorIcon::NeswResize // North-East to South-West
                                                   // CursorIcon::NwseResize
                        } else {
                            CursorIcon::Default
                        }
                    };

                    cursor
                } else {
                    CursorIcon::Default
                }
            }
        };

        let window = self.window.as_ref().expect("Couldn't get window");

        window.set_cursor_icon(cursor);
    }

    pub fn update_date_from_window_resize(
        &mut self,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        let camera = self.camera.as_ref().expect("Couldn't get camera");
        for (poly_index, polygon) in self.polygons.iter_mut().enumerate() {
            polygon.update_data_from_window_size(window_size, device, &camera);
        }
    }

    pub fn handle_mouse_down(&mut self, window_size: &WindowSize, device: &wgpu::Device) {
        let camera = self.camera.as_ref().expect("Couldn't get camera");
        let x = self.ds_ndc_pos.x;
        let y = self.ds_ndc_pos.y;
        let mouse_pos = Point { x, y };
        // let world_pos = camera.screen_to_world(mouse_pos);

        match self.control_mode {
            ControlMode::Point => self.handle_mouse_down_point_mode(mouse_pos, window_size, device),
            ControlMode::Edge => self.handle_mouse_down_edge_mode(mouse_pos, window_size, device),
        }

        self.update_cursor();
    }

    pub fn handle_mouse_down_point_mode(
        &mut self,
        // x: f32,
        // y: f32,
        mouse_pos: Point,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        let camera = self.camera.as_mut().expect("Couldn't get camera");
        // let x = self.last_x;
        // let y = self.last_y;
        // let mouse_pos = Point { x, y };

        // let world_pos = self.camera.screen_to_world(mouse_pos);

        // let last_screen = self.last_screen;

        if let Some(hover_point) = self.hover_point {
            for (poly_index, polygon) in self.polygons.iter_mut().enumerate() {
                if let Some(edge_point) = polygon.closest_point_on_edge(self.last_top_left, &camera)
                {
                    if (edge_point.point.x - hover_point.point.x).abs() < 1.0
                        && (edge_point.point.y - hover_point.point.y).abs() < 1.0
                    {
                        let normalized_point = Point {
                            x: (edge_point.point.x - polygon.transform.position.x)
                                / polygon.dimensions.0,
                            y: (edge_point.point.y - polygon.transform.position.y)
                                / polygon.dimensions.1,
                        };
                        println!("normalized_point {:?}", normalized_point);
                        polygon.add_point(
                            normalized_point,
                            edge_point.edge_index,
                            window_size,
                            device,
                            &camera,
                        );
                        // already done in add_point
                        // polygon.update_data_from_points(
                        //     window_size,
                        //     device,
                        //     polygon.points.clone(),
                        //     &camera,
                        // );
                        self.dragging_point = Some((poly_index, edge_point.edge_index + 1));
                        return;
                    }
                }
            }
        }

        // Check if we're clicking on a polygon to drag
        for (poly_index, polygon) in self.polygons.iter().enumerate() {
            if polygon.contains_point(&self.last_top_left, &camera) {
                self.dragging_polygon = Some(poly_index);
                self.drag_start = Some(self.last_top_left);

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
                if distance(world_point, self.last_top_left) < 5.0 {
                    self.dragging_point = Some((poly_index, point_index));
                    return;
                }
            }
        }

        // If we haven't clicked anything else, start panning
        self.is_panning = true;
        camera.focus_point = Vector2::new(mouse_pos.x, mouse_pos.y);
        self.last_mouse_pos = Some(mouse_pos);
    }

    fn handle_mouse_down_edge_mode(
        &mut self,
        mouse_pos: Point,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        let camera = self.camera.as_ref().expect("Couldn't get camera");
        // let world_pos = self.camera.screen_to_world(mouse_pos);

        if let Some((poly_index, edge_index)) = self.hover_edge {
            self.dragging_edge = Some((poly_index, edge_index));
            return;
        }

        // If not hovering over an edge, check for polygon dragging (same as point mode)
        for (poly_index, polygon) in self.polygons.iter().enumerate() {
            if polygon.contains_point(&self.last_top_left, camera) {
                self.dragging_polygon = Some(poly_index);
                self.drag_start = Some(self.last_top_left);

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
        let camera = self.camera.as_mut().expect("Couldn't get camera");
        let mouse_pos = Point { x, y };
        let ds_ndc = visualize_ray_intersection(window_size, x, y, &camera);
        let ds_ndc_pos = ds_ndc.origin;
        let ds_ndc_pos = Point {
            x: ds_ndc_pos.x,
            y: ds_ndc_pos.y,
        };
        // let top_left = camera.ds_ndc_to_top_left(ds_ndc_pos);
        // let top_left = camera.ndc_to_top_left(ds_ndc.ndc);
        let top_left = ds_ndc.top_left;
        // println!("top_left {:?} {:?}", top_left, mouse_pos);
        self.last_top_left = top_left;
        self.ds_ndc_pos = ds_ndc_pos;
        self.ndc = ds_ndc.ndc;

        self.last_screen = Point { x, y };
        self.last_world = camera.screen_to_world(mouse_pos);

        // Handle panning
        if self.is_panning {
            if let Some(last_pos) = self.last_mouse_pos {
                let delta = Vector2::new(ds_ndc_pos.x - last_pos.x, ds_ndc_pos.y - last_pos.y);

                // println!("is_panning A {:?}", delta);

                let adjusted_delta = Vector2::new(
                    -delta.x, // Invert X
                    -delta.y, // Keep Y as is
                );
                let delta = adjusted_delta / 2.0;

                // println!("is_panning B {:?}", delta);
                // adjusting the camera, so expect delta to be very small like 0-1
                camera.pan(delta);
                let mut camera_binding = self
                    .camera_binding
                    .as_mut()
                    .expect("Couldn't get camera binging");
                let gpu_resources = self
                    .gpu_resources
                    .as_ref()
                    .expect("Couldn't get gpu resources");
                camera_binding.update(&gpu_resources.queue, &camera);
            }
            self.last_mouse_pos = Some(ds_ndc_pos);
            return;
        }

        match self.control_mode {
            ControlMode::Point => {
                self.handle_mouse_move_point_mode(ds_ndc_pos, window_size, device)
            }
            ControlMode::Edge => self.handle_mouse_move_edge_mode(ds_ndc_pos, window_size, device),
        }

        self.update_cursor();
    }

    pub fn handle_mouse_move_point_mode(
        &mut self,
        mouse_pos: Point,
        window_size: &WindowSize,
        device: &wgpu::Device,
        // x: f32,
        // y: f32,
    ) {
        let camera = self.camera.as_ref().expect("Couldn't get camera");
        // let mouse_pos = Point { x, y };
        self.hover_point = None;
        self.guide_lines.clear();

        // self.last_x = x;
        // self.last_y = y;

        // let world_pos = self.camera.screen_to_world(mouse_pos);

        // let last_screen = self.last_screen;

        // println!("last_screen {:?}", last_screen);
        // println!("editor position {:?} {:?}", self.last_x, self.last_y);

        if let Some((poly_index, point_index)) = self.dragging_point {
            let polygon = &mut self.polygons[poly_index];
            let normalized_pos = Point {
                x: (self.last_top_left.x - polygon.transform.position.x) / polygon.dimensions.0,
                y: (self.last_top_left.y - polygon.transform.position.y) / polygon.dimensions.1,
            };
            polygon.move_point(point_index, normalized_pos);
            polygon.update_data_from_points(window_size, device, polygon.points.clone(), &camera);
            self.update_guide_lines(poly_index, window_size);
        } else if let Some(poly_index) = self.dragging_polygon {
            if let Some(start) = self.drag_start {
                self.move_polygon(self.last_top_left, start, poly_index, window_size, device);
            }
        } else {
            for polygon in &self.polygons {
                // let world_pos = camera.screen_to_world(mouse_pos);
                if let Some(edge_point) = polygon.closest_point_on_edge(self.last_top_left, &camera)
                {
                    self.hover_point = Some(edge_point);
                    // println!(
                    //     "hover polygon edge position {:?} {:?} {:?}",
                    //     mouse_pos, edge_point, polygon.transform.position
                    // );
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
        let camera = self.camera.as_ref().expect("Couldn't get camera");
        let aspect_ratio = camera.window_size.width as f32 / camera.window_size.height as f32;
        let dx = mouse_pos.x - start.x;
        let dy = mouse_pos.y - start.y;
        let polygon = &mut self.polygons[poly_index];
        let new_position = Point {
            x: polygon.transform.position.x + (dx * 0.9), // not sure relation with aspect_ratio?
            y: polygon.transform.position.y + dy,
        };
        println!("move_polygon {:?}", new_position);
        polygon.update_data_from_position(window_size, device, new_position, &camera);
        self.drag_start = Some(mouse_pos);
        self.update_guide_lines(poly_index, window_size);
    }

    fn handle_mouse_move_edge_mode(
        &mut self,
        mouse_pos: Point,
        window_size: &WindowSize,
        device: &wgpu::Device,
    ) {
        let camera = self.camera.as_ref().expect("Couldn't get camera");
        self.hover_edge = None;
        self.guide_lines.clear();

        // let world_pos = self.camera.screen_to_world(mouse_pos);

        if let Some((poly_index, edge_index)) = self.dragging_edge {
            let polygon = &mut self.polygons[poly_index];
            polygon.move_edge(edge_index, self.last_top_left, window_size, device, &camera);
            self.update_guide_lines(poly_index, window_size);
        } else if let Some(poly_index) = self.dragging_polygon {
            if let Some(start) = self.drag_start {
                self.move_polygon(self.last_top_left, start, poly_index, window_size, device);
            }
        } else {
            for (poly_index, polygon) in self.polygons.iter().enumerate() {
                if let Some(edge_index) = polygon.closest_edge(self.last_top_left) {
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
        self.is_panning = false;
        self.guide_lines.clear();
        self.update_cursor();
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

use cgmath::InnerSpace;

#[derive(Debug)]
pub struct Ray {
    pub origin: Point3<f32>,
    pub direction: Vector3<f32>,
    pub ndc: Point,
    pub top_left: Point,
}

impl Ray {
    pub fn new(origin: Point3<f32>, direction: Vector3<f32>) -> Self {
        Ray {
            origin,
            direction: direction.normalize(),
            ndc: Point { x: 0.0, y: 0.0 },
            top_left: Point { x: 0.0, y: 0.0 },
        }
    }
}

// Helper function to properly transform screen to NDC coordinates
fn screen_to_ndc(
    screen_x: f32,
    screen_y: f32,
    screen_width: f32,
    screen_height: f32,
    viewport_offset_x: f32, // Usually 0.0
    viewport_offset_y: f32, // Usually 0.0
) -> (f32, f32) {
    // Adjust for viewport offset
    let adjusted_x = screen_x - viewport_offset_x;
    let adjusted_y = screen_y - viewport_offset_y;

    // Use viewport dimensions for correct scaling
    let ndc_x = (2.0 * adjusted_x) / screen_width - 1.0;
    let ndc_y = 1.0 - (2.0 * adjusted_y) / screen_height; // Flip Y coordinates

    (ndc_x, ndc_y)
}

use cgmath::SquareMatrix;
use cgmath::Transform;

pub fn create_ray_from_screen(
    screen_x: f32,
    screen_y: f32,
    screen_width: f32,
    screen_height: f32,
    projection_matrix: &Matrix4<f32>,
    view_matrix: &Matrix4<f32>,
) -> Ray {
    // Convert screen coordinates to NDC space
    let (ndc_x, ndc_y) = screen_to_ndc(
        screen_x,
        screen_y,
        screen_width,
        screen_height,
        0.0, // viewport_offset_x
        0.0, // viewport_offset_y
    );

    // Ensure NDC coordinates are properly bound
    let ndc_x = ndc_x.max(-1.0).min(1.0);
    let ndc_y = ndc_y.max(-1.0).min(1.0);

    // Create homogeneous points in clip space
    let clip_near = Vector4::new(ndc_x, ndc_y, -1.0, 1.0);
    let clip_far = Vector4::new(ndc_x, ndc_y, 1.0, 1.0);

    // Get inverse matrices
    let inv_projection = projection_matrix
        .invert()
        .expect("Failed to invert projection matrix");
    let inv_view = view_matrix.invert().expect("Failed to invert view matrix");

    // Transform to view space (handle w component properly)
    let mut view_near = inv_projection * clip_near;
    let mut view_far = inv_projection * clip_far;

    // Perform perspective divide if w is not 1.0
    if view_near.w.abs() > std::f32::EPSILON {
        view_near = view_near / view_near.w;
    }
    if view_far.w.abs() > std::f32::EPSILON {
        view_far = view_far / view_far.w;
    }

    // Transform to world space
    let world_near = inv_view.transform_point(Point3::new(view_near.x, view_near.y, view_near.z));
    let world_far = inv_view.transform_point(Point3::new(view_far.x, view_far.y, view_far.z));

    // Calculate direction and ensure it's normalized
    let direction = world_far - world_near;
    let direction = direction.normalize();

    Ray::new(world_near, direction)
}

// Example intersection test with an axis-aligned bounding box (AABB)
#[derive(Debug)]
pub struct AABB {
    pub min: Point3<f32>,
    pub max: Point3<f32>,
}

impl AABB {
    pub fn intersects_ray(&self, ray: &Ray) -> Option<f32> {
        let mut tmin = f32::NEG_INFINITY;
        let mut tmax = f32::INFINITY;

        for i in 0..3 {
            let inv_dir = 1.0 / ray.direction[i];
            let mut t0 = (self.min[i] - ray.origin[i]) * inv_dir;
            let mut t1 = (self.max[i] - ray.origin[i]) * inv_dir;

            if inv_dir < 0.0 {
                std::mem::swap(&mut t0, &mut t1);
            }

            tmin = tmin.max(t0);
            tmax = tmax.min(t1);

            if tmax <= tmin {
                return None;
            }
        }

        Some(tmin)
    }
}

pub fn visualize_ray_intersection(
    // device: &wgpu::Device,
    window_size: &WindowSize,
    screen_x: f32,
    screen_y: f32,
    camera: &Camera,
) -> Ray {
    let aspect_ratio = window_size.width as f32 / window_size.height as f32;

    let ndc_x = screen_x / camera.window_size.width as f32;
    let ndc_y = (screen_y / camera.window_size.height as f32);

    // Convert 2D position to 3D (keeping Z at 0 for 2D plane)
    // let camera_pos_ndc = point_to_ndc(
    //     Point {
    //         x: camera.position.x,
    //         y: camera.position.y,
    //     },
    //     window_size,
    // );
    // let camera_pos_ndc = camera.ndc_to_normalized(camera_pos_ndc.x, camera_pos_ndc.y);
    let view_pos = Vector3::new(0.0, 0.0, 0.0); // investigate why moves when this all 0
    let model_view = Matrix4::from_translation(view_pos);

    let plane_size_normal = Vector3::new((1.0 * aspect_ratio) / 2.0, (1.0 * 2.0) / 2.0, 0.0);

    // Transform NDC point to view space
    let view_point_normal = Point3::new(
        ndc_x * plane_size_normal.x,
        ndc_y * plane_size_normal.y,
        0.0,
    );
    let world_point_normal = model_view
        .invert()
        .unwrap()
        .transform_point(view_point_normal);

    println!("normal {:?}", world_point_normal);

    // Create a plane in view space
    let plane_center = Point3::new(
        -(camera.window_size.width as f32),
        -(camera.window_size.height as f32),
        0.0,
    );

    // let plane_center = Point3::new(0.0, 0.0, 0.0);
    let plane_size = Vector3::new(
        (camera.window_size.width as f32) * aspect_ratio, // definitely needed aspect_ratio
        (camera.window_size.height as f32) * 2.0,
        0.0,
    );

    // Transform NDC point to view space, accounting for center offset
    let view_point = Point3::new(
        ndc_x * plane_size.x + plane_center.x,
        ndc_y * plane_size.y + plane_center.y,
        0.0,
    );

    // Transform to world space
    let world_point = model_view.invert().unwrap().transform_point(view_point);

    // Create ray from camera position to point (in 3D space)
    let camera_pos_3d = Point3::new(camera.position.x, camera.position.y, 0.0);
    let direction = (world_point - camera_pos_3d).normalize();

    let origin = Point3 {
        x: world_point.x + camera.position.x + 140.0,
        y: -(world_point.y) + camera.position.y,
        z: world_point.z,
    };

    let ndc = camera.normalized_to_ndc(world_point_normal.x, world_point_normal.y);

    let top_left = Point {
        x: (world_point_normal.x * window_size.width as f32) + (camera.position.x * 0.5) + 70.0, // maybe account for camera position
        y: (world_point_normal.y * window_size.height as f32) - (camera.position.y * 0.5),
    };

    // println!(
    //     "is top_left way higher than middle? {:?} {:?} {:?}",
    //     top_left, screen_x, screen_y
    // );

    Ray {
        direction,
        origin,
        ndc: Point { x: ndc.0, y: ndc.1 },
        top_left,
    }
}
