use crate::basic::Point;
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use basic::rgb_to_wgpu;
use dot::draw_dot;
// use wgpu::{self, core::pipeline};
use winit::dpi::PhysicalSize;
use winit::{
    event::*,
    event_loop::EventLoop,
    window::{Window, WindowBuilder},
};
// use winit::{event_loop, window};
use crate::editor::{Editor, Viewport};

mod basic;
mod dot;
mod editor;
mod path;
mod polygon;
mod transform;
mod vertex;

// Styling information
// struct Style {
//     fill_color: Option<Color>,
//     stroke_color: Option<Color>,
//     stroke_width: f32,
// }

// // Color representation
// struct Color {
//     r: f32,
//     g: f32,
//     b: f32,
//     a: f32,
// }

// // Scene graph node
// struct Node {
//     shape: Box<dyn Shape>,
//     transform: Transform,
//     style: Style,
//     children: Vec<Node>,
// }

// // 2D transformation matrix
// struct Transform {
//     matrix: [[f32; 3]; 3],
// }

// // Layer for grouping and ordering shapes
// struct Layer {
//     nodes: Vec<Node>,
// }

// // Stage to hold all layers
// struct Stage {
//     layers: Vec<Layer>,
//     width: f32,
//     height: f32,
// }

pub struct WindowSize {
    width: u32,
    height: u32,
}

// I sure do love initializing wgpu this way
pub async fn initialize_core(event_loop: EventLoop<()>, window: Window, window_size: WindowSize) {
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
            .create_surface(&window)
            .expect("Couldn't create GPU surface")
    };

    println!("Ready...");

    let viewport = Viewport::new(window_size.width as f32, window_size.height as f32); // Or whatever your window size is
    let mut editor = Editor::new(viewport);

    println!("Setting up adapter...");

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

    let mut config = surface
        .get_default_config(&adapter, window_size.width, window_size.height)
        .unwrap();
    surface.configure(&device, &config);

    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::FilterMode::Nearest,
        ..Default::default()
    });

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

    // Define the layouts
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("Pipeline Layout"),
        // bind_group_layouts: &[&bind_group_layout],
        bind_group_layouts: &[], // No bind group layouts
        push_constant_ranges: &[],
    });

    // Load the shaders
    let shader_module_vert_primary = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("Primary Vert Shader"),
        source: wgpu::ShaderSource::Wgsl(include_str!("shaders/vert_primary.wgsl").into()),
    });

    let shader_module_frag_primary = device.create_shader_module(wgpu::ShaderModuleDescriptor {
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

    println!("Initialized...");

    let mut mouse_position = (0.0, 0.0);

    // test items
    // Create a triangle that fills the entire bounding box
    // let normalized_points = vec![
    //     Point { x: 0.0, y: 0.0 },
    //     Point { x: 1.0, y: 0.0 },
    //     Point { x: 0.5, y: 1.0 },
    // ];
    let normalized_points = vec![
        Point { x: 0.0, y: 0.0 },
        Point { x: 1.0, y: 0.0 },
        Point { x: 1.0, y: 1.0 },
        Point { x: 0.0, y: 1.0 },
    ];
    let dimensions = (100.0, 100.0); // 100x100 pixels bounding box
    let position = Point { x: 100.0, y: 100.0 }; // Position in world space
    let border_radius = 50.0;

    editor.polygons.push(Polygon::new(
        &window_size,
        &device,
        normalized_points,
        dimensions,
        position,
        border_radius,
    ));

    // editor.polygons[0].update_data_from_dimensions(&window_size, &device, (200.0, 50.0));

    // execute winit render loop
    let window = &window;
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
                        mouse_position = (position.x as f32, position.y as f32);
                        editor.handle_mouse_move(
                            &window_size,
                            &device,
                            position.x as f32,
                            position.y as f32,
                        );
                        // window.request_redraw();
                    }
                    WindowEvent::MouseInput { state, button, .. } => {
                        // let window_size = (size.width as f64, size.height as f64);
                        // handle_click(window_size, mouse_position, &buttons, &labels);
                        if button == MouseButton::Left {
                            match state {
                                ElementState::Pressed => editor.handle_mouse_down(
                                    mouse_position.0,
                                    mouse_position.1,
                                    &window_size,
                                    &device,
                                ),
                                ElementState::Released => editor.handle_mouse_up(),
                            }
                        }
                        // window.request_redraw();
                    }
                    WindowEvent::Resized(new_size) => {
                        editor.viewport =
                            Viewport::new(new_size.width as f32, new_size.height as f32);
                        // Reconfigure the surface with the new size
                        // let renderer_state =
                        //     renderer.state.as_mut().expect("Couldn't get RendererState");
                        // renderer_state.config.width = new_size.width.max(1);
                        // renderer_state.config.height = new_size.height.max(1);
                        // surface.configure(&renderer_state.device, &renderer_state.config);
                    }
                    // Event::MainEventsCleared => {
                    //     // If enough time has passed since the last frame, request a redraw
                    //     if last_render_time.elapsed() >= std::time::Duration::from_millis(16) {
                    //         // ~60 FPS
                    //         window.request_redraw();
                    //     }
                    // }
                    WindowEvent::RedrawRequested => {
                        // println!("Redraw");
                        // editor.draw(&mut renderer, &surface, &device);

                        let frame = surface
                            .get_current_texture()
                            .expect("Failed to acquire next swap chain texture");
                        let view = frame
                            .texture
                            .create_view(&wgpu::TextureViewDescriptor::default());

                        // Update the render pass to use the new vertex and index buffers
                        let mut encoder =
                            device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                                label: None,
                            });
                        {
                            let mut render_pass =
                                encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
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
                                    depth_stencil_attachment: Some(
                                        wgpu::RenderPassDepthStencilAttachment {
                                            view: &depth_view, // This is the depth texture view
                                            depth_ops: Some(wgpu::Operations {
                                                load: wgpu::LoadOp::Clear(1.0), // Clear to max depth
                                                store: wgpu::StoreOp::Store,
                                            }),
                                            stencil_ops: None, // Set this if using stencil
                                        },
                                    ),
                                    timestamp_writes: None,
                                    occlusion_query_set: None,
                                });

                            // println!("Render frame...");

                            render_pass.set_pipeline(&render_pipeline);

                            for (poly_index, polygon) in editor.polygons.iter().enumerate() {
                                // println!("Indices length {:?}", polygon.indices.len());
                                render_pass.set_vertex_buffer(0, polygon.vertex_buffer.slice(..));
                                render_pass.set_index_buffer(
                                    polygon.index_buffer.slice(..),
                                    wgpu::IndexFormat::Uint32,
                                );
                                render_pass.draw_indexed(0..polygon.indices.len() as u32, 0, 0..1);
                            }

                            if let Some(edge_point) = editor.hover_point {
                                let (vertices, indices, vertex_buffer, index_buffer) = draw_dot(
                                    &device,
                                    &window_size,
                                    edge_point.point,
                                    rgb_to_wgpu(47, 131, 222, 1.0),
                                ); // Green dot

                                render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
                                render_pass.set_index_buffer(
                                    index_buffer.slice(..),
                                    wgpu::IndexFormat::Uint32,
                                );
                                render_pass.draw_indexed(0..indices.len() as u32, 0, 0..1);
                            }
                        }

                        queue.submit(Some(encoder.finish()));
                        frame.present();

                        // supposed to fall in line with OS refresh rate (?)
                        // std::thread::sleep(std::time::Duration::from_millis(2000));
                        window.request_redraw();
                    }
                    WindowEvent::CloseRequested => target.exit(),
                    _ => {}
                };
            }
        })
        .unwrap();
}

fn main() {
    println!("Waiting for Floem+wgpu power, for now, direct winit / wgpu");

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

    futures::executor::block_on(initialize_core(event_loop, window, window_size));
}
