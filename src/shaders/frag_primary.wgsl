struct FragmentInput {
    @location(0) tex_coords: vec2<f32>,
    @location(1) color: vec4<f32>,  // Receive color from vertex shader
};

@group(0) @binding(0) var myTexture: texture_2d<f32>;
@group(0) @binding(1) var mySampler: sampler;
@group(0) @binding(2) var<uniform> renderMode: i32;

@fragment
fn fs_main(in: FragmentInput) -> @location(0) vec4<f32> {
    let texColor = textureSample(myTexture, mySampler, in.tex_coords);
    // if (renderMode == 1) { // Assume 1 means rendering text
    //     return vec4(1.0, 1.0, 1.0, texColor.r); // Text mode
    // } else {
        return texColor; // Normal rendering
    // }
}