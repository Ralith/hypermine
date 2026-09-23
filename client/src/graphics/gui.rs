use common::math::MVector;
use yakui::{
    Alignment, Color, Response, align, canvas, colored_box, colored_box_container, label, pad,
    widgets::{CanvasResponse, Pad},
};

use crate::{Sim, graphics::Frustum};

pub struct GuiState {
    show_gui: bool,
    show_home_waypoint: bool,
}

impl GuiState {
    pub fn new() -> Self {
        GuiState {
            show_gui: true,
            show_home_waypoint: false,
        }
    }

    /// Toggles whether the GUI is shown
    pub fn toggle_gui(&mut self) {
        self.show_gui = !self.show_gui;
    }

    /// Toggles whether the home waypoint is shown
    pub fn toggle_show_home_waypoint(&mut self) {
        self.show_home_waypoint = !self.show_home_waypoint;
    }

    /// Prepare the GUI for rendering. This should be called between
    /// Yakui::start and Yakui::finish.
    pub fn run(&self, sim: &Sim, frustum: Frustum) {
        if self.show_home_waypoint {
            // Choose a waypoint that appears to be at the origin of the world. 10 steps is sufficient for
            // the waypoint icon's location to be indistinguishable from the correct location.
            let waypoint_location = sim.view_relative_origin(10);

            pad(Pad::all(8.0), || {
                waypoint_icon(waypoint_location, frustum);
            });
        }

        if !self.show_gui {
            return;
        }

        align(Alignment::CENTER, || {
            colored_box(Color::BLACK.with_alpha(0.9), [5.0, 5.0]);
        });

        align(Alignment::TOP_LEFT, || {
            pad(Pad::all(8.0), || {
                colored_box_container(Color::BLACK.with_alpha(0.7), || {
                    let material_count_string = if sim.cfg.gameplay_enabled {
                        sim.count_inventory_entities_matching_material(sim.selected_material())
                            .to_string()
                    } else {
                        "∞".to_string()
                    };
                    label(format!(
                        "Selected material: {:?} (×{})",
                        sim.selected_material(),
                        material_count_string
                    ));
                });
            });
        });
    }
}

/// Add a waypoint icon to the GUI pointing at the given vector
fn waypoint_icon(world_vector: MVector<f32>, frustum: Frustum) -> Response<CanvasResponse> {
    canvas(move |ctx| {
        let viewport = ctx.layout.get(ctx.dom.root()).unwrap().rect;
        let panel_region = ctx.layout.get(ctx.dom.current()).unwrap().rect;

        let c1 = [1.0, 1.0, 1.0, 1.0]; // Fill color
        let c2 = [0.0, 0.0, 0.0, 1.0]; // Outline color
        let r1 = 8.0; // Fill radius
        let r2 = 12.0; // Outline radius

        // Ensure the entire waypoint marker, not just its center, is within `panel_region`.
        let bounds = yakui::Rect::from_pos_size(
            panel_region.pos() + yakui::Vec2::new(r2, r2),
            panel_region.size() - yakui::Vec2::new(r2, r2) * 2.0,
        );

        let target = get_layout_coords_of_world_vector(world_vector, frustum, &viewport, &bounds);
        let vertices = [
            ([r1, 0.0], c1),
            ([0.0, r1], c1),
            ([-r1, 0.0], c1),
            ([0.0, -r1], c1),
            ([r2, 0.0], c2),
            ([0.0, r2], c2),
            ([-r2, 0.0], c2),
            ([0.0, -r2], c2),
        ]
        .map(|(v, c)| new_yakui_vertex(na::Vector2::from(v) + target, [0.0, 0.0], c));

        let indices = [4, 5, 6, 4, 6, 7, 0, 1, 2, 0, 2, 3];
        let mesh = yakui::paint::PaintMesh::new(vertices, indices);
        ctx.paint.add_mesh(mesh);
    })
}

/// Returns the coordinates in Yakui's coordinate system required to indicate a particular world vector.
///
/// For this function to be accurate, the `viewport` rectangle passed in must match the location of the
/// viewport the world is being rendered to. Since Hypermine renders to the entire window, and the UI
/// is an overlay, passing in `ctx.layout.get(ctx.dom.root()).unwrap().rect` where `ctx` refers to `PaintContext`
/// should be sufficient.
///
/// Use the `bounds` rectangle to limit the range of coordinates this function can return. If the world vector
/// cannot be sensibly placed within the bounds, this function will put the marker at the edge of the rectangle in
/// a reasonable location.
fn get_layout_coords_of_world_vector(
    target_world_vector: MVector<f32>,
    frustum: Frustum,
    viewport: &yakui::Rect,
    bounds: &yakui::Rect,
) -> na::Vector2<f32> {
    // The viewport is used to convert from Vulkan's normalized device coordinates to yakui's coordinates.
    let viewport_center: na::Vector2<f32> =
        (viewport.pos() + viewport.size() * 0.5).to_array().into();
    let viewport_scale: na::Vector2<f32> = (viewport.size() * 0.5).to_array().into();

    // The bounds are used to adjust the result if it's too far out of bounds or behind the camera. The
    // computed layout coordinates of a world point are moved towards or away from the center of these
    // bounds to put the marker in a reasonable spot.
    let bounds_center: na::Vector2<f32> = (bounds.pos() + bounds.size() * 0.5).to_array().into();
    let bounds_scale: na::Vector2<f32> = (bounds.size() * 0.5).to_array().into();

    let projected_target = frustum.projection().matrix() * na::Vector4::from(target_world_vector);

    // The following variable represents the target in homogeneous layout-offset coordinates,
    // computed below. These points are in homogeneous layout coordinates (with z being the extra dimension) but
    // centered at `bounds_center` to allow further calculations to scale this vector as needed.
    // Naming note: the word "offset" here is used to note that things are given relative to `bounds_center`.
    let target_homogeneous_layout_offset = na::Vector3::new(
        projected_target.x * viewport_scale.x
            + (viewport_center.x - bounds_center.x) * projected_target.w,
        projected_target.y * viewport_scale.y
            + (viewport_center.y - bounds_center.y) * projected_target.w,
        projected_target.w,
    );

    let target_layout_offset =
        target_homogeneous_layout_offset.xy() / target_homogeneous_layout_offset.z;

    if target_homogeneous_layout_offset.z >= 0.0
        && target_layout_offset.x.abs() <= bounds_scale.x
        && target_layout_offset.y.abs() <= bounds_scale.y
    {
        // If we're in bounds, we already have the answer.
        bounds_center + target_layout_offset
    } else {
        // If we're out of bounds, favor the homogeneous coordinates, ignoring the z-coordinate,
        // as this allows us to reliably get the right direction without dealing with potential
        // infinities or getting the opposite direction if the target is behind the camera.
        let scale_factor = bounds_scale
            .component_div(&target_homogeneous_layout_offset.xy())
            .abs()
            .min();
        let offset = if scale_factor < 1e16 {
            // Scale the offset so that it meets the boundary
            target_homogeneous_layout_offset.xy() * scale_factor
        } else {
            // If the scale factor is too large, the vector we're multiplying is too close to 0.
            // Choose an arbitrary vector on the boundary to be safe.
            na::Vector2::new(0.0, bounds_scale.y)
        };
        bounds_center + offset
    }
}

/// Wrapper function around `yakui::paint::Vertex::new` that takes nalgebra vectors instead of glam vectors
fn new_yakui_vertex(
    position: impl Into<na::Vector2<f32>>,
    texcoord: impl Into<na::Vector2<f32>>,
    color: impl Into<na::Vector4<f32>>,
) -> yakui::paint::Vertex {
    let position = position.into();
    let texcoord = texcoord.into();
    let color = color.into();
    yakui::paint::Vertex::new(
        yakui::Vec2::new(position.x, position.y),
        yakui::Vec2::new(texcoord.x, texcoord.y),
        yakui::Vec4::new(color.x, color.y, color.z, color.w),
    )
}
